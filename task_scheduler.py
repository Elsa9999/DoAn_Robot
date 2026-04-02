# -*- coding: utf-8 -*-
"""
================================================================================
  MODULE: TaskScheduler — Bộ điều phối tác vụ (PLC / Máy trạng thái)
================================================================================
  Vai trò thực tế: Tương đương chương trình PLC viết bằng SFC
  (Sequential Function Chart — Biểu đồ hàm tuần tự theo chuẩn IEC 61131-3).

  Mỗi BƯỚC (Step) trong SFC tương ứng với một STATE ở đây:
    IDLE       → Bước chờ: robot đứng yên, PLC đợi lệnh START
    TO_PREPICK → Bước chạy nhanh đến điểm an toàn trên vật (tránh va chạm)
    TO_PICK    → Bước hạ chậm xuống vật (cẩn thận, vận tốc thấp)
    GRASPING   → Bước kẹp: đóng ngón kẹp, chờ lực kẹp ổn định (~0.8s)
    TO_PREPLACE→ Bước nâng + vận chuyển (đoạn đường dài nhất, tốc độ cao)
    TO_PLACE   → Bước hạ nhẹ xuống điểm đặt
    RELEASING  → Bước nhả: mở ngón kẹp, chờ vật ổn định (~0.8s)
    TO_HOME    → Bước về HOME: robot trở về tư thế chờ an toàn

  Điều kiện chuyển bước (Transition):  alpha >= 1.0  (đã đến đích)
  Tương đương: điều kiện TRANSITION trong SFC (edge/condition)

  Tích hợp:
    - S-Curve Trajectory Planner: làm mượt chuyển động giữa các bước
    - Endless Vision Mode: mỗi chu trình quét camera lại tìm vị trí mới
    - Grasp Yaw: lưu và truyền góc kẹp từ Vision cho IK Solver
    - HMI flags: PAUSE (đóng băng) và E-STOP (nhả vật + về HOME)
================================================================================
"""

import pybullet as p
import time
import math
import numpy as np

from ur5e_robot import TrajectoryPlanner


# ══════════════════════════════════════════════════════════════════════════════
# TÊN CÁC BƯỚC (STEP) — Giống nhãn Step trong SFC của PLC
# Dùng chuỗi thay vì số nguyên để dễ đọc trên HUD và dễ debug
# ══════════════════════════════════════════════════════════════════════════════
STATE_IDLE         = "IDLE"           # Đứng chờ — không làm gì
STATE_TO_PREPICK   = "TO_PREPICK"    # Bay nhanh đến điểm an toàn phía trên vật
STATE_TO_PICK      = "TO_PICK"       # Hạ chậm xuống điểm gắp (vận tốc thấp)
STATE_GRASPING     = "GRASPING"      # Đóng ngón kẹp, giữ nguyên vị trí
STATE_TO_PREPLACE  = "TO_PREPLACE"   # Nâng vật lên cao rồi vận chuyển sang bên kia
STATE_TO_PLACE     = "TO_PLACE"      # Hạ chậm xuống điểm đặt vật
STATE_RELEASING    = "RELEASING"     # Mở ngón kẹp, thả vật, chờ ổn định
STATE_TO_HOME      = "TO_HOME"       # Bay về vị trí HOME an toàn

# Danh sách các bước mà gripper phải GIỮ ĐÚG GÓC XOAY theo vật (Yaw)
# (từ khi chạm vật đến khi nhả vật — không xoay cổ tay ngẫu hứng)
GRASP_STATES = (
    STATE_TO_PICK, STATE_GRASPING,
    STATE_TO_PREPLACE, STATE_TO_PLACE, STATE_RELEASING
)

# Độ cao an toàn khi di chuyển (m) — đủ cao để không va vào vật trên bàn
# Tương đương "clearance height" trong lập trình robot công nghiệp
SAFE_LIFT_HEIGHT = 0.35


# ══════════════════════════════════════════════════════════════════════════════
# TaskScheduler — Máy trạng thái hữu hạn (Finite State Machine)
# ══════════════════════════════════════════════════════════════════════════════

class TaskScheduler:
    """
    Máy trạng thái hữu hạn (FSM) điều khiển tác vụ Pick & Place.

    (Đổi tên từ PickAndPlaceMachine theo yêu cầu refactor OOP)

    Nguyên lý hoạt động:
    - Mỗi trạng thái thực hiện MỘT hành động duy nhất (di chuyển / chờ / kẹp)
    - Điều kiện chuyển State dựa trên:
        + Vị trí EE đã đến đủ gần? (alpha >= 1.0)
        + Đã chờ đủ thời gian ổn định? (elapsed > duration)
    - S-Curve Planner tính điểm đích trung gian tại mỗi bước lặp

    HMI Integration:
    - self.paused  = True → đóng băng segment, robot giữ nguyên vị trí
    - self.e_stop  = True → nhả vật ngay, chuyển về HOME

    Endless Vision Mode:
    - Khi camera != None → mỗi chu trình teleport box + quét camera mới
    - Tự động cập nhật pick_pos + grasp_yaw từ VisionCamera

    Attributes:
        state         : Trạng thái hiện tại
        segment_start : Thời điểm bắt đầu segment hiện tại
        segment_dur   : Thời gian dự kiến cho segment (giây)
        start_pos     : Vị trí EE lúc bắt đầu segment
        target_pos    : Vị trí EE mục tiêu của segment
        is_grasping   : Cờ đang giữ vật hay không
        grasp_yaw     : Góc xoay vật (rad) từ Vision — dùng cho IK
    """

    # ── THỜI GIAN THỰC HIỆN MỖI BƯỚC (giây) ────────────────────────────────
    # Tương đương thông số "Duration" của từng Step trong SFC.
    # Kỹ thuật viên có thể tinh chỉnh các giá trị này để cân bằng tốc độ vs an toàn:
    #   - Tăng TO_PREPICK: robot đi chậm hơn (an toàn hơn khi có người quanh)
    #   - Giảm TO_PICK:    hạ nhanh hơn (năng suất cao hơn nhưng nguy cơ va đập)
    #   - Tăng GRASPING:   chờ lâu hơn để gripper kẹp chắc (vật dễ trượt)
    DURATION = {
        STATE_TO_PREPICK  : 3.0,    # Bay nhanh — đoạn này không gần vật, an toàn
        STATE_TO_PICK     : 1.5,    # Hạ chậm — cần cẩn thận để gripper căn đúng
        STATE_GRASPING    : 0.8,    # Đợi 0.8s để lực kẹp ổn định hoàn toàn
        STATE_TO_PREPLACE : 4.0,    # Đoạn dài nhất: nâng lên + bay qua bàn
        STATE_TO_PLACE    : 1.5,    # Hạ chậm — tránh va đập vào bề mặt
        STATE_RELEASING   : 0.8,    # Đợi 0.8s để vật ổn định trước khi bay đi
        STATE_TO_HOME     : 3.0,    # Bay về HOME — tốc độ vừa phải
    }

    # Điểm đặt vật CỐ ĐỊNH (không thay đổi giữa các chu trình Vision)
    FIXED_PLACE_POS = [0.4, -0.2, 0.025]

    # Vùng ngẫu nhiên trên bàn để teleport khối hộp trước mỗi chu trình mới
    # (Mô phỏng vật thể trên băng tải đến từ vị trí ngẫu nhiên)
    BOX_RAND_X = (0.3, 0.5)    # X ∈ [0.3, 0.5] m tính từ gốc robot
    BOX_RAND_Y = (-0.2, 0.2)   # Y ∈ [-0.2, 0.2] m (hai bên trục giữa)

    def __init__(self, pick_pos: list, place_pos: list, home_pos: list,
                 camera=None, box_id: int = -1):
        """
        Khởi tạo TaskScheduler.

        Args:
            pick_pos  (list): Tọa độ điểm gắp vật [x, y, z]
            place_pos (list): Tọa độ điểm đặt vật [x, y, z]
            home_pos  (list): Tọa độ vị trí Home an toàn [x, y, z]
            camera    (VisionCamera | None):
                        Camera object. Nếu cung cấp → kích hoạt Endless Vision Mode.
            box_id    (int): ID khối hộp đỏ trong PyBullet.
                        Cần thiết để teleport vật sang vị trí ngẫu nhiên.
        """
        self.pick_pos    = pick_pos
        self.place_pos   = place_pos
        self.home_pos    = home_pos

        # ── ENDLESS VISION MODE parameters ───────────────────────────────────
        # Nếu camera và box_id được cung cấp → mỗi chu trình sẽ:
        #   1. Teleport box sang vị trí ngẫu nhiên trên bàn
        #   2. Quét camera để tìm pick_pos mới
        #   3. Đặt lại place_pos về vị trí cố định FIXED_PLACE_POS
        self.camera      = camera       # VisionCamera | None
        self.box_id      = box_id       # int (≥0 nếu hợp lệ)

        # Tính điểm "bay qua" phía TRÊN điểm gắp và điểm đặt (clearance points)
        # Robot cần qua điểm này trước rồi mới hạ xuống — tránh va chạm ngang
        # Chiều cao = SAFE_LIFT_HEIGHT = 0.35m (đủ cao hơn mọi vật trên bàn)
        self.pre_pick_pos  = [pick_pos[0],  pick_pos[1],  SAFE_LIFT_HEIGHT]
        self.pre_place_pos = [place_pos[0], place_pos[1], SAFE_LIFT_HEIGHT]

        # Tọa độ THỰC SỰ mà đầu gripper cần chạm vào vật:
        # Z của vật + 0.06m = offset chiều dài ngón kẹp (gripper nhô ra 6cm trước TCP)
        # Nếu không cộng offset này → TCP đâm vào vật thay vì ngón kẹp bao quanh
        self.pick_contact_pos = [pick_pos[0], pick_pos[1], pick_pos[2] + 0.06]

        # ── TRẠNG THÁI NỘI BỘ CỦA MÁY TRẠNG THÁI ────────────────────────────
        self.state         = STATE_IDLE  # Bước hiện tại đang thực hiện
        self.segment_start = None        # Thời điểm bắt đầu bước hiện tại (giây)
        self.segment_dur   = 0.0         # Thời gian dự kiến của bước hiện tại (giây)
        self.start_pos     = None        # Điểm xuất phát của đoạn chuyển động này
        self.target_pos    = None        # Điểm đích của đoạn chuyển động này
        self.is_grasping   = False       # Cờ báo gripper đang giữ vật (True/False)
        self.status_text_id = -1         # ID text HUD (−1 = chưa tạo)

        self.loop_count    = 0           # Đếm số chu trình hoàn thành (để monitor)
        self.traj_prev_pos = None        # Điểm TCP lần trước (để vẽ đường quỹ đạo)

        # ── GÓC KẸP TỪ VISION — Grasp Orientation Alignment ────────────────
        # Camera đo được khối hộp nghiêng bao nhiêu độ (yaw) so với trục X.
        # Giá trị này được truyền vào IK Solver để gripper xoay cổ tay đúng góc
        # trước khi hạ xuống kẹp → tránh kẹp lệch, không bị trượt vật.
        # Cập nhật mỗi đầu chu trình sau khi camera quét xong.
        self.grasp_yaw = 0.0   # Góc xoay (radian), 0 = song song trục X world

        # ── CỜ TRẠNG THÁI TỪ HMI — Nhận lệnh từ operator ────────────────────
        # Được CẬP NHẬT bởi vòng lặp chính (SimulationApp.run) sau khi đọc HMI.
        # Giống tín hiệu digital I/O từ nút bấm truyền vào PLC.
        self.paused   = False   # True = PAUSE: đóng băng tại vị trí hiện tại
        self.e_stop   = False   # True = E-STOP: nhả vật, về HOME ngay lập tức

    def start(self, current_ee_pos: list):
        """Kích hoạt TaskScheduler, bắt đầu từ trạng thái TO_PREPICK."""
        print("\n" + "=" * 60)
        print("  [TASK] BAT DAU CHU TRINH PICK & PLACE")
        print("=" * 60)
        self._transition(STATE_TO_PREPICK, current_ee_pos, self.pre_pick_pos)

    def _transition(self, new_state: str, from_pos: list, to_pos: list):
        """
        Chuyển sang trạng thái mới, khởi tạo segment quỹ đạo.

        Args:
            new_state (str):  Tên trạng thái tiếp theo
            from_pos  (list): Điểm xuất phát [x,y,z]
            to_pos    (list): Điểm đích [x,y,z]
        """
        self.state         = new_state
        self.segment_start = time.time()
        self.segment_dur   = self.DURATION.get(new_state, 1.0)
        self.start_pos     = list(from_pos)
        self.target_pos    = list(to_pos)

        state_names_vn = {
            STATE_TO_PREPICK  : "[>>] Di chuyen den Pre-Pick",
            STATE_TO_PICK     : "[v]  Ha xuong diem Pick",
            STATE_GRASPING    : "[G]  Dang kep vat...",
            STATE_TO_PREPLACE : "[^]  Nang len va van chuyen",
            STATE_TO_PLACE    : "[v]  Ha xuong diem Place",
            STATE_RELEASING   : "[R]  Dang nha vat...",
            STATE_TO_HOME     : "[H]  Quay ve Home",
        }
        print(f"  [{new_state}] {state_names_vn.get(new_state, '')}")

    def update(self, current_ee_pos: list, current_time: float) -> tuple:
        """
        Hàm cập nhật TaskScheduler — gọi mỗi bước lặp simulation (240Hz).

        Trả về (target_waypoint, is_grasping, grasp_yaw):
        - target_waypoint: Điểm đích TỨC THỜI (đã nội suy S-Curve)
        - is_grasping    : Robot có đang giữ vật không
        - grasp_yaw      : Góc xoay vật (rad) để feed vào IK Solver

        HMI Integration:
        - self.paused = True  → đóng băng segment_start, robot giữ nguyên
        - self.e_stop = True  → chuyển ngay về HOME

        Args:
            current_ee_pos (list):  Vị trí EE hiện tại [x, y, z]
            current_time   (float): Thời gian hiện tại (time.time())

        Returns:
            tuple: (target_waypoint: list, is_grasping: bool, grasp_yaw: float)
        """
        # ── XỬ LÝ E-STOP: Ngắt ngay, nhả vật, bay về HOME ──────────────────
        # Giống tín hiệu HALT trong PLC: bỏ qua bước hiện tại, về vị trí an toàn
        if self.e_stop:
            if self.state != STATE_TO_HOME:
                self.is_grasping = False  # Nhả vật ngay (mở ngón kẹp)
                self._transition(STATE_TO_HOME, current_ee_pos, self.home_pos)
            return TrajectoryPlanner.interpolate(
                self.start_pos, self.target_pos,
                TrajectoryPlanner.s_curve(
                    (current_time - self.segment_start) / self.segment_dur
                )
            ), False, 0.0   # is_grasping=False (đã nhả), yaw=0 (hướng an toàn)

        # ── XỬ LÝ PAUSE: Đóng băng đồng hồ, robot đứng yên tại chỗ ─────────
        # Kỹ thuật: tịnh tiến segment_start về phía trước bằng đúng thời gian đã nghỉ
        # → khi RESUME, alpha tiếp tục từ đúng điểm bị dừng (không nhảy cóc)
        # Tương đương lệnh HOLD trong PLC — robot treo lơ lửng, motor vẫn giữ tải
        if self.paused:
            self.segment_start += current_time - getattr(
                self, '_last_tick', current_time
            )
            self._last_tick = current_time
            return list(current_ee_pos), self.is_grasping, self.grasp_yaw
        self._last_tick = current_time

        if self.state == STATE_IDLE:
            return current_ee_pos, False, 0.0  # Không làm gì khi đang chờ

        # ── TÍNH TIẾN ĐỘ ĐOẠN CHUYỂN ĐỘNG (alpha) ──────────────────────────
        # alpha = thời gian đã qua / thời gian dự kiến → [0.0 → 1.0]
        # alpha=0.0: mới bắt đầu | alpha=0.5: đi được nửa đường | alpha=1.0: đến nơi
        # Capped tại 1.0 để robot không "vượt quá đích"
        # Tương đương: đọc encoder và so sánh với setpoint trong control loop
        elapsed = current_time - self.segment_start
        alpha   = min(elapsed / self.segment_dur, 1.0)

        # Tính tọa độ setpoint tức thời (waypoint) đã làm mượt bằng S-Curve
        # → đây là giá trị được feed vào IK Solver mỗi tick 240Hz
        waypoint = TrajectoryPlanner.interpolate(
            self.start_pos, self.target_pos, alpha
        )

        # ── ĐIỀU KIỆN CHUYỂN BƯỚC (Transitions) ─────────────────────────────
        # Giống điều kiện TRANSITION trong SFC: khi alpha=1.0 → bước hiện tại XONG
        # → kích hoạt bước tiếp theo (tương đương lệnh SET step kế)
        segment_done = (alpha >= 1.0)

        # Đã đến TRÊN điểm gắp → hạ xuống thẳng đứng vào vật
        if self.state == STATE_TO_PREPICK and segment_done:
            self._transition(
                STATE_TO_PICK, self.pre_pick_pos, self.pick_contact_pos
            )

        # Đã hạ xuống đủ → bắt đầu kẹp (đứng yên tại chỗ trong 0.8s)
        elif self.state == STATE_TO_PICK and segment_done:
            self._transition(
                STATE_GRASPING, self.pick_contact_pos, self.pick_contact_pos
            )

        # Đã kẹp ổn định → KÍCH HOẠT lực kẹp và bay sang điểm đặt
        # Đây là bước đánh dấu: từ đây vật theo gripper đến khi RELEASING
        elif self.state == STATE_GRASPING and segment_done:
            self.is_grasping = True   # ← ĐÓNG NGÓN KẸP (gripper.py sẽ kéo vật theo)
            self._transition(
                STATE_TO_PREPLACE, self.pick_contact_pos, self.pre_place_pos
            )

        # Đã đến TRÊN điểm đặt → hạ xuống nhẹ nhàng
        elif self.state == STATE_TO_PREPLACE and segment_done:
            self._transition(
                STATE_TO_PLACE, self.pre_place_pos,
                [self.place_pos[0], self.place_pos[1], self.place_pos[2] + 0.06]
            )

        # Đã hạ xuống điểm đặt → giữ nguyên và chuẩn bị nhả (start = end = vị trí hiện tại)
        elif self.state == STATE_TO_PLACE and segment_done:
            self._transition(
                STATE_RELEASING,
                [self.place_pos[0], self.place_pos[1], self.place_pos[2] + 0.06],
                [self.place_pos[0], self.place_pos[1], self.place_pos[2] + 0.06]
            )

        # Đã nhả xong → MỞ NGÓN KẸP, vật nằm lại trên bàn, robot bay về HOME
        elif self.state == STATE_RELEASING and segment_done:
            self.is_grasping = False  # ← MỞ NGÓN KẸP (gripper.py ngừng kéo vật)
            self._transition(
                STATE_TO_HOME,
                [self.place_pos[0], self.place_pos[1], self.place_pos[2] + 0.06],
                self.home_pos
            )

        # Đã về HOME → kết thúc chu trình, chuẩn bị cho chu trình tiếp theo
        elif self.state == STATE_TO_HOME and segment_done:
            self.loop_count += 1
            print(f"\n  [OK] Hoan thanh chu trinh #{self.loop_count}!")

            # ── CHẾ ĐỘ LẶP VÔ HẠN VỚI VISION (Endless Vision Mode) ──────────
            # Mỗi khi về HOME: đặt lại vật ở vị trí mới → quét camera → lấy pick_pos mới
            # Mô phỏng dây chuyền sản xuất thực: vật liên tục đến từ băng tải
            if self.camera is not None and self.box_id >= 0:
                # ── BƯỚC 1: Dịch chuyển vật đến vị trí ngẫu nhiên mới ────────
                # Tương đương băng tải đưa sản phẩm tiếp theo vào vùng làm việc
                rand_x = np.random.uniform(*self.BOX_RAND_X)
                rand_y = np.random.uniform(*self.BOX_RAND_Y)
                new_box_pos = [rand_x, rand_y, 0.025]
                p.resetBasePositionAndOrientation(
                    self.box_id,
                    new_box_pos,
                    p.getQuaternionFromEuler([0, 0, 0])
                )
                print(f"  [VISION] Box teleport → X={rand_x:+.4f}, Y={rand_y:+.4f}")

                # Chạy 1 bước vật lý để simulation kịp render vật tại vị trí mới
                # (nếu chụp ảnh ngay thì camera vẫn thấy vật ở vị trí CŨ)
                p.stepSimulation()

                # ── BƯỚC 2: Quét camera để lấy pick_pos MỚI + yaw ───────────
                print("  [VISION] Dang quet camera de xac dinh vi tri moi...")
                detected = self.camera.detect_object()

                if detected is not None:
                    # ── BƯỚC 3: Cập nhật pick_pos và grasp_yaw ───────────────
                    self.pick_pos  = [detected.x, detected.y, detected.z]
                    self.grasp_yaw = detected.yaw
                    print(f"  [VISION] pick_pos moi = "
                          f"{[f'{v:+.4f}' for v in self.pick_pos]}")
                    print(f"  [VISION] grasp_yaw    = "
                          f"{math.degrees(self.grasp_yaw):+.1f} deg")
                else:
                    # Fallback: dùng vị trí teleport
                    self.pick_pos = new_box_pos
                    print(f"  [VISION] Fallback: dung vi tri teleport = "
                          f"{new_box_pos}")
                    print(f"  [VISION] Giu nguyen grasp_yaw = "
                          f"{math.degrees(self.grasp_yaw):+.1f} deg")

                # ── BƯỚC 4: Đặt lại place_pos cố định ───────────────────────
                self.place_pos = list(self.FIXED_PLACE_POS)

                # ── BƯỚC 5: Tính lại pre_pick, pick_contact, pre_place ──────
                self.pre_pick_pos     = [self.pick_pos[0],  self.pick_pos[1],
                                         SAFE_LIFT_HEIGHT]
                self.pre_place_pos    = [self.place_pos[0], self.place_pos[1],
                                         SAFE_LIFT_HEIGHT]
                self.pick_contact_pos = [self.pick_pos[0],  self.pick_pos[1],
                                         self.pick_pos[2] + 0.06]

                print(f"  [VISION] place_pos (fixed) = {self.place_pos}")
                print(f"  [VISION] pre_pick_pos      = {self.pre_pick_pos}")
                print(f"  [VISION] pick_contact_pos  = {self.pick_contact_pos}")
                print(f"  [EYES]   Chuan bi chu trinh #{self.loop_count + 1}"
                      f" voi vi tri pick moi...\n")

            else:
                # ── LEGACY MODE: Đảo pick/place (không có Vision) ────────────
                print(f"  [LEGACY] Khong co Vision — dao vi tri pick/place.")
                self.pick_pos, self.place_pos = \
                    self.place_pos, self.pick_pos
                self.pre_pick_pos, self.pre_place_pos = \
                    self.pre_place_pos, self.pre_pick_pos
                self.pick_contact_pos = [
                    self.pick_pos[0], self.pick_pos[1],
                    self.pick_pos[2] + 0.06
                ]

            self.state = STATE_IDLE
            # Nghỉ ngắn để camera có thời gian render box ở vị trí mới
            time.sleep(0.3)
            self.start(current_ee_pos)

        # ── VẼ DẤU VẾT QUỸ ĐẠO TCP (Debug Trail) ────────────────────────────
        # Vẽ đường thẳng nhỏ từ điểm TCP lần trước đến lần này → tạo ra vệt xanh
        # Chỉ vẽ khi robot đang DI CHUYỂN (không vẽ khi đứng yên kẹp/nhả)
        # Lợi ích kỹ thuật: nhìn thấy quỹ đạo thực tế để kiểm tra không va chướng ngại vật
        # Tương đương chức năng "Path Trace" trong phần mềm offline programming (RobotStudio)
        if (self.traj_prev_pos is not None and
                self.state not in (STATE_IDLE, STATE_GRASPING, STATE_RELEASING)):
            dist_sq = sum(
                (a - b) ** 2
                for a, b in zip(current_ee_pos, self.traj_prev_pos)
            )
            # Chỉ vẽ nếu TCP đã di chuyển đủ xa (tránh vẽ điểm chồng nhau khi đứng yên)
            if dist_sq > 1e-5:
                p.addUserDebugLine(
                    self.traj_prev_pos, current_ee_pos,
                    lineColorRGB=[0.2, 0.8, 1.0],  # Màu xanh lam nhạt
                    lineWidth=2, lifeTime=8.0        # Tồn tại 8 giây rồi tự xóa
                )
        self.traj_prev_pos = list(current_ee_pos)  # Lưu lại vị trí hiện tại cho lần sau

        return waypoint, self.is_grasping, self.grasp_yaw

    def update_hud(self, ee_pos: list, status_text_id: int,
                   z_offset: float = 0.0) -> int:
        """
        Cập nhật bảng HUD (Heads-Up Display) trong không gian 3D.

        Hiển thị:
          - Trạng thái HMI (RUN / PAUSE / E-STOP)
          - State hiện tại của FSM
          - Chu trình đang thực hiện
          - Tọa độ EE
          - Z-Offset từ HMI slider
          - Trạng thái kẹp/nhả

        Args:
            ee_pos         (list):  Tọa độ End-Effector [x, y, z]
            status_text_id (int):   ID debug text object (-1 nếu chưa tạo)
            z_offset       (float): Giá trị Z-Offset từ HMI slider (m)

        Returns:
            int: ID debug text object (dùng để update lần sau)
        """
        # Xác định dòng trạng thái HMI
        if self.e_stop:
            hmi_status = "[E-STOP] EMERGENCY HOME"
            hud_color  = [1.0, 0.2, 0.1]       # Đỏ báo động
        elif self.paused:
            hmi_status = "[PAUSE]  Dang tam dung"
            hud_color  = [1.0, 0.6, 0.0]       # Cam cảnh báo
        else:
            hmi_status = "[RUN]    Dang chay"
            hud_color  = ([0.2, 1.0, 0.5] if self.is_grasping
                          else [1.0, 0.95, 0.4])

        hud_text = (
            f"HMI    : {hmi_status}\n"
            f"State  : {self.state}\n"
            f"Cycle  : #{self.loop_count}\n"
            f"EE Pos : ({ee_pos[0]:+.3f}, {ee_pos[1]:+.3f}, "
            f"{ee_pos[2]:+.3f})\n"
            f"Z-Off  : {z_offset:+.3f} m\n"
            f"Grasp  : {'ON  [KEP]' if self.is_grasping else 'OFF [NHA]'}"
        )

        if status_text_id == -1:
            return p.addUserDebugText(
                hud_text, [-0.6, -0.5, 0.9],
                textColorRGB=hud_color, textSize=1.05
            )
        else:
            p.addUserDebugText(
                hud_text, [-0.6, -0.5, 0.9],
                textColorRGB=hud_color, textSize=1.05,
                replaceItemUniqueId=status_text_id
            )
            return status_text_id
