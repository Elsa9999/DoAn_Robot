# -*- coding: utf-8 -*-
"""
================================================================================
  MODULE: UR5eRobot — Bộ điều khiển tay máy UR5e trong môi trường mô phỏng
================================================================================
  Vai trò thực tế: Tương đương bộ Robot Controller của Universal Robots
  (hộp tủ điện UR Control Box đặt cạnh robot ngoài xưởng).

  Nhiệm vụ chính:
    1. Nạp bản vẽ cơ học (URDF) của robot vào môi trường vật lý ảo
    2. Đọc thông số từng khớp (giới hạn góc, lực tối đa, vận tốc tối đa)
    3. Nhận tọa độ đích (X, Y, Z) → giải bài toán Động học Ngược → ra góc 6 khớp
    4. Cấp lệnh setpoint góc cho từng servo motor

  Tham khảo kiến trúc:
    - AIRobot (MIT): gom toàn bộ thao tác robot vào một object duy nhất
    - pybullet_ur5_robotiq: tự động đọc thông số khớp từ file URDF
================================================================================
"""

import pybullet as p
import math
from collections import namedtuple


# ══════════════════════════════════════════════════════════════════════════════
# "Phiếu kỹ thuật" của từng khớp — lưu đầy đủ thông số như datasheet khớp
# (thay vì phải gọi lại PyBullet mỗi lần cần tra cứu)
# ══════════════════════════════════════════════════════════════════════════════
JointInfo = namedtuple('JointInfo', [
    'id', 'name', 'type', 'damping', 'friction',
    'lower_limit', 'upper_limit', 'max_force', 'max_velocity', 'controllable'
])
# Giải thích từng trường:
#   id          → số thứ tự khớp trong file URDF (0=khớp đế, 5=cổ tay)
#   name        → tên khớp theo URDF (vd: "shoulder_pan_joint")
#   type        → loại khớp: quay (REVOLUTE) hay cố định (FIXED)
#   damping     → hệ số cản nhớt của khớp (từ URDF)
#   lower_limit → góc xoay tối thiểu (radian) — giới hạn phần cứng
#   upper_limit → góc xoay tối đa (radian) — giới hạn phần cứng
#   max_force   → mô-men xoắn tối đa servo có thể tạo ra (N·m)
#   max_velocity→ tốc độ góc tối đa (rad/s)
#   controllable→ True nếu là khớp quay (có thể điều khiển), False nếu cố định


# ══════════════════════════════════════════════════════════════════════════════
# BỘ QUY HOẠCH QUỸ ĐẠO — Làm mượt chuyển động theo đường cong hình chữ S
# ══════════════════════════════════════════════════════════════════════════════

class TrajectoryPlanner:
    """
    Tương đương khối Motion Profile Generator trong bộ điều khiển công nghiệp
    (thường gọi là "S-Curve Profile" hay "Jerk-Limited Profile").

    Mục đích: Thay vì cho robot chạy thẳng với tốc độ không đổi (gây giật cơ học),
    bộ này tự động làm mượt: khởi động từ từ → đạt tốc độ max → phanh dần.
    Giống như cabin thang máy — không bao giờ bật/tắt tốc độ đột ngột.
    """

    @staticmethod
    def s_curve(t: float) -> float:
        """
        Tính hệ số làm mượt tại thời điểm t (t chạy từ 0.0 đến 1.0).

        Tại t=0.0 → hệ số = 0.0 (robot chưa nhúc nhích)
        Tại t=0.5 → hệ số = 0.5 (robot đang ở giữa đường, chạy nhanh nhất)
        Tại t=1.0 → hệ số = 1.0 (robot đã đến đích, dừng hoàn toàn)

        Đặc điểm quan trọng: tốc độ thay đổi (gia tốc) bằng 0 tại điểm đầu
        và điểm cuối → không có hiện tượng giật cơ học.

        Công thức Quintic: f(t) = 6t⁵ − 15t⁴ + 10t³
        """
        # Giới hạn t trong [0, 1] để tránh tính toán ngoài vùng
        t = max(0.0, min(1.0, t))
        return 6 * (t ** 5) - 15 * (t ** 4) + 10 * (t ** 3)

    @staticmethod
    def interpolate(start_pos: list, end_pos: list, alpha: float) -> list:
        """
        Tính tọa độ trung gian giữa điểm xuất phát và điểm đích,
        có áp dụng hệ số S-Curve để làm mượt chuyển động.

        Ví dụ thực tế:
          start_pos = [0.3, 0.0, 0.5] (robot đang ở đây)
          end_pos   = [0.4, 0.2, 0.3] (robot cần đến đây)
          alpha=0.5 → trả về điểm ở giữa (nhưng đã qua S-Curve nên
                       không phải giữa đúng—robot cử động mượt hơn nhiều)
        """
        # Tính hệ số làm mượt (s ∈ [0,1])
        s = TrajectoryPlanner.s_curve(alpha)
        # Nội suy tuyến tính trên 3 trục: điểm = start + s × (end - start)
        return [
            start_pos[0] + s * (end_pos[0] - start_pos[0]),
            start_pos[1] + s * (end_pos[1] - start_pos[1]),
            start_pos[2] + s * (end_pos[2] - start_pos[2]),
        ]


# ══════════════════════════════════════════════════════════════════════════════
# BỘ ĐIỀU KHIỂN TAY MÁY UR5e — Quản lý toàn bộ thao tác với robot vật lý ảo
# ══════════════════════════════════════════════════════════════════════════════

class UR5eRobot:
    """
    Tương đương toàn bộ "Robot Controller Cabinet" của UR5e ngoài thực tế:
      - Phần cứng: tủ điện UR Control Box (CB5)
      - Phần mềm: PolyScope OS chạy bên trong

    Khi khởi tạo (bật nguồn robot thực):
      → Nạp bản vẽ cơ học (URDF) — giống load cấu hình robot vào controller
      → Đọc thông số 6 khớp (giới hạn, lực max, vận tốc max)
      → Xác định điểm TCP (Tool Center Point) — đầu mút công cụ
    """

    # ── GIỚI HẠN GÓC XOAY TỪNG KHỚP (lấy từ datasheet UR5e e-Series) ───────
    # Thứ tự 6 khớp: [vai quay, vai nâng, khuỷu tay, cổ tay 1, cổ tay 2, cổ tay 3]
    # ±360° cho hầu hết khớp, riêng khuỷu tay ±180° (tránh va tự thân)
    # Đây là "software limit" — chương trình sẽ không yêu cầu vượt qua các ngưỡng này
    LOWER_LIMITS = [-2*math.pi, -2*math.pi, -math.pi,   -2*math.pi, -2*math.pi, -2*math.pi]
    UPPER_LIMITS = [ 2*math.pi,  2*math.pi,  math.pi,    2*math.pi,  2*math.pi,  2*math.pi]
    # Phạm vi xoay = upper - lower (dùng để bộ giải IK biết "độ tự do" của mỗi khớp)
    JOINT_RANGES = [ 4*math.pi,  4*math.pi,  2*math.pi,  4*math.pi,  4*math.pi,  4*math.pi]

    # ── TƯ THẾ "ĐỨNG NGHỈ" AN TOÀN — Elbow-Up chuẩn công nghiệp ─────────────
    # Khi bộ giải IK có nhiều nghiệm (như bài toán có vô số đáp án), robot sẽ
    # CHỌN nghiệm nào gần với tư thế nghỉ này nhất → luôn giữ khuỷu tay hướng lên.
    # Lợi ích: tránh va đập vật cản, tránh điểm kỳ dị (singularity), trông tự nhiên.
    # [vai quay=0°, vai nâng=−90°, khuỷu=+90°, cổ tay1=−90°, cổ tay2=−90°, cổ tay3=0°]
    REST_POSES = [0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0]

    # ── THÔNG SỐ BỘ GIẢI IK ──────────────────────────────────────────────────
    # (tương đương cấu hình bộ tính toán quỹ đạo trong controller thực)
    IK_MAX_ITERATIONS     = 200     # Cho phép tính lại tối đa 200 lần để hội tụ
    IK_RESIDUAL_THRESHOLD = 1e-5    # Chấp nhận nếu sai số < 0.01mm — đủ chính xác
    IK_JOINT_DAMPING      = 0.01    # Lực cản nhỏ thêm vào mỗi khớp: chống rung
                                    # (giống thêm dashpot vào mỗi trục servo)

    # ── GIỚI HẠN ĐỘNG LỰC HỌC MOTOR ─────────────────────────────────────────
    DEFAULT_MAX_FORCE = 200         # Mô-men xoắn tối đa: 200 N·m (giống thông số servo)
    DEFAULT_MAX_VEL   = 1.5         # Tốc độ góc tối đa: 1.5 rad/s ≈ 86°/giây

    # ── TÊN ĐIỂM TCP THEO CHUẨN URDF UR5e ───────────────────────────────────
    # Danh sách ưu tiên để tìm đúng điểm đầu công cụ (TCP/Tool Center Point)
    EE_CANDIDATE_NAMES = ["ee_link", "tool0", "flange", "wrist_3_link"]

    def __init__(self, urdf_path: str = "ur5e.urdf",
                 base_position: list = None):
        """
        Khởi tạo robot — tương đương BẬT NGUỒN và BOOT hệ thống robot thực.

        urdf_path: đường dẫn file bản vẽ cơ học (URDF).
                   File này mô tả kết cấu cơ khí: chiều dài cánh tay,
                   khối lượng từng bộ phận, giới hạn khớp nối.
        base_position: tọa độ đặt chân đế robot trong không gian 3D.
        """
        if base_position is None:
            base_position = [0, 0, 0]  # Mặc định: đặt robot tại gốc tọa độ

        # ── Nạp bản vẽ robot vào môi trường vật lý ảo ────────────────────────
        # useFixedBase=True: chân đế được bu-lông vào sàn (không bị ngã)
        # Trả về một ID số nguyên — giống "địa chỉ" của robot trong simulation
        self.robot_id = p.loadURDF(
            urdf_path,
            basePosition=base_position,
            useFixedBase=True
        )
        self.base_position = base_position

        # ── Chuẩn bị bộ nhớ lưu thông tin các khớp ──────────────────────────
        self.joints       = []    # Danh sách "phiếu kỹ thuật" của TẤT CẢ khớp
        self.active_joints = []   # Chỉ các khớp QUAY được — đây là khớp cần điều khiển
        self._joint_name_map = {} # Bảng tra nhanh: tên khớp → số thứ tự

        # Đọc và phân loại tất cả khớp từ file URDF
        self._parse_joint_info()

        # Xác định điểm TCP (đầu công cụ) của robot
        self.ee_link_index = self._find_ee_link()

        # ── Tạo bộ hệ số chống rung cho TẤT CẢ khớp (kể cả khớp cố định) ──
        # PyBullet yêu cầu mảng này phải đủ số phần tử bằng tổng số khớp
        self._num_joints    = p.getNumJoints(self.robot_id)
        self._joint_damping = [self.IK_JOINT_DAMPING] * self._num_joints

        print(f"  [ROBOT] UR5e loaded. Active joints: {len(self.active_joints)}. "
              f"EE index: {self.ee_link_index}")

    def _parse_joint_info(self):
        """
        Đọc và lưu thông số kỹ thuật của từng khớp từ file URDF.

        Giống như kỹ thuật viên đọc datasheet của từng servo motor:
        - Motor nào là khớp quay (cần điều khiển)?
        - Motor nào là khớp cố định (bu-lông, bỏ qua)?
        - Giới hạn góc và lực là bao nhiêu?
        """
        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            info = p.getJointInfo(self.robot_id, i)
            # Đóng gói thông số vào namedtuple cho dễ tra cứu sau này
            joint = JointInfo(
                id          = info[0],
                name        = info[1].decode('utf-8'),  # Tên dạng bytes → text
                type        = info[2],
                damping     = info[6],
                friction    = info[7],
                lower_limit = info[8],   # Góc xoay tối thiểu (radian)
                upper_limit = info[9],   # Góc xoay tối đa (radian)
                max_force   = info[10],  # Mô-men xoắn tối đa (N·m)
                max_velocity= info[11],  # Tốc độ góc tối đa (rad/s)
                controllable= (info[2] != p.JOINT_FIXED)  # True nếu là khớp quay
            )
            self.joints.append(joint)
            self._joint_name_map[joint.name] = joint.id

            # Chỉ thêm vào danh sách điều khiển nếu là KHỚP QUAY
            if info[2] == p.JOINT_REVOLUTE:
                self.active_joints.append(joint.id)

    def _find_ee_link(self) -> int:
        """
        Xác định điểm TCP (Tool Center Point) — đầu mút công cụ của robot.

        TCP là điểm tham chiếu quan trọng nhất: tất cả lệnh di chuyển đều
        tính từ điểm này. Trong robot thực, TCP được hiệu chỉnh (calibrate)
        bằng cách dạy robot chạm vào điểm cố định từ nhiều góc.

        Ở đây: tìm link có tên chuẩn trong URDF của UR5e.
        """
        for candidate in self.EE_CANDIDATE_NAMES:
            if candidate in self._joint_name_map:
                idx = self._joint_name_map[candidate]
                print(f"  [EE]   Tim thay TCP link: '{candidate}' (index={idx})")
                return idx

        # Nếu không tìm thấy theo tên → dùng link cuối cùng làm dự phòng
        fallback = p.getNumJoints(self.robot_id) - 1
        print(f"  [EE]   Khong tim thay TCP theo ten, dung link cuoi: index={fallback}")
        return fallback

    def reset_to_pose(self, angles: list = None):
        """
        Đặt robot vào tư thế cho trước ngay lập tức (bỏ qua vật lý).

        Dùng khi khởi động hoặc sau E-Stop — giống chức năng "Jog to position"
        ở tốc độ cao nhất trong chế độ thủ công của PolyScope.

        Không có gia tốc hay làm mượt — robot nhảy thẳng đến tư thế đích.
        Chỉ dùng khi chưa chạy simulation (tránh gây giật cơ học).
        """
        if angles is None:
            angles = self.REST_POSES  # Về tư thế Elbow-Up mặc định

        # Đặt từng khớp về góc cho trước, theo thứ tự từ khớp 0 đến khớp 5
        for i, joint_idx in enumerate(self.active_joints):
            p.resetJointState(
                self.robot_id, joint_idx,
                angles[i] if i < len(angles) else 0
            )

    def get_ee_pose(self) -> tuple:
        """
        Đọc vị trí và hướng hiện tại của điểm TCP.

        Tương đương đọc giá trị "Current Tool Position" trong PolyScope:
          → Vị trí (X, Y, Z) tính bằng mét trong hệ tọa độ thế giới
          → Hướng (quaternion) = cách mà gripper đang xoay trong không gian

        computeForwardKinematics=1: tính toán lại từ góc khớp hiện tại
        (đảm bảo giá trị mới nhất sau khi motor vừa di chuyển)
        """
        state = p.getLinkState(
            self.robot_id, self.ee_link_index,
            computeLinkVelocity=0,        # Không cần tốc độ, tiết kiệm tính toán
            computeForwardKinematics=1    # Tính FK mới nhất
        )
        # state[4] = vị trí thực tế của TCP trong world frame (sau FK)
        # state[5] = hướng TCP dưới dạng quaternion [x, y, z, w]
        return list(state[4]), state[5]

    def get_ee_position(self) -> list:
        """
        Đọc nhanh vị trí TCP (chỉ lấy X, Y, Z, không tính lại FK).

        Dùng trong vòng lặp điều khiển 240Hz — ưu tiên tốc độ hơn độ chính xác.
        Giá trị lấy từ bước tính toán vật lý gần nhất (đủ chính xác cho điều khiển).
        """
        state = p.getLinkState(self.robot_id, self.ee_link_index)
        return list(state[0])  # state[0] = linkWorldPosition (cập nhật sau mỗi stepSimulation)

    def get_joint_states(self) -> list:
        """
        Đọc góc hiện tại của tất cả 6 khớp xoay (radian).

        Tương đương đọc encoder phản hồi từ servo motor —
        biết robot đang ở tư thế nào trong thực tế (feedback position).
        """
        return [p.getJointState(self.robot_id, j)[0] for j in self.active_joints]

    def move_to(self, target_pos: list, orientation=None,
                max_force: float = None, max_vel: float = None):
        """
        LỆNH DI CHUYỂN CHÍNH: Đưa TCP đến tọa độ mục tiêu.

        Đây là bước quan trọng nhất — thực hiện 2 việc:

        BƯỚC 1 — Giải Động học Ngược (Inverse Kinematics):
          "Để điểm TCP đến [X, Y, Z] với hướng [roll, pitch, yaw] cho trước,
           mỗi khớp trong 6 khớp phải xoay bao nhiêu độ?"
           → Đây là bài toán toán học phức tạp, solver tự giải.

        BƯỚC 2 — Cấp setpoint cho servo motor:
          "Khớp 0 → về 15°, Khớp 1 → về −72°, Khớp 2 → về 95°, ..."
           → Giống ghi giá trị vào thanh ghi tham chiếu của biến tần/driver.

        Tham số an toàn:
          - Null-Space: nếu có nhiều nghiệm IK, chọn cái gần Elbow-Up nhất
          - jointDamping: thêm lực cản nhỏ để servo không dao động
        """
        if max_force is None:
            max_force = self.DEFAULT_MAX_FORCE
        if max_vel is None:
            max_vel = self.DEFAULT_MAX_VEL

        if orientation is None:
            # Mặc định: mũi kẹp hướng thẳng đứng xuống dưới — tư thế gắp chuẩn
            # Euler [π, 0, 0] nghĩa là lật úp theo trục X → TCP trỏ xuống
            orientation = p.getQuaternionFromEuler([math.pi, 0, 0])

        # ── BƯỚC 1: Giải Động học Ngược ─────────────────────────────────────
        # calculateInverseKinematics = "bài toán ngược": biết đích, tính góc khớp
        #
        # Null-Space parameters: khi IK có nhiều nghiệm (vô số cách để tay đến đích),
        # chỉ định solver phải chọn nghiệm gần REST_POSES nhất
        # → robot luôn giữ tư thế khuỷu tay hướng lên, không xoay kỳ dị
        #
        # jointDamping: thêm hệ số cản vào từng khớp trong quá trình tính toán
        # → các khớp không cần thiết sẽ "bị giữ lại" thay vì dao động tự do
        joint_angles = p.calculateInverseKinematics(
            self.robot_id,
            self.ee_link_index,
            target_pos,
            orientation,
            lowerLimits      = self.LOWER_LIMITS,      # Giới hạn góc từ datasheet
            upperLimits      = self.UPPER_LIMITS,
            jointRanges      = self.JOINT_RANGES,
            restPoses        = self.REST_POSES,         # Ưu tiên tư thế Elbow-Up
            jointDamping     = self._joint_damping,     # Chống rung
            maxNumIterations = self.IK_MAX_ITERATIONS,  # Tối đa 200 lần thử hội tụ
            residualThreshold= self.IK_RESIDUAL_THRESHOLD  # Chấp nhận nếu lỗi < 0.01mm
        )

        # ── BƯỚC 2: Gửi setpoint góc cho từng servo motor ───────────────────
        # PyBullet trả về mảng joint_angles có ĐỦ cho TẤT CẢ khớp (kể cả fixed).
        # Phải dùng đúng joint_idx làm chỉ số — không phải số đếm 0,1,2,3...
        # Nếu nhầm điều này, robot sẽ điều khiển nhầm khớp!
        for joint_idx in self.active_joints:
            if joint_idx < len(joint_angles):
                p.setJointMotorControl2(
                    bodyIndex     = self.robot_id,
                    jointIndex    = joint_idx,
                    controlMode   = p.POSITION_CONTROL,        # Chế độ điều khiển vị trí
                    targetPosition= joint_angles[joint_idx],   # Setpoint góc (rad)
                    force         = max_force,   # Mô-men xoắn tối đa (N·m)
                    maxVelocity   = max_vel      # Vận tốc góc tối đa (rad/s)
                )
