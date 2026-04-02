# -*- coding: utf-8 -*-
"""
================================================================================
  ĐỒ ÁN TỐT NGHIỆP: Mô phỏng Cánh Tay Máy UR5e - Hệ thống Pick & Place
================================================================================
  Tác giả  : [Tên sinh viên]
  MSSV     : [MSSV]
  Môn học  : Đồ án tốt nghiệp
  Thư viện : PyBullet, NumPy, time, math, OpenCV (cv2)
--------------------------------------------------------------------------------
  KIẾN TRÚC HỆ THỐNG (OOP Refactored):

  Lấy cảm hứng từ:
    - AIRobot (MIT/Improbable-AI): Modular, injectable, unified API
    - pybullet_ur5_robotiq (ElectronicElephant): RobotBase + namedtuple

  Cấu trúc Module:
  ┌─────────────────────────────────────────────────────────────┐
  │  simulation.py   — SimulationApp (Orchestrator chính)       │
  │  ur5e_robot.py   — UR5eRobot + TrajectoryPlanner            │
  │  vision_camera.py— VisionCamera + DetectionResult           │
  │  gripper.py      — Gripper (constraint-based grasping)      │
  │  task_scheduler.py— TaskScheduler (FSM Pick & Place)        │
  │  hmi_panel.py    — HMIPanel (GUI debug parameters)          │
  └─────────────────────────────────────────────────────────────┘

  LUỒNG DỮ LIỆU:
  VisionCamera → DetectionResult → TaskScheduler → TrajectoryPlanner
       ↓                ↓
  UR5eRobot.move_to() ← waypoint + grasp_orn
       ↓
  Gripper.update_pose() ← EE pose (FK)
================================================================================
"""

import pybullet as p
import pybullet_data
import time
import math
import sys

# Force UTF-8 stdout để tránh lỗi UnicodeEncodeError trên Windows (cp1252)
if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

# ── Import các Module OOP ────────────────────────────────────────────────────
from ur5e_robot import UR5eRobot
from vision_camera import VisionCamera
from gripper import Gripper
from task_scheduler import (
    TaskScheduler,
    STATE_IDLE, STATE_TO_PICK, STATE_GRASPING,
    STATE_TO_PREPLACE, STATE_TO_PLACE, STATE_RELEASING,
    GRASP_STATES
)
from hmi_panel import HMIPanel
from scada_gui import ScadaPanel


# ══════════════════════════════════════════════════════════════════════════════
# SimulationApp — Orchestrator chạy toàn bộ hệ thống
# ══════════════════════════════════════════════════════════════════════════════

class SimulationApp:
    """
    Lớp tổng hợp (Orchestrator) — khởi tạo environment, tạo components,
    chạy main loop.

    Design Pattern:
      - AIRobot: pb_client inject vào constructor (robot không tạo connection)
      - pybullet_ur5_robotiq: Environment riêng, Robot riêng
      - Composition over Inheritance: SimulationApp SỞ HỮU các component

    Thứ tự khởi động:
      1. Kết nối PyBullet GUI, cấu hình Physics
      2. Load UR5e (via UR5eRobot class)
      3. Dựng cảnh: bàn + vật + markers
      4. VisionCamera scan vị trí vật lần đầu
      5. Tạo Gripper, TaskScheduler, HMIPanel
      6. Vòng lặp Real-time 240Hz
    """

    def __init__(self):
        """Khởi tạo toàn bộ hệ thống mô phỏng."""

        # ── 1. KẾT NỐI VÀ CẤU HÌNH PYBULLET ────────────────────────────────
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        # Tắt các panel GUI mặc định
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

        # Góc nhìn camera: 3/4 nhìn toàn bộ bàn làm việc
        p.resetDebugVisualizerCamera(
            cameraDistance=1.4,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0.3, 0.0, 0.2]
        )

        # ── 2. LOAD MÔI TRƯỜNG VÀ ROBOT ─────────────────────────────────────
        p.loadURDF("plane.urdf")

        print("\n" + "=" * 60)
        print("  [START] KHOI DONG HE THONG MO PHONG UR5e PICK & PLACE")
        print("=" * 60)
        print("  [INFO] Dang nap Robot UR5e...")

        # Tạo robot qua OOP class (tự động parse joints, tìm EE)
        self.robot = UR5eRobot(urdf_path="ur5e.urdf")

        # Đặt tư thế ban đầu Elbow-Up
        self.robot.reset_to_pose()

        print(f"  [OK]  Robot khoi dong. Tong khop: "
              f"{len(self.robot.active_joints)} (revolute). "
              f"EE index: {self.robot.ee_link_index}")

        # ── 3. DỰNG CẢNH ────────────────────────────────────────────────────
        print("  [INFO] Dang dung canh mo phong...")
        self.scene = self._setup_scene()
        print(f"  [BOX]  Vat the can gap tai (hardcode): "
              f"{self.scene['pick_pos']}")
        print(f"  [TGT]  Vi tri dat vat tai             : "
              f"{self.scene['place_pos']}")

        # ── 3b. KHỞI TẠO CAMERA ẢO ──────────────────────────────────────────
        # Step simulation để render cảnh trước khi chụp ảnh
        for _ in range(10):
            p.stepSimulation()
        self.camera = VisionCamera()

        # ── 3c. NHẬN DIỆN VẬT THỂ LẦN ĐẦU ──────────────────────────────────
        print("\n  [VISION] Dang quet camera de xac dinh vi tri vat...")
        detected = self.camera.detect_object()

        if detected is not None:
            self.scene['pick_pos'] = [detected.x, detected.y, detected.z]
            self.init_grasp_yaw    = detected.yaw
            print(f"  [VISION] Da cap nhat pick_pos = "
                  f"{[f'{v:+.4f}' for v in self.scene['pick_pos']]}")
            print(f"  [VISION] Goc kep ban dau     = "
                  f"{math.degrees(self.init_grasp_yaw):+.1f} deg")
            print(f"  [VISION] Vision Module hoan thanh. Pick pos duoc thay the!")
        else:
            self.init_grasp_yaw = 0.0
            print(f"  [VISION] Fallback: Giu nguyen pick_pos = "
                  f"{self.scene['pick_pos']}")
        print()

        # ── 4. TẠO GRIPPER ───────────────────────────────────────────────────
        self.gripper = Gripper(self.robot)

        # ── 5. KHỞI TẠO TASK SCHEDULER ───────────────────────────────────────
        self.task = TaskScheduler(
            pick_pos  = self.scene['pick_pos'],
            place_pos = self.scene['place_pos'],
            home_pos  = self.scene['home_pos'],
            camera    = self.camera,                  # Kích hoạt Endless Vision
            box_id    = self.scene['box_id']          # ID khối hộp để teleport
        )
        # Seed góc kẹp ban đầu từ lần quét Vision đầu tiên
        self.task.grasp_yaw = self.init_grasp_yaw

        # Lấy vị trí EE ban đầu
        p.stepSimulation()
        init_ee_pos = self.robot.get_ee_position()

        # Cho robot ổn định tư thế ban đầu (chạy 100 bước)
        print("  [WAIT] Robot dang on dinh tu the ban dau...")
        for _ in range(100):
            self.robot.move_to(self.scene['home_pos'])
            p.stepSimulation()
            time.sleep(1. / 240.)

        # Bắt đầu chu trình Pick & Place
        self.task.start(init_ee_pos)

        # ── 6. TẠO HMI PANEL ────────────────────────────────────────────────
        self.hmi = HMIPanel()
        self.hud_id = -1
        self._manual_hud_id = -1   # HUD cảnh báo MANUAL MODE riêng

        # ── 7. TẠO SCADA PANEL (tkinter) ─────────────────────────────────────
        self.scada = ScadaPanel()

        print(f"\n  [RUN] Vong lap Real-time dang chay (240Hz). "
              f"Nhan Ctrl+C de dung.\n")

    @staticmethod
    def _setup_scene() -> dict:
        """
        Dựng đầy đủ cảnh mô phỏng: bàn làm việc + vật thể + vị trí đặt.

        Cấu hình bài toán Pick & Place:
          - Vật cần gắp (Box màu đỏ):   tại PICK_POS
          - Vị trí đặt (Marker xanh lá): tại PLACE_POS
          - Bàn làm việc phẳng:          plane.urdf (đã load trước)

        Returns:
            dict: {
                'box_id'    : ID vật thể cần gắp,
                'pick_pos'  : Tọa độ điểm gắp [x,y,z],
                'place_pos' : Tọa độ điểm đặt [x,y,z],
                'home_pos'  : Vị trí HOME của EE
            }
        """
        # --- Tọa độ cấu hình ---
        PICK_POS    = [0.4,  0.2, 0.025]
        PLACE_POS   = [0.4, -0.2, 0.025]
        HOME_POS    = [0.3,  0.0, 0.6]
        BOX_HALF_Z  = 0.025

        # --- Vật cần gắp: khối hộp màu đỏ ---
        box_collision = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[0.025, 0.025, BOX_HALF_Z]
        )
        box_visual = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[0.025, 0.025, BOX_HALF_Z],
            rgbaColor=[0.9, 0.15, 0.1, 1.0]
        )
        box_id = p.createMultiBody(
            baseMass=0.3,
            baseCollisionShapeIndex=box_collision,
            baseVisualShapeIndex=box_visual,
            basePosition=PICK_POS
        )
        p.changeDynamics(
            box_id, -1,
            lateralFriction=0.8,
            spinningFriction=0.3
        )

        # --- Marker điểm PICK (cột vàng nhỏ) ---
        pick_marker_col = p.createCollisionShape(
            p.GEOM_CYLINDER, radius=0.01, height=0.002
        )
        pick_marker_vis = p.createVisualShape(
            p.GEOM_CYLINDER, radius=0.04, length=0.002,
            rgbaColor=[1.0, 0.8, 0.0, 0.7]
        )
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=pick_marker_col,
            baseVisualShapeIndex=pick_marker_vis,
            basePosition=[PICK_POS[0], PICK_POS[1], 0.001]
        )

        # --- Marker điểm PLACE (đĩa xanh lá) ---
        place_marker_col = p.createCollisionShape(
            p.GEOM_CYLINDER, radius=0.01, height=0.002
        )
        place_marker_vis = p.createVisualShape(
            p.GEOM_CYLINDER, radius=0.04, length=0.002,
            rgbaColor=[0.0, 0.85, 0.2, 0.7]
        )
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=place_marker_col,
            baseVisualShapeIndex=place_marker_vis,
            basePosition=[PLACE_POS[0], PLACE_POS[1], 0.001]
        )

        # --- Nhãn text 3D ---
        p.addUserDebugText(
            "📦 PICK", [PICK_POS[0], PICK_POS[1], 0.12],
            textColorRGB=[1, 0.6, 0], textSize=1.3
        )
        p.addUserDebugText(
            "🎯 PLACE", [PLACE_POS[0], PLACE_POS[1], 0.12],
            textColorRGB=[0, 0.8, 0.2], textSize=1.3
        )

        return {
            'box_id'    : box_id,
            'pick_pos'  : PICK_POS,
            'place_pos' : PLACE_POS,
            'home_pos'  : HOME_POS
        }

    def run(self):
        """
        Vòng lặp Real-time 240Hz — chạy toàn bộ hệ thống.

        Hai chế độ vận hành (điều khiển qua SCADA Panel):

        ═══ CHẾ ĐỘ AUTO ═══
          Giữ nguyên 100% logic Pick & Place tự động:
          HMI → Z-Offset → TaskScheduler → IK → Gripper → HUD

        ═══ CHẾ ĐỘ MANUAL ═══
          Bỏ qua TaskScheduler. Operator trực tiếp điều khiển:
          - IK Mode: Nhập X, Y, Z → robot.move_to()
          - FK Mode: Kéo 6 thanh trượt → setJointMotorControl2()
          Gripper tắt (nhả vật). HUD hiện cảnh báo đỏ.
        """
        try:
            while True:
                current_time = time.time()

                # ── 0. CẬP NHẬT SCADA GUI (tkinter) ──────────────────────────
                # Xử lý sự kiện tkinter (click, kéo slider, gõ phím)
                # Nếu operator đóng cửa sổ SCADA → thoát simulation
                if not self.scada.update_gui():
                    print("  [SCADA] Operator dong cua so SCADA → Dang thoat...")
                    break

                # Đọc chế độ hiện tại từ SCADA Panel
                scada_mode = self.scada.mode   # "AUTO" hoặc "MANUAL"

                # ── LẤY VỊ TRÍ EE HIỆN TẠI (dùng chung cho cả 2 chế độ) ────
                ee_pos = self.robot.get_ee_position()

                # ══════════════════════════════════════════════════════════════
                #  CHẾ ĐỘ AUTO — Giữ nguyên 100% logic Pick & Place
                # ══════════════════════════════════════════════════════════════
                if scada_mode == "AUTO":

                    # ── 1. ĐỌC HMI PANEL ─────────────────────────────────────
                    hmi = self.hmi.read_inputs()

                    # Nút START / PAUSE
                    if hmi.pause_pressed:
                        self.task.paused = not self.task.paused
                        self.task.e_stop = False
                        status_str = "PAUSE" if self.task.paused else "RESUME"
                        print(f"  [HMI]  [{status_str}] Nhan nut START/PAUSE")

                        # Nếu RESUME và đang IDLE → khởi động chu trình mới
                        if (not self.task.paused and
                                self.task.state == STATE_IDLE):
                            self.task.start(ee_pos)

                    # Nút EMERGENCY HOME
                    if hmi.estop_pressed:
                        if not self.task.e_stop:
                            self.task.e_stop  = True
                            self.task.paused  = False
                            print("  [HMI]  [E-STOP] EMERGENCY HOME!")

                    # ── 2. ÁP DỤNG Z-OFFSET ──────────────────────────────────
                    z_offset = hmi.z_offset
                    base_pick_z = self.scene['pick_pos'][2] + 0.06
                    self.task.pick_contact_pos[2] = base_pick_z + z_offset
                    self.task.pick_pos[2] = (
                        self.scene['pick_pos'][2] + z_offset
                    )

                    # ── 3. CẬP NHẬT TASK SCHEDULER ───────────────────────────
                    waypoint, is_grasping, grasp_yaw = self.task.update(
                        ee_pos, current_time
                    )

                    # ── 4. XÂY DỰNG TARGET ORIENTATION ───────────────────────
                    if self.task.state in GRASP_STATES:
                        grasp_orn = p.getQuaternionFromEuler(
                            [math.pi, 0, grasp_yaw]
                        )
                    else:
                        grasp_orn = p.getQuaternionFromEuler(
                            [math.pi, 0, 0]
                        )

                    # ── 5. GIẢI IK + MOTOR ───────────────────────────────────
                    self.robot.move_to(waypoint, orientation=grasp_orn)

                    # ── 6. CẬP NHẬT GRIPPER ──────────────────────────────────
                    self.gripper.update_pose(
                        is_grasping, self.scene['box_id']
                    )

                    # ── 7. CẬP NHẬT HUD (AUTO) ───────────────────────────────
                    self.hud_id = self.task.update_hud(
                        ee_pos, self.hud_id, z_offset
                    )

                    # Xóa HUD cảnh báo MANUAL nếu đang có
                    if self._manual_hud_id != -1:
                        p.removeUserDebugItem(self._manual_hud_id)
                        self._manual_hud_id = -1

                    # ── Cập nhật SCADA status ────────────────────────────────
                    self.scada.update_status(
                        ee_pos,
                        self.robot.get_joint_states(),
                        "AUTO",
                        state=self.task.state,
                        loop_count=self.task.loop_count
                    )

                # ══════════════════════════════════════════════════════════════
                #  CHẾ ĐỘ MANUAL — Operator điều khiển trực tiếp qua SCADA
                # ══════════════════════════════════════════════════════════════
                else:
                    manual_sub = self.scada.manual_sub  # "IK" hoặc "FK"

                    # ── MANUAL IK: Operator nhập tọa độ X, Y, Z ──────────────
                    if manual_sub == "IK":
                        if self.scada.ik_run_flag:
                            target = self.scada.ik_target
                            orn = p.getQuaternionFromEuler(
                                [math.pi, 0, 0]
                            )
                            self.robot.move_to(target, orientation=orn)
                            # Giữ cờ True để robot tiếp tục hội tụ về target
                            # (IK cần nhiều bước stepSimulation)

                    # ── MANUAL FK: Operator kéo 6 thanh trượt ────────────────
                    elif manual_sub == "FK":
                        angles = self.scada.fk_angles
                        for i, joint_idx in enumerate(
                            self.robot.active_joints
                        ):
                            if i < len(angles):
                                p.setJointMotorControl2(
                                    bodyIndex=self.robot.robot_id,
                                    jointIndex=joint_idx,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=angles[i],
                                    force=self.robot.DEFAULT_MAX_FORCE,
                                    maxVelocity=self.robot.DEFAULT_MAX_VEL
                                )
                        # Tắt cờ IK khi chuyển sang FK
                        self.scada.ik_run_flag = False

                    # ── Tắt gripper (nhả vật) trong MANUAL ───────────────────
                    self.gripper.update_pose(
                        False, self.scene['box_id']
                    )

                    # ── HUD cảnh báo MANUAL MODE (chữ đỏ) ────────────────────
                    manual_hud_text = (
                        f"⚠️  SCADA MANUAL MODE ({manual_sub})\n"
                        f"EE Pos : ({ee_pos[0]:+.3f}, {ee_pos[1]:+.3f}, "
                        f"{ee_pos[2]:+.3f})\n"
                        f"Grasp  : OFF [NHA] — Manual Override"
                    )
                    if self._manual_hud_id == -1:
                        self._manual_hud_id = p.addUserDebugText(
                            manual_hud_text, [-0.6, -0.5, 0.9],
                            textColorRGB=[1.0, 0.2, 0.1],
                            textSize=1.1
                        )
                    else:
                        p.addUserDebugText(
                            manual_hud_text, [-0.6, -0.5, 0.9],
                            textColorRGB=[1.0, 0.2, 0.1],
                            textSize=1.1,
                            replaceItemUniqueId=self._manual_hud_id
                        )

                    # Xóa HUD AUTO nếu đang có
                    if self.hud_id != -1:
                        p.removeUserDebugItem(self.hud_id)
                        self.hud_id = -1

                    # ── Cập nhật SCADA status ────────────────────────────────
                    self.scada.update_status(
                        ee_pos,
                        self.robot.get_joint_states(),
                        "MANUAL"
                    )

                # ── BƯỚC VẬT LÝ (chung cho cả AUTO và MANUAL) ───────────────
                p.stepSimulation()
                time.sleep(1. / 240.)

        except KeyboardInterrupt:
            print("\n  [STOP] Mo phong da dung theo yeu cau nguoi dung.")
        finally:
            # Đóng SCADA Panel trước
            self.scada.destroy()
            try:
                p.disconnect()
            except p.error:
                pass   # Đã disconnect trước đó (user đóng cửa sổ GUI)
            print("  [BYE]  Da ngat ket noi PyBullet.\n")


# ══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    app = SimulationApp()
    app.run()