# -*- coding: utf-8 -*-
"""
================================================================================
  MODULE: Gripper — Cơ cấu kẹp (End-Effector / Gripper)
================================================================================
  Vai trò thực tế: Tương đương cơ cấu kẹp khí nén hoặc điện (Electric/Pneumatic
  Gripper) gắn vào mặt bích (flange) tại đầu cánh tay robot.

  Ví dụ ngoài thực tế: Gripper Robotiq 2F-85, Gripper Schunk, kẹp chân không...

  Nhiệm vụ chính:
    1. Luôn bám (follow) theo đầu công cụ TCP của robot mỗi chu kỳ điều khiển
    2. Khi nhận lệnh GRASP: siết chặt vật (mô phỏng bằng cách kéo vật đi cùng)
    3. Khi nhận lệnh RELEASE: nhả vật tại vị trí hiện tại

  Lưu ý kỹ thuật:
    Mô phỏng này dùng "kẹp lý tưởng" (perfect grasp) — không mô hình hóa
    lực tiếp xúc ngón kẹp, slip, hay deformation của vật.
    Trong thực tế: cần thuật toán force control để tránh bóp vỡ/tuột vật.
================================================================================
"""

import pybullet as p


# ══════════════════════════════════════════════════════════════════════════════
# CƠ CẤU KẸP — Gripper bám theo TCP của robot
# ══════════════════════════════════════════════════════════════════════════════

class Gripper:
    """
    Mô phỏng cơ cấu kẹp gắn tại đầu cánh tay robot.

    Khi robot di chuyển, gripper phải luôn bám chính xác theo đầu công cụ (TCP).
    Điều này tương đương gripper thực được bu-lông cứng vào mặt bích (flange).

    Khi kẹp vật (GRASPING):
      → Vật được "dán" theo gripper (mô phỏng lực kẹp giữ vật)
      → Mỗi khi gripper di chuyển, vật di chuyển theo y hệt
    """

    def __init__(self, robot):
        """
        Lắp cơ cấu kẹp vào đầu cánh tay robot.

        robot: đối tượng UR5eRobot — cần để đọc vị trí và hướng TCP
               (giống gripper vật lý cần biết mặt bích nó đang gắn vào đâu)

        Gripper ảo là một khối hộp nhỏ:
          Kích thước: 6cm × 6cm × 10cm
          Màu: xám đậm (giả kim loại)
          Khối lượng: 0 (không chịu trọng lực — luôn bám theo lệnh)
        """
        self.robot = robot  # Lưu tham chiếu đến robot để đọc vị trí TCP

        # ── Tạo hình học 3D cho gripper ──────────────────────────────────────
        # halfExtents: bán kích thước theo 3 trục (mét)
        # Khối đầy đủ = 2 × halfExtents = 6cm × 6cm × 10cm
        gripper_shape = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[0.03, 0.03, 0.05]   # Hình dạng va chạm (invisble)
        )
        gripper_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.03, 0.03, 0.05],
            rgbaColor=[0.3, 0.3, 0.3, 0.95]  # Xám đậm, alpha 95% (hơi trong)
        )

        # Tạo vật thể trong simulation: mass=0 → không chịu gravity, không bị ngã
        # Vị trí [0, 0, 0.3] chỉ là vị trí tạm thời khi khởi tạo — sẽ cập nhật ngay
        self.gripper_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=gripper_shape,
            baseVisualShapeIndex=gripper_visual,
            basePosition=[0, 0, 0.3]
        )

        print("  [OK]  Gripper da duoc tao.")

    @staticmethod
    def _rotate_by_quat(vec: list, quat: tuple) -> list:
        """
        Xoay một vector theo hướng của gripper.

        Bài toán thực tế: gripper có trục Z riêng (trục của ngón kẹp).
        Khi cần đặt gripper phía TRƯỚC TCP (theo hướng gripper đang quay),
        phải tính vector offset theo LOCAL FRAME của gripper, không phải world frame.

        Ví dụ: Nếu gripper đang nghiêng 45°, "phía trước" của nó không còn
        là hướng Z của world frame nữa — phải xoay vector theo đúng góc đó.

        Dùng công thức nhân quaternion: v' = q × v × q⁻¹
        (Rodrigues rotation formula — tính xoay vector bằng quaternion)
        """
        qx, qy, qz, qw = quat  # 4 thành phần quaternion biểu diễn hướng gripper
        vx, vy, vz = vec        # Vector cần xoay (offset trong local frame)

        # Bước 1: Tính tích có hướng q_vec × v (cross product)
        cx = qy * vz - qz * vy
        cy = qz * vx - qx * vz
        cz = qx * vy - qy * vx

        # Bước 2: Tính q_vec × (q_vec × v) lần thứ 2
        cx2 = qy * cz - qz * cy
        cy2 = qz * cx - qx * cz
        cz2 = qx * cy - qy * cx

        # Kết quả: vector đã được xoay sang world frame
        return [
            vx + 2 * qw * cx + 2 * cx2,
            vy + 2 * qw * cy + 2 * cy2,
            vz + 2 * qw * cz + 2 * cz2,
        ]

    def update_pose(self, is_grasping: bool, grasped_object_id: int = -1,
                    offset_z: float = 0.05):
        """
        Cập nhật vị trí gripper — gọi MỖI VÒNG LẶP 240Hz.

        Đây là vòng điều khiển vị trí của gripper — tương đương việc
        servo motor liên tục điều chỉnh vị trí để bám theo setpoint.

        offset_z: gripper không đặt thẳng tại TCP mà hơi lùi ra ngoài
                  theo trục tool (như độ nhô của ngón kẹp thực tế)

        is_grasping=True: gripper đang giữ vật → kéo vật đi theo
        is_grasping=False: gripper rỗng → chỉ cập nhật vị trí gripper
        """
        # ── Đọc vị trí và hướng chính xác (FK) của đầu công cụ TCP ──────────
        # Đây là "feedback" từ encoder của robot — biết TCP đang ở đâu
        ee_pos, ee_orn = self.robot.get_ee_pose()

        # ── Tính vị trí gripper theo local frame của TCP ─────────────────────
        # Gripper nằm "phía trước" TCP, lệch 5cm dọc theo trục tool
        # Không thể chỉ cộng Z thẳng vào world frame vì gripper có thể đang nghiêng!
        local_offset = [0.0, 0.0, offset_z]  # 5cm theo trục Z LOCAL của gripper
        world_offset = self._rotate_by_quat(local_offset, ee_orn)  # Chuyển sang world frame

        # Vị trí thực tế của gripper trong không gian 3D
        gripper_pos = [
            ee_pos[0] + world_offset[0],
            ee_pos[1] + world_offset[1],
            ee_pos[2] + world_offset[2],
        ]

        # Đặt gripper vào đúng vị trí và hướng (dùng reset vì mass=0)
        # Hướng gripper = hướng TCP (gắn cứng vào mặt bích)
        p.resetBasePositionAndOrientation(self.gripper_id, gripper_pos, ee_orn)

        # ── Kéo vật theo gripper khi đang kẹp ───────────────────────────────
        # Đây là "lực kẹp lý tưởng" — vật bám cứng vào gripper không bị trượt
        # Vật nằm thêm 4cm nữa theo trục tool, bên dưới gripper
        if is_grasping and grasped_object_id >= 0:
            obj_local_offset = [0.0, 0.0, offset_z + 0.04]  # 5cm + 4cm = 9cm từ TCP
            obj_world_offset = self._rotate_by_quat(obj_local_offset, ee_orn)
            obj_pos = [
                ee_pos[0] + obj_world_offset[0],
                ee_pos[1] + obj_world_offset[1],
                ee_pos[2] + obj_world_offset[2],
            ]
            # Vật xoay cùng hướng với gripper (bị siết chặt, không trượt)
            p.resetBasePositionAndOrientation(
                grasped_object_id, obj_pos, ee_orn
            )
