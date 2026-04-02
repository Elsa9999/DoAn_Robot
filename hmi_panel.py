# -*- coding: utf-8 -*-
"""
================================================================================
  MODULE: HMIPanel — Bảng điều khiển vận hành (Human-Machine Interface)
================================================================================
  Vai trò thực tế: Tương đương một màn hình HMI cảm ứng (Siemens SIMATIC HMI,
  Allen-Bradley PanelView) hoặc hộp pendant cầm tay của operator.

  3 Control trên bảng vận hành:

    [NÚT 1] >> START / PAUSE Cycle
      → Operator bấm để bắt đầu hoặc tạm dừng chu trình tự động
      → Tương đương nút RUN/HOLD trên PLC Panel
      → Mỗi lần BẤM (không phải giữ) → thay đổi trạng thái

    [NÚT 2] !! EMERGENCY HOME !!
      → Bấm khi có người/vật lạ vào vùng làm việc
      → Robot nhả vật ngay lập tức và về HOME an toàn
      → KHÁC với E-Stop cứng (E-Stop cứng ngắt nguồn hoàn toàn)

    [SLIDER] Z-Offset Fine Tune (m)
      → Operator kéo để tinh chỉnh chiều cao điểm gắp ±50mm
      → Dùng khi vật thể cao hơn/thấp hơn thiết kế, hoặc khi gripper bị mòn
      → Tương đương chỉnh offset tọa độ TCP trong PolyScope

  Nguyên lý Edge Detection (phát hiện sự kiện NHẤN NÚT):
    PyBullet không có sự kiện "click" — chỉ đọc được GIÁ TRỊ số.
    Mỗi lần bấm nút, giá trị tăng thêm 1.
    → So sánh giá trị lần này với lần trước → nếu KHÁC NHAU thì vừa bấm.
    (Cách làm này y hệt cách PLC đọc Rising Edge của tín hiệu digital I/O)
================================================================================
"""

import pybullet as p
from collections import namedtuple


# ── Gói gọn kết quả đọc HMI trong 1 "bản tin" hữu danh ─────────────────────
# Thay vì trả về 3 biến riêng lẻ, đóng gói thành 1 namedtuple có nhãn rõ ràng
# Giống như một "bản tin trạng thái HMI" truyền về PLC qua Modbus/Profinet
HMIState = namedtuple('HMIState', ['pause_pressed', 'estop_pressed', 'z_offset'])
# pause_pressed : True nếu nút START/PAUSE VỪA được bấm trong chu kỳ này
# estop_pressed : True nếu nút E-STOP VỪA được bấm trong chu kỳ này
# z_offset      : Giá trị liên tục từ slider (mét), cập nhật mỗi tick


# ══════════════════════════════════════════════════════════════════════════════
# BẢNG ĐIỀU KHIỂN HMI — Tạo và đọc 3 control trên giao diện PyBullet
# ══════════════════════════════════════════════════════════════════════════════

class HMIPanel:
    """
    Bảng điều khiển HMI xuất hiện trong cửa sổ PyBullet GUI (tab Parameters).

    Operator tương tác trực tiếp với 3 control này trong khi robot đang chạy.
    Không cần dừng simulation để thay đổi — giống HMI cảm ứng thực tế.
    """

    def __init__(self):
        """
        Tạo và hiển thị 3 control trên bảng HMI.

        Nên gọi SAU KHI robot đã ổn định tư thế ban đầu (sau task.start()).
        Lý do: operator không nên can thiệp trong khi robot đang boot lên.
        """
        print("  [HMI]  Dang tao bang dieu khien HMI...")

        # ── Nút START / PAUSE ─────────────────────────────────────────────────
        # rangeMin=0, rangeMax=1, startValue=0: button sẽ lần lượt là 0, 1, 0, 1...
        # Thực ra PyBullet tăng giá trị +1 mỗi lần bấm → ta chỉ cần phát hiện "thay đổi"
        self.btn_pause_id = p.addUserDebugParameter(
            paramName  = ">> START / PAUSE Cycle",
            rangeMin   = 0,
            rangeMax   = 1,
            startValue = 0
        )

        # ── Nút EMERGENCY HOME ────────────────────────────────────────────────
        # Nguyên lý hoàn toàn giống nút START/PAUSE — chỉ khác nhãn và chức năng
        self.btn_estop_id = p.addUserDebugParameter(
            paramName  = "!! EMERGENCY HOME !!",
            rangeMin   = 0,
            rangeMax   = 1,
            startValue = 0
        )

        # ── Slider Z-Offset ───────────────────────────────────────────────────
        # rangeMin=-0.05m (-50mm), rangeMax=+0.05m (+50mm), mặc định=0
        # Đây là slider liên tục — giá trị thay đổi khi kéo, không cần click
        self.slider_z_id = p.addUserDebugParameter(
            paramName  = "Z-Offset Fine Tune (m)",
            rangeMin   = -0.05,    # Nâng gripper lên 50mm so với tọa độ pick gốc
            rangeMax   =  0.05,    # Hạ gripper xuống 50mm
            startValue =  0.0      # Mặc định: không offset
        )

        # ── Lưu giá trị ban đầu để làm "điểm tham chiếu" phát hiện cạnh ──────
        # Kỹ thuật này gọi là "Edge Detection" — giống đọc Rising Edge trong PLC
        # Nếu giá trị HỌC lần này ≠ lần trước → nút VỪA được bấm
        self._prev_pause_val = p.readUserDebugParameter(self.btn_pause_id)
        self._prev_estop_val = p.readUserDebugParameter(self.btn_estop_id)

        print("  [HMI]  Panel HMI san sang:")
        print("         [1] START/PAUSE  — bat/tam dung chu trinh")
        print("         [2] EMERGENCY HOME — khan cap ve HOME")
        print("         [3] Z-Offset slider — tinh chinh chieu cao grasping")

    def read_inputs(self) -> HMIState:
        """
        Đọc trạng thái tất cả control HMI — gọi MỖI VÒNG LẶP 240Hz.

        Tương đương chu kỳ SCAN của PLC — đọc toàn bộ input I/O một lượt.
        Phát hiện "Rising Edge" cho các nút bấm (chỉ phản ứng khi VỪA bấm,
        không phải khi đang giữ).

        Returns:
            HMIState — "bản tin" trạng thái HMI của chu kỳ này
        """
        # ── Đọc trạng thái nút START/PAUSE và phát hiện sự kiện nhấn ────────
        cur_pause_val = p.readUserDebugParameter(self.btn_pause_id)
        # So sánh với lần đọc trước: nếu khác → nút VỪA được bấm trong tick này
        pause_pressed = (cur_pause_val != self._prev_pause_val)
        self._prev_pause_val = cur_pause_val  # Cập nhật giá trị tham chiếu

        # ── Đọc trạng thái nút E-STOP ────────────────────────────────────────
        cur_estop_val = p.readUserDebugParameter(self.btn_estop_id)
        estop_pressed = (cur_estop_val != self._prev_estop_val)
        self._prev_estop_val = cur_estop_val

        # ── Đọc giá trị Z-Offset từ slider (đọc trực tiếp, không cần edge) ──
        # Slider là tín hiệu analog liên tục → đọc giá trị hiện tại mỗi tick
        z_offset = p.readUserDebugParameter(self.slider_z_id)

        # Đóng gói vào namedtuple và trả về cho vòng điều khiển chính
        return HMIState(
            pause_pressed = pause_pressed,
            estop_pressed = estop_pressed,
            z_offset      = z_offset
        )
