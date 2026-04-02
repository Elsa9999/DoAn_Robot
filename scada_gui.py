# -*- coding: utf-8 -*-
"""
================================================================================
  MODULE: ScadaPanel — Bảng điều khiển SCADA ảo (External GUI — tkinter)
================================================================================
  Vai trò thực tế: Tương đương màn hình SCADA (Supervisory Control and Data
  Acquisition) trong phòng điều khiển trung tâm của nhà máy.

  Trong công nghiệp thực:
    - SCADA chạy trên MÁY TÍNH RIÊNG, giao tiếp với PLC qua Ethernet/Modbus
    - Operator nhìn màn hình SCADA để giám sát và can thiệp khi cần
    - Có 2 chế độ: AUTO (PLC tự chạy) và MANUAL (operator điều khiển tay)

  Ở đây: tkinter đóng vai trò "máy tính SCADA", giao tiếp với PyBullet
  thông qua các biến chia sẻ (shared flags) trong vòng lặp 240Hz.

  Giao diện gồm 3 phần chính:
    ┌─────────────────────────────────────────┐
    │  🔘 SYSTEM MODE: AUTO / MANUAL          │  ← Chọn chế độ vận hành
    ├─────────────────────────────────────────┤
    │  📐 MANUAL — INVERSE KINEMATICS (IK)    │  ← Nhập X, Y, Z + nút RUN
    ├─────────────────────────────────────────┤
    │  🎚️ MANUAL — FORWARD KINEMATICS (FK)    │  ← 6 thanh trượt góc khớp
    ├─────────────────────────────────────────┤
    │  📊 LIVE STATUS                         │  ← Hiển thị EE pos + joints
    └─────────────────────────────────────────┘
================================================================================
"""

import tkinter as tk
from tkinter import ttk
import math


class ScadaPanel:
    """
    Bảng điều khiển SCADA ảo — cửa sổ tkinter luôn nổi trên PyBullet.

    Giao tiếp với SimulationApp thông qua các thuộc tính công khai:
      - self.mode          : "AUTO" hoặc "MANUAL"
      - self.manual_sub    : "IK" hoặc "FK" (chế độ phụ trong MANUAL)
      - self.ik_target     : [X, Y, Z] mục tiêu cho IK
      - self.ik_run_flag   : True khi operator bấm "RUN IK"
      - self.fk_angles     : [q0..q5] góc 6 khớp từ thanh trượt (rad)

    Vòng lặp chính của simulation gọi update_gui() mỗi tick để xử lý
    sự kiện tkinter (non-blocking — không dùng mainloop()).
    """

    # ── Tên hiển thị cho 6 khớp UR5e ─────────────────────────────────────────
    JOINT_NAMES = [
        "J0  Shoulder Pan",     # Khớp đế — xoay toàn bộ robot
        "J1  Shoulder Lift",    # Khớp vai — nâng/hạ cánh tay trên
        "J2  Elbow",            # Khớp khuỷu — gập/duỗi cẳng tay
        "J3  Wrist 1",          # Cổ tay 1 — xoay cẳng tay
        "J4  Wrist 2",          # Cổ tay 2 — nghiêng cổ tay
        "J5  Wrist 3",          # Cổ tay 3 — xoay ngón kẹp
    ]

    # Tư thế REST mặc định (Elbow-Up) — dùng làm giá trị khởi tạo thanh trượt
    REST_POSES = [0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0]

    def __init__(self):
        """Tạo cửa sổ SCADA và dựng toàn bộ giao diện."""

        # ── Khởi tạo cửa sổ chính ────────────────────────────────────────────
        self.root = tk.Tk()
        self.root.title("🏭 SCADA Panel — UR5e Pick & Place System")
        self.root.geometry("460x780+20+20")   # Kích thước + vị trí góc trái trên
        self.root.resizable(False, False)
        self.root.attributes('-topmost', True) # Luôn nổi trên PyBullet GUI
        self.root.configure(bg="#1e1e2e")      # Nền tối (dark theme)

        # ── Biến chia sẻ với SimulationApp ────────────────────────────────────
        self.mode        = "AUTO"       # Chế độ hiện tại: AUTO / MANUAL
        self.manual_sub  = "IK"         # Chế độ phụ MANUAL: IK / FK
        self.ik_target   = [0.3, 0.0, 0.4]  # Tọa độ IK mục tiêu
        self.ik_run_flag = False        # Cờ bấm nút RUN IK
        self.fk_angles   = list(self.REST_POSES)  # Góc 6 khớp từ slider

        # ── Biến tkinter ──────────────────────────────────────────────────────
        self._mode_var    = tk.StringVar(value="AUTO")
        self._sub_var     = tk.StringVar(value="IK")
        self._ik_x_var    = tk.StringVar(value="0.30")
        self._ik_y_var    = tk.StringVar(value="0.00")
        self._ik_z_var    = tk.StringVar(value="0.40")
        self._fk_vars     = []  # DoubleVar cho 6 thanh trượt
        self._status_var  = tk.StringVar(value="Đang chờ khởi động...")

        # ── Tạo style cho ttk widgets ─────────────────────────────────────────
        self._setup_styles()

        # ── Dựng giao diện ────────────────────────────────────────────────────
        self._build_header()
        self._build_mode_section()
        self._build_ik_section()
        self._build_fk_section()
        self._build_status_section()

        print("  [SCADA] Cua so SCADA Panel da khoi tao (tkinter).")

    # ══════════════════════════════════════════════════════════════════════════
    # SETUP STYLES
    # ══════════════════════════════════════════════════════════════════════════

    def _setup_styles(self):
        """Cấu hình style cho ttk widgets — dark theme."""
        style = ttk.Style()
        style.theme_use('clam')

        # Scale (thanh trượt) — màu xanh dương trên nền tối
        style.configure("FK.Horizontal.TScale",
                        background="#1e1e2e",
                        troughcolor="#2d2d44",
                        sliderthickness=18)

        # Label frame
        style.configure("Dark.TLabelframe",
                        background="#1e1e2e",
                        foreground="#cdd6f4")
        style.configure("Dark.TLabelframe.Label",
                        background="#1e1e2e",
                        foreground="#89b4fa",
                        font=("Segoe UI", 10, "bold"))

        # Radiobutton
        style.configure("Dark.TRadiobutton",
                        background="#1e1e2e",
                        foreground="#cdd6f4",
                        font=("Segoe UI", 10))

    # ══════════════════════════════════════════════════════════════════════════
    # BUILD UI SECTIONS
    # ══════════════════════════════════════════════════════════════════════════

    def _build_header(self):
        """Tiêu đề SCADA Panel."""
        header = tk.Frame(self.root, bg="#181825", pady=8)
        header.pack(fill="x")

        tk.Label(header,
                 text="🏭  SCADA  PANEL",
                 font=("Segoe UI", 16, "bold"),
                 fg="#89b4fa", bg="#181825").pack()
        tk.Label(header,
                 text="UR5e Pick & Place — Supervisory Control",
                 font=("Segoe UI", 9),
                 fg="#6c7086", bg="#181825").pack()

    def _build_mode_section(self):
        """Phần chọn SYSTEM MODE: AUTO / MANUAL."""
        frame = tk.LabelFrame(self.root,
                               text="  ⚙️  SYSTEM MODE  ",
                               font=("Segoe UI", 10, "bold"),
                               fg="#89b4fa", bg="#1e1e2e",
                               bd=1, relief="groove",
                               padx=12, pady=8)
        frame.pack(fill="x", padx=12, pady=(8, 4))

        # Radio buttons AUTO / MANUAL
        btn_frame = tk.Frame(frame, bg="#1e1e2e")
        btn_frame.pack(fill="x")

        for mode_text, mode_val, color in [
            ("🟢  AUTO  (PLC tự động)", "AUTO", "#a6e3a1"),
            ("🟠  MANUAL  (Điều khiển tay)", "MANUAL", "#fab387"),
        ]:
            rb = tk.Radiobutton(
                btn_frame, text=mode_text, variable=self._mode_var,
                value=mode_val, command=self._on_mode_change,
                font=("Segoe UI", 10), fg=color, bg="#1e1e2e",
                selectcolor="#313244", activebackground="#1e1e2e",
                activeforeground=color, indicatoron=True,
                anchor="w", padx=8, pady=3)
            rb.pack(fill="x")

    def _build_ik_section(self):
        """Phần MANUAL — Inverse Kinematics: nhập X, Y, Z + nút RUN."""
        self._ik_frame = tk.LabelFrame(
            self.root,
            text="  📐  MANUAL — INVERSE KINEMATICS  ",
            font=("Segoe UI", 10, "bold"),
            fg="#89b4fa", bg="#1e1e2e",
            bd=1, relief="groove",
            padx=12, pady=8)
        self._ik_frame.pack(fill="x", padx=12, pady=4)

        # ── Sub-mode radio: IK / FK ──────────────────────────────────────────
        sub_frame = tk.Frame(self._ik_frame, bg="#1e1e2e")
        sub_frame.pack(fill="x", pady=(0, 6))

        for sub_text, sub_val in [("IK Mode", "IK"), ("FK Mode", "FK")]:
            tk.Radiobutton(
                sub_frame, text=sub_text, variable=self._sub_var,
                value=sub_val, command=self._on_sub_change,
                font=("Segoe UI", 9), fg="#cdd6f4", bg="#1e1e2e",
                selectcolor="#313244", activebackground="#1e1e2e",
                indicatoron=True, padx=4
            ).pack(side="left", padx=8)

        # ── Entry fields X, Y, Z ─────────────────────────────────────────────
        coords_frame = tk.Frame(self._ik_frame, bg="#1e1e2e")
        coords_frame.pack(fill="x", pady=4)

        for col, (label, var) in enumerate([
            ("X (m)", self._ik_x_var),
            ("Y (m)", self._ik_y_var),
            ("Z (m)", self._ik_z_var),
        ]):
            cell = tk.Frame(coords_frame, bg="#1e1e2e")
            cell.pack(side="left", expand=True, fill="x", padx=4)

            tk.Label(cell, text=label,
                     font=("Segoe UI", 9), fg="#a6adc8",
                     bg="#1e1e2e").pack()
            entry = tk.Entry(cell, textvariable=var,
                             font=("Consolas", 11), width=8,
                             bg="#313244", fg="#cdd6f4",
                             insertbackground="#cdd6f4",
                             relief="flat", justify="center",
                             bd=0, highlightthickness=1,
                             highlightcolor="#89b4fa",
                             highlightbackground="#45475a")
            entry.pack(ipady=4)

        # ── Nút RUN IK ──────────────────────────────────────────────────────
        self._ik_btn = tk.Button(
            self._ik_frame, text="▶  RUN IK",
            font=("Segoe UI", 11, "bold"),
            fg="#1e1e2e", bg="#a6e3a1",
            activebackground="#94e2d5", activeforeground="#1e1e2e",
            relief="flat", cursor="hand2",
            padx=16, pady=6,
            command=self._on_run_ik)
        self._ik_btn.pack(pady=(8, 2))

    def _build_fk_section(self):
        """Phần MANUAL — Forward Kinematics: 6 thanh trượt góc khớp."""
        self._fk_frame = tk.LabelFrame(
            self.root,
            text="  🎚️  MANUAL — FORWARD KINEMATICS  ",
            font=("Segoe UI", 10, "bold"),
            fg="#89b4fa", bg="#1e1e2e",
            bd=1, relief="groove",
            padx=12, pady=6)
        self._fk_frame.pack(fill="x", padx=12, pady=4)

        self._fk_vars = []
        self._fk_value_labels = []

        for i, name in enumerate(self.JOINT_NAMES):
            row = tk.Frame(self._fk_frame, bg="#1e1e2e")
            row.pack(fill="x", pady=1)

            # Nhãn tên khớp
            tk.Label(row, text=name,
                     font=("Consolas", 8), fg="#a6adc8", bg="#1e1e2e",
                     width=20, anchor="w").pack(side="left")

            # Biến DoubleVar cho slider
            var = tk.DoubleVar(value=self.REST_POSES[i])
            self._fk_vars.append(var)

            # Thanh trượt (Scale)
            scale = ttk.Scale(
                row, from_=-math.pi, to=math.pi,
                orient="horizontal", variable=var,
                style="FK.Horizontal.TScale",
                command=lambda val, idx=i: self._on_fk_change(idx))
            scale.pack(side="left", expand=True, fill="x", padx=4)

            # Hiển thị giá trị góc
            val_label = tk.Label(row,
                                  text=f"{self.REST_POSES[i]:+.2f}",
                                  font=("Consolas", 9), fg="#f9e2af",
                                  bg="#1e1e2e", width=6)
            val_label.pack(side="right")
            self._fk_value_labels.append(val_label)

        # ── Nút Reset FK về REST_POSES ───────────────────────────────────────
        tk.Button(self._fk_frame, text="🔄 Reset về Elbow-Up",
                  font=("Segoe UI", 9),
                  fg="#cdd6f4", bg="#45475a",
                  activebackground="#585b70",
                  relief="flat", cursor="hand2",
                  padx=8, pady=3,
                  command=self._on_fk_reset).pack(pady=(4, 2))

    def _build_status_section(self):
        """Phần hiển thị trạng thái live từ simulation."""
        frame = tk.LabelFrame(self.root,
                               text="  📊  LIVE STATUS  ",
                               font=("Segoe UI", 10, "bold"),
                               fg="#89b4fa", bg="#1e1e2e",
                               bd=1, relief="groove",
                               padx=12, pady=6)
        frame.pack(fill="x", padx=12, pady=(4, 8))

        self._status_label = tk.Label(
            frame, textvariable=self._status_var,
            font=("Consolas", 9), fg="#cdd6f4", bg="#1e1e2e",
            justify="left", anchor="w")
        self._status_label.pack(fill="x")

    # ══════════════════════════════════════════════════════════════════════════
    # EVENT HANDLERS — Xử lý sự kiện từ operator
    # ══════════════════════════════════════════════════════════════════════════

    def _on_mode_change(self):
        """Operator chuyển chế độ AUTO ↔ MANUAL."""
        self.mode = self._mode_var.get()
        print(f"  [SCADA] Chuyen che do: {self.mode}")

    def _on_sub_change(self):
        """Operator chuyển chế độ phụ IK ↔ FK trong MANUAL."""
        self.manual_sub = self._sub_var.get()
        print(f"  [SCADA] Che do MANUAL: {self.manual_sub}")

    def _on_run_ik(self):
        """Operator bấm nút RUN IK — đọc tọa độ và bật cờ."""
        try:
            x = float(self._ik_x_var.get())
            y = float(self._ik_y_var.get())
            z = float(self._ik_z_var.get())
            self.ik_target = [x, y, z]
            self.ik_run_flag = True
            print(f"  [SCADA] RUN IK → target = [{x:.3f}, {y:.3f}, {z:.3f}]")
        except ValueError:
            print("  [SCADA] [LOI] Toa do IK khong hop le! Nhap so thuc.")

    def _on_fk_change(self, joint_idx: int):
        """Operator kéo thanh trượt FK — cập nhật góc khớp."""
        val = self._fk_vars[joint_idx].get()
        self.fk_angles[joint_idx] = val
        self._fk_value_labels[joint_idx].config(text=f"{val:+.2f}")

    def _on_fk_reset(self):
        """Reset tất cả thanh trượt FK về tư thế Elbow-Up."""
        for i, rest in enumerate(self.REST_POSES):
            self._fk_vars[i].set(rest)
            self.fk_angles[i] = rest
            self._fk_value_labels[i].config(text=f"{rest:+.2f}")
        print("  [SCADA] FK Reset → Elbow-Up REST_POSES")

    # ══════════════════════════════════════════════════════════════════════════
    # PUBLIC API — Gọi từ vòng lặp chính simulation.py
    # ══════════════════════════════════════════════════════════════════════════

    def update_status(self, ee_pos: list, joint_angles: list, mode: str,
                      state: str = "", loop_count: int = 0):
        """
        Cập nhật mục LIVE STATUS trên SCADA Panel.

        Args:
            ee_pos       : Vị trí EE hiện tại [x, y, z]
            joint_angles : Góc 6 khớp hiện tại (rad)
            mode         : "AUTO" hoặc "MANUAL"
            state        : Tên state FSM hiện tại (chế độ AUTO)
            loop_count   : Số chu trình đã hoàn thành
        """
        if mode == "AUTO":
            lines = [
                f"Mode   : 🟢 AUTO",
                f"State  : {state}",
                f"Cycle  : #{loop_count}",
                f"EE Pos : ({ee_pos[0]:+.3f}, {ee_pos[1]:+.3f}, {ee_pos[2]:+.3f})",
            ]
        else:
            lines = [
                f"Mode   : 🟠 MANUAL ({self.manual_sub})",
                f"EE Pos : ({ee_pos[0]:+.3f}, {ee_pos[1]:+.3f}, {ee_pos[2]:+.3f})",
            ]
        # Thêm góc khớp
        if joint_angles:
            angles_str = ", ".join(f"{math.degrees(a):+6.1f}°"
                                    for a in joint_angles[:6])
            lines.append(f"Joints : [{angles_str}]")

        self._status_var.set("\n".join(lines))

    def update_gui(self) -> bool:
        """
        Cập nhật cửa sổ tkinter — gọi MỖI VÒNG LẶP 240Hz.

        Không dùng mainloop() vì vòng lặp chính là của PyBullet.
        Thay vào đó, gọi root.update() để xử lý sự kiện tkinter
        (click, kéo slider, gõ phím) rồi trả quyền điều khiển
        ngay cho simulation loop.

        Returns:
            True  : cửa sổ SCADA còn mở, tiếp tục simulation
            False : operator đã đóng cửa sổ SCADA → thoát simulation
        """
        try:
            self.root.update()
            return True
        except tk.TclError:
            # Người dùng đóng cửa sổ tkinter → TclError
            print("  [SCADA] Cua so SCADA da dong. Dang thoat simulation...")
            return False

    def destroy(self):
        """Đóng cửa sổ SCADA an toàn."""
        try:
            self.root.destroy()
        except tk.TclError:
            pass
