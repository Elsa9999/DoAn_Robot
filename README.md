# 🤖 Mô phỏng Robot UR5e — Pick & Place với PyBullet

> **Đồ án tốt nghiệp** | Mô phỏng tay máy công nghiệp UR5e (Universal Robots e-Series)
> thực hiện nhiệm vụ gắp-thả vật tự động với hệ thống thị giác máy (Computer Vision).

---

## 📖 Đọc trước khi xem code — Bức tranh tổng thể

Hãy tưởng tượng bạn đang đứng trước **một dây chuyền công nghiệp thực thụ** với các thành phần sau:

```
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│   📷 Camera công nghiệp          🖥️ Màn hình SCADA / HMI Panel     │
│   (nhìn xuống băng tải)          (nút bấm, đèn trạng thái)         │
│           │                                  │                      │
│           ▼                                  ▼                      │
│   🧠 Bộ xử lý ảnh            📋 Bộ điều phối tác vụ (PLC/FSM)     │
│   (tính tọa độ vật)           (ra lệnh: gắp → vận chuyển → thả)    │
│                                              │                      │
│                               ┌─────────────┘                      │
│                               ▼                                     │
│                    🦾 Bộ điều khiển tay máy                        │
│                    (giải IK, cấp setpoint cho 6 servo)              │
│                               │                                     │
│                               ▼                                     │
│                    🤏 Cơ cấu kẹp (Gripper)                         │
│                    (đóng ngón kẹp, giữ vật)                        │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**Bộ phần mềm này mô phỏng đúng kiến trúc đó.** Mỗi file Python tương ứng với một thiết bị/module riêng biệt trong hệ thống thực.

---

## 🗂️ 6 File — 6 Bộ phận của hệ thống

| File Python | Tương đương ngoài thực tế | Vai trò cụ thể |
|---|---|---|
| `ur5e_robot.py` | **Bộ điều khiển tay máy** (Robot Controller) | Nhận lệnh tọa độ → giải bài toán động học ngược → cấp setpoint góc cho 6 khớp |
| `vision_camera.py` | **Camera công nghiệp + bộ xử lý ảnh** | Chụp ảnh từ trên cao → lọc màu đỏ → tính tọa độ vật → đo góc nghiêng |
| `gripper.py` | **Cơ cấu kẹp + bộ điều khiển gripper** | Bám theo đầu công cụ (TCP), kẹp cứng vật khi nhận lệnh GRASP |
| `task_scheduler.py` | **PLC / Máy trạng thái (FSM)** | Điều phối toàn bộ chu trình: IDLE → PICK → PLACE → HOME, lặp vô hạn |
| `hmi_panel.py` | **Màn hình HMI / Bàn phím vận hành** | 3 control: nút START/PAUSE, nút E-STOP khẩn cấp, núm chỉnh cao độ |
| `simulation.py` | **Phòng điều khiển trung tâm (SCADA)** | Khởi động toàn bộ hệ thống, kết nối các module, chạy vòng quét 240Hz |

---

## 🔄 Luồng chạy — Từ lúc bấm nút Run đến khi robot gắp vật

### GIAI ĐOẠN 0 — Khởi động hệ thống (`simulation.py`)

```
Bấm [Run] python simulation.py
        │
        ▼
[1] Bật môi trường vật lý (PyBullet) — giống bật nguồn toàn bộ cabinet điện
[2] Nạp bản vẽ robot từ file URDF — giống "import" thông số cơ học vào controller
[3] Dựng cảnh: bàn làm việc + khối hộp đỏ + điểm PLACE
[4] Gọi Camera chụp ảnh lần đầu → xác định vị trí vật ban đầu
[5] Tạo gripper, khởi động máy trạng thái
[6] Hiển thị bảng HMI → SẴNSÀNG VẬN HÀNH
```

---

### GIAI ĐOẠN 1 — Vision System đo tọa độ vật (`vision_camera.py`)

```
Camera chụp ảnh từ trên cao (top-down, cao 1m)
        │
        ▼
Lọc màu HSV → chỉ giữ lại vùng màu ĐỎ trong ảnh
        │
        ▼
Tìm viền (contour) của khối đỏ → tính tâm (centroid)
        │
        ▼
Dùng Depth Buffer → "hỏi" độ sâu tại pixel đó → tính ra X, Y, Z thực tế (mét)
        │
        ▼
Đo góc nghiêng của hộp bằng hình chữ nhật bao ngoài → góc kẹp (Yaw)
        │
        ▼
Kiểm tra an toàn: vật có nằm trong tầm với ≤ 0.75m không?
        │
        ▼
Xuất ra: DetectionResult(X=0.42, Y=0.15, Z=0.025, Yaw=+23°)
```

> 📌 **Analog thực tế**: Giống hệ thống vision 2D/3D của Cognex hay Keyence đặt trên băng chuyền.  
> Camera đo tọa độ, truyền qua Ethernet/OPC-UA về PLC.

---

### GIAI ĐOẠN 2 — Máy trạng thái lập lịch tác vụ (`task_scheduler.py`)

Đây là **trái tim** của hệ thống — tương đương một chương trình PLC viết bằng Ladder Diagram hoặc Sequential Function Chart (SFC):

```
                    ┌──────────────────────┐
                    │        IDLE          │ ← Đứng yên, chờ lệnh
                    └──────────┬───────────┘
                               │ Nhận pick_pos từ Vision
                    ┌──────────▼───────────┐
                    │     TO_PREPICK       │ ← Bay nhanh lên phía trên vật
                    └──────────┬───────────┘
                               │ Đến nơi
                    ┌──────────▼───────────┐
                    │      TO_PICK         │ ← Hạ chậm xuống điểm gắp
                    └──────────┬───────────┘
                               │ Đến nơi
                    ┌──────────▼───────────┐
                    │      GRASPING        │ ← Kẹp chặt vật (0.8 giây)
                    └──────────┬───────────┘
                               │ Kẹp xong
                    ┌──────────▼───────────┐
                    │    TO_PREPLACE       │ ← Nâng + vận chuyển sang vị trí thả
                    └──────────┬───────────┘
                               │ Đến nơi
                    ┌──────────▼───────────┐
                    │      TO_PLACE        │ ← Hạ chậm xuống điểm thả
                    └──────────┬───────────┘
                               │ Đến nơi
                    ┌──────────▼───────────┐
                    │      RELEASING       │ ← Nhả vật (0.8 giây)
                    └──────────┬───────────┘
                               │ Nhả xong
                    ┌──────────▼───────────┐
                    │      TO_HOME         │ ← Về vị trí HOME an toàn
                    └──────────┬───────────┘
                               │ Về đến HOME
                               │ [Endless Mode: teleport hộp ngẫu nhiên]
                               │ [Vision scan vị trí mới]
                               └──────────► Lặp lại từ đầu ∞
```

---

### GIAI ĐOẠN 3 — Bộ điều khiển tay máy giải IK (`ur5e_robot.py`)

Mỗi **bước lặp 240Hz** (tức là 240 lần mỗi giây), máy trạng thái gửi **1 setpoint tọa độ (X, Y, Z)** xuống bộ điều khiển:

```
Máy trạng thái gửi: "Hãy đến tọa độ [X=0.42, Y=0.15, Z=0.30]"
        │
        ▼
Bộ giải Động học Ngược (IK Solver) tính toán:
"Để bàn tay đến đúng điểm đó, 6 khớp phải quay góc bao nhiêu?"
  → θ₁(vai quay) = +15°
  → θ₂(vai nâng) = -72°
  → θ₃(khuỷu tay) = +95°
  → θ₄(cổ tay 1) = -113°
  → θ₅(cổ tay 2) = -90°
  → θ₆(cổ tay 3) = +15°
        │
        ▼
Gửi setpoint góc cho từng servo motor (tương đương ghi thanh ghi Position
vào biến tần/driver của từng khớp qua EtherCAT)

        ┌─── Chọn nghiệm "cùi chỏ hướng lên" (tránh va chạm)
        └─── Thêm jointDamping để servo không rung lắc
```

> 📌 **Analog thực tế**: Trong robot Fanuc/KUKA thực, controller giải IK nội bộ trong 8ms mỗi chu kỳ. Đây chính là việc đó — chỉ chạy trong phần mềm.

---

### GIAI ĐOẠN 4 — Cơ cấu kẹp bám theo EE (`gripper.py`)

```
Mỗi vòng lặp:
  1. Đọc vị trí + hướng đầu công cụ (TCP) từ robot
  2. Dịch gripper đến đúng vị trí TCP (offset 5cm theo trục tool)
  3. Nếu đang KẸPVẬT → kéo khối hộp đỏ đi theo gripper
     (mô phỏng lực ma sát của ngón kẹp giữ vật)
```

---

### GIAI ĐOẠN 5 — Người vận hành tương tác qua HMI (`hmi_panel.py`)

```
Operator nhìn vào màn hình HMI thấy 3 nút:

[>> START / PAUSE]  →  Bấm lần 1: PAUSE (robot dừng đứng yên tại chỗ)
                        Bấm lần 2: RESUME (tiếp tục chu trình)

[!! EMERGENCY HOME] →  Bấm: Robot nhả vật NGAY LẬP TỨC, về HOME
                        (tương đương nút E-STOP trên tủ điện)

[Z-Offset slider]   →  Kéo ±50mm để tinh chỉnh chiều cao gắp
                        (operator thấy gripper hơi cao → kéo xuống)
```

---

## 📡 Sơ đồ truyền dữ liệu giữa các module

```
                    vision_camera.py
                    ┌──────────────┐
                    │  VisionCamera│
                    │  .detect()   │──── DetectionResult(X,Y,Z,Yaw)
                    └──────────────┘              │
                                                  ▼
    hmi_panel.py            task_scheduler.py ◄───┘
    ┌──────────────┐        ┌─────────────────┐
    │  HMIPanel    │        │  TaskScheduler  │
    │  .read()     │──────► │  .update()      │──── waypoint(X,Y,Z)
    └──────────────┘        │                 │──── grasp_yaw (rad)
     HMIState(              └─────────────────┘         │
      pause_pressed,                                     ▼
      estop_pressed,               ur5e_robot.py
      z_offset)             ┌─────────────────────┐
                            │  UR5eRobot           │
                            │  .move_to(X,Y,Z,orn) │──► 6 servo motors
                            └─────────────────────┘
                                       │ get_ee_pose()
                                       ▼
                            gripper.py
                            ┌─────────────────────┐
                            │  Gripper             │
                            │  .update_pose()      │──► kéo object theo
                            └─────────────────────┘

    Tất cả được điều phối bởi: simulation.py / SimulationApp.run() (240Hz)
```

---

## 🚀 Cài đặt & Chạy

### Yêu cầu hệ thống

```bash
pip install pybullet numpy opencv-python
```

### Chạy mô phỏng

```bash
cd d:\DoAn_UR5e
python simulation.py
```

### Chạy kiểm tra module

```bash
python test_refactor.py
```

---

## 🎮 Hướng dẫn vận hành

Khi cửa sổ PyBullet mở ra:

| Thao tác | Kết quả |
|---|---|
| Bấm **`>> START / PAUSE`** | Tạm dừng / tiếp tục chu trình |
| Bấm **`!! EMERGENCY HOME !!`** | Robot nhả vật, về HOME ngay lập tức |
| Kéo **`Z-Offset`** sang phải | Gripper hạ thêm (+mm) khi gắp |
| Kéo **`Z-Offset`** sang trái | Gripper nâng lên (−mm) khi gắp |
| Giữ chuột + kéo (trong cửa sổ 3D) | Xoay góc nhìn camera |
| Scroll chuột | Zoom in/out |
| **Đóng cửa sổ** | Kết thúc mô phỏng |

---

## 📐 Thông số kỹ thuật

| Thông số | Giá trị | Ghi chú |
|---|---|---|
| Robot | UR5e (e-Series) | 6 bậc tự do, revolute joints |
| Tầm với tối đa | 0.75 m | Giới hạn an toàn do Vision Module kiểm tra |
| Vùng làm việc Pick | X ∈ [0.3, 0.5] m, Y ∈ [−0.2, 0.2] m | Ngẫu nhiên mỗi chu trình |
| Vị trí Place cố định | [0.4, −0.2, 0.025] m | Luôn đặt đây |
| Tần suất vòng lặp | 240 Hz | Tương đương chu kỳ scan PLC 4.2 ms |
| Phương pháp IK | Null-Space + Joint Damping | Tránh singularity, chống rung |
| Nội suy quỹ đạo | S-Curve bậc 5 (Quintic) | Gia tốc mượt, không giật cơ học |
| Camera ảo | Top-down, FOV 60°, 640×480 | Tại [0.4, 0.0, 1.0] m |

---

## 🏗️ Cấu trúc thư mục

```
d:\DoAn_UR5e\
├── simulation.py        ← ĐIỂM KHỞI ĐỘNG — chạy file này
├── ur5e_robot.py        ← Bộ điều khiển tay máy
├── vision_camera.py     ← Camera + xử lý ảnh
├── gripper.py           ← Cơ cấu kẹp
├── task_scheduler.py    ← PLC / Máy trạng thái
├── hmi_panel.py         ← Màn hình HMI
├── ur5e.urdf            ← Bản vẽ 3D/cơ học của robot
└── test_refactor.py     ← Kiểm tra độc lập từng module
```

---

## 💡 Ghi chú cho sinh viên

> **Tại sao lại tách thành 6 file riêng?**
>
> Trong hệ thống công nghiệp thực, Camera, Robot Controller, PLC, và HMI là **các thiết bị vật lý riêng biệt**, giao tiếp qua bus trường (EtherCAT, Profibus, OPC-UA).
>
> Tách code thành 6 file mô phỏng đúng kiến trúc đó:
> - Muốn đổi camera khác → thay `vision_camera.py`
> - Muốn đổi robot model khác → thay `ur5e_robot.py`
> - Muốn thêm trạng thái SCAN → chỉ sửa `task_scheduler.py`
>
> Không có module nào biết chi tiết bên trong của module kia — giống như Camera Cognex không cần biết Robot Fanuc giải IK như thế nào.

---

*Phiên bản: 2.0 (OOP Refactored) | Framework tham khảo: AIRobot (MIT), pybullet_ur5_robotiq*
