# -*- coding: utf-8 -*-
"""
================================================================================
  MODULE: VisionCamera — Hệ thống thị giác máy (Machine Vision System)
================================================================================
  Vai trò thực tế: Tương đương một Camera công nghiệp (Cognex/Keyence)
  kết hợp với bộ xử lý ảnh đặt phía trên băng tải.

  Nhiệm vụ chính:
    1. Chụp ảnh từ góc nhìn từ trên cao (top-down view)
    2. Lọc màu sắc → xác định vùng có vật cần gắp
    3. Tính tọa độ thực (X, Y, Z) của tâm vật trong hệ tọa độ robot (mét)
    4. Đo góc nghiêng của vật → truyền về cho gripper xoay đúng hướng kẹp
    5. Kiểm tra an toàn: vật có nằm trong vùng làm việc không?

  Output truyền về cho PLC (TaskScheduler):
    DetectionResult(X=0.42, Y=0.15, Z=0.025, Yaw=+23°)
    → PLC dùng X,Y,Z để lập kế hoạch di chuyển
    → PLC dùng Yaw để xoay cổ tay gripper đúng góc kẹp

  Camera ảo:
    Vị trí: [0.4, 0.0, 1.0] m (treo trên không, cao 1m)
    Hướng: nhìn thẳng xuống bàn làm việc
    FOV: 60° — nhìn thấy toàn bộ vùng gắp
================================================================================
"""

import pybullet as p
import numpy as np
import math
from collections import namedtuple

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("  [WARN] OpenCV (cv2) chua duoc cai dat. Vision module se bi TAT.")
    print("         Chay: pip install opencv-python")


# ── Kết quả nhận diện — có tên rõ ràng thay vì danh sách vô danh [x,y,z,yaw]─
# Giống như đầu ra chuẩn của camera Cognex: một bộ thông số được đánh nhãn đầy đủ
DetectionResult = namedtuple('DetectionResult', ['x', 'y', 'z', 'yaw'])
# x, y, z : tọa độ tâm vật trong hệ tọa độ robot (mét)
# yaw     : góc xoay của vật quanh trục Z (radian) — để gripper căn chỉnh hướng kẹp


# ══════════════════════════════════════════════════════════════════════════════
# HỆ THỐNG THỊ GIÁC MÁY — Camera ảo + Pipeline nhận diện vật đỏ
# ══════════════════════════════════════════════════════════════════════════════

class VisionCamera:
    """
    Mô phỏng hệ thống Camera công nghiệp + bộ xử lý ảnh.

    Trong thực tế, đây là 2 thiết bị vật lý riêng:
      - Camera GigE/USB 3.0 gắn trên khung (cố định, nhìn xuống)
      - PC công nghiệp chạy phần mềm xử lý ảnh (Cognex VisionPro, Halcon, OpenCV)

    Toàn bộ tham số cấu hình (vị trí camera, góc nhìn, màu cần lọc)
    được truyền vào lúc khởi tạo — dễ thay đổi mà không cần sửa code bên trong.
    """

    # ── CẤU HÌNH CAMERA MẶC ĐỊNH ─────────────────────────────────────────────
    # (tương đương cài đặt trong phần mềm camera: focal length, mounting position)
    DEFAULT_CONFIG = {
        'target_pos': [0.4, 0.0, 0.0],    # Tâm điểm camera nhìn vào (trung tâm bàn)
        'eye_pos':    [0.4, 0.0, 1.0],    # Vị trí đặt camera trong không gian 3D (m)
        'up_vector':  [1.0, 0.0, 0.0],    # Hướng "lên" của camera (trục X world)
        'fov':        60.0,                # Góc nhìn dọc: 60° — đủ bao phủ toàn bàn
        'near':       0.01,                # Khoảng cách cận cảnh (m) — vật sát ống kính
        'far':        2.0,                 # Khoảng cách xa nhất camera thấy được (m)
        'width':      640,                 # Độ phân giải ngang: 640 pixels
        'height':     480,                 # Độ phân giải dọc: 480 pixels
    }

    # ── DẢI MÀU CẦN LỌC — Màu ĐỎ trong không gian HSV ──────────────────────
    # Tại sao dùng HSV thay vì RGB? → HSV tách biệt "màu sắc" khỏi "độ sáng"
    # → ít bị ảnh hưởng bởi thay đổi ánh sáng công xưởng hơn so với RGB
    #
    # Màu đỏ trong HSV bị tách thành 2 dải do cách HSV quản lý vòng màu:
    #   Dải 1: Hue 0°—10°  (đỏ bên trái bánh màu)
    #   Dải 2: Hue 165°—180° (đỏ bên phải bánh màu)
    DEFAULT_HSV_RANGES = {
        'lower1': np.array([  0,  80,  60], dtype=np.uint8),  # [Hue, Saturation, Value] min
        'upper1': np.array([ 10, 255, 255], dtype=np.uint8),  # [Hue, Saturation, Value] max
        'lower2': np.array([165,  80,  60], dtype=np.uint8),
        'upper2': np.array([180, 255, 255], dtype=np.uint8),
    }

    def __init__(self, config: dict = None, hsv_ranges: dict = None,
                 max_reach: float = 0.75):
        """
        Lắp đặt camera và hiệu chỉnh (calibrate) lần đầu.

        Trong thực tế: bước này tương đương lắp camera lên giá, cấu hình
        thông số ống kính, và chạy chương trình calibration để biết
        mỗi pixel ảnh tương ứng với vị trí (X, Y, Z) thực nào.

        max_reach: giới hạn vùng làm việc an toàn — vật nằm xa hơn 0.75m
                   tính từ gốc robot sẽ bị bỏ qua để bảo vệ robot.
        """
        # Hợp nhất cấu hình người dùng với mặc định (người dùng có thể override)
        cfg = {**self.DEFAULT_CONFIG, **(config or {})}
        self.hsv = {**self.DEFAULT_HSV_RANGES, **(hsv_ranges or {})}
        self.max_reach = max_reach  # Bán kính tầm với tối đa (m)

        # Lưu thông số ảnh để dùng trong các bước tính toán sau
        self.width  = cfg['width']
        self.height = cfg['height']
        self.near   = cfg['near']
        self.far    = cfg['far']

        # ── Tính ma trận View và Projection — "hiệu chỉnh camera" ────────────
        # View Matrix: mô tả camera ở đâu và nhìn về đâu
        #              (như đặt vị trí và hướng của máy ảnh)
        self.view_matrix = p.computeViewMatrix(
            cameraEyePosition    = cfg['eye_pos'],
            cameraTargetPosition = cfg['target_pos'],
            cameraUpVector       = cfg['up_vector']
        )
        # Projection Matrix: mô tả ống kính camera (góc nhìn, tỷ lệ khung hình)
        #                    (như chọn tiêu cự ống kính: góc rộng vs. telephoto)
        self.proj_matrix = p.computeProjectionMatrixFOV(
            fov     = cfg['fov'],
            aspect  = self.width / self.height,
            nearVal = self.near,
            farVal  = self.far
        )

        print("  [CAM]  Camera ao da duoc khoi tao.")
        print(f"         Vi tri: {cfg['eye_pos']} → {cfg['target_pos']}")
        print(f"         Goc nhin FOV={cfg['fov']}°, {self.width}x{self.height}px")

    def detect_object(self) -> 'DetectionResult | None':
        """
        Chụp ảnh và xử lý để tìm vật cần gắp.

        Đây là PIPELINE xử lý ảnh hoàn chỉnh — 7 bước liên tiếp:

        [1] Chụp ảnh RGB + bản đồ độ sâu (Depth Map)
        [2] Chuyển ảnh sang không gian màu HSV → lọc lấy vùng màu đỏ
        [3] Làm sạch mask: xóa nhiễu nhỏ, lấp lỗ hổng
        [4] Tìm đường viền vật → tính tâm điểm (centroid)
        [4b]Đo góc nghiêng vật (Yaw) bằng hình chữ nhật bao ngoài nhỏ nhất
        [5] Đọc độ sâu tại tâm điểm → biết vật cách camera bao nhiêu mét
        [6] Chuyển từ tọa độ pixel → tọa độ 3D thực (X, Y, Z) trong hệ robot
        [7] Kiểm tra tầm với: vật có trong vùng làm việc an toàn không?

        Trả về: DetectionResult(x, y, z, yaw) hoặc None nếu không tìm thấy
        """
        if not CV2_AVAILABLE:
            print("  [VISION] OpenCV khong co san. Giu nguyen pick_pos mac dinh.")
            return None

        # ── BƯỚC 1: Chụp ảnh từ Camera ảo ───────────────────────────────────
        # Tương đương trigger camera bằng tín hiệu I/O từ PLC
        # getCameraImage trả về: ảnh màu (RGB) + bản đồ độ sâu (Depth Buffer)
        # Depth Buffer: mỗi pixel lưu khoảng cách từ camera đến điểm đó (0→1)
        _, _, rgb_pixels, depth_pixels, _ = p.getCameraImage(
            width            = self.width,
            height           = self.height,
            viewMatrix       = self.view_matrix,       # Vị trí camera
            projectionMatrix = self.proj_matrix,        # Thông số ống kính
            renderer         = p.ER_TINY_RENDERER       # Dùng renderer nhẹ (CPU)
        )

        # ── BƯỚC 2: Chuyển định dạng ảnh PyBullet → OpenCV ──────────────────
        # PyBullet: ảnh RGBA 4 kênh (Red, Green, Blue, Alpha/transparency)
        # OpenCV:   ảnh BGR 3 kênh (Blue, Green, Red — thứ tự ngược!)
        # Phải reshape về đúng kích thước (height × width × 4 channels)
        rgb_array = np.array(rgb_pixels, dtype=np.uint8).reshape(
            self.height, self.width, 4
        )
        bgr_image = cv2.cvtColor(rgb_array, cv2.COLOR_RGBA2BGR)

        # ── BƯỚC 3: Lọc màu đỏ bằng HSV ─────────────────────────────────────
        # Chuyển từ BGR sang HSV để lọc màu chính xác hơn dưới ánh sáng thay đổi
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

        # Tạo mask nhị phân: pixel trắng = đỏ, pixel đen = không đỏ
        mask1 = cv2.inRange(hsv_image, self.hsv['lower1'], self.hsv['upper1'])
        mask2 = cv2.inRange(hsv_image, self.hsv['lower2'], self.hsv['upper2'])
        red_mask = cv2.bitwise_or(mask1, mask2)  # Gộp 2 dải đỏ lại

        # Làm sạch mask: loại bỏ nhiễu nhỏ và lấp các lỗ hổng trong vùng vật
        # MORPH_CLOSE: lấp lỗ hổng bên trong vùng trắng (vùng đỏ bị đứt quãng)
        # MORPH_OPEN:  xóa các điểm trắng nhỏ li ti (nhiễu độc lập)
        kernel   = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN,  kernel)

        # ── BƯỚC 4: Tìm đường viền và chọn vật thể chính ────────────────────
        # findContours: tìm tất cả vùng trắng liền kề trong mask
        # Chọn contour LỚN NHẤT → đó là vật thể chính, loại bỏ nhiễu nhỏ
        contours, _ = cv2.findContours(
            red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            print("  [VISION] Khong phat hien vat do trong anh camera!")
            return None

        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)

        # Bỏ qua nếu vùng phát hiện quá nhỏ (< 50 pixel²) — chắc là nhiễu
        if area < 50:
            print(f"  [VISION] Vung mau do qua nho ({area:.0f}px2), co the la nhieu.")
            return None

        # ── BƯỚC 4b: Đo góc nghiêng vật (Yaw) bằng minAreaRect ──────────────
        # minAreaRect: tìm hình chữ nhật bao ngoài nhỏ nhất bao quanh vật
        # Từ đó tính được góc nghiêng của cạnh dài hình chữ nhật = góc xoay vật
        #
        # Tại sao cần đo góc? → Để gripper xoay cổ tay đúng góc trước khi kẹp.
        # Nếu vật nằm nghiêng 30° mà gripper kẹp thẳng → kẹp hụt hoặc kẹp méo.
        rect = cv2.minAreaRect(largest_contour)
        angle_deg = rect[2]  # Góc của hình chữ nhật bao ngoài (−90° đến 0°)

        # Quy chuẩn: nếu chiều rộng < chiều cao thì cộng thêm 90° để lấy hướng cạnh dài
        if rect[1][0] < rect[1][1]:
            angle_deg += 90.0

        # Chuyển sang radian và chuẩn hóa về [−π/2, +π/2]
        # Vì gripper đối xứng 180°: kẹp xuôi = kẹp ngược → không cần phân biệt
        yaw_rad = math.radians(angle_deg)
        if yaw_rad > math.pi / 2:
            yaw_rad -= math.pi
        elif yaw_rad < -math.pi / 2:
            yaw_rad += math.pi

        print(f"  [VISION] Goc xoay hop (Yaw): {math.degrees(yaw_rad):+.1f} deg "
              f"({yaw_rad:+.4f} rad)")

        # ── Tính tâm điểm (centroid) của vùng vật ───────────────────────────
        # Moment tích phân: tính tọa độ trọng tâm của vùng màu trắng trong mask
        # cx_px, cy_px = cột và hàng của điểm trung tâm vật (tính bằng pixel)
        M = cv2.moments(largest_contour)
        if M['m00'] == 0:
            return None  # Không có diện tích → không tính được tâm

        cx_px = int(M['m10'] / M['m00'])   # Tọa độ cột (trục ngang) của tâm
        cy_px = int(M['m01'] / M['m00'])   # Tọa độ hàng (trục dọc) của tâm

        print(f"  [VISION] Phat hien vat tai pixel: ({cx_px}, {cy_px}), "
              f"Area={area:.0f}px2")

        # ── BƯỚC 5: Đọc Depth Buffer để biết độ sâu tại tâm vật ─────────────
        # Depth Buffer: mỗi giá trị trong [0, 1] tương ứng với khoảng cách
        # 0.0 = sát ống kính (near plane), 1.0 = xa nhất (far plane)
        depth_buffer = np.array(depth_pixels).reshape(self.height, self.width)
        depth_ndc    = float(depth_buffer[cy_px, cx_px])  # Giá trị tại điểm tâm vật

        # Chuyển giá trị NDC (phi tuyến) → khoảng cách tuyến tính thực (mét)
        # Công thức này xuất phát từ cách OpenGL mã hóa depth buffer (non-linear)
        z_eye = (2.0 * self.far * self.near) / \
                (self.far + self.near - depth_ndc * (self.far - self.near))

        # ── BƯỚC 6: Chuyển tọa độ pixel → tọa độ 3D thực tế (Unproject) ─────
        # NDC (Normalized Device Coordinates): hệ tọa độ chuẩn hóa [-1, +1]
        # NDC_x: pixel bên trái = -1, pixel bên phải = +1
        # NDC_y: pixel dưới cùng = -1, pixel trên cùng = +1 (ngược với pixel)
        ndc_x = (cx_px + 0.5) / self.width  * 2.0 - 1.0
        ndc_y = 1.0 - (cy_px + 0.5) / self.height * 2.0
        ndc_z = (self.far + self.near - 2.0 * self.near * self.far / z_eye) / \
                (self.far - self.near)

        # "Unproject": đảo ngược quá trình camera chiếu 3D→2D
        # Dùng ma trận nghịch đảo để tính ngược từ pixel → điểm 3D trong world
        # (tương đương bài toán ray-casting trong calibration camera)
        proj_matrix_np = np.array(self.proj_matrix).reshape(4, 4).T
        view_matrix_np = np.array(self.view_matrix).reshape(4, 4).T

        clip_coord  = np.array([ndc_x, ndc_y, ndc_z, 1.0])
        view_coord  = np.linalg.inv(proj_matrix_np) @ clip_coord  # Clip → View space
        view_coord /= view_coord[3]                                 # Chia tọa độ thuần nhất
        world_coord  = np.linalg.inv(view_matrix_np) @ view_coord  # View → World space
        world_coord /= world_coord[3]

        world_x = float(world_coord[0])   # Tọa độ X thực (mét) trong hệ tọa độ robot
        world_y = float(world_coord[1])   # Tọa độ Y thực (mét)
        world_z = float(world_coord[2])   # Tọa độ Z thực (mét) — chiều cao

        # Kẹp Z về đúng tầm bàn làm việc (vật được đặt trên bề mặt Z≈0.025m)
        # Sai số depth buffer nhỏ → cần clamp để IK không bị sai chiều cao
        world_z = max(0.02, min(0.05, world_z))

        # ── BƯỚC 7: Kiểm tra an toàn tầm với ────────────────────────────────
        # Tính khoảng cách từ gốc robot (0,0) đến vị trí vật trên mặt bàn (2D)
        # Nếu vượt quá tầm với → bỏ qua để tránh robot bị kéo căng, nguy hiểm
        reach = math.sqrt(world_x ** 2 + world_y ** 2)
        if reach > self.max_reach:
            print(f"  [VISION] [CANH BAO] Vat qua xa goc robot: "
                  f"{reach:.3f}m > {self.max_reach}m")
            print(f"           Bo qua vi tri nay de bao ve robot!")
            return None  # Trả về None → TaskScheduler sẽ dùng vị trí dự phòng

        print(f"  [VISION] Toa do vat trong World Frame:")
        print(f"           X={world_x:+.4f} m, Y={world_y:+.4f} m, "
              f"Z={world_z:+.4f} m")
        print(f"           Tam voi={reach:.3f}m (gioi han {self.max_reach}m)  "
              f"Yaw={math.degrees(yaw_rad):+.1f}deg")

        # Trả về kết quả đầy đủ: vị trí + góc xoay để PLC lập kế hoạch gắp
        return DetectionResult(x=world_x, y=world_y, z=world_z, yaw=yaw_rad)
