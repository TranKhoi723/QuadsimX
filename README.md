# QUADSIM — Phần mềm mô phỏng Quadcopter và lập kế hoạch đường bay

## 1. Giới thiệu

QUADSIM là phần mềm mô phỏng "Digital Twin" cho máy bay không người lái quadcopter. Phần mềm tích hợp các mô-đun chính:

- Mô hình động lực học 6 bậc tự do (6-DOF) dựa trên phương trình Newton-Euler.
- Bộ điều khiển Cascade PID 4 tầng (Vị trí → Vận tốc → Góc → Tốc độ góc).
- Thuật toán lập kế hoạch đường bay A* kết hợp với làm mịn đường đi bằng Ramer-Douglas-Peucker (RDP).
- Bốn phương án nhận diện vật cản: ngưỡng độ sáng, phân loại màu HSV, truy vấn OpenStreetMap, và SAM click-to-segment.
- Giao diện web (Streamlit) cho phép tương tác bằng chuột, và giao diện dòng lệnh (CLI) cho các thao tác nhanh.

Phần mềm được thiết kế theo kiến trúc module, dễ dàng mở rộng và tích hợp.

## 2. Cài đặt

### 2.1. Yêu cầu hệ thống

- Python 3.10 trở lên.
- Pip (có sẵn khi cài Python).
- (Khuyến nghị) Git để sao chép mã nguồn.

### 2.2. Tải mã nguồn

Mở terminal (Command Prompt trên Windows) và chạy:

```bash
git clone https://github.com/yourusername/quadsim.git
cd quadsim
(Nếu không có Git, bạn có thể tải file ZIP từ GitHub và giải nén.)

2.3. Tạo môi trường ảo (khuyến nghị)
bash
python -m venv .venv
Kích hoạt môi trường ảo:

Windows: .venv\Scripts\activate

Linux / macOS: source .venv/bin/activate

2.4. Cài đặt các thư viện phụ thuộc
2.4.1. Phiên bản cơ bản (chạy CLI)
bash
pip install -r requirements.txt
2.4.2. Phiên bản có giao diện web (GUI)
bash
pip install -r requirements_gui.txt
2.4.3. Phiên bản đầy đủ (GUI + SAM hỗ trợ nhận diện vật cản bằng AI)
bash
pip install -r requirements_gui.txt
pip install -r requirements_gui_sam.txt
pip install "git+https://github.com/ChaoningZhang/MobileSAM.git"
Lưu ý: Phần SAM (Segment Anything) yêu cầu tải thêm model khoảng 40MB và thư viện PyTorch, có thể mất vài phút. Nếu bạn không cần tính năng này, có thể bỏ qua.

2.4.4. Cài đặt tự động trên Windows (dùng script)
Chạy file setup.bat trong thư mục dự án và chọn loại cài đặt mong muốn.

3. Hướng dẫn sử dụng
3.1. Chạy giao diện dòng lệnh (CLI)
bash
python main.py
Menu chính sẽ hiện ra với các tùy chọn:

[1] Đổi drone (preset hiện có: Crazyflie 2.0 và X-Custom)

[2] Xem thông số drone

[3] Chạy kịch bản vòng hở (leo cao → roll → pitch → yaw)

[4] Chạy kịch bản rotor lệch (mô phỏng lỗi động cơ)

[5] Chạy mô phỏng vòng kín với bộ điều khiển Cascade PID (có thể nhập setpoint)

[0] Thoát

Các đồ thị kết quả sẽ được lưu vào thư mục outputs/ dưới dạng file PNG.

3.2. Chạy giao diện web (GUI)
bash
streamlit run app_gui.py
Trình duyệt sẽ tự động mở tại địa chỉ http://localhost:8501.

Quy trình thao tác trên GUI:

Tải ảnh bản đồ (định dạng PNG, JPG, JPEG, BMP).

Hiệu chỉnh tỉ lệ (m/pixel) để chuyển đổi tọa độ pixel sang mét.

Phát hiện vùng cấm bay (chọn một trong bốn phương án):

Ngưỡng độ sáng: thích hợp với sơ đồ vẽ tay.

Phân loại màu HSV: cho ảnh vệ tinh (phân biệt cây, nước, mái nhà).

OpenStreetMap: cần nhập tọa độ GPS của hai góc ảnh, độ chính xác cao.

SAM click-to-segment: dùng chuột click lên vật thể, AI sẽ tự khoanh vùng.

Đặt waypoint:

Thủ công: click lên ảnh để thêm từng điểm.

Tự động (A*): click điểm đầu và điểm cuối, hệ thống tự tìm đường né vật cản.

Chỉnh sửa waypoint trong bảng bên phải: độ cao (m), góc yaw (độ), và bán kính epsilon (m) riêng từng điểm.

Chạy mô phỏng: nhấn nút "Chạy mô phỏng" và xem quỹ đạo bay, đồ thị vị trí/góc/tốc độ động cơ.

Lưu / tải file JSON: xuất waypoints và vùng cấm bay để dùng lại sau.

3.3. Tối ưu tham số PID tự động
Chạy script:

bash
python tune_altitude_gain.py
Script sử dụng thuật toán Nelder-Mead để tìm bộ ba (Kp, Ki, Kd) tối ưu cho kênh vận tốc đứng (vel_z). Kết quả sẽ được lưu vào file CSV và đồ thị so sánh đáp ứng trước/sau tối ưu trong thư mục outputs/.

3.4. Chạy bộ kiểm thử tự động
bash
pytest tests/ -v
Dự kiến 24 test case đều PASSED, kiểm tra các bất biến vật lý như rơi tự do, hover, tính trực giao của ma trận xoay, và hội tụ của RK4.

4. Cấu trúc thư mục
text
quadsim/
├── quadsim/                      # Package chính (các module tính toán)
│   ├── __init__.py
│   ├── params.py                 # Tham số drone (Crazyflie 2.0, X-Custom)
│   ├── mixer.py                  # Phân bổ điều khiển (tốc độ động cơ ↔ lực/mô-men)
│   ├── dynamics.py               # Động lực học Newton-Euler 6-DOF + RK4
│   ├── scenarios.py              # Kịch bản vòng hở
│   ├── controllers.py            # Cascade PID và WaypointManager
│   ├── simulate.py               # Vòng lặp mô phỏng
│   ├── plotting.py               # Vẽ đồ thị
│   ├── cli.py                    # Giao diện dòng lệnh
│   ├── pathfinding.py            # Thuật toán A* và RDP
│   ├── color_obstacles.py        # Nhận diện vật cản theo màu HSV
│   ├── osm_obstacles.py          # Lấy dữ liệu vật cản từ OpenStreetMap
│   ├── sam_obstacles.py          # MobileSAM click-to-segment
│   ├── waypoint_io.py            # Đọc/ghi JSON, hiệu chỉnh tọa độ
│   ├── waypoint_editor.py        # Công cụ chọn waypoint độc lập
│   └── metrics.py                # Các chỉ số đánh giá hiệu năng
│
├── assets/                        # Tài nguyên (font chữ)
│   └── fonts/
│       ├── DejaVuSans.ttf
│       └── DejaVuSans-Bold.ttf
│
├── main.py                        # Điểm vào CLI
├── app_gui.py                     # Điểm vào GUI (Streamlit)
├── tune_altitude_gain.py          # Tối ưu PID
│
├── requirements.txt               # Thư viện cơ bản
├── requirements_gui.txt           # Thư viện GUI
├── requirements_gui_sam.txt       # Thư viện GUI + SAM
├── setup.bat                      # Script cài đặt tự động (Windows)
│
└── README.md                      # File hướng dẫn này
5. Thông số drone mặc định
Phần mềm sử dụng thông số của drone Crazyflie 2.0 (từ nghiên cứu của Förster 2015):

Tham số	Giá trị
Khối lượng	0.027 kg
Đường chéo (giữa hai động cơ đối diện)	0.092 m
Mô-men quán tính Ixx, Iyy	1.66e-5 kg·m²
Mô-men quán tính Izz	2.93e-5 kg·m²
Hệ số lực đẩy kF	2.359e-8 N/(rad/s)²
Hệ số mô-men cản kM	1.297e-10 N·m/(rad/s)²
Tốc độ góc cực đại của động cơ	2513 rad/s
Bạn có thể chọn drone khác (ví dụ X-Custom) hoặc thêm preset mới trong file params.py.

6. Các tài liệu tham khảo chính
T. Luukkonen, "Modelling and control of quadcopter," Aalto University, 2011.

J. Förster, "System Identification of the Crazyflie 2.0 Nano Quadrocopter," ETH Zurich, 2015.

K. Bouzgou et al., "Dynamic modeling, simulation and PID controller of UAV," INTECH, 2017.

L. Carlone, M. Ryll, "Quadrotor Dynamics," MIT 16.485, 2023.

D. Mellinger, V. Kumar, "Minimum snap trajectory generation," ICRA, 2011.

7. Liên hệ và giấy phép
Tác giả: Trần Anh Khôi

Giấy phép: MIT (xem file LICENSE)

Chúc bạn sử dụng phần mềm hiệu quả!
