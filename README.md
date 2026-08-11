markdown
<div align="center">
  <a href="#-english">🇬🇧 English</a> | <a href="#-tiếng-việt">🇻🇳 Tiếng Việt</a>
</div>

---

<a id="-english"></a>
## 🇬🇧 English

# QUADSIM — Quadcopter Simulation and Path Planning

**QUADSIM** is a Digital Twin simulation software for quadcopters, integrating 6-DOF dynamics, Cascade PID control, A* path planning, and multiple obstacle detection methods. Designed with a clear modular architecture, it is suitable for both beginners and advanced developers in flight mechanics and control systems.

Users can interact with the software through:
- **Web GUI** (Streamlit) — click-based interaction
- **Command Line Interface (CLI)** — terminal menu
- **Python Library** — import and extend

---

## Features

| Feature | Description |
|---------|-------------|
| **6-DOF Dynamics** | Newton-Euler equations, RK4 integration, Euler angle representation |
| **Cascade PID** | 4 layers: Position -> Velocity -> Attitude -> Rate with anti-windup |
| **Path Planning** | A* algorithm with obstacle inflation and RDP simplification |
| **Obstacle Detection** | 4 methods: threshold, HSV color, OpenStreetMap, SAM click-to-segment |
| **Waypoint Manager** | Per-waypoint altitude, yaw, and epsilon (acceptance radius) |
| **GUI** | Streamlit web interface with mouse interaction |
| **Unit Testing** | 24 test cases validating physical invariants |
| **Auto Tuning** | Nelder-Mead optimization for PID gains |

---

## Installation

### System Requirements
- **Python 3.10 or higher**
- pip package manager
- (Optional) Git for cloning

### Quick Install (Windows)

```cmd
setup.bat
Follow the menu to choose installation type:

CLI only (minimal)

GUI (with Streamlit)

GUI + SAM (includes AI segmentation)

Manual Install
bash
cd quadsim

# Create virtual environment
python -m venv .venv
source .venv/bin/activate      # Linux/macOS
.venv\Scripts\activate          # Windows

# Install dependencies
pip install -r requirements_gui.txt

# Optional: Install SAM for AI obstacle detection
pip install -r requirements_gui_sam.txt
pip install "git+https://github.com/ChaoningZhang/MobileSAM.git"
Quick Start
1. Web Interface (GUI)
bash
streamlit run app_gui.py
Workflow:

Upload map image

Set scale (m/pixel)

Detect obstacles (4 methods available)

Place waypoints (manual click or A* auto)

Adjust altitude, yaw, epsilon per waypoint

Run simulation and view results

2. Command Line Interface (CLI)
bash
python main.py
Menu options:

[1] Switch drone preset

[2] View drone parameters

[3] Open-loop scenario (ascend -> roll -> pitch -> yaw)

[4] Single rotor offset test

[5] Closed-loop PID control

[0] Exit

3. PID Auto-Tuning
bash
python tune_altitude_gain.py
Results saved to outputs/pid_altitude_tuning_real.csv

4. Run Unit Tests
bash
pytest tests/ -v
Expected: 24/24 tests passed.

Project Structure
text
quadsim/
├── quadsim/                      # Core package
│   ├── __init__.py
│   ├── params.py                 # Drone parameters (Crazyflie 2.0)
│   ├── mixer.py                  # Control allocation (omega ↔ wrench)
│   ├── dynamics.py               # Newton-Euler 6-DOF + RK4
│   ├── scenarios.py              # Open-loop test scenarios
│   ├── controllers.py            # Cascade PID + WaypointManager
│   ├── simulate.py               # Simulation loops
│   ├── plotting.py               # Plot generation and saving
│   ├── cli.py                    # Terminal interface
│   ├── pathfinding.py            # A* + inflate + RDP
│   ├── color_obstacles.py        # HSV-based obstacle detection
│   ├── osm_obstacles.py          # OpenStreetMap query
│   ├── sam_obstacles.py          # MobileSAM click-to-segment
│   ├── waypoint_io.py            # JSON I/O and coordinate calibration
│   ├── waypoint_editor.py        # Standalone waypoint picker
│   └── metrics.py                # Performance metrics
│
├── assets/                        # Resources
│   └── fonts/
│       ├── DejaVuSans.ttf
│       └── DejaVuSans-Bold.ttf
│
├── main.py                        # CLI entry point
├── app_gui.py                     # GUI entry point
├── tune_altitude_gain.py          # PID optimization script
│
├── requirements.txt               # Core dependencies
├── requirements_gui.txt           # GUI dependencies
├── requirements_gui_sam.txt       # GUI + SAM dependencies
├── setup.bat                      # Windows auto-install script
│
├── README.md                      # This file
└── .gitignore                     # Git ignore rules
Technical Specifications
Drone Model: Crazyflie 2.0
Parameter	Value
Mass	0.027 kg
Wheelbase (diagonal)	0.092 m
Ixx, Iyy	1.66e-5 kg·m²
Izz	2.93e-5 kg·m²
Thrust coefficient (kF)	2.359e-8 N/(rad/s)²
Drag coefficient (kM)	1.297e-10 N·m/(rad/s)²
Max motor speed	2513 rad/s
Control Structure
Layer	Time Constant	Gain Type
Position	1.5 s	P
Velocity	0.4 s	PID
Attitude	0.12 s	P
Rate	0.03 s	PID
PID Optimization Result (vel_z channel)
Gain Set	Overshoot	Settling Time	Steady Error	Saturation	Cost
Baseline (2.500, 0.625, 0.125)	1.90%	4.30s	0.038m	0.0%	4.225
Optimized (15.000, 2.999, 0.234)	0.00%	5.47s	0.025m	0.5%	3.253
References
T. Luukkonen, "Modelling and control of quadcopter," Aalto University, 2011.

J. Forster, "System Identification of the Crazyflie 2.0 Nano Quadrocopter," ETH Zurich, 2015.

K. Bouzgou, Y. Bestaoui, L. Benchikh, B. Ibari, Z. Ahmed-Foitih, "Dynamic modeling, simulation and PID controller of unmanned aerial vehicle UAV," 7th International Conference on Innovative Computing Technology (INTECH), 2017.

L. Carlone, M. Ryll, "Quadrotor Dynamics," 16.485 Visual Navigation for Autonomous Vehicles (VNAV), MIT, Fall 2023.

D. Mellinger, V. Kumar, "Minimum snap trajectory generation and control for quadrotors," IEEE International Conference on Robotics and Automation (ICRA), 2011.

License
MIT License

Contact
Author: Tran Anh Khoi


<a id="-tiếng-việt"></a>

🇻🇳 Tiếng Việt
QUADSIM — Mô phỏng Quadcopter và Lập kế hoạch đường bay
QUADSIM là phần mềm mô phỏng Digital Twin cho quadcopter, tích hợp động lực học 6-DOF, điều khiển Cascade PID, lập kế hoạch đường bay A* và nhiều phương án nhận diện vật cản. Với kiến trúc module rõ ràng, phần mềm phù hợp cho cả người mới bắt đầu và các nhà phát triển chuyên sâu về cơ học bay và hệ thống điều khiển.

Người dùng có thể tương tác với phần mềm qua:

Giao diện Web (GUI) — Streamlit, tương tác bằng chuột

Giao diện dòng lệnh (CLI) — menu terminal

Thư viện Python — import và mở rộng

Tính năng
Tính năng	Mô tả
Động lực học 6-DOF	Phương trình Newton-Euler, tích phân RK4, biểu diễn góc Euler
Cascade PID	4 tầng: Vị trí -> Vận tốc -> Góc -> Tốc độ góc, có anti-windup
Lập kế hoạch đường bay	Thuật toán A* với phình vật cản và rút gọn RDP
Nhận diện vật cản	4 phương án: ngưỡng tối, màu HSV, OpenStreetMap, SAM click-to-segment
Quản lý waypoint	Độ cao, góc yaw và epsilon (bán kính chấp nhận) riêng từng điểm
Giao diện GUI	Streamlit web, tương tác bằng chuột
Kiểm thử tự động	24 test case kiểm tra bất biến vật lý
Tối ưu tự động	Nelder-Mead tối ưu tham số PID
Cài đặt
Yêu cầu hệ thống
Python 3.10 trở lên

pip

(Tùy chọn) Git để clone

Cài đặt nhanh (Windows)
cmd
setup.bat
Chọn loại cài đặt:

CLI (cơ bản)

GUI (có Streamlit)

GUI + SAM (có AI nhận diện vật cản)

Cài đặt thủ công
bash
# Clone repository
git clone https://github.com/yourusername/quadsim.git
cd quadsim

# Tạo môi trường ảo
python -m venv .venv
source .venv/bin/activate      # Linux/macOS
.venv\Scripts\activate          # Windows

# Cài đặt thư viện
pip install -r requirements_gui.txt

# Tùy chọn: Cài SAM cho nhận diện vật cản bằng AI
pip install -r requirements_gui_sam.txt
pip install "git+https://github.com/ChaoningZhang/MobileSAM.git"
Bắt đầu nhanh
1. Giao diện Web (GUI)
bash
streamlit run app_gui.py
Quy trình:

Tải ảnh bản đồ

Hiệu chỉnh tỉ lệ (m/pixel)

Nhận diện vật cản (4 phương án)

Đặt waypoint (click thủ công hoặc A* tự động)

Chỉnh sửa độ cao, yaw, epsilon từng điểm

Chạy mô phỏng và xem kết quả

2. Giao diện dòng lệnh (CLI)
bash
python main.py
Menu chính:

[1] Đổi drone preset

[2] Xem thông số drone

[3] Chạy kịch bản vòng hở

[4] Kiểm tra rotor lệch

[5] Điều khiển vòng kín PID

[0] Thoát

3. Tối ưu PID tự động
bash
python tune_altitude_gain.py
Kết quả lưu tại outputs/pid_altitude_tuning_real.csv

4. Chạy kiểm thử
bash
pytest tests/ -v
Kết quả: 24/24 test PASSED.

Cấu trúc dự án
text
quadsim/
├── quadsim/                      # Package lõi
│   ├── __init__.py
│   ├── params.py                 # Tham số drone (Crazyflie 2.0)
│   ├── mixer.py                  # Phân bổ điều khiển (omega ↔ wrench)
│   ├── dynamics.py               # Newton-Euler 6-DOF + RK4
│   ├── scenarios.py              # Kịch bản vòng hở
│   ├── controllers.py            # Cascade PID + WaypointManager
│   ├── simulate.py               # Vòng lặp mô phỏng
│   ├── plotting.py               # Vẽ và lưu đồ thị
│   ├── cli.py                    # Giao diện terminal
│   ├── pathfinding.py            # A* + inflate + RDP
│   ├── color_obstacles.py        # Nhận diện vật cản HSV
│   ├── osm_obstacles.py          # Truy vấn OpenStreetMap
│   ├── sam_obstacles.py          # SAM click-to-segment
│   ├── waypoint_io.py            # JSON + hiệu chỉnh tọa độ
│   ├── waypoint_editor.py        # Công cụ chọn waypoint độc lập
│   └── metrics.py                # Tiêu chí đánh giá
│
├── assets/                        # Tài nguyên
│   └── fonts/
│       ├── DejaVuSans.ttf
│       └── DejaVuSans-Bold.ttf
│
├── main.py                        # Điểm vào CLI
├── app_gui.py                     # Điểm vào GUI
├── tune_altitude_gain.py          # Tối ưu PID
│
├── requirements.txt               # Thư viện cơ bản
├── requirements_gui.txt           # Thư viện GUI
├── requirements_gui_sam.txt       # Thư viện GUI + SAM
├── setup.bat                      # Cài đặt tự động (Windows)
│
├── README.md                      # File này
└── .gitignore                     # Bỏ qua file rác
Thông số kỹ thuật
Drone: Crazyflie 2.0
Tham số	Giá trị
Khối lượng	0.027 kg
Đường chéo	0.092 m
Ixx, Iyy	1.66e-5 kg·m²
Izz	2.93e-5 kg·m²
Hệ số lực đẩy (kF)	2.359e-8 N/(rad/s)²
Hệ số mô men cản (kM)	1.297e-10 N·m/(rad/s)²
Tốc độ động cơ max	2513 rad/s
Cấu trúc điều khiển
Tầng	Hằng số thời gian	Loại
Vị trí	1.5 s	P
Vận tốc	0.4 s	PID
Góc	0.12 s	P
Tốc độ góc	0.03 s	PID
Kết quả tối ưu PID (kênh vel_z)
Bộ gain	Vọt lố	Thời gian xác lập	Sai số xác lập	Bão hòa	Cost
Baseline (2.500, 0.625, 0.125)	1.90%	4.30s	0.038m	0.0%	4.225
Tối ưu (15.000, 2.999, 0.234)	0.00%	5.47s	0.025m	0.5%	3.253
Tài liệu tham khảo
T. Luukkonen, "Modelling and control of quadcopter," Aalto University, 2011.

J. Forster, "System Identification of the Crazyflie 2.0 Nano Quadrocopter," ETH Zurich, 2015.

K. Bouzgou, Y. Bestaoui, L. Benchikh, B. Ibari, Z. Ahmed-Foitih, "Dynamic modeling, simulation and PID controller of unmanned aerial vehicle UAV," 7th International Conference on Innovative Computing Technology (INTECH), 2017.

L. Carlone, M. Ryll, "Quadrotor Dynamics," 16.485 Visual Navigation for Autonomous Vehicles (VNAV), MIT, Fall 2023.

D. Mellinger, V. Kumar, "Minimum snap trajectory generation and control for quadrotors," IEEE International Conference on Robotics and Automation (ICRA), 2011.

Giấy phép
MIT License

Liên hệ
Tác giả: Trần Anh Khôi
