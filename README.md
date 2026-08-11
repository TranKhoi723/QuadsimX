<div align="center">
  <a href="#english">English</a> &nbsp; | &nbsp; <a href="#vietnamese">Vietnamese</a>
</div>


# QUADSIM — Quadcopter Simulation and Path Planning
---

<a name="english"></a>
## English

### Introduction

QUADSIM is a Digital Twin simulation software for quadcopter UAVs, integrating 6-DOF dynamics, Cascade PID control, A* path planning, and multiple obstacle detection methods. The software is built with a modular architecture, making it easy to extend and integrate.

**Key features:**
- 6-DOF rigid body dynamics (Newton-Euler equations)
- Runge-Kutta 4th order numerical integration
- Cascade PID control with 4 layers: Position -> Velocity -> Attitude -> Rate
- A* path planning with obstacle inflation and RDP path simplification
- 4 obstacle detection methods: intensity threshold, HSV color, OpenStreetMap, SAM click-to-segment
- Web GUI (Streamlit) with mouse interaction
- Command Line Interface (CLI) for quick operations
- Automated PID gain optimization (Nelder-Mead)
- Unit testing with 24 test cases

---

### Installation

#### System Requirements
- Python 3.10 or higher
- pip package manager
- (Optional) Git for cloning

#### Quick Install (Windows)

```cmd
setup.bat
```

Select installation type:
1. CLI only
2. GUI (with Streamlit)
3. GUI + SAM (with AI segmentation)

#### Manual Install

```bash
# Clone repository
git clone [https://github.com/yourusername/quadsim.git](https://github.com/yourusername/quadsim.git)
cd quadsim

# Create virtual environment
python -m venv .venv
source .venv/bin/activate      # Linux/macOS
.venv\Scripts\activate         # Windows

# Install dependencies
pip install -r requirements_gui.txt

# Optional: Install SAM for AI obstacle detection
pip install -r requirements_gui_sam.txt
pip install "git+[https://github.com/ChaoningZhang/MobileSAM.git](https://github.com/ChaoningZhang/MobileSAM.git)"
```

---

### Quick Start

#### 1. Web Interface (GUI)

```bash
streamlit run app_gui.py
```

**Workflow:**
1. Upload map image
2. Set scale (m/pixel)
3. Detect obstacles (4 methods available)
4. Place waypoints (manual click or A* auto)
5. Adjust altitude, yaw, epsilon per waypoint
6. Run simulation and view results

#### 2. Command Line Interface (CLI)

```bash
python main.py
```

Menu options:
- [1] Switch drone preset
- [2] View drone parameters
- [3] Open-loop scenario
- [4] Single rotor offset test
- [5] Closed-loop PID control
- [0] Exit

#### 3. PID Auto-Tuning

```bash
python tune_altitude_gain.py
```

Results saved to `outputs/pid_altitude_tuning_real.csv`

#### 4. Run Unit Tests

```bash
pytest tests/ -v
```

Expected: 24/24 tests passed.

---

### Project Structure

```
quadsim/
├── quadsim/                      # Core package
│   ├── __init__.py
│   ├── params.py                 # Drone parameters (Crazyflie 2.0)
│   ├── mixer.py                  # Control allocation
│   ├── dynamics.py               # Newton-Euler 6-DOF + RK4
│   ├── scenarios.py              # Open-loop scenarios
│   ├── controllers.py            # Cascade PID + WaypointManager
│   ├── simulate.py               # Simulation loops
│   ├── plotting.py               # Plot generation
│   ├── cli.py                    # Terminal interface
│   ├── pathfinding.py            # A* + RDP
│   ├── color_obstacles.py        # HSV obstacle detection
│   ├── osm_obstacles.py          # OpenStreetMap query
│   ├── sam_obstacles.py          # MobileSAM segmenter
│   ├── waypoint_io.py            # JSON I/O + calibration
│   ├── waypoint_editor.py        # Standalone waypoint picker
│   └── metrics.py                # Performance metrics
│
├── assets/fonts/                  # Font resources
│   ├── DejaVuSans.ttf
│   └── DejaVuSans-Bold.ttf
│
├── main.py                        # CLI entry
├── app_gui.py                     # GUI entry
├── tune_altitude_gain.py          # PID optimizer
│
├── requirements.txt               # Core dependencies
├── requirements_gui.txt           # GUI dependencies
├── requirements_gui_sam.txt       # SAM dependencies
├── setup.bat                      # Windows installer
│
└── README.md                      # This file
```

---

### Technical Specifications

#### Drone Model: Crazyflie 2.0

| Parameter | Value |
|-----------|-------|
| Mass | 0.027 kg |
| Wheelbase | 0.092 m |
| Ixx, Iyy | 1.66e-5 kg·m² |
| Izz | 2.93e-5 kg·m² |
| kF | 2.359e-8 N/(rad/s)² |
| kM | 1.297e-10 N·m/(rad/s)² |
| omega_max | 2513 rad/s |

#### PID Optimization Result (vel_z channel)

| Gain Set | Overshoot | Settling Time | Steady Error |
|----------|-----------|---------------|--------------|
| Baseline | 1.90% | 4.30s | 0.038m |
| Optimized | 0.00% | 5.47s | 0.025m |

---

### References

1. T. Luukkonen, "Modelling and control of quadcopter," Aalto University, 2011.
2. J. Forster, "System Identification of the Crazyflie 2.0," ETH Zurich, 2015.
3. K. Bouzgou et al., "Dynamic modeling of UAV," INTECH, 2017.
4. L. Carlone, M. Ryll, "Quadrotor Dynamics," MIT 16.485, 2023.
5. D. Mellinger, V. Kumar, "Minimum snap trajectory generation," ICRA, 2011.

---

### License

MIT License

---

### Contact

- **Author:** Tran Anh Khoi

---

---
---

<a name="vietnamese"></a>

## Tiếng Việt

### Giới thiệu

QUADSIM là phần mềm mô phỏng Digital Twin cho máy bay không người lái quadcopter, tích hợp động lực học 6-DOF, điều khiển Cascade PID, lập kế hoạch đường bay A* và nhiều phương án nhận diện vật cản. Phần mềm được xây dựng với kiến trúc module, dễ dàng mở rộng và tích hợp.

**Tính năng chính:**
- Động lực học vật rắn 6-DOF (phương trình Newton-Euler)
- Tích phân Runge-Kutta bậc 4
- Điều khiển Cascade PID 4 tầng: Vị trí -> Vận tốc -> Góc -> Tốc độ góc
- Lập kế hoạch đường bay A* với phình vật cản và rút gọn RDP
- 4 phương án nhận diện vật cản: ngưỡng độ sáng, màu HSV, OpenStreetMap, SAM click-to-segment
- Giao diện web (Streamlit) tương tác bằng chuột
- Giao diện dòng lệnh (CLI) cho thao tác nhanh
- Tối ưu tham số PID tự động (Nelder-Mead)
- Kiểm thử tự động với 24 test case

---

### Cài đặt

#### Yêu cầu hệ thống
- Python 3.10 trở lên
- pip
- (Tùy chọn) Git để sao chép mã nguồn

#### Cài đặt nhanh (Windows)

```cmd
setup.bat
```

Chọn loại cài đặt:
1. CLI (cơ bản)
2. GUI (có Streamlit)
3. GUI + SAM (có AI nhận diện)

#### Cài đặt thủ công

```bash
# Sao chép mã nguồn
git clone [https://github.com/yourusername/quadsim.git](https://github.com/yourusername/quadsim.git)
cd quadsim

# Tạo môi trường ảo
python -m venv .venv
source .venv/bin/activate      # Linux/macOS
.venv\Scripts\activate         # Windows

# Cài đặt thư viện
pip install -r requirements_gui.txt

# Tùy chọn: Cài SAM cho nhận diện vật cản bằng AI
pip install -r requirements_gui_sam.txt
pip install "git+[https://github.com/ChaoningZhang/MobileSAM.git](https://github.com/ChaoningZhang/MobileSAM.git)"
```

---

### Bắt đầu nhanh

#### 1. Giao diện Web (GUI)

```bash
streamlit run app_gui.py
```

**Quy trình:**
1. Tải ảnh bản đồ
2. Hiệu chỉnh tỉ lệ (m/pixel)
3. Nhận diện vật cản (4 phương án)
4. Đặt waypoint (click thủ công hoặc A* tự động)
5. Chỉnh sửa độ cao, yaw, epsilon từng điểm
6. Chạy mô phỏng và xem kết quả

#### 2. Giao diện dòng lệnh (CLI)

```bash
python main.py
```

Menu chính:
- [1] Đổi drone preset
- [2] Xem thông số drone
- [3] Chạy kịch bản vòng hở
- [4] Kiểm tra rotor lệch
- [5] Điều khiển vòng kín PID
- [0] Thoát

#### 3. Tối ưu PID tự động

```bash
python tune_altitude_gain.py
```

Kết quả lưu tại `outputs/pid_altitude_tuning_real.csv`

#### 4. Chạy kiểm thử

```bash
pytest tests/ -v
```

Kết quả: 24/24 test PASSED.

---

### Cấu trúc dự án

```
quadsim/
├── quadsim/                      # Package lõi
│   ├── __init__.py
│   ├── params.py                 # Tham số drone (Crazyflie 2.0)
│   ├── mixer.py                  # Phân bổ điều khiển
│   ├── dynamics.py               # Newton-Euler 6-DOF + RK4
│   ├── scenarios.py              # Kịch bản vòng hở
│   ├── controllers.py            # Cascade PID + WaypointManager
│   ├── simulate.py               # Vòng lặp mô phỏng
│   ├── plotting.py               # Vẽ đồ thị
│   ├── cli.py                    # Giao diện dòng lệnh
│   ├── pathfinding.py            # A* + RDP
│   ├── color_obstacles.py        # Nhận diện vật cản HSV
│   ├── osm_obstacles.py          # Truy vấn OpenStreetMap
│   ├── sam_obstacles.py          # MobileSAM
│   ├── waypoint_io.py            # JSON + hiệu chỉnh tọa độ
│   ├── waypoint_editor.py        # Công cụ chọn waypoint độc lập
│   └── metrics.py                # Chỉ số đánh giá
│
├── assets/fonts/                  # Font chữ
│   ├── DejaVuSans.ttf
│   └── DejaVuSans-Bold.ttf
│
├── main.py                        # Điểm vào CLI
├── app_gui.py                     # Điểm vào GUI
├── tune_altitude_gain.py          # Tối ưu PID
│
├── requirements.txt               # Thư viện cơ bản
├── requirements_gui.txt           # Thư viện GUI
├── requirements_gui_sam.txt       # Thư viện SAM
├── setup.bat                      # Cài đặt tự động (Windows)
│
└── README.md                      # File này
```

---

### Thông số kỹ thuật

#### Drone: Crazyflie 2.0

| Tham số | Giá trị |
|---------|---------|
| Khối lượng | 0.027 kg |
| Đường chéo | 0.092 m |
| Ixx, Iyy | 1.66e-5 kg·m² |
| Izz | 2.93e-5 kg·m² |
| kF | 2.359e-8 N/(rad/s)² |
| kM | 1.297e-10 N·m/(rad/s)² |
| omega_max | 2513 rad/s |

#### Kết quả tối ưu PID (kênh vel_z)

| Bộ gain | Vọt lố | Thời gian xác lập | Sai số xác lập |
|---------|--------|-------------------|----------------|
| Baseline | 1.90% | 4.30s | 0.038m |
| Tối ưu | 0.00% | 5.47s | 0.025m |

---

### Tài liệu tham khảo

1. T. Luukkonen, "Modelling and control of quadcopter," Aalto University, 2011.
2. J. Forster, "System Identification of the Crazyflie 2.0," ETH Zurich, 2015.
3. K. Bouzgou et al., "Dynamic modeling of UAV," INTECH, 2017.
4. L. Carlone, M. Ryll, "Quadrotor Dynamics," MIT 16.485, 2023.
5. D. Mellinger, V. Kumar, "Minimum snap trajectory generation," ICRA, 2011.

---

### Giấy phép

MIT License

---

### Liên hệ

- **Tác giả:** Tran Anh Khoi
