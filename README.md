README.md 
markdown
<div align="center">
  <a href="#english">🇬🇧 English</a> | <a href="#vietnamese">🇻🇳 Tiếng Việt</a>
</div>

---

# <a id="english"></a>🇬🇧 English

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
| **Cascade PID** | 4 layers: Position → Velocity → Attitude → Rate with anti-windup |
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
# Clone repository
git clone https://github.com/yourusername/quadsim.git
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

[3] Open-loop scenario (ascend → roll → pitch → yaw)

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
│   ├── dynamics.py               # Newton-Euler 6-DOF + RK4
│   ├── controllers.py            # Cascade PID + WaypointManager
│   ├── mixer.py                  # Control allocation (omega ↔ wrench)
│   ├── pathfinding.py            # A* + inflate + RDP
│   ├── color_obstacles.py        # HSV obstacle detection
│   ├── osm_obstacles.py          # OpenStreetMap query
│   ├── sam_obstacles.py          # MobileSAM click-to-segment
│   ├── waypoint_io.py            # JSON I/O + coordinate calibration
│   └── ...
├── main.py                        # CLI entry
├── app_gui.py                     # GUI entry
├── tune_altitude_gain.py          # PID optimizer
├── requirements.txt               # Core dependencies
├── requirements_gui.txt           # GUI dependencies
├── requirements_gui_sam.txt       # SAM dependencies
├── setup.bat                      # Windows auto-installer
└── README.md                      # This file
Technical Specifications
Drone Model: Crazyflie 2.0
Parameter	Value
Mass	0.027 kg
Wheelbase	0.092 m
Ixx, Iyy	1.66e-5 kg·m²
Izz	2.93e-5 kg·m²
kF	2.359e-8 N/(rad/s)²
kM	1.297e-10 N·m/(rad/s)²
omega_max	2513 rad/s
Control Structure
Layer	Time Constant	Gain Type
Position	1.5 s	P
Velocity	0.4 s	PID
Attitude	0.12 s	P
Rate	0.03 s	PID
PID Optimization Result (vel_z channel)
Gain Set	Overshoot	Settling Time	Steady Error
Baseline	1.90%	4.30s	0.038m
Optimized	0.00%	5.47s	0.025m
References
T. Luukkonen, "Modelling and control of quadcopter," Aalto University, 2011.

J. Forster, "System Identification of the Crazyflie 2.0," ETH Zurich, 2015.

K. Bouzgou et al., "Dynamic modeling of UAV," INTECH, 2017.

L. Carlone, M. Ryll, "Quadrotor Dynamics," MIT 16.485, 2023.

D. Mellinger, V. Kumar, "Minimum snap trajectory generation," ICRA, 2011.

License
MIT License

Contact
Author: Tran Anh Khoi

Student ID: 2211696

Institution: HCMUT — Faculty of Transportation Engineering

GitHub: github.com/yourusername

<a id="vietnamese"></a>🇻🇳 Tiếng Việt
QUADSIM — Mo phong Quadcopter va Lap ke hoach duong bay
QUADSIM la phan mem mo phong Digital Twin cho quadcopter, tich hop dong luc hoc 6-DOF, dieu khien Cascade PID, lap ke hoach duong bay A* va nhieu phuong an nhan dien vat can. Voi kien truc module ro rang, phan mem phu hop cho ca nguoi moi bat dau va cac nha phat trien chuyen sau ve co hoc bay va he thong dieu khien.

Nguoi dung co the tuong tac voi phan mem qua:

Giao dien Web (GUI) — Streamlit, tuong tac bang chuot

Giao dien dong lenh (CLI) — menu terminal

Thu vien Python — import va mo rong

Tinh nang
Tinh nang	Mo ta
Dong luc hoc 6-DOF	Phuong trinh Newton-Euler, tich phan RK4, bieu dien goc Euler
Cascade PID	4 tang: Vi tri → Van toc → Goc → Toc do goc, co anti-windup
Lap ke hoach duong bay	Thuat toan A* voi phinh vat can va rut gon RDP
Nhan dien vat can	4 phuong an: nguong toi, mau HSV, OpenStreetMap, SAM click-to-segment
Quan ly waypoint	Do cao, goc yaw va epsilon (ban kinh chap nhan) rieng tung diem
Giao dien GUI	Streamlit web, tuong tac bang chuot
Kiem thu tu dong	24 test case kiem tra bat bien vat ly
Toi uu tu dong	Nelder-Mead toi uu tham so PID
Cai dat
Yeu cau he thong
Python 3.10 tro len

pip

(Tuy chon) Git de clone

Cai dat nhanh (Windows)
cmd
setup.bat
Chon loai cai dat:

CLI (co ban)

GUI (co Streamlit)

GUI + SAM (co AI nhan dien vat can)

Cai dat thu cong
bash
# Clone repository
git clone https://github.com/yourusername/quadsim.git
cd quadsim

# Tao moi truong ao
python -m venv .venv
source .venv/bin/activate      # Linux/macOS
.venv\Scripts\activate          # Windows

# Cai dat thu vien
pip install -r requirements_gui.txt

# Tuy chon: Cai SAM cho nhan dien vat can bang AI
pip install -r requirements_gui_sam.txt
pip install "git+https://github.com/ChaoningZhang/MobileSAM.git"
Bat dau nhanh
1. Giao dien Web (GUI)
bash
streamlit run app_gui.py
Quy trinh:

Tai anh ban do

Hieu chinh ti le (m/pixel)

Nhan dien vat can (4 phuong an)

Dat waypoint (click thu cong hoac A* tu dong)

Chinh sua do cao, yaw, epsilon tung diem

Chay mo phong va xem ket qua

2. Giao dien dong lenh (CLI)
bash
python main.py
Menu chinh:

[1] Doi drone preset

[2] Xem thong so drone

[3] Chay kich ban vong ho

[4] Kiem tra rotor lech

[5] Dieu khien vong kin PID

[0] Thoat

3. Toi uu PID tu dong
bash
python tune_altitude_gain.py
Ket qua luu tai outputs/pid_altitude_tuning_real.csv

4. Chay kiem thu
bash
pytest tests/ -v
Ket qua: 24/24 test PASSED.

Cau truc du an
text
quadsim/
├── quadsim/                      # Package loi
│   ├── dynamics.py               # Newton-Euler 6-DOF + RK4
│   ├── controllers.py            # Cascade PID + WaypointManager
│   ├── mixer.py                  # Phan bo dieu khien
│   ├── pathfinding.py            # A* + inflate + RDP
│   ├── color_obstacles.py        # Nhan dien vat can HSV
│   ├── osm_obstacles.py          # Truy van OpenStreetMap
│   ├── sam_obstacles.py          # SAM click-to-segment
│   ├── waypoint_io.py            # JSON + hieu chinh toa do
│   └── ...
├── main.py                        # Diem vao CLI
├── app_gui.py                     # Diem vao GUI
├── tune_altitude_gain.py          # Toi uu PID
├── requirements.txt               # Thu vien co ban
├── requirements_gui.txt           # Thu vien GUI
├── requirements_gui_sam.txt       # Thu vien SAM
├── setup.bat                      # Cai dat tu dong (Windows)
└── README.md                      # File nay
Thong so ky thuat
Drone: Crazyflie 2.0
Tham so	Gia tri
Khoi luong	0.027 kg
Duong cheo	0.092 m
Ixx, Iyy	1.66e-5 kg·m²
Izz	2.93e-5 kg·m²
kF	2.359e-8 N/(rad/s)²
kM	1.297e-10 N·m/(rad/s)²
omega_max	2513 rad/s
Cau truc dieu khien
Tang	Hang so thoi gian	Loai
Vi tri	1.5 s	P
Van toc	0.4 s	PID
Goc	0.12 s	P
Toc do goc	0.03 s	PID
Ket qua toi uu PID (kenh vel_z)
Bo gain	Vot lo	Thoi gian xac lap	Sai so xac lap
Baseline	1.90%	4.30s	0.038m
Toi uu	0.00%	5.47s	0.025m
Tai lieu tham khao
T. Luukkonen, "Modelling and control of quadcopter," Aalto University, 2011.

J. Forster, "System Identification of the Crazyflie 2.0," ETH Zurich, 2015.

K. Bouzgou et al., "Dynamic modeling of UAV," INTECH, 2017.

L. Carlone, M. Ryll, "Quadrotor Dynamics," MIT 16.485, 2023.

D. Mellinger, V. Kumar, "Minimum snap trajectory generation," ICRA, 2011.

Giay phep
MIT License

Lien he
Tac gia: Tran Anh Khoi

MSSV: 2211696

Truong: DHBK TPHCM — Khoa Ky thuat Giao thong

GitHub: github.com/yourusername
