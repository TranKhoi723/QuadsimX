"""
quadsim
=========
Phan mem mo phong Quadcopter (Digital Twin + PID) - kien truc module hoa:

    quadsim/
        params.py       - Nhan dang he thong (DroneParams, get_preset)
        mixer.py        - Control allocation (Mixer matrix)
        dynamics.py     - Newton-Euler 6-DOF + RK4
        scenarios.py    - Cac kich ban omega(t) mau (vong ho)
        controllers.py  - Cascade PID (vong kin)
        simulate.py     - Vong lap mo phong (open-loop + closed-loop)
        plotting.py     - Ve va luu do thi
        cli.py          - Giao dien dong lenh (terminal menu)

Diem vao chay: `python main.py` (o thu muc goc).
"""

__version__ = "1.0.0"
