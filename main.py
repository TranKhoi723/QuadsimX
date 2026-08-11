#!/usr/bin/env python3
"""
main.py
=========
DIEM VAO CHAY PHAN MEM QUADSIM (giao dien terminal).

Chay:
    python main.py

Toan bo logic tinh toan nam trong package quadsim/ (params, mixer, dynamics,
scenarios, controllers, simulate, plotting) - file nay chi khoi dong menu.
"""

import sys

# --- An toan Unicode tren Windows ---
# Console Windows (cmd.exe / PowerShell cu) mac dinh dung codepage KHONG PHAI
# UTF-8 (vd cp1252, cp437) - cac chu co dau tieng Viet (ạ, ọ, đ, ...) trong
# menu/thong bao se lam crash UnicodeEncodeError ngay khi print() chay. Doan
# duoi day CO GANG chuyen stdout/stderr sang UTF-8; neu khong duoc (Python cu
# hon 3.7, hoac console dac biet), bo qua trong im lang - phan mem van chay,
# chi co the hien thi loi ky tu thay vi crash.
if sys.stdout.encoding and sys.stdout.encoding.lower() != "utf-8":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

from quadsim.cli import run_app

if __name__ == "__main__":
    run_app(initial_preset="crazyflie")
