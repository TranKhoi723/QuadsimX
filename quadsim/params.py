"""
quadsim.params
================
NHAN DANG HE THONG - FILE DUY NHAT chua thong so vat ly cua drone. Moi
module khac (mixer, dynamics, controllers, simulate) deu doc tu day qua
class DroneParams. Muon them drone moi: goi DroneParams(...) truc tiep,
hoac them 1 preset moi trong get_preset() o cuoi file.
"""

import numpy as np


class DroneParams:
    """
    Tham so vat ly cua 1 drone quadcopter cau hinh X (quad_x_standard).

    Nhan tham so co ban qua constructor, TU TINH cac gia tri phai sinh:
        d           = L/sqrt(2)              (tay don hieu dung cau hinh X)
        omega_hover = sqrt(mass*g / (4*kF))   (tot do rotor de hover)
        rotor_pos, rotor_spin                  (vi tri + chieu quay 4 rotor)
    """

    def __init__(self, name, mass, diagonal_wheelbase, Ixx, Iyy, Izz,
                 kF, kM, omega_max, omega_min=0.0, drag_lin=None, notes=""):
        self.name = name
        self.mass = mass
        self.g = 9.81

        self.diagonal_wheelbase = diagonal_wheelbase
        self.L = diagonal_wheelbase / 2
        self.d = self.L / np.sqrt(2)          # cau hinh X: rotor lech 45 do so voi truc

        self.Ixx, self.Iyy, self.Izz = Ixx, Iyy, Izz
        self.I = np.diag([Ixx, Iyy, Izz])

        self.kF, self.kM = kF, kM
        self.omega_max, self.omega_min = omega_max, omega_min
        self.omega_hover = np.sqrt(self.mass * self.g / (4 * self.kF))

        self.drag_lin = drag_lin if drag_lin is not None else np.array([0.05, 0.05, 0.08])
        self.notes = notes

        d = self.d
        # Cau hinh X chuan (giong PX4): Rotor 1,2 = CW; Rotor 3,4 = CCW
        self.rotor_pos = np.array([[d, d], [-d, -d], [d, -d], [-d, d]])   # (4,2)
        self.rotor_spin = np.array([-1, -1, 1, 1])

    def thrust_to_weight_ratio(self):
        """Ty le luc day toi da / trong luong - chi so kha thi bay (>=1.5 la an toan)."""
        return 4 * self.kF * self.omega_max ** 2 / (self.mass * self.g)

    def summary(self):
        tw = self.thrust_to_weight_ratio()
        lines = [
            "=" * 62,
            f"THONG SO DRONE: {self.name}",
            "=" * 62,
            f"Khoi luong           : {self.mass} kg",
            f"Diagonal wheelbase    : {self.diagonal_wheelbase} m (L={self.L:.4f} m, d={self.d:.4f} m)",
            f"Quan tinh Ixx,Iyy,Izz : {self.Ixx:.3e}, {self.Iyy:.3e}, {self.Izz:.3e} kg.m^2",
            f"He so luc day kF      : {self.kF:.4e} N/(rad/s)^2",
            f"He so mo-men kM       : {self.kM:.4e} N.m/(rad/s)^2",
            f"Omega max             : {self.omega_max:.1f} rad/s ({self.omega_max*60/(2*np.pi):.0f} RPM)",
            f"Omega hover           : {self.omega_hover:.1f} rad/s "
            f"({self.omega_hover/self.omega_max*100:.1f}% cua omega_max)",
            f"Thrust/Weight         : {tw:.2f} "
            f"({'AN TOAN' if tw >= 1.5 else 'CANH BAO: QUA THAP'})",
        ]
        if self.notes:
            lines.append(f"Ghi chu               : {self.notes}")
        lines.append("=" * 62)
        text = "\n".join(lines)
        print(text)
        return text


def get_preset(name):
    """Tra ve DroneParams theo preset co san: 'crazyflie' hoac 'x_custom'."""
    if name == "crazyflie":
        return DroneParams(
            name="Crazyflie 2.0 (so lieu thuc nghiem cong khai)",
            mass=0.027, diagonal_wheelbase=0.092,
            Ixx=1.66e-5, Iyy=1.66e-5, Izz=2.93e-5,
            kF=2.359e-8, kM=2.359e-8 * 0.0055,
            omega_max=2513.3, omega_min=0.0,
            drag_lin=np.array([0.01, 0.01, 0.02]),
            notes="Bitcraze/ETH Zurich - mau KIEM CHUNG code dung.",
        )
    elif name == "x_custom":
        return DroneParams(
            name="X Custom (UOC LUONG, can Sizing xac nhan)",
            mass=5.0, diagonal_wheelbase=0.35,
            Ixx=0.018, Iyy=0.018, Izz=0.032,
            kF=3.411e-6, kM=3.949e-8,
            omega_max=1570.0, omega_min=0.0,
            drag_lin=np.array([0.08, 0.08, 0.12]),
            notes="UOC LUONG hinh hoc - CANH BAO: Thrust/Weight co the < 1.5.",
        )
    raise ValueError(f"Khong co preset '{name}'. Dung 'crazyflie' hoac 'x_custom'.")


PRESET_NAMES = ["crazyflie", "x_custom"]
