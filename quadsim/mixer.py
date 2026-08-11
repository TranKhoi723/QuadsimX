"""
quadsim.mixer
===============
CONTROL ALLOCATION - noi 4 toc do rotor [w1,w2,w3,w4] voi wrench tong hop
[F, tau_x, tau_y, tau_z], qua ma tran Mixer A (4x4):

    F_i = kF * w_i^2         (luc day tung rotor)
    M_i = kM * w_i^2         (mo-men phan luc tung rotor, dau theo chieu quay)

    [F, tau_x, tau_y, tau_z]^T = A @ [w1^2, w2^2, w3^2, w4^2]^T
"""

import numpy as np


def build_mixer_matrix(params):
    """Xay ma tran Mixer A (4x4) tu vi tri + chieu quay 4 rotor cua params."""
    kF, kM = params.kF, params.kM
    A = np.zeros((4, 4))
    for col in range(4):
        xi, yi = params.rotor_pos[col]
        spin_i = params.rotor_spin[col]
        A[0, col] = kF                  # dong gop vao F
        A[1, col] = kF * yi             # dong gop vao tau_x (roll)
        A[2, col] = -kF * xi            # dong gop vao tau_y (pitch)
        A[3, col] = spin_i * kM         # dong gop vao tau_z (yaw)
    return A


def omega_to_wrench(omega, params):
    """Chieu THUAN: omega thuc te (rad/s, 4 phan tu) -> wrench [F,tau_x,tau_y,tau_z]."""
    A = build_mixer_matrix(params)
    return A @ (np.asarray(omega, dtype=float) ** 2)


def wrench_to_omega(wrench, params):
    """Chieu NGUOC: wrench mong muon -> omega can thiet (CHUA bao hoa phan cung)."""
    A = build_mixer_matrix(params)
    omega_sq = np.clip(np.linalg.solve(A, np.asarray(wrench, dtype=float)), 0, None)
    return np.sqrt(omega_sq)


def wrench_to_motor_command(wrench, params):
    """Nhu wrench_to_omega, nhung da GIOI HAN trong [omega_min, omega_max] that."""
    omega = wrench_to_omega(wrench, params)
    return np.clip(omega, params.omega_min, params.omega_max)


def mixer_signs(params):
    """
    Dau +/- ma tung rotor dong gop vao moi truc, suy TRUC TIEP tu ma tran
    Mixer A - dung de xay cac kich ban mo phong (vd 1 xung roll thuan tuy)
    ma KHONG can doan tay rotor nao tang/giam, dung cho MOI cau hinh X.

    Tra ve: dict {'roll':.., 'pitch':.., 'yaw':..} moi gia tri la mang (4,) chi +-1.
    """
    A = build_mixer_matrix(params)
    return {
        "roll": np.sign(A[1, :]),
        "pitch": np.sign(A[2, :]),
        "yaw": np.sign(A[3, :]),
    }
