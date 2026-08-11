"""
quadsim.scenarios
====================
Cac ham xay dung omega(t) cho mo phong VONG HO (open-loop, khong co bo dieu
khien - drone lam DUNG NHUNG GI omega(t) yeu cau).

Ham `smooth_pulse()` duoi day dung **1 CHU KY SIN DAY DU** (tang toc nua dau,
TU HAM LAI o nua sau) - dam bao van toc goc ~0 tai cuoi moi doan, giong dung
nguyen tac "bang-bang" (tang toc roi giam toc) cua kich ban bac thang ban dau,
nhung van LIEN TUC VA MUOT (dao ham lien tuc, khong buoc nhay).
"""

import numpy as np
from .mixer import mixer_signs


def smooth_pulse(t, t0, T, amplitude):
    """
    Xung 1 CHU KY SIN DAY DU trong [t0, t0+T]: 0 -> +dinh -> 0 -> -dinh -> 0.
    Nua dau tang toc, nua sau TU HAM LAI - dam bao ket thuc doan gan nhu
    dung yen ve mat toc do goc (khac han "half-sine" chi co 1 chieu).
    Ngoai khoang [t0, t0+T] tra ve 0 (khong dong gop).
    """
    if t < t0 or t > t0 + T:
        return 0.0
    return amplitude * np.sin(2 * np.pi * (t - t0) / T)


def hover_scenario(params):
    """Kich ban baseline: ca 4 rotor giu nguyen omega_hover mai mai."""
    w_h = params.omega_hover

    def omega_cmd(t):
        return [w_h] * 4

    return omega_cmd


def luukkonen_scenario(params, t_ascend=0.5, t_roll=0.5, t_pitch=0.5, t_yaw=0.5,
                        amp_thrust=0.05, amp_roll=0.03, amp_pitch=0.03, amp_yaw=0.05):
    """
    Kich ban 4 giai doan tuan tu (leo cao -> roll -> pitch -> yaw), moi giai
    doan la 1 xung smooth_pulse() (mot chu ky sin day du, tu hoan tat tang
    toc + giam toc). Tai tao dung cau truc Figure 2-3-4 cua Luukkonen (2011)
    nhung dung DUNG tham so drone dang xet (khong copy so lieu bai bao).

    Tham so:
        params      : DroneParams (vd get_preset("crazyflie"))
        t_*         : [s] thoi luong tung giai doan (mac dinh 0.5s/giai doan)
        amp_*       : bien do %omega_hover cho tung giai doan (CHINH TAY de
                      ra goc nghieng mong muon - drone nhe hon can amp nho hon)

    Tra ve: (omega_cmd, t_total)
    """
    w_h = params.omega_hover
    signs = mixer_signs(params)

    t0_ascend = 0.0
    t0_roll = t0_ascend + t_ascend
    t0_pitch = t0_roll + t_roll
    t0_yaw = t0_pitch + t_pitch
    t_total = t0_yaw + t_yaw

    def omega_cmd(t):
        thrust_pulse = smooth_pulse(t, t0_ascend, t_ascend, amp_thrust * w_h)
        roll_pulse = smooth_pulse(t, t0_roll, t_roll, amp_roll * w_h)
        pitch_pulse = smooth_pulse(t, t0_pitch, t_pitch, amp_pitch * w_h)
        yaw_pulse = smooth_pulse(t, t0_yaw, t_yaw, amp_yaw * w_h)
        return (w_h
                + thrust_pulse
                + signs["roll"] * roll_pulse
                + signs["pitch"] * pitch_pulse
                + signs["yaw"] * yaw_pulse)

    return omega_cmd, t_total


def single_rotor_offset_scenario(params, rotor_index, delta_frac, t_start=0.0, t_duration=None):
    """
    Chi 1 rotor lech omega_hover 1 luong co dinh (vd mo phong loi dong co),
    3 rotor con lai giu nguyen. delta_frac la ty le %omega_hover (vd 0.05).
    """
    w_h = params.omega_hover

    def omega_cmd(t):
        w = [w_h] * 4
        if t_start <= t and (t_duration is None or t < t_start + t_duration):
            w[rotor_index] = w_h * (1 + delta_frac)
        return w

    return omega_cmd
