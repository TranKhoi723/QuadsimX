#!/usr/bin/env python3
"""
tune_altitude_gain.py
========================
TOI UU HOA THAT (khac voi bang minh hoa trong cli.py::action_compare_and_export_gains)
cho 1 kenh PID cu the: kenh VAN TOC-Z (vel_z), la kenh quyet dinh toc do +
do vot cua dap ung do cao. Dung scipy.optimize.minimize (Nelder-Mead) de
tim (Kp, Ki, Kd) toi thieu hoa 1 ham chi phi thuc te tren dap ung step
(nhay do cao 0m -> 2m), thay vi so tay chon so nhu ban minh hoa cu.

Ham chi phi = ISE (tich phan binh phuong sai so) + phat 20% neu vuot qua
(overshoot) + phat nho neu chua on dinh luc ket thuc mo phong - la 3 tieu
chi CHUAN trong dieu chinh PID (tra loi truc tiep cho phan "phuong phap
danh gia" trong bao cao).

Chay:
    python tune_altitude_gain.py

Ket qua IN RA man hinh + luu CSV that (khong phai so minh hoa) vao
outputs/pid_altitude_tuning_real.csv de dua vao bao cao.
"""

import os
import csv
import numpy as np
from scipy.optimize import minimize

from quadsim.params import get_preset
from quadsim.controllers import CascadePIDController, PIDChannel, default_gains
from quadsim.simulate import simulate_closed_loop


ALT_STEP_M = 2.0     # nhay do cao tu 0 -> 2m
T_FINAL = 6.0
DT = 0.01


def run_altitude_step(params, gains_vel_z):
    """Chay 1 lan mo phong vong kin CHI xoay quanh truc Z (giu x=y=0,yaw=0),
    tra ve mang do cao(t), van toc-z(t) va co bao hoa dong co hay khong."""
    gains = default_gains(params)
    Kp, Ki, Kd = gains_vel_z
    gains["vel_z"] = PIDChannel(Kp=Kp, Ki=Ki, Kd=Kd, integral_limit=3.0)

    ctrl = CascadePIDController(params, gains=gains)
    setpoint = {"pos": [0.0, 0.0, -ALT_STEP_M], "yaw": 0.0}
    state0 = np.zeros(12)

    t, state, omega, saturated = simulate_closed_loop(
        ctrl, params, setpoint, state0=state0, dt=DT, t_final=T_FINAL)
    altitude = -state[:, 2]
    return t, altitude, saturated


# Gioi han bien TREN cho Kp,Ki,Kd - dong vai tro "gioi han vat ly hop ly"
# (Kp qua lon se khien dong co bao hoa/dieu khien giat cuc trong thuc te,
# du cost ISE thuan tuy van co the "thich" vi dap ung nhanh hon). Neu khong
# co bien nay, Nelder-Mead se troi den Kp cang lon cang tot tren mo hinh ly
# tuong hoa (khong mo hinh do tre dong co/nhieu do), la loi kinh dien khi
# tu dong toi uu PID chi tren 1 tieu chi ISE don thuan.
GAIN_UPPER_BOUND = (15.0, 3.0, 3.0)


def cost_function(gains_vel_z, params):
    Kp, Ki, Kd = gains_vel_z
    if Kp <= 0 or Ki < 0 or Kd < 0:
        return 1e6
    if Kp > GAIN_UPPER_BOUND[0] or Ki > GAIN_UPPER_BOUND[1] or Kd > GAIN_UPPER_BOUND[2]:
        return 1e6  # ngoai vung gain "hop ly ve mat vat ly" - loai ngay

    t, altitude, saturated = run_altitude_step(params, gains_vel_z)
    error = ALT_STEP_M - altitude

    ise = np.trapezoid(error ** 2, t) if hasattr(np, "trapezoid") else np.trapz(error ** 2, t)
    overshoot = max(0.0, altitude.max() - ALT_STEP_M)
    overshoot_penalty = 5.0 * overshoot ** 2

    final_error = abs(error[-1])
    settle_penalty = 10.0 * final_error ** 2

    # Phat bao hoa dong co: 1 bo gain khien dong co bao hoa nhieu buoc la
    # DAU HIEU thieu thuc te (mo hinh ly tuong "cho phep" nhung dong co that
    # khong dap ung noi) - CAN co trong ham cost, khong chi ISE don thuan.
    saturation_penalty = 2.0 * float(np.mean(saturated))

    return float(ise + overshoot_penalty + settle_penalty + saturation_penalty)


def step_metrics(t, altitude, target=ALT_STEP_M):
    """Cac chi so dinh luong CHUAN de bao cao: overshoot %, thoi gian xac
    lap (settling time, sai so con lai <2%), sai so xac lap (steady-state)."""
    overshoot_pct = max(0.0, (altitude.max() - target) / target * 100.0)
    band = 0.02 * target
    within_band = np.abs(altitude - target) <= band
    settle_idx = len(t) - 1
    for i in range(len(t) - 1, -1, -1):
        if not within_band[i]:
            settle_idx = min(i + 1, len(t) - 1)
            break
    else:
        settle_idx = 0
    settling_time_s = t[settle_idx]
    steady_state_error = abs(target - altitude[-1])
    return overshoot_pct, settling_time_s, steady_state_error


def main():
    params = get_preset("crazyflie")
    baseline_gains = default_gains(params)
    baseline = (baseline_gains["vel_z"].Kp, baseline_gains["vel_z"].Ki, baseline_gains["vel_z"].Kd)

    print("=" * 70)
    print("TOI UU HOA THAT (Nelder-Mead, scipy.optimize) - kenh do cao (vel_z)")
    print("=" * 70)
    print(f"Gain khoi tao (tu default_gains()): Kp={baseline[0]:.4f}, "
          f"Ki={baseline[1]:.4f}, Kd={baseline[2]:.4f}")

    t0, alt0, sat0 = run_altitude_step(params, baseline)
    os0, st0, sse0 = step_metrics(t0, alt0)
    cost0 = cost_function(baseline, params)
    print(f"  -> Overshoot={os0:.2f}%  Settling time={st0:.2f}s  "
          f"Sai so xac lap={sse0:.4f}m  Bao hoa={sat0.mean()*100:.1f}%  Cost={cost0:.4f}")

    print("\nDang chay Nelder-Mead (co the mat vai chuc giay)...")
    result = minimize(
        cost_function, x0=np.array(baseline), args=(params,),
        method="Nelder-Mead",
        options={"xatol": 1e-4, "fatol": 1e-4, "maxiter": 200, "adaptive": True},
    )
    tuned = tuple(result.x)
    print(f"\nKet qua toi uu: Kp={tuned[0]:.4f}, Ki={tuned[1]:.4f}, Kd={tuned[2]:.4f}")
    print(f"  So lan danh gia ham cost: {result.nfev}, hoi tu: {result.success}")

    t1, alt1, sat1 = run_altitude_step(params, tuned)
    os1, st1, sse1 = step_metrics(t1, alt1)
    cost1 = cost_function(tuned, params)
    print(f"  -> Overshoot={os1:.2f}%  Settling time={st1:.2f}s  "
          f"Sai so xac lap={sse1:.4f}m  Bao hoa={sat1.mean()*100:.1f}%  Cost={cost1:.4f}")

    improve_cost = (1 - cost1 / cost0) * 100 if cost0 > 0 else 0.0
    print(f"\nCai thien cost function: {improve_cost:+.1f}%")

    os.makedirs("outputs", exist_ok=True)
    csv_path = os.path.join("outputs", "pid_altitude_tuning_real.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["Bo_gain", "Kp", "Ki", "Kd", "Overshoot_%", "Settling_time_s",
                    "Sai_so_xac_lap_m", "Cost_function", "So_lan_danh_gia_cost"])
        w.writerow(["Baseline (default_gains)", *[f"{x:.6f}" for x in baseline],
                    f"{os0:.4f}", f"{st0:.4f}", f"{sse0:.6f}", f"{cost0:.6f}", "-"])
        w.writerow(["Toi_uu_Nelder-Mead", *[f"{x:.6f}" for x in tuned],
                    f"{os1:.4f}", f"{st1:.4f}", f"{sse1:.6f}", f"{cost1:.6f}", result.nfev])
    print(f"\nDa luu ket qua THAT (khong phai so minh hoa) vao: {csv_path}")

    # Ve do thi so sanh dap ung step truoc/sau toi uu - dung THANG cho bao cao
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot(t0, alt0, label=f"Trước tối ưu (Kp={baseline[0]:.2f})", linewidth=1.6)
    ax.plot(t1, alt1, label=f"Sau tối ưu (Kp={tuned[0]:.2f})", linewidth=1.6)
    ax.axhline(ALT_STEP_M, color="gray", linestyle=":", label="Setpoint (2m)")
    ax.set_xlabel("Thời gian (s)")
    ax.set_ylabel("Độ cao (m)")
    ax.set_title("So sánh đáp ứng bậc thang độ cao — trước / sau tối ưu Nelder-Mead")
    ax.legend()
    ax.grid(alpha=0.3)
    fig_path = os.path.join("outputs", "pid_altitude_tuning_real.png")
    fig.tight_layout()
    fig.savefig(fig_path, dpi=130)
    print(f"Da luu do thi vao: {fig_path}")


if __name__ == "__main__":
    main()
