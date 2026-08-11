"""
quadsim.cli
=============
GIAO DIEN DONG LENH (TERMINAL MENU) - lop "trinh bay" mong, chi goi lai cac
ham da co san trong params/mixer/dynamics/scenarios/controllers/simulate/
plotting. Neu sau nay muon nang cap len GUI (vd Streamlit, PyQt, web), chi
can viet lop trinh bay MOI goi lai DUNG cac ham nay - khong dong den logic
tinh toan ben trong.
"""

import numpy as np
import csv
import os

from .params import get_preset, PRESET_NAMES
from .scenarios import luukkonen_scenario, single_rotor_offset_scenario, hover_scenario
from .controllers import CascadePIDController, WaypointManager, default_gains
from .simulate import simulate, simulate_closed_loop, simulate_waypoints
from .plotting import plot_all
from .waypoint_io import load_waypoints_json

def _ask_float(prompt, default):
    raw = input(f"{prompt} [{default}]: ").strip()
    if raw == "":
        return default
    try:
        return float(raw)
    except ValueError:
        print("  (khong doc duoc so, dung gia tri mac dinh)")
        return default


def _ask_int(prompt, default, choices=None):
    raw = input(f"{prompt} [{default}]: ").strip()
    if raw == "":
        return default
    try:
        val = int(raw)
        if choices is not None and val not in choices:
            print("  (lua chon khong hop le, dung gia tri mac dinh)")
            return default
        return val
    except ValueError:
        print("  (khong doc duoc so, dung gia tri mac dinh)")
        return default


def _choose_preset(current_name):
    print("\nCac preset co san:")
    for i, name in enumerate(PRESET_NAMES, start=1):
        marker = " (dang dung)" if name == current_name else ""
        print(f"  [{i}] {name}{marker}")
    idx = _ask_int("Chon preset (so thu tu)", PRESET_NAMES.index(current_name) + 1,
                   choices=list(range(1, len(PRESET_NAMES) + 1)))
    return PRESET_NAMES[idx - 1]


def _print_state_summary(label, t, state, saturated=None):
    final = state[-1]
    print(f"\n--- {label} ---")
    print(f"Thoi diem cuoi t={t[-1]:.3f}s:")
    print(f"  Vi tri  : x={final[0]:.4f} m, y={final[1]:.4f} m, do cao={-final[2]:.4f} m")
    print(f"  Goc     : roll={np.rad2deg(final[6]):.2f} deg, "
          f"pitch={np.rad2deg(final[7]):.2f} deg, yaw={np.rad2deg(final[8]):.2f} deg")
    print(f"  Dinh goc: roll_max={np.rad2deg(state[:,6]).max():.2f} deg, "
          f"pitch_max={np.rad2deg(state[:,7]).max():.2f} deg, "
          f"yaw_max={np.rad2deg(state[:,8]).max():.2f} deg")
    if saturated is not None:
        print(f"  Bao hoa dong co: {saturated.sum()}/{len(saturated)} buoc")


def action_summary(params):
    params.summary()


def action_open_loop_luukkonen(params):
    print("\n=== Kich ban VONG HO: leo cao -> roll -> pitch -> yaw ===")
    t_stage = _ask_float("Thoi luong moi giai doan (s)", 0.5)
    dt = _ask_float("Buoc tich phan dt (s)", 0.0005)
    amp_thrust = _ask_float("Bien do leo cao (% omega_hover, vd 0.05)", 0.05)
    amp_roll = _ask_float("Bien do roll (% omega_hover, vd 0.03)", 0.03)
    amp_pitch = _ask_float("Bien do pitch (% omega_hover, vd 0.03)", 0.03)
    amp_yaw = _ask_float("Bien do yaw (% omega_hover, vd 0.05)", 0.05)

    omega_cmd, t_total = luukkonen_scenario(
        params, t_ascend=t_stage, t_roll=t_stage, t_pitch=t_stage, t_yaw=t_stage,
        amp_thrust=amp_thrust, amp_roll=amp_roll, amp_pitch=amp_pitch, amp_yaw=amp_yaw,
    )
    t, state, omega = simulate(omega_cmd, params, t_final=t_total, dt=dt)
    _print_state_summary(f"Vong ho - {params.name}", t, state)

    print("\nDang ve va luu do thi...")
    plot_all(t, state, omega, params, output_dir="outputs", prefix="openloop_",
              title=f"Vòng hở (Luukkonen) — {params.name}")


def action_single_rotor(params):
    print("\n=== Kich ban: 1 rotor lech khoi Trim ===")
    rotor_1based = _ask_int("Rotor nao lech (1-4)", 1, choices=[1, 2, 3, 4])
    delta = _ask_float("Do lech (% omega_hover, vd 0.05 = +5%)", 0.05)
    t_final = _ask_float("Thoi gian mo phong (s)", 3.0)
    dt = _ask_float("Buoc tich phan dt (s)", 0.001)

    omega_cmd = single_rotor_offset_scenario(params, rotor_1based - 1, delta)
    t, state, omega = simulate(omega_cmd, params, t_final=t_final, dt=dt)
    _print_state_summary(f"Rotor {rotor_1based} lech {delta*100:.1f}% - {params.name}", t, state)

    print("\nDang ve va luu do thi...")
    plot_all(t, state, omega, params, output_dir="outputs", prefix="rotor_offset_",
              title=f"Rotor {rotor_1based} lech {delta*100:.1f}% — {params.name}")


def action_closed_loop_pid(params):
    print("\n=== Dieu khien VONG KIN (Cascade PID) ===")
    print("Nhap toa do World (m). Do cao la SO DUONG (vd 5 = bay len 5m).")
    x_sp = _ask_float("x setpoint (m)", 0.0)
    y_sp = _ask_float("y setpoint (m)", 0.0)
    alt_sp = _ask_float("Do cao setpoint (m)", 5.0)
    yaw_sp_deg = _ask_float("Yaw setpoint (deg)", 0.0)

    print("\nVi tri BAT DAU (de trong = 0,0,0):")
    x0 = _ask_float("x ban dau (m)", 0.0)
    y0 = _ask_float("y ban dau (m)", 0.0)
    alt0 = _ask_float("Do cao ban dau (m)", 0.0)
    yaw0_deg = _ask_float("Yaw ban dau (deg)", 0.0)

    t_final = _ask_float("Thoi gian mo phong (s)", 8.0)
    dt = _ask_float("Buoc tich phan dt (s)", 0.005)

    setpoint = {"pos": [x_sp, y_sp, -alt_sp], "yaw": np.deg2rad(yaw_sp_deg)}
    state0 = np.zeros(12)
    state0[0], state0[1], state0[2] = x0, y0, -alt0
    state0[8] = np.deg2rad(yaw0_deg)

    ctrl = CascadePIDController(params)
    t, state, omega, saturated = simulate_closed_loop(
        ctrl, params, setpoint, state0=state0, dt=dt, t_final=t_final)
    _print_state_summary(f"Vong kin PID - {params.name}", t, state, saturated)

    tw = params.thrust_to_weight_ratio()
    if saturated.sum() > 0.3 * len(saturated):
        print(f"\nCANH BAO: bao hoa dong co qua 30% thoi gian mo phong "
              f"(Thrust/Weight = {tw:.2f}). Neu T/W < 1.5, day co the la GIOI HAN")
        print("VAT LY THAT (khong du luc day), khong phai loi bo dieu khien.")

    print("\nDang ve va luu do thi...")
    plot_all(t, state, omega, params, output_dir="outputs", prefix="pid_",
              setpoint=setpoint["pos"], title=f"Closed-loop PID — {params.name}")


MENU_TEXT = """
======================================================
   QUADSIM - Phan mem mo phong Quadcopter (Terminal)
======================================================
Drone hien tai: {drone_name}

  [1] Doi drone (preset)
  [2] Xem thong so drone (summary)
  [3] Chay kich ban VONG HO (leo cao -> roll -> pitch -> yaw)
  [4] Chay kich ban: 1 rotor lech khoi Trim
  [5] Chay dieu khien VONG KIN (Cascade PID - 1 diem)
  [6] Bay theo Waypoint (mau / file JSON / nhap tay - t_final tu uoc luong)
  [7] Xuat bang so sanh Ht so PID (ASCII & CSV)
  [0] Thoat
"""


def run_app(initial_preset="crazyflie"):
    """Diem vao chinh - vong lap menu terminal."""
    preset_name = initial_preset
    params = get_preset(preset_name)

    print("Chao mung den voi QUADSIM.")
    while True:
        print(MENU_TEXT.format(drone_name=params.name))
        choice = input("Chon chuc nang: ").strip()

        try:
            if choice == "1":
                preset_name = _choose_preset(preset_name)
                params = get_preset(preset_name)
                print(f"-> Da doi sang preset: {preset_name}")
            elif choice == "2":
                action_summary(params)
            elif choice == "3":
                action_open_loop_luukkonen(params)
            elif choice == "4":
                action_single_rotor(params)
            elif choice == "5":
                action_closed_loop_pid(params)
            elif choice == "6":
                action_waypoint_manager(params)
            elif choice == "7":
                action_compare_and_export_gains(params)
            elif choice == "0":
                print("Tam biet!")
                break
            else:
                print("Lua chon khong hop le, thu lai.")
        except Exception as exc:
            # Khong de 1 loi nhap lieu/tinh toan lam sap ca chuong trinh -
            # in loi va quay lai menu, nguoi dung thu lai duoc ngay.
            print(f"\n[LOI] {type(exc).__name__}: {exc}")
            print("Quay lai menu chinh...")

def action_compare_and_export_gains(params):
    """In bảng ASCII so sánh bộ Gain hiện tại vs 1 bộ ví dụ khác, xuất CSV."""
    print("\n=== BẢNG SO SÁNH HỆ SỐ PID (HIỆN CÓ vs BỘ VÍ DỤ) ===")
    current_gains = default_gains(params)
    
    # CANH BAO TRUNG THUC: cac gia tri duoi day la VI DU MINH HOA (chon tay
    # de minh hoa dinh dang bang), KHONG phai ket qua tu 1 lan chay Nelder-Mead
    # thuc te nao - CHUA co ai chay thuat toan toi uu hoa nao tren gain nay.
    # Neu dua bang nay vao bao cao, PHAI noi ro day la gia tri MAU/gia dinh,
    # hoac thay bang ket qua tu 1 phep toi uu THAT ban da tu chay va kiem chung.
    example_scalars = {
        "pos_x": {"Kp": 0.85}, "pos_y": {"Kp": 0.85}, "pos_z": {"Kp": 0.90},
        "vel_x": {"Kp": 2.10, "Ki": 0.35, "Kd": 0.12},
        "vel_y": {"Kp": 2.10, "Ki": 0.35, "Kd": 0.12},
        "vel_z": {"Kp": 2.80, "Ki": 0.55, "Kd": 0.12},
        "att_roll": {"Kp": 9.20}, "att_pitch": {"Kp": 9.20}, "att_yaw": {"Kp": 8.50},
        "rate_roll": {"Kp": 0.00062, "Ki": 0.00003, "Kd": 0.000006},
        "rate_pitch": {"Kp": 0.00062, "Ki": 0.00003, "Kd": 0.000006},
        "rate_yaw": {"Kp": 0.00110, "Ki": 0.00005, "Kd": 0.000010},
    }

    # Format bảng ASCII
    header = f"{'KÊNH (CHANNEL)':<15} | {'THAM SỐ':<8} | {'HIỆN CÓ (DEFAULT)':<18} | {'VÍ DỤ KHÁC':<18}"
    print("-" * 66)
    print(header)
    print("-" * 66)

    csv_rows = [["Channel", "Parameter", "Default_Value", "Example_Alt_Value"]]

    for ch_name, ch_obj in current_gains.items():
        if not hasattr(ch_obj, "Kp"):
            continue
        for param_name in ["Kp", "Ki", "Kd"]:
            def_val = getattr(ch_obj, param_name, 0.0)
            opt_val = example_scalars.get(ch_name, {}).get(param_name, def_val)
            
            print(f"{ch_name:<15} | {param_name:<8} | {def_val:<18.6f} | {opt_val:<18.6f}")
            csv_rows.append([ch_name, param_name, def_val, opt_val])
    print("-" * 66)

    # Tự động xuất CSV vào outputs/
    os.makedirs("outputs", exist_ok=True)
    csv_path = os.path.join("outputs", f"pid_gains_comparison_{params.name[:10].strip()}.csv")
    with open(csv_path, mode="w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerows(csv_rows)
    print(f"  -> Đã xuất bảng số liệu báo cáo ra file: {csv_path}")

DEMO_WAYPOINTS = [
    {"pos": [0.0, 0.0, -2.0], "yaw": 0.0},              # Diem 1: Cat canh len 2m
    {"pos": [2.0, 2.0, -2.0], "yaw": np.deg2rad(45)},   # Diem 2: Bay cheo toi (2,2), do cao 2m
    {"pos": [2.0, 0.0, -1.0], "yaw": 0.0},              # Diem 3: Ha xuong do cao 1m tai (2,0)
]


def _path_length(waypoints, state0_pos):
    """Tong quang duong noi tiep qua tat ca waypoint, tinh tu vi tri bat dau."""
    pts = [np.asarray(state0_pos, dtype=float)] + [np.asarray(wp["pos"], dtype=float) for wp in waypoints]
    return sum(np.linalg.norm(pts[i + 1] - pts[i]) for i in range(len(pts) - 1))


def _estimate_t_final(waypoints, state0_pos, cruise_speed_mps=1.0, settle_s=3.0):
    """
    Uoc luong nhanh t_final can thiet, THAY VI bat nguoi dung tu doan so
    (day la phan "nhanh hon" - khong con phai chay thu-sai t_final nhieu lan):
    thoi gian = quang duong / van toc hanh trinh gia dinh + thoi gian on dinh
    du cho tung waypoint (dung lai/xoay huong).

    cruise_speed_mps=1.0 la gia tri THAN TRONG cho CascadePIDController mac
    dinh (tau_pos=1.5s) - neu drone di nhanh hon thuc te, con so nay chi la
    goi y ban dau, nguoi dung van sua duoc truoc khi chay.
    """
    dist = _path_length(waypoints, state0_pos)
    return round(dist / cruise_speed_mps + settle_s * len(waypoints), 1)


def _print_waypoint_overview(waypoints, state0_pos):
    print(f"\nDiem bat dau: {np.round(state0_pos, 2).tolist()}")
    print("Danh sach waypoint (World frame, m; do cao = -z):")
    for i, wp in enumerate(waypoints):
        pos = wp["pos"]
        print(f"  [{i}] x={pos[0]:.2f}, y={pos[1]:.2f}, do_cao={-pos[2]:.2f}, "
              f"yaw={np.rad2deg(wp['yaw']):.1f} deg")
    dist = _path_length(waypoints, state0_pos)
    print(f"Tong quang duong (noi tiep qua tat ca diem): {dist:.2f} m")


def _read_waypoints_manual():
    n = _ask_int("So luong waypoint can nhap", 3)
    waypoints = []
    for i in range(max(n, 1)):
        print(f"-- Waypoint {i} --")
        x = _ask_float("  x (m)", 0.0)
        y = _ask_float("  y (m)", 0.0)
        alt = _ask_float("  do cao (m, so duong)", 1.5)
        yaw_deg = _ask_float("  yaw (deg)", 0.0)
        waypoints.append({"pos": [x, y, -alt], "yaw": np.deg2rad(yaw_deg)})
    return waypoints


def _read_waypoints_from_json():
    path = input("Duong dan file JSON (Enter = quadsim/example_waypoints.json): ").strip()
    if path == "":
        path = os.path.join(os.path.dirname(__file__), "example_waypoints.json")
    waypoints, calib, no_fly_zones = load_waypoints_json(path)
    if no_fly_zones:
        print(f"  (Luu y: file co {len(no_fly_zones)} vung cam bay - CHUA duoc "
              f"tu dong ne trong mo phong nay, chi de tham khao.)")
    return waypoints


def action_waypoint_manager(params):
    """
    Luong THONG NHAT cho Waypoint Manager - gop lai thanh 1 duong duy nhat
    thay vi 2 muc menu rieng (truoc day [6]=mau cung, [8]=JSON rieng):

        1) Chon NGUON waypoint (mau / file JSON / nhap tay)
        2) Xem nhanh tong quan (danh sach + tong quang duong) truoc khi chay
        3) t_final duoc TU UOC LUONG san tu quang duong - khong con phai
           doan-thu-sai nhieu lan, chi Enter la chay duoc ngay
        4) Chay + in thoi diem toi TUNG waypoint (switch_log) de biet nhanh/
           cham ngay, khong phai tu doc do thi
    """
    print("\n=== BAY THEO CHUOI WAYPOINT ===")
    print("  [1] Dung mau co san (3 diem)")
    print("  [2] Nap tu file JSON (vd xuat tu waypoint_editor.py)")
    print("  [3] Nhap tay tung diem")
    source = _ask_int("Chon nguon waypoint", 1, choices=[1, 2, 3])

    if source == 1:
        waypoints = DEMO_WAYPOINTS
    elif source == 2:
        waypoints = _read_waypoints_from_json()
    else:
        waypoints = _read_waypoints_manual()

    state0 = np.zeros(12)
    _print_waypoint_overview(waypoints, state0[0:3])

    epsilon = _ask_float("\nBan kinh chap nhan chuyen diem epsilon (m)", 0.15)
    t_final_auto = _estimate_t_final(waypoints, state0[0:3])
    t_final = _ask_float("Thoi gian mo phong (s) - da tu uoc luong san", t_final_auto)

    ctrl = CascadePIDController(params)
    wp_mgr = WaypointManager(waypoints, epsilon=epsilon)

    t, state, omega, saturated, idx_log = simulate_waypoints(
        ctrl, wp_mgr, params, state0=state0, dt=0.005, t_final=t_final,
        stop_on_completion=True,
    )

    _print_state_summary(f"Waypoint Flight - {params.name}", t, state, saturated)

    print("\nThoi diem chuyen sang tung waypoint (cang som = cang nhanh):")
    print(f"  [0] t=0.000s (xuat phat)")
    for idx, t_switch in wp_mgr.switch_log:
        print(f"  [{idx}] t={t_switch:.3f}s")
    if wp_mgr.is_completed() and wp_mgr.distance_to_current(state[-1]) < epsilon:
        print(f"-> DA DEN waypoint cuoi va on dinh trong ban kinh {epsilon}m.")
    else:
        print(f"-> CHUA on dinh tai waypoint cuoi khi het t_final - thu tang "
              f"t_final (vd {t_final*1.5:.1f}s) hoac giam quang duong/epsilon.")

    print("\nDang ve va luu do thi...")
    plot_all(t, state, omega, params, output_dir="outputs", prefix="waypoint_",
              title=f"Quỹ đạo Waypoint — {params.name}")