"""
quadsim.metrics
==================
TIEU CHI DANH GIA cho bao cao thuc tap - tach rieng khoi simulate.py de KHONG
dung vao vong lap tich phan (chi doc log sau khi mo phong xong).

3 nhom chi so, dung chung 1 nguyen tac: CHUAN HOA ve [0,1] hoac % de so sanh
duoc giua cac lan chay khac nhau (vd doi epsilon, doi gain PID, doi drone).

    1) "Hieu suat Motor" (motor_smoothness_index / motor_efficiency_proxy)
       DANH GIA THEO DO MUOT CUA XUNG OMEGA(t), KHONG PHAI hieu suat dien
       that (mo hinh nay khong co dong co dien/pin - xem CANH BAO trong
       docstring motor_efficiency_proxy). Muot = it giat cuc = it hao phi
       nhiet do dong/xa dong co lien tuc trong thuc te.

    2) "Chi so vot so" (leg_overshoot_table / overshoot_index)
       % vuot qua setpoint tren truc dich chuyen chinh cua tung chang bay,
       CUNG CONG THUC voi step_metrics() trong tune_altitude_gain.py, chi
       tong quat hoa tu 1-truc-Z sang 3-truc va tu 1-buoc-nhay sang NHIEU
       waypoint lien tiep.

    3) "Tieu chi chon" (score_config / compare_epsilon_values)
       Ham diem TONG HOP (weighted sum) de CHON epsilon/gain "tot nhat" -
       tra loi cau hoi "ban kinh waypoint lon/nho anh huong toc do muot
       dong co the nao" bang thuc nghiem quet nhieu gia tri epsilon.
"""

import numpy as np


# ----------------------------------------------------------------------
# 1) HIEU SUAT MOTOR - theo do muot xung omega
# ----------------------------------------------------------------------
def motor_smoothness_index(omega_log, dt, params, alpha=20.0):
    """
    Chi so do MUOT cua xung dieu khien omega(t), CHUAN HOA ve khoang (0, 1]:

        jerk_rms   = RMS( d(omega)/dt ) tren CA 4 dong co, ca qua trinh bay
        sigma_w    = jerk_rms / omega_max        (khong thu nguyen, cang
                                                    lon = dong co doi toc
                                                    cang gap/giat cang manh)
        S_w        = exp(-alpha * sigma_w)       -> (0, 1]

    S_w = 1   : omega gan nhu hang so (ly tuong, hover tuyet doi).
    S_w -> 0  : omega dao dong/giat cuc lien tuc (dong co mau chong hong,
                rung khung, hao pin nhanh trong thuc te).

    alpha=20.0 la he so ty le CHON THUC NGHIEM sao cho S_w roi vao khoang
    de doc (~0.3-0.9) voi cac kich ban bay thong thuong cua project nay -
    NEU doi drone/preset khac qua khac biet ve omega_max, nen ve lai
    duong cong S_w(alpha) 1 lan de kiem tra truoc khi dua vao bao cao.
    """
    omega_log = np.asarray(omega_log, dtype=float)
    if omega_log.shape[0] < 2 or dt <= 0:
        return 1.0
    domega_dt = np.diff(omega_log, axis=0) / dt
    jerk_rms = float(np.sqrt(np.mean(domega_dt ** 2)))
    sigma_w = jerk_rms / max(params.omega_max, 1e-9)
    return float(np.exp(-alpha * sigma_w))


def motor_efficiency_proxy(omega_log, dt, params, saturated_log, alpha=20.0,
                            weights=(0.5, 0.3, 0.2)):
    """
    UOC LUONG (proxy) "hieu suat dong co", KHONG PHAI hieu suat dien thuc
    (W thuc = P_co_hoc / P_dien, can mo hinh dong co DC/BLDC + pin ma
    project nay KHONG co - dynamics.py chi mo phong CO HOC 6-DOF). Ket hop
    3 thanh phan, moi thanh phan da nam trong [0,1]:

        S      = motor_smoothness_index(...)          it giat cuc
        U      = 1 - |mean(omega)/omega_max - omega_hover/omega_max|
                                                        gan vung hover ly
                                                        tuong (khong chay
                                                        du thua toc do)
        SAT    = 1 - ty_le_buoc_bao_hoa                khong bi ep kich
                                                        (clip) lien tuc

        eta_proxy = w1*S + w2*U + w3*SAT   (w1+w2+w3 = 1, mac dinh 0.5/0.3/0.2)

    Dung de SO SANH TUONG DOI giua cac cau hinh (vd epsilon khac nhau, gain
    PID khac nhau) trong CUNG 1 mo hinh - KHONG dung con so nay nhu % hieu
    suat dien thuc trong bao cao neu khong chua giai thich ro day la proxy.
    """
    w1, w2, w3 = weights
    if abs(w1 + w2 + w3 - 1.0) > 1e-6:
        raise ValueError(f"weights phai cong lai bang 1.0, nhan duoc {weights}")

    omega_log = np.asarray(omega_log, dtype=float)
    S = motor_smoothness_index(omega_log, dt, params, alpha=alpha)

    mean_ratio = float(np.mean(omega_log)) / max(params.omega_max, 1e-9)
    hover_ratio = params.omega_hover / max(params.omega_max, 1e-9)
    U = 1.0 - min(abs(mean_ratio - hover_ratio), 1.0)

    sat_ratio = float(np.mean(saturated_log)) if len(saturated_log) else 0.0
    SAT = 1.0 - sat_ratio

    eta_proxy = w1 * S + w2 * U + w3 * SAT
    return {
        "eta_proxy": float(np.clip(eta_proxy, 0.0, 1.0)),
        "smoothness_S": S,
        "hover_utilization_U": U,
        "saturation_free_SAT": SAT,
    }


# ----------------------------------------------------------------------
# 2) CHI SO VOT SO (overshoot) - tung chang bay va toan hanh trinh
# ----------------------------------------------------------------------
def leg_overshoot_table(t_log, state_log, waypoint_idx_log, world_wps):
    """
    Chi so vot so (%) CHO TUNG CHANG BAY (leg = doan tu waypoint i-1 den
    waypoint i), TONG QUAT HOA cong thuc step_metrics() trong
    tune_altitude_gain.py (overshoot_pct = (peak-target)/amplitude*100)
    tu 1-truc sang CHON TRUC CO BIEN DO DICH CHUYEN LON NHAT trong chang do
    (truc "chu dao" cua chang bay).

    Tra ve list[dict]: {"leg", "axis", "amplitude_m", "overshoot_pct",
    "overshoot_m"}. Chang co bien do qua nho (<1cm, vd chi doi yaw) duoc
    BO QUA (khong co "buoc nhay" vi tri de tinh vot so).
    """
    t_log = np.asarray(t_log)
    state_log = np.asarray(state_log)
    waypoint_idx_log = np.asarray(waypoint_idx_log)
    axis_names = ["x", "y", "z"]

    rows = []
    n_legs = len(world_wps)
    for leg in range(n_legs):
        mask = waypoint_idx_log == leg
        if not np.any(mask):
            continue
        idx = np.where(mask)[0]
        start_pos = state_log[idx[0], 0:3]
        target_pos = np.asarray(world_wps[leg]["pos"], dtype=float)
        amplitude_vec = target_pos - start_pos
        axis = int(np.argmax(np.abs(amplitude_vec)))
        amplitude = amplitude_vec[axis]
        if abs(amplitude) < 1e-2:
            continue  # khong co dich chuyen dang ke tren truc nao - bo qua

        response = state_log[idx, axis]
        if amplitude > 0:
            overshoot_m = max(0.0, float(response.max()) - target_pos[axis])
        else:
            overshoot_m = max(0.0, target_pos[axis] - float(response.min()))
        overshoot_pct = overshoot_m / abs(amplitude) * 100.0

        rows.append({
            "leg": leg,
            "axis": axis_names[axis],
            "amplitude_m": float(amplitude),
            "overshoot_m": float(overshoot_m),
            "overshoot_pct": float(overshoot_pct),
        })
    return rows


def overshoot_index(leg_table):
    """
    Gop bang leg_overshoot_table() thanh 1 CHI SO VOT SO tong the cua ca
    hanh trinh: (trung binh, lon nhat) tinh theo % - dung trong bang tong
    hop / "tieu chi chon" ben duoi.
    """
    if not leg_table:
        return {"mean_overshoot_pct": 0.0, "max_overshoot_pct": 0.0, "n_legs_evaluated": 0}
    pct = np.array([row["overshoot_pct"] for row in leg_table])
    return {
        "mean_overshoot_pct": float(pct.mean()),
        "max_overshoot_pct": float(pct.max()),
        "n_legs_evaluated": len(leg_table),
    }


# ----------------------------------------------------------------------
# 3) TIEU CHI CHON - diem tong hop de chon cau hinh (vd epsilon) "tot nhat"
# ----------------------------------------------------------------------
def compute_run_metrics(t_log, state_log, omega_log, saturated_log,
                         waypoint_idx_log, world_wps, params, dt):
    """Goi tat ca chi so tren cho 1 lan chay mo phong, tra ve 1 dict phang
    - dung lam 1 HANG trong bang so sanh nhieu cau hinh (vd nhieu epsilon)."""
    eff = motor_efficiency_proxy(omega_log, dt, params, saturated_log)
    legs = leg_overshoot_table(t_log, state_log, waypoint_idx_log, world_wps)
    ov = overshoot_index(legs)
    return {
        "flight_time_s": float(t_log[-1]) if len(t_log) else 0.0,
        "saturation_pct": float(np.mean(saturated_log) * 100.0) if len(saturated_log) else 0.0,
        "motor_smoothness_S": eff["smoothness_S"],
        "motor_efficiency_proxy": eff["eta_proxy"],
        "overshoot_mean_pct": ov["mean_overshoot_pct"],
        "overshoot_max_pct": ov["max_overshoot_pct"],
        "_leg_table": legs,
    }


def score_config(run_metrics, weights=None, targets=None):
    """
    TIEU CHI CHON: quy 1 dict run_metrics (tu compute_run_metrics) ve 1 DIEM
    SO DUY NHAT trong [0,1] de xep hang nhieu cau hinh (nhieu epsilon, nhieu
    bo gain...) - diem CANG CAO cang tot.

    Mac dinh 4 tieu chi, trong so cong lai = 1:
        thoi_gian_bay   (nho hon tot hon, so voi target_time_s)   15%
        vot_so          (nho hon tot hon, target 0%)              30%
        do_muot_motor   (S_w, lon hon tot hon)                    35%
        bao_hoa_dong_co (nho hon tot hon)                         20%

    targets: dict tuy chon {"time_s": ..., } de chuan hoa thoi gian bay
    tuong doi (mac dinh so voi chinh gia tri dang xet, tuc thanh phan thoi
    gian se ~trung lap - NEN truyen targets["time_s"] = thoi gian nhanh
    nhat trong tap cau hinh dang so sanh khi dung trong compare_epsilon_values).
    """
    w = weights or {"time": 0.15, "overshoot": 0.30, "smoothness": 0.35, "saturation": 0.20}
    if abs(sum(w.values()) - 1.0) > 1e-6:
        raise ValueError(f"Tong trong so phai = 1.0, nhan duoc {w}")

    t_ref = (targets or {}).get("time_s", run_metrics["flight_time_s"])
    time_score = 1.0 if run_metrics["flight_time_s"] <= 1e-9 else \
        float(np.clip(t_ref / run_metrics["flight_time_s"], 0.0, 1.0))
    overshoot_score = float(np.clip(1.0 - run_metrics["overshoot_mean_pct"] / 100.0, 0.0, 1.0))
    smoothness_score = float(np.clip(run_metrics["motor_smoothness_S"], 0.0, 1.0))
    saturation_score = float(np.clip(1.0 - run_metrics["saturation_pct"] / 100.0, 0.0, 1.0))

    total = (w["time"] * time_score + w["overshoot"] * overshoot_score +
             w["smoothness"] * smoothness_score + w["saturation"] * saturation_score)
    return {
        "total_score": float(total),
        "time_score": time_score,
        "overshoot_score": overshoot_score,
        "smoothness_score": smoothness_score,
        "saturation_score": saturation_score,
    }


def compare_epsilon_values(epsilon_values, controller_factory, waypoints_no_epsilon,
                            params, dt=0.005, t_final=30.0):
    """
    KHAO SAT anh huong BAN KINH WAYPOINT (epsilon) len do muot dong co /
    vot so / thoi gian bay - chay lai simulate_waypoints() 1 lan cho MOI
    gia tri epsilon trong epsilon_values, dung CHUNG 1 bo waypoint + gain.

    controller_factory: ham khong tham so -> tra ve 1 CascadePIDController
    MOI (moi lan chay can controller/gain rieng, tranh loi tich luy integral
    tu lan chay truoc).
    waypoints_no_epsilon: list[dict] {"pos":[..], "yaw":..} - KHONG kem
    epsilon rieng (epsilon se lay tu epsilon_values dang quet).

    Tra ve list[dict], moi dict = 1 hang: {"epsilon", **compute_run_metrics(...),
    **score_config(...)}. Sap xep GIAM DAN theo total_score (cau hinh de xuat
    dau bang).
    """
    # import cuc bo de tranh vong lap import (metrics.py <-> simulate.py)
    from .simulate import simulate_waypoints
    from .controllers import WaypointManager

    rows = []
    fastest_time = None
    for eps in epsilon_values:
        wm = WaypointManager(
            [dict(pos=wp["pos"], yaw=wp.get("yaw", 0.0)) for wp in waypoints_no_epsilon],
            epsilon=eps,
        )
        ctrl = controller_factory()
        state0 = np.zeros(12)
        state0[0:3] = waypoints_no_epsilon[0]["pos"]

        t, state, omega, saturated, wp_idx = simulate_waypoints(
            ctrl, wm, params, state0=state0, dt=dt, t_final=t_final,
            stop_on_completion=True, settle_time=1.0,
        )
        m = compute_run_metrics(t, state, omega, saturated, wp_idx,
                                 waypoints_no_epsilon, params, dt)
        m["epsilon"] = float(eps)
        rows.append(m)
        if fastest_time is None or m["flight_time_s"] < fastest_time:
            fastest_time = m["flight_time_s"]

    for m in rows:
        s = score_config(m, targets={"time_s": fastest_time})
        m.update(s)

    rows.sort(key=lambda r: r["total_score"], reverse=True)
    return rows
