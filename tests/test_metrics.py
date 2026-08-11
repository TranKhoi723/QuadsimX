"""
tests/test_metrics.py
========================
Kiem tra quadsim/metrics.py: 3 nhom chi so moi them theo yeu cau bao cao
thuc tap (hieu suat motor theo do muot omega, chi so vot so, tieu chi chon).
"""

import numpy as np
import pytest

from quadsim.params import get_preset
from quadsim.controllers import CascadePIDController, WaypointManager
from quadsim.simulate import simulate_waypoints
from quadsim.metrics import (
    motor_smoothness_index, motor_efficiency_proxy,
    leg_overshoot_table, overshoot_index,
    compute_run_metrics, score_config, compare_epsilon_values,
)


@pytest.fixture
def params():
    return get_preset("crazyflie")


def test_motor_smoothness_perfect_constant_omega(params):
    """Omega hang so tuyet doi -> do muot phai bang 1.0 (Sw toi da)."""
    omega_log = np.full((100, 4), params.omega_hover)
    s = motor_smoothness_index(omega_log, dt=0.01, params=params)
    assert s == pytest.approx(1.0, abs=1e-9)


def test_motor_smoothness_decreases_with_jitter(params):
    """Xung dao dong manh phai co Sw THAP HON xung on dinh."""
    smooth = np.full((200, 4), params.omega_hover)
    rng = np.random.default_rng(0)
    jittery = params.omega_hover + rng.normal(0, params.omega_max * 0.1, size=(200, 4))

    s_smooth = motor_smoothness_index(smooth, dt=0.005, params=params)
    s_jittery = motor_smoothness_index(jittery, dt=0.005, params=params)
    assert s_smooth > s_jittery
    assert 0.0 <= s_jittery <= 1.0


def test_motor_efficiency_proxy_bounds_and_weights(params):
    omega_log = np.full((150, 4), params.omega_hover)
    saturated = np.zeros(150, dtype=bool)
    result = motor_efficiency_proxy(omega_log, dt=0.005, params=params, saturated_log=saturated)
    assert 0.0 <= result["eta_proxy"] <= 1.0
    assert result["smoothness_S"] == pytest.approx(1.0, abs=1e-9)

    with pytest.raises(ValueError):
        motor_efficiency_proxy(omega_log, 0.005, params, saturated, weights=(0.5, 0.5, 0.5))


def test_leg_overshoot_table_detects_overshoot(params):
    """Dung 1 waypoint step ro rang tren truc X, kiem tra bang vot so tra
    ve dung so chang va truc chinh - so lieu overshoot >= 0."""
    waypoints = [{"pos": [0, 0, -1], "yaw": 0.0}, {"pos": [4, 0, -1], "yaw": 0.0}]
    wm = WaypointManager(waypoints, epsilon=0.2)
    ctrl = CascadePIDController(params)
    state0 = np.zeros(12)
    state0[0:3] = waypoints[0]["pos"]

    t, state, omega, saturated, wp_idx = simulate_waypoints(
        ctrl, wm, params, state0=state0, dt=0.01, t_final=10.0,
        stop_on_completion=True, settle_time=1.0,
    )
    table = leg_overshoot_table(t, state, wp_idx, waypoints)
    assert len(table) >= 1
    row = table[0]
    assert row["axis"] == "x"
    assert row["overshoot_pct"] >= 0.0

    idx = overshoot_index(table)
    assert idx["n_legs_evaluated"] == len(table)
    assert idx["max_overshoot_pct"] >= idx["mean_overshoot_pct"]


def test_leg_overshoot_table_skips_tiny_amplitude(params):
    """Chang gan nhu khong dich chuyen vi tri (chi doi yaw) phai bi BO QUA,
    khong duoc chia cho bien do gan 0."""
    waypoints = [{"pos": [0, 0, -1], "yaw": 0.0}, {"pos": [0.001, 0, -1], "yaw": 1.0}]
    wm = WaypointManager(waypoints, epsilon=0.2)
    ctrl = CascadePIDController(params)
    state0 = np.zeros(12)
    state0[0:3] = waypoints[0]["pos"]

    t, state, omega, saturated, wp_idx = simulate_waypoints(
        ctrl, wm, params, state0=state0, dt=0.01, t_final=5.0,
        stop_on_completion=True, settle_time=1.0,
    )
    table = leg_overshoot_table(t, state, wp_idx, waypoints)
    assert table == []


def test_compute_run_metrics_and_score_config(params):
    waypoints = [{"pos": [0, 0, -1], "yaw": 0.0}, {"pos": [3, 1, -1.5], "yaw": 0.0}]
    wm = WaypointManager(waypoints, epsilon=0.2)
    ctrl = CascadePIDController(params)
    state0 = np.zeros(12)
    state0[0:3] = waypoints[0]["pos"]

    t, state, omega, saturated, wp_idx = simulate_waypoints(
        ctrl, wm, params, state0=state0, dt=0.01, t_final=15.0,
        stop_on_completion=True, settle_time=1.0,
    )
    m = compute_run_metrics(t, state, omega, saturated, wp_idx, waypoints, params, dt=0.01)
    for key in ("flight_time_s", "saturation_pct", "motor_smoothness_S",
                "motor_efficiency_proxy", "overshoot_mean_pct", "overshoot_max_pct"):
        assert key in m

    score = score_config(m)
    assert 0.0 <= score["total_score"] <= 1.0

    with pytest.raises(ValueError):
        score_config(m, weights={"time": 0.5, "overshoot": 0.5, "smoothness": 0.5, "saturation": 0.5})


def test_compare_epsilon_values_ranks_and_sorts(params):
    waypoints = [{"pos": [0, 0, -1], "yaw": 0.0}, {"pos": [3, 0, -1], "yaw": 0.0}]
    rows = compare_epsilon_values(
        [0.2, 0.6],
        controller_factory=lambda: CascadePIDController(params),
        waypoints_no_epsilon=waypoints,
        params=params, dt=0.01, t_final=12.0,
    )
    assert len(rows) == 2
    assert {r["epsilon"] for r in rows} == {0.2, 0.6}
    # Sap xep giam dan theo total_score
    assert rows[0]["total_score"] >= rows[1]["total_score"]
