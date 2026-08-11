"""
tests/test_waypoint_manager.py
=================================
Kiem tra WaypointManager - phan quan trong nhat cua tinh nang moi (epsilon
rieng tung waypoint) ma khong co bo test nao truoc do (diem yeu duoc ca 2
ban danh gia deu neu ra).
"""

import numpy as np
import pytest

from quadsim.params import get_preset
from quadsim.controllers import CascadePIDController, WaypointManager, default_gains
from quadsim.simulate import simulate_waypoints


@pytest.fixture
def params():
    return get_preset("crazyflie")


def test_waypoint_manager_advances_on_arrival():
    """Khi state nam trong epsilon cua waypoint hien tai -> phai CHUYEN sang
    waypoint tiep theo (current_idx tang)."""
    waypoints = [{"pos": [0, 0, -1], "yaw": 0.0}, {"pos": [5, 0, -1], "yaw": 0.0}]
    wm = WaypointManager(waypoints, epsilon=0.5)
    assert wm.current_idx == 0

    state_far = np.zeros(12)
    state_far[0:3] = [0, 0, -1]  # dung tai waypoint 0
    _, switched = wm.get_current_setpoint(state_far)
    assert switched is True
    assert wm.current_idx == 1  # da chuyen sang waypoint 1

    state_at_wp1 = np.zeros(12)
    state_at_wp1[0:3] = [5, 0, -1]
    _, switched2 = wm.get_current_setpoint(state_at_wp1)
    # waypoint cuoi cung - KHONG chuyen qua nua (khong co waypoint 2)
    assert switched2 is False
    assert wm.current_idx == 1


def test_per_waypoint_epsilon_overrides_default():
    """Waypoint co epsilon RIENG phai dung epsilon do, KHONG dung epsilon
    mac dinh cua WaypointManager - day la tinh nang moi quan trong nhat."""
    waypoints = [
        {"pos": [0, 0, -1], "yaw": 0.0, "epsilon": 2.0},   # de toi (epsilon lon)
        {"pos": [10, 0, -1], "yaw": 0.0, "epsilon": 0.05},  # kho toi (epsilon nho)
    ]
    wm = WaypointManager(waypoints, epsilon=0.5)  # epsilon mac dinh = 0.5

    state = np.zeros(12)
    state[0:3] = [1.5, 0, -1]  # cach waypoint 0 la 1.5m: > 0.5 (mac dinh) NHUNG < 2.0 (rieng)
    wm.get_current_setpoint(state)
    assert wm.current_idx == 1, "Phai dung epsilon RIENG (2.0m) cua waypoint 0, khong phai mac dinh (0.5m)"

    assert wm.epsilon_of_current() == pytest.approx(0.05)

    state2 = np.zeros(12)
    state2[0:3] = [9.8, 0, -1]  # cach waypoint 1 (epsilon rieng=0.05m) 0.2m: < mac dinh(0.5) NHUNG > rieng(0.05)
    wm.get_current_setpoint(state2)
    assert wm.current_idx == 1, "Khong duoc chuyen qua khi > epsilon RIENG (0.05m), du < epsilon mac dinh"


def test_waypoint_epsilon_none_falls_back_to_default():
    """Waypoint KHONG khai bao epsilon rieng -> phai dung dung epsilon mac dinh."""
    waypoints = [{"pos": [0, 0, -1], "yaw": 0.0}]  # khong co key "epsilon"
    wm = WaypointManager(waypoints, epsilon=0.33)
    assert wm.epsilon_of_current() == pytest.approx(0.33)


def test_distance_to_current_is_euclidean():
    waypoints = [{"pos": [3, 4, 0], "yaw": 0.0}]
    wm = WaypointManager(waypoints, epsilon=0.1)
    state = np.zeros(12)
    state[0:3] = [0, 0, 0]
    assert wm.distance_to_current(state) == pytest.approx(5.0)  # 3-4-5


def test_simulate_waypoints_reaches_final_waypoint(params):
    """Kiem tra tich hop: bay het 1 chuoi 2 waypoint that su bang PID, phai
    HOI TU ve gan waypoint cuoi trong pham vi epsilon truoc khi het t_final."""
    waypoints = [
        {"pos": [0.0, 0.0, -1.0], "yaw": 0.0, "epsilon": 0.2},
        {"pos": [1.0, 0.0, -1.0], "yaw": 0.0, "epsilon": 0.2},
    ]
    ctrl = CascadePIDController(params)
    wm = WaypointManager(waypoints, epsilon=0.2)
    state0 = np.zeros(12)
    state0[0:3] = waypoints[0]["pos"]

    t, state, omega, saturated, wp_idx = simulate_waypoints(
        ctrl, wm, params, state0=state0, dt=0.01, t_final=15.0, stop_on_completion=True)

    final_pos = state[-1, 0:3]
    dist_to_final_wp = np.linalg.norm(final_pos - np.array(waypoints[-1]["pos"]))
    assert dist_to_final_wp < 0.3, (
        f"Drone khong bay toi gan waypoint cuoi (khoang cach con lai={dist_to_final_wp:.3f}m)"
    )
    assert len(wm.switch_log) >= 1, "Phai co it nhat 1 lan chuyen waypoint trong switch_log"
