"""
quadsim.simulate
===================
2 VONG LAP MO PHONG:

    simulate()             - VONG HO: omega(t) do NGUOI DUNG dinh nghia,
                              khong co phan hoi. Dung cho scenarios.py.
    simulate_closed_loop()  - VONG KIN: 1 CascadePIDController tinh wrench
                              tu sai so hien tai moi buoc.

Luong du lieu chung ca 2 truong hop (chi khac o BUOC 0):
    [omega hoac state] -> Mixer -> [F,tau] -> Newton-Euler (RK4) -> state moi
"""

import numpy as np
from .mixer import omega_to_wrench, wrench_to_motor_command
from .dynamics import rk4_step
from .controllers import WaypointManager


def simulate(omega_cmd, params, t_final=2.0, dt=0.0005, state0=None):
    """
    VONG HO. omega_cmd: ham nhan t (giay) -> [w1,w2,w3,w4] (rad/s).
    Tra ve (t_log, state_log, omega_log).
    """
    if state0 is None:
        state0 = np.zeros(12)
    if dt <= 0 or t_final <= 0:
        raise ValueError(f"t_final ({t_final}) va dt ({dt}) phai > 0.")

    N = max(int(t_final / dt), 1)
    t_log = np.zeros(N + 1)
    state_log = np.zeros((N + 1, 12))
    omega_log = np.zeros((N + 1, 4))
    state = state0.copy()
    state_log[0] = state

    for k in range(N):
        t = t_log[k]
        omega = np.clip(np.asarray(omega_cmd(t), dtype=float), params.omega_min, params.omega_max)
        omega_log[k] = omega

        wrench = omega_to_wrench(omega, params)
        F_body = np.array([0, 0, -wrench[0]])
        M_body = wrench[1:4]

        state = rk4_step(state, F_body, M_body, params, dt)
        t_log[k + 1] = t + dt
        state_log[k + 1] = state

    omega_log[N] = omega_log[N - 1]
    return t_log, state_log, omega_log


def simulate_closed_loop(controller, params, setpoint, state0=None, dt=0.005, t_final=8.0):
    """
    VONG KIN. controller: doi tuong co .update(state, setpoint, dt) -> wrench
    (vd CascadePIDController). Tra ve (t_log, state_log, omega_log, saturated_log).
    """
    if state0 is None:
        state0 = np.zeros(12)
    if dt <= 0 or t_final <= 0:
        raise ValueError(f"t_final ({t_final}) va dt ({dt}) phai > 0.")

    n_steps = max(int(t_final / dt), 1)
    t_log = np.zeros(n_steps)
    state_log = np.zeros((n_steps, 12))
    omega_log = np.zeros((n_steps, 4))
    saturated_log = np.zeros(n_steps, dtype=bool)

    state = state0.copy()
    for k in range(n_steps):
        wrench_cmd = controller.update(state, setpoint, dt)
        omega_cmd = wrench_to_motor_command(wrench_cmd, params)
        wrench_actual = omega_to_wrench(omega_cmd, params)
        F_actual, tau_x, tau_y, tau_z = wrench_actual
        F_body = np.array([0, 0, -F_actual])
        M_body = np.array([tau_x, tau_y, tau_z])

        t_log[k] = k * dt
        state_log[k] = state
        omega_log[k] = omega_cmd
        saturated_log[k] = np.any(omega_cmd >= params.omega_max - 1e-6)

        state = rk4_step(state, F_body, M_body, params, dt)

    return t_log, state_log, omega_log, saturated_log


def simulate_waypoints(controller, waypoint_manager, params, state0=None,
                        dt=0.005, t_final=30.0, stop_on_completion=False,
                        settle_time=1.0):
    """
    VONG KIN + WAYPOINT: giong simulate_closed_loop(), nhung setpoint moi
    buoc lay tu waypoint_manager.get_current_setpoint(state, t) thay vi 1
    setpoint co dinh duy nhat.

    Tra ve (t_log, state_log, omega_log, saturated_log, waypoint_idx_log).
    """
    if not isinstance(waypoint_manager, WaypointManager):
        raise TypeError("waypoint_manager phai la instance cua WaypointManager.")
    if state0 is None:
        state0 = np.zeros(12)
    if dt <= 0 or t_final <= 0:
        raise ValueError(f"t_final ({t_final}) va dt ({dt}) phai > 0.")

    n_steps = max(int(t_final / dt), 1)
    t_log = np.zeros(n_steps)
    state_log = np.zeros((n_steps, 12))
    omega_log = np.zeros((n_steps, 4))
    saturated_log = np.zeros(n_steps, dtype=bool)
    waypoint_idx_log = np.zeros(n_steps, dtype=int)

    waypoint_manager.reset()
    state = state0.copy()
    settled_since = None

    for k in range(n_steps):
        t = k * dt
        setpoint, _switched = waypoint_manager.get_current_setpoint(state, t)

        wrench_cmd = controller.update(state, setpoint, dt)
        omega_cmd = wrench_to_motor_command(wrench_cmd, params)
        wrench_actual = omega_to_wrench(omega_cmd, params)
        F_actual, tau_x, tau_y, tau_z = wrench_actual
        F_body = np.array([0, 0, -F_actual])
        M_body = np.array([tau_x, tau_y, tau_z])

        t_log[k] = t
        state_log[k] = state
        omega_log[k] = omega_cmd
        saturated_log[k] = np.any(omega_cmd >= params.omega_max - 1e-6)
        waypoint_idx_log[k] = waypoint_manager.current_idx

        state = rk4_step(state, F_body, M_body, params, dt)

        if stop_on_completion and waypoint_manager.is_completed():
            if waypoint_manager.distance_to_current(state) < waypoint_manager.epsilon_of_current():
                if settled_since is None:
                    settled_since = t
                elif t - settled_since >= settle_time:
                    k_end = k + 1
                    return (t_log[:k_end], state_log[:k_end], omega_log[:k_end],
                            saturated_log[:k_end], waypoint_idx_log[:k_end])
            else:
                settled_since = None

    return t_log, state_log, omega_log, saturated_log, waypoint_idx_log
