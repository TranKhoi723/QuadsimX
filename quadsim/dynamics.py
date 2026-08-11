"""
quadsim.dynamics
==================
DIGITAL TWIN - phuong trinh Newton-Euler cho vat ran 6-DOF. Quy uoc NED
(North-East-Down): z am = do cao duong.

State (12): [x y z  u v w  phi theta psi  p q r]
    x,y,z          : vi tri World frame (m)
    u,v,w          : van toc thang Body frame (m/s)
    phi,theta,psi  : goc Euler Roll-Pitch-Yaw (rad)
    p,q,r          : van toc goc Body frame (rad/s)
"""

import numpy as np


def rotation_matrix_body_to_world(phi, theta, psi):
    """Ma tran xoay Body->World, thu tu ZYX (chuan hang khong)."""
    cphi, sphi = np.cos(phi), np.sin(phi)
    cth, sth = np.cos(theta), np.sin(theta)
    cpsi, spsi = np.cos(psi), np.sin(psi)
    Rx = np.array([[1, 0, 0], [0, cphi, -sphi], [0, sphi, cphi]])
    Ry = np.array([[cth, 0, sth], [0, 1, 0], [-sth, 0, cth]])
    Rz = np.array([[cpsi, -spsi, 0], [spsi, cpsi, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def euler_kinematics_matrix(phi, theta):
    """
    Chuyen van toc goc Body (p,q,r) -> dao ham goc Euler (dphi,dtheta,dpsi).
    Ky di (gimbal lock) tai theta = +-90 do (chia cho cos(theta)=0).
    """
    cphi, sphi = np.cos(phi), np.sin(phi)
    cth, tth = np.cos(theta), np.tan(theta)
    return np.array([
        [1, sphi * tth, cphi * tth],
        [0, cphi, -sphi],
        [0, sphi / cth, cphi / cth],
    ])


def sixdof_derivatives(state, F_body, M_body, params):
    """Newton-Euler: (state, luc/mo-men Body frame) -> dao ham trang thai."""
    u, v, w = state[3], state[4], state[5]
    phi, theta, psi = state[6], state[7], state[8]
    p, q, r = state[9], state[10], state[11]

    R = rotation_matrix_body_to_world(phi, theta, psi)
    gravity_body = R.T @ np.array([0, 0, params.mass * params.g])
    drag_force = -params.drag_lin * np.array([u, v, w])
    F_total_body = np.array(F_body) + gravity_body + drag_force

    omega_vec = np.array([p, q, r])
    V_vec = np.array([u, v, w])

    dV = F_total_body / params.mass - np.cross(omega_vec, V_vec)          # Newton
    V_world = R @ V_vec
    dEuler = euler_kinematics_matrix(phi, theta) @ omega_vec
    domega = np.linalg.solve(
        params.I, np.array(M_body) - np.cross(omega_vec, params.I @ omega_vec)
    )                                                                     # Euler

    dstate = np.zeros(12)
    dstate[0:3], dstate[3:6], dstate[6:9], dstate[9:12] = V_world, dV, dEuler, domega
    return dstate


def rk4_step(state, F_body, M_body, params, dt):
    """Tich phan 1 buoc bang Runge-Kutta bac 4."""
    k1 = sixdof_derivatives(state, F_body, M_body, params)
    k2 = sixdof_derivatives(state + 0.5 * dt * k1, F_body, M_body, params)
    k3 = sixdof_derivatives(state + 0.5 * dt * k2, F_body, M_body, params)
    k4 = sixdof_derivatives(state + dt * k3, F_body, M_body, params)
    return state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
