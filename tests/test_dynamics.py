"""
tests/test_dynamics.py
=========================
Kiem tra CAC BAT BIEN VAT LY co ban cua lop dynamics.py - KHONG kiem tra
"gia tri dung tuyet doi" (vi khong co ground-truth phan tich cho he phi
tuyen day du), ma kiem tra cac tinh chat PHAI dung theo dinh luat vat ly:

    1. Roi tu do (khong luc, khong drag) phai khop CHINH XAC voi cong thuc
       Galileo z(t) = 1/2 g t^2 (day la truong hop CO loi giai giai tich).
    2. Hover (luc day = trong luong) phai giu nguyen do cao (khong roi,
       khong bay len).
    3. Bao toan dong luong goc khi khong co mo-men ngoai tac dung (torque-free
       rotation) - chuan Euler's equations, khong doi voi vat the doi xung
       truc (Ixx=Iyy).
    4. Ma tran xoay body->world phai la ma tran TRUC GIAO (RR^T = I) tai moi
       goc Euler - tinh chat bat bien cua SO(3).
"""

import numpy as np
import pytest

from quadsim.params import get_preset
from quadsim.dynamics import (
    rotation_matrix_body_to_world, sixdof_derivatives, rk4_step,
)


@pytest.fixture
def params():
    return get_preset("crazyflie")


def test_free_fall_matches_galileo(params):
    """Khong luc day, khong drag -> phai roi dung z(t) = 1/2 g t^2 (drag=0)."""
    params.drag_lin = np.zeros(3)  # tat drag de so sanh CHINH XAC voi cong thuc giai tich
    state = np.zeros(12)
    dt = 0.001
    t_final = 1.0
    n = int(t_final / dt)

    for _ in range(n):
        state = rk4_step(state, F_body=np.zeros(3), M_body=np.zeros(3), params=params, dt=dt)

    expected_altitude_drop = 0.5 * params.g * t_final ** 2
    # He NED: z=0 la diem xuat phat, roi XUONG lam z TANG DUONG (z am = do cao).
    actual_drop = state[2]
    assert actual_drop == pytest.approx(expected_altitude_drop, rel=1e-3)


def test_hover_thrust_holds_altitude(params):
    """Luc day = dung trong luong -> gia toc doc = 0 -> do cao khong doi."""
    state = np.zeros(12)
    F_body = np.array([0, 0, -params.mass * params.g])  # thrust bu trong luong (huong -z body = len)
    dt = 0.001
    for _ in range(2000):
        state = rk4_step(state, F_body=F_body, M_body=np.zeros(3), params=params, dt=dt)

    assert state[2] == pytest.approx(0.0, abs=1e-6)   # do cao khong doi
    assert state[5] == pytest.approx(0.0, abs=1e-6)   # van toc doc khong doi


def test_rotation_matrix_is_orthogonal():
    """R phai truc giao (R @ R.T = I) va det(R)=1 voi MOI goc Euler."""
    rng = np.random.default_rng(42)
    for _ in range(20):
        phi, theta, psi = rng.uniform(-np.pi / 2 + 0.01, np.pi / 2 - 0.01, size=3)
        R = rotation_matrix_body_to_world(phi, theta, psi)
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-10)
        assert np.linalg.det(R) == pytest.approx(1.0, abs=1e-10)


def test_torque_free_symmetric_body_keeps_constant_omega_z(params):
    """Vat the doi xung truc (Ixx=Iyy, khong mo-men ngoai) quay quanh z thuan
    tuy -> p=q=0 khong doi, r khong doi (khong co tien dong/nutation)."""
    assert params.Ixx == pytest.approx(params.Iyy)  # crazyflie doi xung truc x=y

    state = np.zeros(12)
    state[11] = 5.0  # r = 5 rad/s quanh truc z, khong co p,q ban dau
    dt = 0.001
    for _ in range(1000):
        state = rk4_step(state, F_body=np.zeros(3), M_body=np.zeros(3), params=params, dt=dt)

    assert state[9] == pytest.approx(0.0, abs=1e-8)    # p van = 0
    assert state[10] == pytest.approx(0.0, abs=1e-8)   # q van = 0
    assert state[11] == pytest.approx(5.0, abs=1e-6)   # r khong doi (bao toan)


def test_sixdof_derivatives_zero_state_zero_input_only_gravity():
    """State=0, luc/mo-men ngoai=0 -> chi con gia toc trong truong theo z,
    khong co gia toc ngang, khong co gia toc goc."""
    params = get_preset("crazyflie")
    state = np.zeros(12)
    dstate = sixdof_derivatives(state, F_body=np.zeros(3), M_body=np.zeros(3), params=params)

    assert dstate[3] == pytest.approx(0.0, abs=1e-10)   # du = 0
    assert dstate[4] == pytest.approx(0.0, abs=1e-10)   # dv = 0
    assert dstate[5] == pytest.approx(params.g, rel=1e-6)  # dw = +g (roi tu do trong body frame)
    np.testing.assert_allclose(dstate[9:12], 0.0, atol=1e-10)  # khong gia toc goc
