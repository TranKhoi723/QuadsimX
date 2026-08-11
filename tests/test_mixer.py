"""
tests/test_mixer.py
======================
Kiem tra Mixer (control allocation): wrench_to_omega() phai la NGHICH DAO
CHINH XAC cua omega_to_wrench() (A^-1 @ A = I), va cac dau +/- cua
mixer_signs() phai KHOP voi ket qua thuc te tu omega_to_wrench() - neu 2
cai nay lech nhau, scenarios.py (dung mixer_signs de tao xung roll/pitch/yaw)
se sinh chuyen dong SAI HUONG ma khong ai phat hien ra khi chi nhin do thi.
"""

import numpy as np
import pytest

from quadsim.params import get_preset
from quadsim.mixer import omega_to_wrench, wrench_to_omega, mixer_signs


@pytest.fixture
def params():
    return get_preset("crazyflie")


def test_wrench_omega_roundtrip(params):
    """omega -> wrench -> omega phai ra LAI DUNG omega ban dau (A kha nghich)."""
    omega = np.array([params.omega_hover * 1.05, params.omega_hover * 0.95,
                       params.omega_hover * 1.02, params.omega_hover * 0.98])
    wrench = omega_to_wrench(omega, params)
    omega_recovered = wrench_to_omega(wrench, params)
    np.testing.assert_allclose(omega_recovered, omega, rtol=1e-6)


def test_hover_wrench_has_zero_torque(params):
    """4 rotor cung toc do (hover) -> chi co luc day, KHONG co mo-men nao."""
    omega = np.full(4, params.omega_hover)
    F, tau_x, tau_y, tau_z = omega_to_wrench(omega, params)
    assert F == pytest.approx(params.mass * params.g, rel=1e-6)
    assert tau_x == pytest.approx(0.0, abs=1e-9)
    assert tau_y == pytest.approx(0.0, abs=1e-9)
    assert tau_z == pytest.approx(0.0, abs=1e-9)


def test_mixer_signs_match_actual_wrench_direction(params):
    """
    mixer_signs() cho biet rotor nao TANG toc do se tao mo-men DUONG theo
    tung truc. Kiem tra dieu nay dung voi CHINH omega_to_wrench(): tang nhe
    1 rotor theo dung dau mixer_signs() cho truc roll phai lam tau_x TANG
    (cung dau voi quy uoc), lam tuong tu cho pitch/yaw.
    """
    signs = mixer_signs(params)
    base = np.full(4, params.omega_hover)
    delta = 1.0  # rad/s, nhieu loan nho

    for axis, tau_index in [("roll", 1), ("pitch", 2), ("yaw", 3)]:
        omega_perturbed = base + signs[axis] * delta
        wrench = omega_to_wrench(omega_perturbed, params)
        # Mo-men tren dung truc phai CUNG DAU voi huong nhieu loan (>0)
        assert wrench[tau_index] > 0, (
            f"mixer_signs()['{axis}'] khong khop chieu thuc te tu omega_to_wrench() "
            f"- kich ban scenarios.py se sinh chuyen dong SAI huong cho truc {axis}."
        )


def test_negative_wrench_clipped_to_nonnegative_omega(params):
    """wrench_to_omega() phai CLIP omega^2 am ve 0 (khong sqrt() so am ra NaN)
    khi wrench yeu cau mot to hop vat ly khong kha thi."""
    extreme_wrench = np.array([0.0, 100.0, 100.0, 100.0])  # mo-men rat lon, luc day = 0
    omega = wrench_to_omega(extreme_wrench, params)
    assert np.all(np.isfinite(omega))
    assert np.all(omega >= 0)
