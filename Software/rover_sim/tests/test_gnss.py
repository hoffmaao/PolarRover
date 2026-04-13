import math

import pytest

from rover_sim.control import FixType
from rover_sim.sensors import FIX_NOISE_STD_M, GnssConfig, GnssSensor
from rover_sim.vehicle.base import VehicleState


def _truth(x: float = 10.0, y: float = 20.0) -> VehicleState:
    return VehicleState(x=x, y=y, heading=0.5, speed=1.0)


def test_rate_limit_returns_none_between_samples():
    s = GnssSensor(GnssConfig(rate_hz=10.0, seed=0))
    assert s.sample(_truth(), t=0.0) is not None
    assert s.sample(_truth(), t=0.05) is None
    assert s.sample(_truth(), t=0.1) is not None


def test_fixed_noise_envelope_matches_zed_f9p_rtk_fixed():
    s = GnssSensor(GnssConfig(rate_hz=100.0, seed=42, default_fix=FixType.FIXED))
    samples = []
    for i in range(2000):
        r = s.sample(_truth(), t=i * 0.01)
        assert r is not None
        samples.append((r.x - 10.0, r.y - 20.0))

    dxs = [a for a, _ in samples]
    dys = [b for _, b in samples]
    expected = FIX_NOISE_STD_M[FixType.FIXED]
    sigma_x = math.sqrt(sum(d * d for d in dxs) / len(dxs))
    sigma_y = math.sqrt(sum(d * d for d in dys) / len(dys))
    assert sigma_x == pytest.approx(expected, rel=0.2)
    assert sigma_y == pytest.approx(expected, rel=0.2)


def test_float_envelope_is_order_of_magnitude_worse_than_fixed():
    fixed = GnssSensor(GnssConfig(rate_hz=100.0, seed=1, default_fix=FixType.FIXED))
    floaty = GnssSensor(GnssConfig(rate_hz=100.0, seed=1, default_fix=FixType.FLOAT))
    truth = _truth()
    r_fixed = fixed.sample(truth, 0.0)
    r_float = floaty.sample(truth, 0.0)
    assert r_fixed is not None and r_float is not None
    assert r_fixed.fix_type is FixType.FIXED
    assert r_float.fix_type is FixType.FLOAT
    assert r_float.position_std_m > r_fixed.position_std_m * 5


def test_noiseless_override_produces_exact_truth():
    s = GnssSensor(
        GnssConfig(
            rate_hz=10.0,
            default_fix=FixType.FIXED,
            noise_std_m={FixType.FIXED: 0.0},
            seed=0,
        )
    )
    truth = _truth(x=123.0, y=456.0)
    r = s.sample(truth, t=0.0)
    assert r is not None
    assert r.x == 123.0
    assert r.y == 456.0


def test_dropout_produces_fix_none():
    s = GnssSensor(GnssConfig(rate_hz=10.0, prob_dropout=1.0, seed=0))
    r = s.sample(_truth(), t=0.0)
    assert r is not None
    assert r.fix_type is FixType.NONE
    assert r.position_std_m == 0.0


def test_reset_rewinds_fix_and_rng():
    s = GnssSensor(GnssConfig(rate_hz=10.0, seed=7, default_fix=FixType.FIXED))
    a = s.sample(_truth(), t=0.0)
    s.reset()
    b = s.sample(_truth(), t=0.0)
    assert a is not None and b is not None
    assert a.x == b.x
    assert a.y == b.y
    assert s.current_fix is FixType.FIXED
