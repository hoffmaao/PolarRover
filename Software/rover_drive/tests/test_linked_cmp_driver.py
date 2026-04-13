import math

import numpy as np
import pytest

from rover_sim.control import FixType, RoverState
from rover_sim.missions import Mission, MissionKind, Waypoint

from rover_drive.estimation import EnKFConfig, EnsembleKalmanFilter
from rover_drive.modes.linked_cmp import CMPFormationConfig, CMPMetrics, LinkedCMPDriver


def _cmp_mission(midpoint: tuple[float, float] = (0, 0),
                 direction: tuple[float, float] = (1, 0)) -> Mission:
    """Mission with midpoint at wp[0] and survey direction toward wp[1]."""
    return Mission(
        kind=MissionKind.CMP,
        waypoints=[
            Waypoint(x=midpoint[0], y=midpoint[1]),
            Waypoint(x=midpoint[0] + direction[0] * 50, y=midpoint[1] + direction[1] * 50),
        ],
    )


def _state(t=0.0, x=0.0, y=0.0, heading=0.0, fix=FixType.FIXED) -> RoverState:
    return RoverState(t=t, x=x, y=y, heading=heading, fix_type=fix, position_std_m=0.015)


def _make_driver(start=2.0, end=20.0, rate=1.0, **kw) -> LinkedCMPDriver:
    cfg = CMPFormationConfig(start_spread_m=start, end_spread_m=end,
                             spread_rate_m_per_s=rate, **kw)
    return LinkedCMPDriver(
        config=cfg,
        enkf_a=EnsembleKalmanFilter(EnKFConfig(seed=42)),
        enkf_b=EnsembleKalmanFilter(EnKFConfig(seed=43)),
    )


# ---------- validation ----------

def test_needs_at_least_two_waypoints():
    d = _make_driver()
    m = Mission(kind=MissionKind.CMP, waypoints=[Waypoint(x=0, y=0)])
    with pytest.raises(RuntimeError, match="2 waypoints"):
        d.update(_state(), _state(), m, dt=0.05)


# ---------- initialization ----------

def test_waits_for_both_fixes():
    d = _make_driver()
    m = _cmp_mission()
    cmd_a, cmd_b = d.update(
        _state(x=1), _state(fix=FixType.NONE), m, dt=0.05
    )
    assert cmd_a.brake > 0

    cmd_a, cmd_b = d.update(
        _state(t=0.1, x=1), _state(t=0.1, x=-1), m, dt=0.05
    )
    assert cmd_a.throttle > 0 or cmd_b.throttle > 0


# ---------- spread increases over time ----------

def test_target_spread_increases():
    d = _make_driver(start=2.0, end=20.0, rate=2.0)
    m = _cmp_mission()
    for step in range(20):
        t = step * 0.1
        d.update(_state(t=t, x=1 + step * 0.1, heading=0.0),
                 _state(t=t, x=-1 - step * 0.1, heading=math.pi),
                 m, dt=0.1)
    assert d.current_target_spread > 2.0
    assert len(d.metrics.target_spread_samples) > 0
    assert d.metrics.target_spread_samples[-1] > d.metrics.target_spread_samples[0]


def test_target_spread_clamped_to_end():
    d = _make_driver(start=2.0, end=5.0, rate=100.0)
    m = _cmp_mission()
    for step in range(10):
        d.update(_state(t=step * 0.1, x=step),
                 _state(t=step * 0.1, x=-step, heading=math.pi),
                 m, dt=0.1)
    assert d.current_target_spread == pytest.approx(5.0)


# ---------- midpoint stability ----------

def test_symmetric_motion_preserves_midpoint():
    d = _make_driver(start=2.0, end=10.0, rate=1.0)
    m = _cmp_mission()
    for step in range(30):
        t = step * 0.1
        offset = 1.0 + step * 0.05
        d.update(_state(t=t, x=offset, heading=0.0),
                 _state(t=t, x=-offset, heading=math.pi),
                 m, dt=0.1)
    assert d.metrics.midpoint_drift_rms_m < 1.0


# ---------- metrics ----------

def test_metrics_accumulate():
    d = _make_driver()
    m = _cmp_mission()
    for step in range(10):
        t = step * 0.1
        d.update(_state(t=t, x=1 + step * 0.05, heading=0.0),
                 _state(t=t, x=-1 - step * 0.05, heading=math.pi),
                 m, dt=0.1)
    assert len(d.metrics.midpoint_drift_samples) >= 9
    assert len(d.metrics.spread_error_samples) >= 9


def test_empty_metrics():
    m = CMPMetrics()
    assert m.midpoint_drift_rms_m == 0.0
    assert m.spread_error_rms_m == 0.0
    assert m.spread_error_max_m == 0.0
    assert m.midpoint_drift_max_m == 0.0


# ---------- completion ----------

def test_completes_when_at_end_spread_and_near_targets():
    d = _make_driver(start=2.0, end=6.0, rate=5.0, target_tolerance_m=1.5)
    m = _cmp_mission()
    for step in range(100):
        t = step * 0.1
        spread = min(2.0 + 5.0 * t, 6.0)
        half = spread / 2.0
        d.update(_state(t=t, x=half, heading=0.0),
                 _state(t=t, x=-half, heading=math.pi),
                 m, dt=0.1)
    assert d.complete


# ---------- reset ----------

def test_reset_clears_all():
    d = _make_driver()
    m = _cmp_mission()
    for step in range(10):
        d.update(_state(t=step * 0.1, x=step, heading=0.0),
                 _state(t=step * 0.1, x=-step, heading=math.pi),
                 m, dt=0.1)
    assert len(d.metrics.midpoint_drift_samples) > 0
    d.reset()
    assert len(d.metrics.midpoint_drift_samples) == 0
    assert not d.enkf_a.initialized
