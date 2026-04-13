import math

import numpy as np
import pytest

from rover_sim.control import FixType, RoverState
from rover_sim.missions import Mission, MissionKind, Waypoint

from rover_drive.estimation import EnKFConfig, EnsembleKalmanFilter
from rover_drive.modes.multipass import (
    MultipassControllerConfig,
    MultipassDriver,
    MultipassMetrics,
)


def _line_mission(
    start: tuple[float, float],
    end: tuple[float, float],
    n_points: int = 50,
) -> Mission:
    """Create a densely sampled straight-line mission in ENU meters."""
    xs = np.linspace(start[0], end[0], n_points)
    ys = np.linspace(start[1], end[1], n_points)
    wps = [Waypoint(x=float(x), y=float(y)) for x, y in zip(xs, ys)]
    return Mission(kind=MissionKind.MULTIPASS, waypoints=wps)


def _state(
    t: float = 0.0,
    x: float = 0.0,
    y: float = 0.0,
    heading: float = 0.0,
    speed: float = 0.0,
    fix: FixType = FixType.FIXED,
) -> RoverState:
    return RoverState(
        t=t, x=x, y=y, heading=heading, speed=speed,
        fix_type=fix, position_std_m=0.015,
    )


def _seeded_driver(**kwargs) -> MultipassDriver:
    cfg = MultipassControllerConfig(**kwargs)
    return MultipassDriver(config=cfg, enkf=EnsembleKalmanFilter(EnKFConfig(seed=42)))


# ---------- validation ----------


def test_missing_mission_raises():
    d = _seeded_driver()
    with pytest.raises(RuntimeError, match="non-empty"):
        d.update(_state(), mission=None, dt=0.05)


def test_empty_mission_raises():
    d = _seeded_driver()
    m = Mission(kind=MissionKind.MULTIPASS, waypoints=[])
    with pytest.raises(RuntimeError, match="non-empty"):
        d.update(_state(), m, dt=0.05)


# ---------- initialization ----------


def test_waits_for_valid_fix():
    d = _seeded_driver()
    m = _line_mission((5.0, 0.0), (20.0, 0.0))
    cmd = d.update(_state(fix=FixType.NONE), m, dt=0.05)
    assert not d.enkf.initialized
    assert cmd.brake > 0


def test_initializes_on_valid_fix_and_commands_forward():
    d = _seeded_driver()
    m = _line_mission((5.0, 0.0), (20.0, 0.0))
    cmd = d.update(_state(), m, dt=0.05)
    assert d.enkf.initialized
    assert cmd.throttle > 0


# ---------- cross-track metric ----------


def test_cross_track_recorded_each_step():
    d = _seeded_driver()
    m = _line_mission((0.0, 0.0), (20.0, 0.0), n_points=100)
    for step in range(10):
        d.update(_state(t=step * 0.1, x=step * 0.3), m, dt=0.1)
    assert len(d.metrics.cross_track_samples) >= 9  # first step may init only


def test_on_track_gives_small_cross_track():
    d = _seeded_driver()
    m = _line_mission((0.0, 0.0), (20.0, 0.0), n_points=100)
    for step in range(40):
        d.update(_state(t=step * 0.1, x=step * 0.2, y=0.0), m, dt=0.1)
    assert d.metrics.cross_track_rms_m < 0.5


def test_off_track_gives_larger_cross_track():
    d = _seeded_driver()
    m = _line_mission((0.0, 0.0), (20.0, 0.0), n_points=100)
    for step in range(40):
        d.update(_state(t=step * 0.1, x=step * 0.2, y=3.0), m, dt=0.1)
    assert d.metrics.cross_track_rms_m > 2.0


def test_metrics_rms_max_mean():
    m = MultipassMetrics(cross_track_samples=[1.0, 2.0, 3.0])
    assert m.cross_track_rms_m == pytest.approx(math.sqrt((1 + 4 + 9) / 3), rel=1e-9)
    assert m.cross_track_max_m == pytest.approx(3.0)
    assert m.cross_track_mean_m == pytest.approx(2.0)


def test_empty_metrics():
    m = MultipassMetrics()
    assert m.cross_track_rms_m == 0.0
    assert m.cross_track_max_m == 0.0


# ---------- completion ----------


def test_completes_when_reaching_end_of_path():
    d = _seeded_driver(path_complete_tolerance_m=2.0)
    m = _line_mission((0.0, 0.0), (10.0, 0.0), n_points=50)
    for step in range(100):
        x_pos = min(step * 0.2, 10.0)
        d.update(_state(t=step * 0.1, x=x_pos, y=0.0), m, dt=0.1)
    assert d.complete


# ---------- pure-pursuit steering ----------


def test_lookahead_steers_toward_path_not_just_nearest():
    """With a path curving north, the lookahead target should cause a positive
    steer even when the nearest point is directly beside the rover."""
    # L-shaped path: east then north
    wps = (
        [Waypoint(x=float(x), y=0.0) for x in np.linspace(0, 10, 50)]
        + [Waypoint(x=10.0, y=float(y)) for y in np.linspace(0, 10, 50)]
    )
    m = Mission(kind=MissionKind.MULTIPASS, waypoints=wps)
    d = _seeded_driver(lookahead_m=3.0)

    # Position the rover near the bend, heading east
    for step in range(5):
        d.update(_state(t=step * 0.1, x=9.0, y=0.0, heading=0.0), m, dt=0.1)

    cmd = d.update(_state(t=0.5, x=9.5, y=0.0, heading=0.0), m, dt=0.1)
    # Lookahead lands on the north-going segment → steer positive (left turn)
    assert cmd.steer > 0.1


# ---------- reset ----------


def test_reset_clears_metrics_and_filter():
    d = _seeded_driver()
    m = _line_mission((0.0, 0.0), (10.0, 0.0))
    for step in range(20):
        d.update(_state(t=step * 0.1, x=step * 0.3), m, dt=0.1)
    assert len(d.metrics.cross_track_samples) > 0
    d.reset()
    assert len(d.metrics.cross_track_samples) == 0
    assert not d.enkf.initialized
    assert d.progress_index == 0
