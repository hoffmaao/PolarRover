import math

import numpy as np
import pytest

from rover_sim.control import FixType, RoverState
from rover_sim.missions import Mission, MissionKind, Waypoint

from rover_drive.estimation import EnKFConfig, EnsembleKalmanFilter
from rover_drive.modes.waypoint import WaypointControllerConfig, WaypointDriver


def _mission(*coords: tuple[float, float]) -> Mission:
    return Mission(
        kind=MissionKind.WAYPOINT,
        waypoints=[Waypoint(x=x, y=y) for x, y in coords],
    )


def _state(
    t: float = 0.0,
    x: float = 0.0,
    y: float = 0.0,
    heading: float = 0.0,
    speed: float = 0.0,
    fix: FixType = FixType.FIXED,
) -> RoverState:
    return RoverState(
        t=t,
        x=x,
        y=y,
        heading=heading,
        speed=speed,
        fix_type=fix,
        position_std_m=0.015,
    )


def _seeded_driver(**kwargs) -> WaypointDriver:
    cfg = WaypointControllerConfig(**kwargs)
    return WaypointDriver(config=cfg, enkf=EnsembleKalmanFilter(EnKFConfig(seed=42)))


# ---------- protocol / validation ----------


def test_mission_required():
    driver = _seeded_driver()
    with pytest.raises(RuntimeError, match="mission"):
        driver.update(_state(), mission=None, dt=0.05)


def test_empty_mission_raises():
    driver = _seeded_driver()
    mission = Mission(kind=MissionKind.WAYPOINT, waypoints=[])
    with pytest.raises(RuntimeError, match="non-empty"):
        driver.update(_state(), mission, dt=0.05)


# ---------- initialization & fix handling ----------


def test_waits_for_valid_fix_before_initializing_filter():
    driver = _seeded_driver()
    mission = _mission((10.0, 0.0))
    cmd = driver.update(_state(fix=FixType.NONE), mission, dt=0.05)
    assert not driver.enkf.initialized
    assert cmd.brake > 0.0
    assert cmd.throttle == 0.0


def test_initializes_filter_on_first_valid_fix_and_commands_forward():
    driver = _seeded_driver()
    mission = _mission((10.0, 0.0))
    cmd = driver.update(_state(), mission, dt=0.05)
    assert driver.enkf.initialized
    # Waypoint is directly ahead → forward throttle, near-zero steer
    assert cmd.throttle > 0.0
    assert abs(cmd.steer) < 0.1


# ---------- steering geometry ----------


def test_waypoint_to_the_left_produces_positive_steer():
    driver = _seeded_driver(heading_saturation_rad=math.radians(90.0))
    mission = _mission((0.0, 10.0))  # due north when heading east
    cmd = driver.update(_state(heading=0.0), mission, dt=0.05)
    # target_heading = pi/2, rover_heading = 0, alpha = pi/2 ≈ 1.57
    # Saturation is 90° (pi/2), so steer ≈ 1.57 / 1.57 = 1.0 (max left)
    assert cmd.steer > 0.5


def test_waypoint_to_the_right_produces_negative_steer():
    driver = _seeded_driver(heading_saturation_rad=math.radians(90.0))
    mission = _mission((0.0, -10.0))
    cmd = driver.update(_state(heading=0.0), mission, dt=0.05)
    assert cmd.steer < -0.5


def test_steer_saturates_at_config_bound():
    driver = _seeded_driver(heading_saturation_rad=math.radians(10.0))
    mission = _mission((0.0, 10.0))
    cmd = driver.update(_state(heading=0.0), mission, dt=0.05)
    # heading error ≈ 90°, saturation 10° → should clamp at 1.0
    assert cmd.steer == pytest.approx(1.0)


# ---------- waypoint advancement ----------


def test_advances_past_nearby_waypoint_and_targets_next():
    driver = _seeded_driver(waypoint_tolerance_m=1.0)
    mission = _mission((1.5, 0.0), (20.0, 0.0))

    # First call: initialize filter at origin
    driver.update(_state(), mission, dt=0.05)
    assert driver.active_waypoint_index == 0

    # Walk the "rover" through the first waypoint's tolerance radius
    for step in range(30):
        driver.update(
            _state(t=(step + 1) * 0.1, x=0.1 + step * 0.1),
            mission,
            dt=0.1,
        )
    assert driver.active_waypoint_index >= 1


def test_mission_complete_when_all_waypoints_reached():
    driver = _seeded_driver(waypoint_tolerance_m=2.0)
    mission = _mission((2.0, 0.0), (4.0, 0.0))

    # Drive the rover past both waypoints
    for step, x in enumerate(np.linspace(0.0, 6.0, 60)):
        driver.update(_state(t=step * 0.1, x=x), mission, dt=0.1)

    assert driver.complete
    last_cmd = driver.update(_state(t=10.0, x=6.0), mission, dt=0.1)
    assert last_cmd.throttle == 0.0
    assert last_cmd.brake > 0.0


# ---------- filter integration ----------


def test_filter_advance_sees_last_commanded_action():
    """After a non-trivial update, _last_cmd should equal the returned command
    so the next predict step uses the right propagator input."""
    driver = _seeded_driver()
    mission = _mission((10.0, 0.0))
    cmd1 = driver.update(_state(), mission, dt=0.05)
    assert driver._last_cmd == cmd1
    # And the EnKF has advanced (ensemble count unchanged but mean shifted)
    assert driver.enkf.initialized


def test_stale_observation_not_double_counted():
    """The runner re-presents the same rover_state between GNSS ticks. The
    driver must not fuse the same observation twice, or the filter will
    artificially over-weight a stale fix."""
    driver = _seeded_driver()
    mission = _mission((10.0, 0.0))

    # First update at t=0 initializes the filter from the observation
    s0 = _state(t=0.0, x=1.0, y=0.0)
    driver.update(s0, mission, dt=0.05)
    assert driver._last_obs_t == 0.0

    # Re-presenting the same observation (same state.t) must not re-fuse.
    driver.update(s0, mission, dt=0.05)
    assert driver._last_obs_t == 0.0

    # A fresh observation at a later t advances the fusion bookkeeping.
    s1 = _state(t=0.1, x=1.05, y=0.0)
    driver.update(s1, mission, dt=0.05)
    assert driver._last_obs_t == pytest.approx(0.1)


# ---------- reset ----------


def test_reset_rewinds_all_state():
    driver = _seeded_driver()
    mission = _mission((5.0, 0.0), (10.0, 0.0))
    for step in range(20):
        driver.update(_state(t=step * 0.1, x=step * 0.5), mission, dt=0.1)

    driver.reset()
    assert driver.active_waypoint_index == 0
    assert not driver.complete
    assert not driver.enkf.initialized
    assert driver._last_cmd is None
