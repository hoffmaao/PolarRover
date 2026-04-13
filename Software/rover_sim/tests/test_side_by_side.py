import pytest

from rover_sim.control import CommandBus, Direction
from rover_sim.vehicle import SideBySideSkidSteer, SideBySideSkidSteerConfig


def _run(vehicle: SideBySideSkidSteer, cmd: CommandBus, steps: int, dt: float = 0.01) -> None:
    for _ in range(steps):
        vehicle.step(cmd, dt)


def test_straight_line():
    cfg = SideBySideSkidSteerConfig(max_speed_mps=2.0, accel_mps2=100.0)
    v = SideBySideSkidSteer(cfg)
    _run(v, CommandBus(throttle=1.0), steps=1000)

    s = v.state
    assert abs(s.y) < 1e-9
    assert s.x == pytest.approx(20.0, rel=0.02)
    assert abs(s.heading) < 1e-9
    assert s.v_left == pytest.approx(2.0)
    assert s.v_right == pytest.approx(2.0)


def test_in_place_rotation_yaw_rate_matches_differential_formula():
    cfg = SideBySideSkidSteerConfig(
        track_width_m=1.0, max_speed_mps=2.0, accel_mps2=100.0
    )
    v = SideBySideSkidSteer(cfg)
    _run(v, CommandBus(steer=1.0), steps=100)

    s = v.state
    # target_vL = -1.0, target_vR = +1.0 -> yaw_rate = (1 - (-1)) / 1.0 = 2.0 rad/s
    # After 1 s, heading ≈ 2.0 rad; center velocity = 0, so x, y ≈ 0.
    assert s.v_left == pytest.approx(-1.0)
    assert s.v_right == pytest.approx(1.0)
    assert s.yaw_rate == pytest.approx(2.0, rel=1e-6)
    assert s.heading == pytest.approx(2.0, rel=0.02)
    assert abs(s.x) < 1e-9
    assert abs(s.y) < 1e-9


def test_reverse_direction():
    cfg = SideBySideSkidSteerConfig(max_speed_mps=2.0, accel_mps2=100.0)
    v = SideBySideSkidSteer(cfg)
    _run(v, CommandBus(throttle=1.0, direction=Direction.REVERSE), steps=500)

    s = v.state
    assert s.speed == pytest.approx(-2.0)
    assert s.x == pytest.approx(-10.0, rel=0.02)


def test_neutral_stops_commands():
    cfg = SideBySideSkidSteerConfig(max_speed_mps=2.0, accel_mps2=100.0)
    v = SideBySideSkidSteer(cfg)
    _run(v, CommandBus(throttle=1.0), steps=100)
    assert v.state.speed > 1.9

    _run(v, CommandBus(neutral=True), steps=100)
    assert abs(v.state.speed) < 1e-6
    assert abs(v.state.v_left) < 1e-6
    assert abs(v.state.v_right) < 1e-6
