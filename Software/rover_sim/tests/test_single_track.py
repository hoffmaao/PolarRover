import math

import pytest

from rover_sim.control import CommandBus, Direction, DirectionMode
from rover_sim.vehicle import SingleTrackArticulated, SingleTrackArticulatedConfig


def _run(vehicle: SingleTrackArticulated, cmd: CommandBus, steps: int, dt: float = 0.01) -> None:
    for _ in range(steps):
        vehicle.step(cmd, dt)


def test_straight_line_constant_velocity():
    cfg = SingleTrackArticulatedConfig(max_speed_mps=2.0, accel_mps2=100.0)
    v = SingleTrackArticulated(cfg)
    _run(v, CommandBus(throttle=1.0), steps=1000)

    s = v.state
    assert abs(s.y) < 1e-6
    assert abs(s.heading) < 1e-9
    assert abs(s.gamma) < 1e-9
    assert s.x == pytest.approx(20.0, rel=0.02)
    assert s.speed == pytest.approx(2.0, rel=1e-6)


def test_closed_loop_constant_turn_matches_analytical_radius():
    cfg = SingleTrackArticulatedConfig(
        arm_length_m=2.0,
        max_speed_mps=1.0,
        accel_mps2=100.0,
        max_gamma_rad=math.radians(30.0),
        max_gamma_rate_rad_s=math.radians(1000.0),
        closed_loop_gain=50.0,
    )
    v = SingleTrackArticulated(cfg)
    cmd = CommandBus(throttle=1.0, steer=1.0, direction_mode=DirectionMode.CLOSED_LOOP)
    _run(v, cmd, steps=1010)

    expected_R = cfg.arm_length_m / math.tan(cfg.max_gamma_rad)
    s = v.state
    assert abs(s.yaw_rate) > 0
    R = abs(s.speed) / abs(s.yaw_rate)
    assert R == pytest.approx(expected_R, rel=0.02)
    assert s.gamma == pytest.approx(cfg.max_gamma_rad, rel=1e-3)


def test_zero_speed_no_heading_change_even_with_gamma():
    cfg = SingleTrackArticulatedConfig(
        arm_length_m=2.0,
        max_gamma_rad=math.radians(30.0),
        max_gamma_rate_rad_s=math.radians(1000.0),
        closed_loop_gain=50.0,
    )
    v = SingleTrackArticulated(cfg)
    cmd = CommandBus(steer=1.0, direction_mode=DirectionMode.CLOSED_LOOP)
    _run(v, cmd, steps=200)

    s = v.state
    assert s.gamma == pytest.approx(cfg.max_gamma_rad, rel=1e-3)
    assert abs(s.heading) < 1e-12
    assert abs(s.x) < 1e-12
    assert abs(s.y) < 1e-12


def test_forward_stop_reverse_sequence():
    cfg = SingleTrackArticulatedConfig(max_speed_mps=2.0, accel_mps2=100.0)
    v = SingleTrackArticulated(cfg)

    _run(v, CommandBus(throttle=1.0, direction=Direction.FORWARD), steps=100)
    assert v.state.speed > 1.9

    _run(v, CommandBus(brake=1.0), steps=100)
    assert abs(v.state.speed) < 1e-6

    _run(v, CommandBus(throttle=1.0, direction=Direction.REVERSE), steps=100)
    assert v.state.speed < -1.9


def test_estop_halts_vehicle():
    cfg = SingleTrackArticulatedConfig(max_speed_mps=2.0, accel_mps2=100.0)
    v = SingleTrackArticulated(cfg)

    _run(v, CommandBus(throttle=1.0), steps=100)
    assert v.state.speed > 1.9

    _run(v, CommandBus(estop_machine=True), steps=100)
    assert v.state.speed == pytest.approx(0.0, abs=1e-6)


def test_speed_cap_enforced():
    cfg = SingleTrackArticulatedConfig(max_speed_mps=1.0, accel_mps2=100.0)
    v = SingleTrackArticulated(cfg)
    _run(v, CommandBus(throttle=1.0), steps=500)
    assert v.state.speed == pytest.approx(1.0, rel=1e-6)


def test_reset_restores_zero_state():
    v = SingleTrackArticulated()
    _run(v, CommandBus(throttle=1.0), steps=100)
    assert v.state.x != 0.0
    v.reset()
    assert v.state.x == 0.0
    assert v.state.speed == 0.0
    assert v.state.gamma == 0.0


def test_safety_locked_prevents_motion():
    cfg = SingleTrackArticulatedConfig(max_speed_mps=2.0, accel_mps2=100.0)
    v = SingleTrackArticulated(cfg)
    _run(v, CommandBus(throttle=1.0, safety_unlocked=False), steps=200)
    assert v.state.speed == pytest.approx(0.0, abs=1e-6)
    assert v.state.x == pytest.approx(0.0, abs=1e-6)
