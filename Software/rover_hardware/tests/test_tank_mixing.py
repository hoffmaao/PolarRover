import pytest

from rover_sim.control.command import CommandBus, Direction, WinchCommand

from rover_hardware.mtt154.tank import TankMixConfig, mix_differential


def test_straight_forward_both_sides_equal():
    cmd = CommandBus(throttle=0.5, steer=0.0, direction=Direction.FORWARD)
    left, right = mix_differential(cmd, TankMixConfig())

    assert left.throttle == pytest.approx(0.5)
    assert right.throttle == pytest.approx(0.5)
    assert left.direction == Direction.FORWARD
    assert right.direction == Direction.FORWARD


def test_straight_reverse_both_sides_equal():
    cmd = CommandBus(throttle=0.3, steer=0.0, direction=Direction.REVERSE)
    left, right = mix_differential(cmd, TankMixConfig())

    assert left.throttle == pytest.approx(0.3)
    assert right.throttle == pytest.approx(0.3)
    assert left.direction == Direction.REVERSE
    assert right.direction == Direction.REVERSE


def test_pivot_turn_opposite_directions():
    """Pure steer at zero throttle should pivot: left reverses, right forwards
    for a positive (left-turning) steer command."""
    cmd = CommandBus(throttle=0.0, steer=1.0)
    left, right = mix_differential(cmd, TankMixConfig())

    assert left.direction == Direction.REVERSE
    assert right.direction == Direction.FORWARD
    assert left.throttle == pytest.approx(0.5)
    assert right.throttle == pytest.approx(0.5)


def test_forward_turn_asymmetric_throttle():
    """Forward throttle plus left steer: right side faster than left, both
    still moving forward."""
    cmd = CommandBus(throttle=0.8, steer=0.4, direction=Direction.FORWARD)
    left, right = mix_differential(cmd, TankMixConfig())

    assert left.direction == Direction.FORWARD
    assert right.direction == Direction.FORWARD
    assert right.throttle > left.throttle


def test_per_side_steer_is_zero():
    """Individual MTT units should never see an articulation command in tank
    mode — the turn comes from the differential, not the hitch."""
    cmd = CommandBus(throttle=0.5, steer=0.3, direction=Direction.FORWARD)
    left, right = mix_differential(cmd, TankMixConfig())

    assert left.steer == 0.0
    assert right.steer == 0.0


def test_brake_and_neutral_propagate():
    cmd = CommandBus(throttle=0.0, steer=0.0, brake=0.7, neutral=True)
    left, right = mix_differential(cmd, TankMixConfig())

    assert left.brake == pytest.approx(0.7)
    assert right.brake == pytest.approx(0.7)
    assert left.neutral is True
    assert right.neutral is True


def test_safety_winch_and_headlight_propagate():
    cmd = CommandBus(
        throttle=0.4,
        steer=0.0,
        safety_unlocked=False,
        winch=WinchCommand.OUT,
        headlight=True,
    )
    left, right = mix_differential(cmd, TankMixConfig())

    assert left.safety_unlocked is False
    assert right.safety_unlocked is False
    assert left.winch is WinchCommand.OUT
    assert right.winch is WinchCommand.OUT
    assert left.headlight is True
    assert right.headlight is True
