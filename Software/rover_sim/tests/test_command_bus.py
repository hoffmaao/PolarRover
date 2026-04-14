import pytest

from rover_sim.control import (
    CommandBus,
    Direction,
    DirectionMode,
    WinchCommand,
)


def test_zero_command_defaults():
    c = CommandBus.zero()
    assert c.throttle == 0.0
    assert c.brake == 0.0
    assert c.steer == 0.0
    assert c.direction is Direction.FORWARD
    assert c.neutral is False
    assert c.estop_handle is False
    assert c.estop_machine is False
    assert c.direction_mode is DirectionMode.OPEN_LOOP
    assert c.safety_unlocked is True
    assert c.winch is WinchCommand.NEUTRAL
    assert c.headlight is False


def test_throttle_out_of_range_rejected():
    with pytest.raises(ValueError):
        CommandBus(throttle=1.5)
    with pytest.raises(ValueError):
        CommandBus(throttle=-0.1)


def test_brake_out_of_range_rejected():
    with pytest.raises(ValueError):
        CommandBus(brake=1.5)
    with pytest.raises(ValueError):
        CommandBus(brake=-0.1)


def test_steer_out_of_range_rejected():
    with pytest.raises(ValueError):
        CommandBus(steer=1.5)
    with pytest.raises(ValueError):
        CommandBus(steer=-1.5)


def test_is_frozen():
    c = CommandBus(throttle=0.5)
    with pytest.raises(Exception):
        c.throttle = 0.7  # type: ignore[misc]


def test_safety_locked_is_settable():
    c = CommandBus(throttle=0.5, safety_unlocked=False)
    assert c.safety_unlocked is False
    assert c.throttle == 0.5


def test_winch_states():
    assert CommandBus(winch=WinchCommand.IN).winch is WinchCommand.IN
    assert CommandBus(winch=WinchCommand.OUT).winch is WinchCommand.OUT
    assert CommandBus(winch=WinchCommand.NEUTRAL).winch is WinchCommand.NEUTRAL


def test_headlight_defaults_off_and_can_toggle():
    assert CommandBus().headlight is False
    assert CommandBus(headlight=True).headlight is True
