import pytest

from rover_sim.control import CommandBus, Direction, DirectionMode


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
