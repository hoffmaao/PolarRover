import math

import numpy as np
import pytest

from rover_sim.control import CommandBus, Direction, DirectionMode

from rover_drive.estimation import (
    STATE_HEADING,
    STATE_SPEED,
    STATE_X,
    STATE_Y,
    STATE_YAW_RATE,
    BicycleModelConfig,
    BicycleStateModel,
)


def test_straight_line_no_command_persists_velocity():
    model = BicycleStateModel()
    x = np.array([0.0, 0.0, 0.0, 1.0, 0.0])
    out = model.propagate(x, cmd=None, dt=1.0)

    assert out[STATE_X] == pytest.approx(1.0, rel=1e-9)
    assert out[STATE_Y] == pytest.approx(0.0, abs=1e-9)
    assert out[STATE_SPEED] == pytest.approx(1.0, rel=1e-9)
    assert out[STATE_YAW_RATE] == pytest.approx(0.0, abs=1e-9)


def test_closed_loop_steer_creates_positive_yaw_rate():
    model = BicycleStateModel(
        BicycleModelConfig(
            arm_length_m=2.0,
            max_gamma_rad=math.radians(30.0),
            yaw_time_constant_s=0.1,
            speed_time_constant_s=0.1,
        )
    )
    x = np.array([0.0, 0.0, 0.0, 1.0, 0.0])
    cmd = CommandBus(throttle=0.2, steer=1.0, direction_mode=DirectionMode.CLOSED_LOOP)
    for _ in range(20):
        x = model.propagate(x, cmd, dt=0.05)

    assert x[STATE_YAW_RATE] > 0.0
    assert x[STATE_HEADING] > 0.0


def test_estop_relaxes_speed_toward_zero():
    model = BicycleStateModel(
        BicycleModelConfig(speed_time_constant_s=0.1)
    )
    x = np.array([0.0, 0.0, 0.0, 2.0, 0.0])
    cmd = CommandBus(estop_machine=True)
    for _ in range(50):
        x = model.propagate(x, cmd, dt=0.05)

    assert abs(x[STATE_SPEED]) < 0.01


def test_reverse_direction_produces_negative_speed():
    model = BicycleStateModel(BicycleModelConfig(speed_time_constant_s=0.05))
    x = np.zeros(5)
    cmd = CommandBus(throttle=1.0, direction=Direction.REVERSE)
    for _ in range(50):
        x = model.propagate(x, cmd, dt=0.05)

    assert x[STATE_SPEED] < -5.0


def test_zero_dt_returns_unchanged_state():
    model = BicycleStateModel()
    x = np.array([1.0, 2.0, 0.5, 1.0, 0.1])
    out = model.propagate(x, cmd=None, dt=0.0)
    np.testing.assert_array_equal(out, x)


def test_zero_speed_closed_loop_steer_holds_yaw_rate():
    # At zero speed, closed-loop steer should not produce a yaw rate (the
    # bicycle model's yaw depends on forward motion).
    model = BicycleStateModel()
    x = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    cmd = CommandBus(throttle=0.0, steer=1.0, direction_mode=DirectionMode.CLOSED_LOOP)
    out = model.propagate(x, cmd, dt=0.1)
    assert out[STATE_YAW_RATE] == pytest.approx(0.0, abs=1e-9)
