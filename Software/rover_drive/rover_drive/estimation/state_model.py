"""Forward dynamics model used inside the rover_drive EnKF."""

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from rover_sim.control import CommandBus, Direction, DirectionMode

# State vector layout shared across the estimation package
STATE_X = 0
STATE_Y = 1
STATE_HEADING = 2
STATE_SPEED = 3
STATE_YAW_RATE = 4
STATE_DIM = 5


@dataclass
class BicycleModelConfig:
    """
    Parameters for the bicycle-style forward dynamics model the EnKF uses to
    propagate ensemble members. These describe the filter's *expectation* of
    rover motion; they do not need to exactly match rover_sim's kinematics
    because the EnKF's process noise absorbs model mismatch.
    """

    arm_length_m: float = 2.5
    max_speed_mps: float = 5.56
    max_gamma_rad: float = math.radians(45.0)
    speed_time_constant_s: float = 0.5
    yaw_time_constant_s: float = 0.3


class BicycleStateModel:
    """
    Forward dynamics model for a single-track articulated rover. Given a state
    vector ``[x, y, heading, speed, yaw_rate]`` and (optionally) the last
    commanded ``CommandBus``, propagate the state by ``dt`` seconds.

    When no command is available, the model uses a persistence assumption
    (speed and yaw rate held constant), relying on process noise and
    measurement updates in the EnKF to correct the drift. When a command is
    available the model uses a first-order relaxation toward the target speed
    and (in closed-loop direction mode) the target yaw rate implied by the
    commanded steering value. This mirrors the behavior of rover_sim's vehicle
    model closely enough that the filter converges quickly in practice.
    """

    def __init__(self, config: Optional[BicycleModelConfig] = None) -> None:
        self.config = config or BicycleModelConfig()

    def propagate(
        self,
        x: np.ndarray,
        cmd: Optional[CommandBus],
        dt: float,
    ) -> np.ndarray:
        if dt <= 0.0:
            return x.copy()

        cfg = self.config
        new = x.copy()

        # ---- speed update ----
        if cmd is not None:
            if cmd.estop_machine or cmd.estop_handle or cmd.neutral:
                target_speed = 0.0
            else:
                sign = 1.0 if cmd.direction is Direction.FORWARD else -1.0
                target_speed = sign * cmd.throttle * cfg.max_speed_mps
            alpha_v = min(1.0, dt / cfg.speed_time_constant_s)
            new[STATE_SPEED] = x[STATE_SPEED] + alpha_v * (target_speed - x[STATE_SPEED])
        # else: persistence — new[STATE_SPEED] == x[STATE_SPEED] already

        # ---- yaw rate update ----
        if cmd is not None:
            if cmd.estop_machine or cmd.estop_handle or cmd.neutral:
                target_yaw_rate = 0.0
            elif cmd.direction_mode is DirectionMode.CLOSED_LOOP and abs(new[STATE_SPEED]) > 1e-6:
                target_gamma = cmd.steer * cfg.max_gamma_rad
                if cfg.arm_length_m > 0.0:
                    target_yaw_rate = (new[STATE_SPEED] / cfg.arm_length_m) * math.tan(target_gamma)
                else:
                    target_yaw_rate = 0.0
            else:
                # Open loop or stopped: the filter's best guess is persistence
                target_yaw_rate = x[STATE_YAW_RATE]
            alpha_w = min(1.0, dt / cfg.yaw_time_constant_s)
            new[STATE_YAW_RATE] = x[STATE_YAW_RATE] + alpha_w * (
                target_yaw_rate - x[STATE_YAW_RATE]
            )
        # else: persistence

        # ---- integrate heading and position (midpoint method) ----
        v_avg = 0.5 * (x[STATE_SPEED] + new[STATE_SPEED])
        w_avg = 0.5 * (x[STATE_YAW_RATE] + new[STATE_YAW_RATE])
        new[STATE_HEADING] = x[STATE_HEADING] + w_avg * dt
        h_avg = 0.5 * (x[STATE_HEADING] + new[STATE_HEADING])
        new[STATE_X] = x[STATE_X] + v_avg * math.cos(h_avg) * dt
        new[STATE_Y] = x[STATE_Y] + v_avg * math.sin(h_avg) * dt

        return new
