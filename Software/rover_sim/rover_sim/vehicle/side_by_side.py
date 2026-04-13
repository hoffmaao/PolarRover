import math
from dataclasses import dataclass
from typing import Optional

from rover_sim.control.command import CommandBus, Direction
from rover_sim.vehicle.base import Vehicle, VehicleState


@dataclass
class SideBySideSkidSteerConfig:
    """
    Parameters for two MTT-154 units coupled side-by-side as the tracks of a
    tank (the "Side by side left/right" setting in the Vehicle Type menu).
    """

    track_width_m: float = 0.584  # distance between the two coupled track centerlines
    max_speed_mps: float = 5.56
    accel_mps2: float = 1.5
    brake_decel_mps2: float = 4.0


class SideBySideSkidSteer(Vehicle):
    """
    Differential-drive kinematic model for two MTTs coupled side-by-side.
    Throttle sets the commanded center velocity; steer sets the commanded
    per-track differential so that:
        target_v_left  = v_center - (steer * max_speed) / 2
        target_v_right = v_center + (steer * max_speed) / 2
    Positive steer yields a positive yaw rate (counter-clockwise / left turn),
    matching the convention used by SingleTrackArticulated.
    """

    def __init__(self, config: Optional[SideBySideSkidSteerConfig] = None) -> None:
        self.config = config or SideBySideSkidSteerConfig()
        self._state = VehicleState()

    @property
    def state(self) -> VehicleState:
        return self._state

    def reset(self, initial: Optional[VehicleState] = None) -> None:
        self._state = initial if initial is not None else VehicleState()

    def step(self, cmd: CommandBus, dt: float) -> VehicleState:
        if dt <= 0:
            return self._state

        cfg = self.config
        s = self._state

        stop = cmd.estop_machine or cmd.estop_handle or cmd.neutral
        if stop:
            target_center = 0.0
            target_diff = 0.0
        else:
            sign = 1.0 if cmd.direction == Direction.FORWARD else -1.0
            target_center = sign * cmd.throttle * cfg.max_speed_mps
            target_diff = cmd.steer * cfg.max_speed_mps

        target_vL = target_center - target_diff / 2.0
        target_vR = target_center + target_diff / 2.0

        new_vL = _first_order(s.v_left, target_vL, cfg.accel_mps2, dt)
        new_vR = _first_order(s.v_right, target_vR, cfg.accel_mps2, dt)

        if cmd.brake > 0:
            b = cfg.brake_decel_mps2 * cmd.brake
            new_vL = _decelerate(new_vL, b, dt)
            new_vR = _decelerate(new_vR, b, dt)

        new_vL = _clamp(new_vL, -cfg.max_speed_mps, cfg.max_speed_mps)
        new_vR = _clamp(new_vR, -cfg.max_speed_mps, cfg.max_speed_mps)

        v = 0.5 * (new_vL + new_vR)
        yaw_rate = (new_vR - new_vL) / cfg.track_width_m

        new_heading = s.heading + yaw_rate * dt
        h_avg = 0.5 * (s.heading + new_heading)
        new_x = s.x + v * math.cos(h_avg) * dt
        new_y = s.y + v * math.sin(h_avg) * dt

        self._state = VehicleState(
            x=new_x,
            y=new_y,
            heading=new_heading,
            speed=v,
            yaw_rate=yaw_rate,
            v_left=new_vL,
            v_right=new_vR,
        )
        return self._state


def _first_order(current: float, target: float, rate: float, dt: float) -> float:
    delta = target - current
    max_step = rate * dt
    if abs(delta) <= max_step:
        return target
    return current + math.copysign(max_step, delta)


def _decelerate(speed: float, decel: float, dt: float) -> float:
    if speed == 0.0:
        return 0.0
    step = decel * dt
    if abs(speed) <= step:
        return 0.0
    return speed - math.copysign(step, speed)


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))
