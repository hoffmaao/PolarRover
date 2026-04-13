import math
from dataclasses import dataclass
from typing import Optional

from rover_sim.control.command import CommandBus, Direction, DirectionMode
from rover_sim.vehicle.base import Vehicle, VehicleState


@dataclass
class SingleTrackArticulatedConfig:
    """
    Kinematic / hardware parameters for the MTT-154 in single-track mode
    pulling a towed accessory (sled, trailer, or radar rig). `arm_length_m`
    is the free parameter the user tunes per sled — it is the effective
    distance from the MTT articulated hitch to the sled center of resistance.
    """

    arm_length_m: float = 2.5
    max_speed_mps: float = 5.56  # 20 km/h hardware cap
    max_gamma_rad: float = math.radians(45.0)
    max_gamma_rate_rad_s: float = math.radians(30.0)
    closed_loop_gain: float = 3.0
    accel_mps2: float = 1.5
    brake_decel_mps2: float = 4.0


class SingleTrackArticulated(Vehicle):
    """
    Bicycle-style kinematic model for an MTT-154 pulling an articulated towed
    accessory. State is (x, y, heading, gamma); yaw rate emerges from the
    current speed and articulation angle via theta_dot = v / L * tan(gamma),
    with L = arm_length_m.

    The model handles both direction modes from the MTT Option menu:
      OPEN_LOOP  — steer in [-1, 1] is gamma_dot (commanded cylinder rate),
                   so the cylinder holds its position when the stick is at 0.
      CLOSED_LOOP — steer in [-1, 1] is the normalized target gamma; a simple
                    proportional controller slews the cylinder toward it,
                    clamped by max_gamma_rate_rad_s.

    At zero speed, the yaw rate is zero regardless of gamma, which matches the
    physical behavior of an articulated vehicle: you can command an articulation
    angle while stopped, but heading only changes once the vehicle moves.
    """

    def __init__(self, config: Optional[SingleTrackArticulatedConfig] = None) -> None:
        self.config = config or SingleTrackArticulatedConfig()
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

        if cmd.estop_machine or cmd.estop_handle:
            new_speed = _decelerate(s.speed, cfg.brake_decel_mps2, dt)
            return self._integrate(new_speed, s.gamma, dt)

        if cmd.neutral:
            target_v = 0.0
        else:
            sign = 1.0 if cmd.direction == Direction.FORWARD else -1.0
            target_v = sign * cmd.throttle * cfg.max_speed_mps

        new_speed = _first_order(s.speed, target_v, cfg.accel_mps2, dt)
        if cmd.brake > 0:
            new_speed = _decelerate(new_speed, cfg.brake_decel_mps2 * cmd.brake, dt)
        new_speed = _clamp(new_speed, -cfg.max_speed_mps, cfg.max_speed_mps)

        if cmd.direction_mode == DirectionMode.OPEN_LOOP:
            gamma_rate = cmd.steer * cfg.max_gamma_rate_rad_s
        else:
            target_gamma = cmd.steer * cfg.max_gamma_rad
            gamma_rate = cfg.closed_loop_gain * (target_gamma - s.gamma)
            gamma_rate = _clamp(
                gamma_rate, -cfg.max_gamma_rate_rad_s, cfg.max_gamma_rate_rad_s
            )

        new_gamma = _clamp(s.gamma + gamma_rate * dt, -cfg.max_gamma_rad, cfg.max_gamma_rad)

        return self._integrate(new_speed, new_gamma, dt)

    def _integrate(self, new_speed: float, new_gamma: float, dt: float) -> VehicleState:
        cfg = self.config
        s = self._state

        v_avg = 0.5 * (s.speed + new_speed)
        yaw_rate = (
            (v_avg / cfg.arm_length_m) * math.tan(new_gamma)
            if cfg.arm_length_m > 0
            else 0.0
        )

        new_heading = s.heading + yaw_rate * dt
        h_avg = 0.5 * (s.heading + new_heading)
        new_x = s.x + v_avg * math.cos(h_avg) * dt
        new_y = s.y + v_avg * math.sin(h_avg) * dt

        self._state = VehicleState(
            x=new_x,
            y=new_y,
            heading=new_heading,
            speed=new_speed,
            yaw_rate=yaw_rate,
            gamma=new_gamma,
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
