from dataclasses import dataclass
from enum import Enum


class Direction(Enum):
    FORWARD = "F"
    REVERSE = "R"


class DirectionMode(Enum):
    """Steering-cylinder control mode — mirrors the MTT-154 Option menu setting."""

    OPEN_LOOP = "open"
    CLOSED_LOOP = "closed"


@dataclass(frozen=True)
class CommandBus:
    """
    Commands sent from a Driver to a Vehicle. Mirrors the MTT-154 joystick
    command surface (D1 throttle, D2 brake, D3 steering, D4 F/R, D9 neutral,
    D8/A5 e-stops) so code written against this interface can later drive
    real hardware through a CAN-side backend.

    The meaning of `steer` depends on `direction_mode`:
      - OPEN_LOOP: steer in [-1, 1] is the commanded rate of the steering
        cylinder (gamma-dot). Releasing the stick keeps the cylinder where it is.
      - CLOSED_LOOP: steer in [-1, 1] is the absolute target position of the
        cylinder as a fraction of max travel. Releasing returns it to center.
    Positive steer produces a positive yaw rate (counter-clockwise, left turn)
    when the vehicle is moving forward.
    """

    throttle: float = 0.0
    brake: float = 0.0
    steer: float = 0.0
    direction: Direction = Direction.FORWARD
    neutral: bool = False
    estop_handle: bool = False
    estop_machine: bool = False
    direction_mode: DirectionMode = DirectionMode.OPEN_LOOP

    def __post_init__(self) -> None:
        if not 0.0 <= self.throttle <= 1.0:
            raise ValueError(f"throttle must be in [0, 1], got {self.throttle}")
        if not 0.0 <= self.brake <= 1.0:
            raise ValueError(f"brake must be in [0, 1], got {self.brake}")
        if not -1.0 <= self.steer <= 1.0:
            raise ValueError(f"steer must be in [-1, 1], got {self.steer}")

    @classmethod
    def zero(cls) -> "CommandBus":
        return cls()
