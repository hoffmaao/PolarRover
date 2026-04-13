from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

from rover_sim.control.command import CommandBus


@dataclass(frozen=True)
class VehicleState:
    """
    Complete ground-truth state of a sim vehicle. The Sensors layer (Phase 2)
    derives a noisy, observable RoverState from this.

    Fields used by all models:
      x, y       — ENU position, meters
      heading    — radians, 0 = east, counter-clockwise positive
      speed      — m/s, signed (negative = reverse)
      yaw_rate   — rad/s, instantaneous
    Model-specific extras (0 when not applicable):
      gamma      — articulation angle, rad (single-track articulated only)
      v_left     — left-track velocity, m/s (side-by-side only)
      v_right    — right-track velocity, m/s (side-by-side only)
    """

    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0
    speed: float = 0.0
    yaw_rate: float = 0.0
    gamma: float = 0.0
    v_left: float = 0.0
    v_right: float = 0.0


class Vehicle(ABC):
    """Abstract base for sim vehicle kinematic models."""

    @abstractmethod
    def step(self, cmd: CommandBus, dt: float) -> VehicleState:
        """Integrate forward by dt seconds and return the new state."""

    @abstractmethod
    def reset(self, initial: Optional[VehicleState] = None) -> None:
        """Reset to the given initial state (or a zeroed state if None)."""

    @property
    @abstractmethod
    def state(self) -> VehicleState:
        """Read-only access to the current state."""
