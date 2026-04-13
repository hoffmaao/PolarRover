"""Base interface for pluggable path-following controllers."""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

from rover_sim.missions import Waypoint


@dataclass(frozen=True)
class ControlOutput:
    """What a path controller produces each step."""
    steer: float          # [-1, 1] normalized
    throttle: float       # [0, 1]
    cross_track_m: float  # signed cross-track error (+ = left of path)


class PathController(ABC):
    """
    A path controller takes the rover's fused state and a reference path,
    and produces a steer + throttle command that minimizes cross-track error.
    Different algorithms (pure pursuit, Stanley, MPC) implement this interface
    so they can be swapped in the multipass driver and compared.
    """

    @abstractmethod
    def compute(
        self,
        rx: float, ry: float, rh: float, rv: float,
        path: list[Waypoint],
        closest_idx: int,
    ) -> ControlOutput:
        """
        Compute steer + throttle given:
          rx, ry, rh, rv — rover fused position, heading, speed
          path — the reference waypoint list
          closest_idx — hint for the nearest path point
        """
        ...
