"""Rosette crossing maneuver for radar waypoint surveys.

The radar antenna is in the sled trailing behind the rover. To measure at a
specific waypoint, the sled must pass over the point. A rosette makes the
sled cross the waypoint in the arrival direction, loop around, and cross it
again in the departure direction — giving two radar measurements at different
azimuths at the same surface location.

    Arrival →  ========●========→  sled crosses waypoint
                       |
                       ↻ loop (radius = R_min)
                       |
    Departure  ←=======●========  sled crosses waypoint again on new heading
"""

import math
from dataclasses import dataclass
from typing import Optional

from rover_drive.planning.dubins import dubins_path


@dataclass(frozen=True)
class RosetteManeuver:
    """A computed rosette crossing pattern at a waypoint."""

    waypoint: tuple[float, float]
    arrival_heading: float
    departure_heading: float
    radius: float
    loop_points: list[tuple[float, float, float]]  # sampled (x, y, heading)
    overshoot_m: float


def rosette_waypoints(
    waypoint: tuple[float, float],
    arrival_heading: float,
    departure_heading: float,
    radius: float,
    overshoot_m: float = 3.0,
    sample_spacing: float = 0.5,
) -> Optional[RosetteManeuver]:
    """
    Compute a rosette crossing pattern at a waypoint.

    The rover:
      1. Drives past the waypoint by ``overshoot_m`` on the arrival heading.
      2. Executes a Dubins turn from (overshoot_point, arrival_heading) to
         (approach_point, departure_heading), where approach_point is
         ``overshoot_m`` before the waypoint on the departure heading.
      3. Drives through the waypoint on the departure heading.

    The result is a set of intermediate waypoints that, when followed by the
    path-tracking controller, produce the rosette crossing pattern. The sled
    passes over the waypoint twice: once on arrival and once on departure.

    Returns None if no feasible Dubins turn exists (shouldn't happen for R > 0
    and sufficient overshoot, but guarded).
    """
    wx, wy = waypoint
    ah = arrival_heading
    dh = departure_heading

    overshoot_x = wx + overshoot_m * math.cos(ah)
    overshoot_y = wy + overshoot_m * math.sin(ah)

    approach_x = wx - overshoot_m * math.cos(dh)
    approach_y = wy - overshoot_m * math.sin(dh)

    start_pose = (overshoot_x, overshoot_y, ah)
    end_pose = (approach_x, approach_y, dh)

    path = dubins_path(start_pose, end_pose, radius)
    if path is None:
        return None

    loop_points = path.sample(spacing=sample_spacing)

    return RosetteManeuver(
        waypoint=waypoint,
        arrival_heading=ah,
        departure_heading=dh,
        radius=radius,
        loop_points=loop_points,
        overshoot_m=overshoot_m,
    )
