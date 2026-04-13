"""Stanley path controller — cross-track + heading error formulation.

Reference: Snider, J.M. (2009). "Automatic Steering Methods for Autonomous
Automobile Path Tracking." CMU-RI-TR-09-08.

The Stanley controller was used by Stanford's DARPA Grand Challenge vehicle.
It combines heading error (align with the path tangent) and cross-track error
(converge back to the path) in a single steering law:

    steer = heading_error + atan(k * cross_track_error / (speed + k_soft))

where k is the cross-track gain and k_soft prevents division by zero at low
speed. This directly targets cross-track minimization, making it well-suited
for multipass repeat-track surveys where path fidelity is the primary metric.
"""

import math
from dataclasses import dataclass

from rover_sim.missions import Waypoint
from rover_drive.controllers.base import ControlOutput, PathController


@dataclass
class StanleyConfig:
    k_crosstrack: float = 2.0         # cross-track gain
    k_soft: float = 0.5              # softening term to prevent div-by-zero
    cruise_throttle: float = 0.25
    approach_throttle: float = 0.10
    heading_saturation_rad: float = math.radians(50.0)
    slowdown_radius_m: float = 3.0


class StanleyController(PathController):
    """
    Stanley controller: steers proportional to heading error from the path
    tangent PLUS an arctan term that pulls the rover back toward the path
    when cross-track error builds up. The cross-track gain ``k_crosstrack``
    can be derived from the calibrated turn radius: higher k means more
    aggressive correction, which is safe when R_min is small.
    """

    def __init__(self, config: StanleyConfig | None = None) -> None:
        self.config = config or StanleyConfig()

    def compute(self, rx, ry, rh, rv, path, closest_idx):
        cfg = self.config

        # Find closest point and signed cross-track error
        ci, signed_ct = _signed_cross_track(rx, ry, rh, path, closest_idx)

        # Path tangent heading at the closest point
        tangent_h = _tangent_heading(path, ci)

        # Heading error: difference between rover heading and path tangent
        heading_err = _wrap(tangent_h - rh)

        # Stanley steering law
        ct_term = math.atan2(cfg.k_crosstrack * signed_ct, abs(rv) + cfg.k_soft)
        raw_steer = heading_err + ct_term
        steer = _clamp(raw_steer / cfg.heading_saturation_rad, -1, 1)

        # Throttle with slowdown near end
        dist_to_end = _remaining(path, ci, rx, ry)
        if dist_to_end < cfg.slowdown_radius_m:
            frac = dist_to_end / cfg.slowdown_radius_m
            throttle = cfg.approach_throttle + frac * (cfg.cruise_throttle - cfg.approach_throttle)
        else:
            throttle = cfg.cruise_throttle

        return ControlOutput(steer=steer, throttle=throttle, cross_track_m=abs(signed_ct))


def _signed_cross_track(rx, ry, rh, path, hint):
    """Signed cross-track: positive = rover is left of path, negative = right."""
    best_idx = hint
    best_d2 = (path[hint].x - rx)**2 + (path[hint].y - ry)**2
    for i in range(max(0, hint - 3), min(len(path), hint + 15)):
        d2 = (path[i].x - rx)**2 + (path[i].y - ry)**2
        if d2 < best_d2:
            best_d2 = d2
            best_idx = i

    cp = path[best_idx]
    dx = cp.x - rx
    dy = cp.y - ry
    th = _tangent_heading(path, best_idx)
    # Cross product of tangent × (point - rover) gives signed distance
    sign = math.cos(th) * dy - math.sin(th) * dx
    return best_idx, math.copysign(math.sqrt(best_d2), sign)


def _tangent_heading(path, idx):
    if idx < len(path) - 1:
        dx = path[idx + 1].x - path[idx].x
        dy = path[idx + 1].y - path[idx].y
    elif idx > 0:
        dx = path[idx].x - path[idx - 1].x
        dy = path[idx].y - path[idx - 1].y
    else:
        return 0.0
    return math.atan2(dy, dx)


def _remaining(path, idx, rx, ry):
    if idx >= len(path) - 1:
        return math.hypot(path[-1].x - rx, path[-1].y - ry)
    total = math.hypot(path[idx].x - rx, path[idx].y - ry)
    for i in range(idx, len(path) - 1):
        total += math.hypot(path[i+1].x - path[i].x, path[i+1].y - path[i].y)
    return total


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))
