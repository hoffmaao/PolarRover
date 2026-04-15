"""Pure pursuit path controller — the existing baseline, refactored as a pluggable backend."""

import math
from dataclasses import dataclass

from rover_sim.surveys import Waypoint
from rover_drive.controllers.base import ControlOutput, PathController


@dataclass
class PurePursuitConfig:
    lookahead_m: float = 3.0
    cruise_throttle: float = 0.25
    approach_throttle: float = 0.10
    heading_saturation_rad: float = math.radians(40.0)
    slowdown_radius_m: float = 3.0


class PurePursuitController(PathController):
    """Heading-error proportional controller with lookahead on the path."""

    def __init__(self, config: PurePursuitConfig | None = None) -> None:
        self.config = config or PurePursuitConfig()

    def compute(self, rx, ry, rh, rv, path, closest_idx):
        cfg = self.config
        ct_dist = _cross_track(rx, ry, path, closest_idx)
        la_idx = _advance(path, closest_idx, cfg.lookahead_m)
        target = path[la_idx]

        dx = target.x - rx
        dy = target.y - ry
        dist_to_end = _remaining(path, closest_idx, rx, ry)

        alpha = _wrap(math.atan2(dy, dx) - rh)
        steer = _clamp(alpha / cfg.heading_saturation_rad, -1, 1)

        if dist_to_end < cfg.slowdown_radius_m:
            frac = dist_to_end / cfg.slowdown_radius_m
            throttle = cfg.approach_throttle + frac * (cfg.cruise_throttle - cfg.approach_throttle)
        else:
            throttle = cfg.cruise_throttle

        return ControlOutput(steer=steer, throttle=throttle, cross_track_m=ct_dist)


def _cross_track(rx, ry, path, hint):
    best = math.hypot(path[hint].x - rx, path[hint].y - ry)
    for i in range(max(0, hint - 2), min(len(path), hint + 10)):
        d = math.hypot(path[i].x - rx, path[i].y - ry)
        if d < best:
            best = d
    return best


def _advance(path, start, distance):
    remaining = distance
    idx = start
    while idx < len(path) - 1:
        seg = math.hypot(path[idx+1].x - path[idx].x, path[idx+1].y - path[idx].y)
        if seg >= remaining:
            return idx + 1
        remaining -= seg
        idx += 1
    return len(path) - 1


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
