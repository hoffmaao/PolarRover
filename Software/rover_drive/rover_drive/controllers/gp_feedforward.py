"""GP-learned feedforward + Stanley feedback controller.

Instead of precomputing curvature from the reference path geometry (noisy),
this controller uses the GP-learned steer→curvature mapping to compute the
feedforward. The GP knows the rover's ACTUAL steering response (from
calibration), so the feedforward is calibrated to the real vehicle.

The feedback is the same Stanley formulation (heading error + cross-track).

This is the GP-MPC concept, simplified: instead of optimizing a steer sequence
over a horizon (slow), we compute a single feedforward steer from the path
curvature using the GP's inverse model, and add Stanley feedback. This runs
at microsecond speed instead of the GP-MPC's milliseconds.

    δ = GP_inverse(κ(s)) + heading_error + atan(k · cross_track / (v + k_soft))
"""

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from rover_sim.surveys import Waypoint
from rover_drive.controllers.base import ControlOutput, PathController
from rover_drive.learning.gp_dynamics import GPDynamicsModel


@dataclass
class GPFeedforwardConfig:
    k_crosstrack: float = 1.5
    k_soft: float = 0.3
    ff_blend: float = 0.8
    smooth_window: int = 15
    cruise_throttle: float = 0.25
    approach_throttle: float = 0.10
    heading_saturation_rad: float = math.radians(50.0)
    slowdown_radius_m: float = 3.0


class GPFeedforwardController(PathController):
    """
    GP-learned feedforward + Stanley feedback. The GP's learned curvature
    model replaces the analytic arctan(L*κ) feedforward with a data-driven
    one that accounts for snow slip, sled drag, and other real-world effects
    the analytic model misses.

    Call ``observe()`` after each step to continue online learning.
    """

    def __init__(
        self,
        gp_model: GPDynamicsModel,
        config: Optional[GPFeedforwardConfig] = None,
    ) -> None:
        self.config = config or GPFeedforwardConfig()
        self.gp_model = gp_model
        self._curvatures: Optional[np.ndarray] = None
        self._tangent_headings: Optional[np.ndarray] = None
        self._steer_for_curvature_cache: Optional[np.ndarray] = None
        self._obs_since_cache: int = 0

    def observe(self, steer: float, speed: float, yaw_rate: float) -> None:
        """Feed driving data into the GP for online learning."""
        self.gp_model.observe(steer, speed, yaw_rate)
        self._obs_since_cache += 1
        if self._obs_since_cache >= 50:
            self._steer_for_curvature_cache = None  # rebuild periodically, not every step
            self._obs_since_cache = 0

    def precompute_path(self, path: list[Waypoint]) -> None:
        """Compute smoothed curvature and tangent headings for the path."""
        n = len(path)
        raw_curv = np.zeros(n)
        self._tangent_headings = np.zeros(n)
        for i in range(n):
            self._tangent_headings[i] = _tangent_heading(path, i)
            raw_curv[i] = _curvature_at(path, i)

        w = self.config.smooth_window
        if w > 1 and n > w:
            kernel = np.ones(w) / w
            self._curvatures = np.convolve(raw_curv, kernel, mode='same')
        else:
            self._curvatures = raw_curv

    def _build_ff_cache(self) -> None:
        """Use the GP's inverse model to find the steer that produces each path curvature."""
        if self._curvatures is None or not self.gp_model.fitted:
            return

        # For each path curvature, find the steer that produces it via GP lookup
        # GP maps: steer → curvature. We need: curvature → steer (inverse).
        # Approach: evaluate GP on a grid of steers, then interpolate.
        steer_grid = np.linspace(-0.8, 0.8, 50)
        curv_means, _ = self.gp_model.predict_curvature_batch(steer_grid)

        # For each path curvature, find the closest GP-predicted curvature
        n = len(self._curvatures)
        self._steer_for_curvature_cache = np.zeros(n)
        for i in range(n):
            target_curv = self._curvatures[i]
            idx = np.argmin(np.abs(curv_means - target_curv))
            self._steer_for_curvature_cache[i] = steer_grid[idx]

    def compute(self, rx, ry, rh, rv, path, closest_idx):
        cfg = self.config

        if self._curvatures is None:
            self.precompute_path(path)

        if self._steer_for_curvature_cache is None and self.gp_model.fitted:
            self._build_ff_cache()

        ci, signed_ct = _signed_cross_track(rx, ry, rh, path, closest_idx)

        # Feedforward from GP inverse model (or zero if GP not ready)
        if self._steer_for_curvature_cache is not None:
            delta_ff = cfg.ff_blend * self._steer_for_curvature_cache[ci]
        else:
            delta_ff = 0.0

        # Stanley feedback (same sign convention as working stanley.py)
        tangent_h = float(self._tangent_headings[ci])
        heading_err = _wrap(tangent_h - rh)
        ct_term = math.atan2(cfg.k_crosstrack * signed_ct, abs(rv) + cfg.k_soft)

        delta_total = delta_ff + heading_err + ct_term
        steer = _clamp(delta_total / cfg.heading_saturation_rad, -1.0, 1.0)

        dist_to_end = _remaining(path, ci, rx, ry)
        if dist_to_end < cfg.slowdown_radius_m:
            frac = dist_to_end / cfg.slowdown_radius_m
            throttle = cfg.approach_throttle + frac * (cfg.cruise_throttle - cfg.approach_throttle)
        else:
            throttle = cfg.cruise_throttle

        return ControlOutput(steer=steer, throttle=throttle, cross_track_m=abs(signed_ct))


def _signed_cross_track(rx, ry, rh, path, hint):
    best_idx = hint
    best_d2 = (path[hint].x - rx)**2 + (path[hint].y - ry)**2
    for i in range(max(0, hint - 5), min(len(path), hint + 20)):
        d2 = (path[i].x - rx)**2 + (path[i].y - ry)**2
        if d2 < best_d2:
            best_d2 = d2
            best_idx = i
    cp = path[best_idx]
    dx, dy = cp.x - rx, cp.y - ry
    th = _tangent_heading(path, best_idx)
    sign = math.cos(th) * dy - math.sin(th) * dx
    return best_idx, math.copysign(math.sqrt(best_d2), sign)


def _curvature_at(path, idx):
    n = len(path)
    if idx <= 0 or idx >= n - 1:
        return 0.0
    p0, p1, p2 = path[idx-1], path[idx], path[idx+1]
    ax, ay = p1.x - p0.x, p1.y - p0.y
    bx, by = p2.x - p1.x, p2.y - p1.y
    cross = ax * by - ay * bx
    da, db = math.hypot(ax, ay), math.hypot(bx, by)
    if da < 1e-9 or db < 1e-9:
        return 0.0
    return 2.0 * cross / (da * db * (da + db))


def _tangent_heading(path, idx):
    if idx < len(path) - 1:
        return math.atan2(path[idx+1].y - path[idx].y, path[idx+1].x - path[idx].x)
    elif idx > 0:
        return math.atan2(path[idx].y - path[idx-1].y, path[idx].x - path[idx-1].x)
    return 0.0


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
