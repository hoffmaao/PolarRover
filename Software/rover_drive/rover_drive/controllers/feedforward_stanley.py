"""Feedforward + Stanley feedback controller for repeat-track path following.

The key insight: on a pre-driven reference track where curvature is known at
every point, feedforward from the curvature should do 90%+ of the steering
work. The feedback (Stanley cross-track + heading error) only handles small
disturbances: GNSS noise, snow slip, wind, sled drag.

    δ = δ_ff(s) + θ_e + atan(k · e / (v + k_soft))

where:
  δ_ff(s) = arctan(L_cal · κ(s))     — feedforward from calibrated model
  θ_e                                  — heading error from path tangent
  atan(k · e / (v + k_soft))          — Stanley cross-track correction

This is computationally trivial (one lookup + one arctan + two multiplies)
and leverages the calibration data directly. The feedforward anticipates
curvature instead of reacting to it, which is why pure pursuit (reactive
only) can't compete on repeat tracks.

References:
  Hoffmann et al. (2007). "Autonomous Automobile Trajectory Tracking for
  Off-Road Driving" — feedforward + Stanley achieving sub-10cm tracking.
  Snider (2009). CMU-RI-TR-09-08 — canonical feedforward/feedback decomposition.
"""

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from rover_sim.surveys import Waypoint
from rover_drive.controllers.base import ControlOutput, PathController


@dataclass
class FeedforwardStanleyConfig:
    arm_length_m: float = 2.5         # calibrated effective arm length
    k_crosstrack: float = 1.5         # Stanley cross-track gain
    k_soft: float = 0.3               # softening at low speed
    ff_blend: float = 0.5             # feedforward blend: 0=pure feedback, 1=full feedforward
    smooth_window: int = 15           # curvature smoothing kernel width (points)
    cruise_throttle: float = 0.25
    approach_throttle: float = 0.10
    heading_saturation_rad: float = math.radians(50.0)
    slowdown_radius_m: float = 3.0


class FeedforwardStanleyController(PathController):
    """
    Feedforward from precomputed path curvature + Stanley feedback for
    disturbance rejection. The feedforward does the heavy lifting (steering
    through known curves); the feedback only corrects small perturbations.
    """

    def __init__(self, config: Optional[FeedforwardStanleyConfig] = None) -> None:
        self.config = config or FeedforwardStanleyConfig()
        self._curvatures: Optional[np.ndarray] = None
        self._arc_lengths: Optional[np.ndarray] = None
        self._tangent_headings: Optional[np.ndarray] = None

    def precompute_path(self, path: list[Waypoint], smooth_window: int = 11) -> None:
        """Precompute curvature, arc-length, and tangent heading along the path.
        Curvature is smoothed with a moving average to suppress noise from
        discretized reference track positions — critical for feedforward stability.
        Call once when the reference track is loaded."""
        n = len(path)
        raw_curvatures = np.zeros(n)
        self._arc_lengths = np.zeros(n)
        self._tangent_headings = np.zeros(n)

        for i in range(n):
            self._tangent_headings[i] = _tangent_heading(path, i)
            raw_curvatures[i] = _curvature_at(path, i)
            if i > 0:
                self._arc_lengths[i] = self._arc_lengths[i-1] + math.hypot(
                    path[i].x - path[i-1].x, path[i].y - path[i-1].y
                )

        # Smooth curvature to suppress noise (position noise / spacing² → curvature noise)
        if smooth_window > 1 and n > smooth_window:
            kernel = np.ones(smooth_window) / smooth_window
            self._curvatures = np.convolve(raw_curvatures, kernel, mode='same')
        else:
            self._curvatures = raw_curvatures

    def compute(self, rx, ry, rh, rv, path, closest_idx):
        cfg = self.config

        if self._curvatures is None:
            self.precompute_path(path, smooth_window=cfg.smooth_window)

        ci, signed_ct = _signed_cross_track(rx, ry, rh, path, closest_idx)

        # Feedforward: steering command that would perfectly track the curvature
        # Blended to avoid amplifying curvature estimation noise
        kappa = float(self._curvatures[ci])
        delta_ff = cfg.ff_blend * math.atan(cfg.arm_length_m * kappa)

        # Feedback: Stanley heading error + cross-track correction
        # Sign convention matches the working StanleyController in stanley.py
        tangent_h = float(self._tangent_headings[ci])
        heading_err = _wrap(tangent_h - rh)  # path_tangent minus rover_heading
        ct_term = math.atan2(cfg.k_crosstrack * signed_ct, abs(rv) + cfg.k_soft)

        # Combined: feedforward + Stanley feedback
        delta_total = delta_ff + heading_err + ct_term
        steer = _clamp(delta_total / cfg.heading_saturation_rad, -1.0, 1.0)

        # Throttle with slowdown near end
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
    """Three-point curvature estimate using the circumscribed circle formula."""
    n = len(path)
    if idx <= 0 or idx >= n - 1:
        return 0.0
    p0, p1, p2 = path[idx-1], path[idx], path[idx+1]
    ax, ay = p1.x - p0.x, p1.y - p0.y
    bx, by = p2.x - p1.x, p2.y - p1.y
    cross = ax * by - ay * bx
    da = math.hypot(ax, ay)
    db = math.hypot(bx, by)
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
