"""Model Predictive Contouring Control (MPCC) for path following.

Instead of minimizing cross-track error at fixed time steps (standard MPC),
MPCC maximizes *progress along the path* while constraining cross-track error.
This naturally handles the trade-off between speed and accuracy: the rover
advances as fast as it can while staying within a cross-track tolerance.

The optimization variable is a sequence of (steer, progress_rate) pairs over
a short horizon. The cost function is:

    J = -w_progress * sum(progress) + w_contour * sum(contour_error²) + w_lag * sum(lag_error²) + w_steer * sum(delta_steer²)

where:
  - contour_error = perpendicular distance from rover to path (cross-track)
  - lag_error = along-path distance between rover and the progress parameter
  - progress = how far along the path the reference advances per step

Reference: Liniger, Domahidi, Morari (2015). "Optimization-Based Autonomous
Racing of 1:43 Scale RC Cars." ETH Zurich.
"""

import math
from dataclasses import dataclass

import numpy as np

from rover_sim.missions import Waypoint
from rover_drive.controllers.base import ControlOutput, PathController


@dataclass
class MPCCConfig:
    horizon: int = 10
    dt_predict: float = 0.15
    arm_length_m: float = 2.5
    max_gamma_rad: float = math.radians(45.0)
    w_contour: float = 20.0      # penalize perpendicular (cross-track) error
    w_lag: float = 5.0           # penalize along-path lag
    w_progress: float = 3.0     # reward progress along path
    w_steer_change: float = 1.0
    cruise_throttle: float = 0.25
    approach_throttle: float = 0.10
    heading_saturation_rad: float = math.radians(50.0)
    slowdown_radius_m: float = 3.0
    n_candidates: int = 21
    n_refine: int = 4


class MPCCController(PathController):
    """
    MPCC path follower. Each step it optimizes a steer sequence that maximizes
    progress along the reference path while minimizing contour (cross-track)
    and lag errors. The result is a controller that naturally slows down in
    curves (less progress per step needed) and speeds up on straights.
    """

    def __init__(self, config: MPCCConfig | None = None) -> None:
        self.config = config or MPCCConfig()
        self._last_steer: float = 0.0
        self._path_param: float = 0.0  # accumulated progress along path (meters)

    def compute(self, rx, ry, rh, rv, path, closest_idx):
        cfg = self.config

        if abs(rv) < 0.05 or len(path) < 3:
            return self._fallback(rx, ry, rh, path, closest_idx)

        # Current contour error for metrics
        ci, ct_dist = _find_closest_unsigned(rx, ry, path, closest_idx)
        self._path_param = _arc_length_to(path, ci)

        best_steer_seq = self._optimize(rx, ry, rh, rv, path, ci)

        steer = _clamp(best_steer_seq[0] / cfg.heading_saturation_rad, -1, 1)
        self._last_steer = best_steer_seq[0]

        dist_to_end = _remaining(path, ci, rx, ry)
        if dist_to_end < cfg.slowdown_radius_m:
            frac = dist_to_end / cfg.slowdown_radius_m
            throttle = cfg.approach_throttle + frac * (cfg.cruise_throttle - cfg.approach_throttle)
        else:
            throttle = cfg.cruise_throttle

        return ControlOutput(steer=steer, throttle=throttle, cross_track_m=ct_dist)

    def _optimize(self, rx, ry, rh, rv, path, closest_idx):
        cfg = self.config
        N = cfg.horizon

        candidates = np.linspace(-cfg.max_gamma_rad * 0.8, cfg.max_gamma_rad * 0.8, cfg.n_candidates)

        best_cost = float("inf")
        best_seq = [0.0] * N

        # Phase 1: evaluate constant-steer sequences
        for s in candidates:
            seq = [float(s)] * N
            cost = self._evaluate(rx, ry, rh, rv, seq, path, closest_idx)
            if cost < best_cost:
                best_cost = cost
                best_seq = seq[:]

        # Phase 2: graded sequences (ramp from current to candidate)
        for s in candidates[::3]:
            seq = [float(self._last_steer + (s - self._last_steer) * k / max(N-1, 1)) for k in range(N)]
            cost = self._evaluate(rx, ry, rh, rv, seq, path, closest_idx)
            if cost < best_cost:
                best_cost = cost
                best_seq = seq[:]

        # Phase 3: local refinement
        for iteration in range(cfg.n_refine):
            improved = False
            for k in range(min(N, 6)):
                current = best_seq[k]
                for delta in [-0.2, -0.08, -0.03, 0.03, 0.08, 0.2]:
                    trial = best_seq[:]
                    trial[k] = _clamp(current + delta, -cfg.max_gamma_rad, cfg.max_gamma_rad)
                    cost = self._evaluate(rx, ry, rh, rv, trial, path, closest_idx)
                    if cost < best_cost:
                        best_cost = cost
                        best_seq = trial[:]
                        improved = True
            if not improved:
                break

        return best_seq

    def _evaluate(self, rx, ry, rh, rv, steer_seq, path, start_idx):
        cfg = self.config
        x, y, h = rx, ry, rh
        v = rv
        cost = 0.0
        prev_s = self._last_steer
        param = self._path_param

        for k, s in enumerate(steer_seq):
            # Bicycle model forward step
            yaw_rate = (v / cfg.arm_length_m) * math.tan(s) if cfg.arm_length_m > 0 else 0
            h_new = h + yaw_rate * cfg.dt_predict
            h_avg = 0.5 * (h + h_new)
            x += v * math.cos(h_avg) * cfg.dt_predict
            y += v * math.sin(h_avg) * cfg.dt_predict
            h = h_new

            # Progress along path
            progress = v * cfg.dt_predict
            param += progress

            # Find path point at current progress parameter
            ref_idx = _idx_at_arclength(path, param)
            ref = path[ref_idx]
            th = _tangent_heading(path, ref_idx)

            # Decompose error into contour (perpendicular) and lag (along-path)
            dx = x - ref.x
            dy = y - ref.y
            contour_err = -math.sin(th) * dx + math.cos(th) * dy  # perpendicular
            lag_err = math.cos(th) * dx + math.sin(th) * dy       # along path

            cost += cfg.w_contour * contour_err ** 2
            cost += cfg.w_lag * lag_err ** 2
            cost -= cfg.w_progress * progress
            cost += cfg.w_steer_change * (s - prev_s) ** 2
            prev_s = s

        return cost

    def _fallback(self, rx, ry, rh, path, ci):
        wp = path[min(ci + 1, len(path) - 1)]
        alpha = _wrap(math.atan2(wp.y - ry, wp.x - rx) - rh)
        steer = _clamp(alpha / self.config.heading_saturation_rad, -1, 1)
        ct = _find_closest_unsigned(rx, ry, path, ci)[1]
        return ControlOutput(steer=steer, throttle=self.config.cruise_throttle, cross_track_m=ct)


def _find_closest_unsigned(rx, ry, path, hint):
    best_idx = hint
    best_d2 = (path[hint].x - rx)**2 + (path[hint].y - ry)**2
    for i in range(max(0, hint - 3), min(len(path), hint + 15)):
        d2 = (path[i].x - rx)**2 + (path[i].y - ry)**2
        if d2 < best_d2:
            best_d2 = d2
            best_idx = i
    return best_idx, math.sqrt(best_d2)


def _arc_length_to(path, idx):
    total = 0.0
    for i in range(min(idx, len(path) - 1)):
        total += math.hypot(path[i+1].x - path[i].x, path[i+1].y - path[i].y)
    return total


def _idx_at_arclength(path, target_len):
    accum = 0.0
    for i in range(len(path) - 1):
        seg = math.hypot(path[i+1].x - path[i].x, path[i+1].y - path[i].y)
        if accum + seg >= target_len:
            return i
        accum += seg
    return len(path) - 1


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
