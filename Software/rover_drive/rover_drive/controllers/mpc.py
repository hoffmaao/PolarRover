"""Short-horizon Model Predictive Controller for path following.

Optimizes a sequence of steer commands over a prediction horizon using the
calibrated bicycle kinematic model. The cost function penalizes cross-track
error, heading error, and control effort (steer change). The optimization
uses a simple shooting method with scipy.optimize.minimize.

This is intentionally a lightweight MPC — short horizon (N=10–15 steps),
single-input (steer only, throttle is set by a separate speed policy), and
no inequality constraints beyond steer saturation. It's enough to demonstrate
the benefit of predictive control for path following without the complexity
of a full nonlinear MPC framework.
"""

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from rover_sim.surveys import Waypoint
from rover_drive.controllers.base import ControlOutput, PathController


@dataclass
class MPCConfig:
    horizon: int = 12                    # prediction steps
    dt_predict: float = 0.2             # prediction time step (coarser than sim dt for speed)
    arm_length_m: float = 2.5           # bicycle model L (from calibration)
    max_gamma_rad: float = math.radians(45.0)
    w_crosstrack: float = 10.0          # cost weight: cross-track error²
    w_heading: float = 2.0              # cost weight: heading error²
    w_steer_change: float = 0.5         # cost weight: (steer[i] - steer[i-1])²
    cruise_throttle: float = 0.25
    approach_throttle: float = 0.10
    heading_saturation_rad: float = math.radians(50.0)
    slowdown_radius_m: float = 3.0


class MPCController(PathController):
    """
    Short-horizon MPC that predicts the rover's trajectory over N steps using
    the bicycle kinematic model and optimizes steer commands to minimize a
    weighted sum of cross-track error, heading error from the path tangent,
    and control effort. Returns the first steer command of the optimal
    sequence (receding horizon).
    """

    def __init__(self, config: MPCConfig | None = None) -> None:
        self.config = config or MPCConfig()
        self._last_steer: float = 0.0

    def compute(self, rx, ry, rh, rv, path, closest_idx):
        cfg = self.config
        ct_dist = _cross_track_unsigned(rx, ry, path, closest_idx)

        # If nearly stopped or very few path points, fall back to simple heading control
        if abs(rv) < 0.05 or len(path) < 3:
            return self._fallback(rx, ry, rh, path, closest_idx, ct_dist)

        # Build the reference trajectory (path points the MPC should track)
        ref_points = _build_reference(path, closest_idx, cfg.horizon, cfg.dt_predict, abs(rv))

        # Optimize steer sequence
        best_steers = self._optimize(rx, ry, rh, rv, ref_points)

        steer = _clamp(best_steers[0] / cfg.heading_saturation_rad, -1, 1)
        self._last_steer = best_steers[0]

        dist_to_end = _remaining(path, closest_idx, rx, ry)
        if dist_to_end < cfg.slowdown_radius_m:
            frac = dist_to_end / cfg.slowdown_radius_m
            throttle = cfg.approach_throttle + frac * (cfg.cruise_throttle - cfg.approach_throttle)
        else:
            throttle = cfg.cruise_throttle

        return ControlOutput(steer=steer, throttle=throttle, cross_track_m=ct_dist)

    def _optimize(self, rx, ry, rh, rv, ref_points):
        cfg = self.config
        N = min(cfg.horizon, len(ref_points))
        if N < 2:
            return [0.0] * cfg.horizon

        # Simple grid search + refinement (fast, no scipy dependency)
        best_cost = float("inf")
        best_seq = [0.0] * N

        # Try a few constant-steer candidates, then perturb the best
        candidates = np.linspace(-cfg.max_gamma_rad, cfg.max_gamma_rad, 15)
        for s in candidates:
            seq = [float(s)] * N
            cost = self._evaluate(rx, ry, rh, rv, seq, ref_points)
            if cost < best_cost:
                best_cost = cost
                best_seq = seq[:]

        # Local refinement: perturb each step
        for iteration in range(3):
            for k in range(min(N, 5)):
                current = best_seq[k]
                for delta in [-0.15, -0.05, 0.05, 0.15]:
                    trial = best_seq[:]
                    trial[k] = _clamp(current + delta, -cfg.max_gamma_rad, cfg.max_gamma_rad)
                    cost = self._evaluate(rx, ry, rh, rv, trial, ref_points)
                    if cost < best_cost:
                        best_cost = cost
                        best_seq = trial[:]

        return best_seq

    def _evaluate(self, rx, ry, rh, rv, steer_seq, ref_points):
        cfg = self.config
        x, y, h = rx, ry, rh
        v = rv
        cost = 0.0
        prev_s = self._last_steer

        for k, (s, ref) in enumerate(zip(steer_seq, ref_points)):
            # Bicycle model forward step
            gamma = s
            yaw_rate = (v / cfg.arm_length_m) * math.tan(gamma) if cfg.arm_length_m > 0 else 0
            h_new = h + yaw_rate * cfg.dt_predict
            h_avg = 0.5 * (h + h_new)
            x += v * math.cos(h_avg) * cfg.dt_predict
            y += v * math.sin(h_avg) * cfg.dt_predict
            h = h_new

            # Cross-track cost
            ct = math.hypot(x - ref[0], y - ref[1])
            cost += cfg.w_crosstrack * ct * ct

            # Heading error cost (align with path tangent at reference point)
            if len(ref) > 2:
                he = _wrap(ref[2] - h)
                cost += cfg.w_heading * he * he

            # Control effort
            cost += cfg.w_steer_change * (s - prev_s) ** 2
            prev_s = s

        return cost

    def _fallback(self, rx, ry, rh, path, ci, ct_dist):
        """Simple heading-toward-nearest when MPC can't run."""
        wp = path[min(ci + 1, len(path) - 1)]
        alpha = _wrap(math.atan2(wp.y - ry, wp.x - rx) - rh)
        steer = _clamp(alpha / self.config.heading_saturation_rad, -1, 1)
        return ControlOutput(steer=steer, throttle=self.config.cruise_throttle, cross_track_m=ct_dist)


def _build_reference(path, closest_idx, horizon, dt, speed):
    """Sample reference points along the path at the prediction spacing."""
    ref = []
    dist_per_step = speed * dt
    accumulated = 0.0
    idx = closest_idx

    for _ in range(horizon):
        accumulated += dist_per_step
        while idx < len(path) - 1:
            seg = math.hypot(path[idx+1].x - path[idx].x, path[idx+1].y - path[idx].y)
            if seg >= accumulated:
                break
            accumulated -= seg
            idx += 1
        wp = path[min(idx, len(path) - 1)]
        th = _tangent_heading(path, min(idx, len(path) - 1))
        ref.append((wp.x, wp.y, th))

    return ref


def _tangent_heading(path, idx):
    if idx < len(path) - 1:
        return math.atan2(path[idx+1].y - path[idx].y, path[idx+1].x - path[idx].x)
    elif idx > 0:
        return math.atan2(path[idx].y - path[idx-1].y, path[idx].x - path[idx-1].x)
    return 0.0


def _cross_track_unsigned(rx, ry, path, hint):
    best = math.hypot(path[hint].x - rx, path[hint].y - ry)
    for i in range(max(0, hint - 3), min(len(path), hint + 15)):
        d = math.hypot(path[i].x - rx, path[i].y - ry)
        if d < best:
            best = d
    return best


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
