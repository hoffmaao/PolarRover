"""GP-MPC: Model Predictive Control with a Gaussian Process dynamics model.

This is the full GP-MPC implementation that combines:
  1. GP-learned feedforward (from the working GP-feedforward controller)
  2. Short-horizon trajectory optimization using the GP dynamics
  3. Warm-start from the GP-feedforward+Stanley solution
  4. Cached GP curvature predictions (one batch predict per step, not per candidate)

The key insight from the GP-feedforward experiments: the GP's value is in the
dynamics model, not the optimization. So we use the GP for accurate trajectory
prediction, warm-start the optimizer from a good initial guess (what the
feedforward+Stanley controller would do), and only optimize a small refinement.

References:
  Hewing, Kabzan, Zeilinger (2020). "Cautious Model Predictive Control
  Using Gaussian Process Regression." IEEE T-CST.
  Hoffmann et al. (2007). "Autonomous Automobile Trajectory Tracking."
"""

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy.optimize import minimize

from rover_sim.surveys import Waypoint
from rover_drive.controllers.base import ControlOutput, PathController
from rover_drive.learning.gp_dynamics import GPDynamicsModel


@dataclass
class GPMPCFullConfig:
    horizon: int = 8
    dt_predict: float = 0.15
    max_steer_rad: float = math.radians(45.0)
    w_contour: float = 20.0        # cross-track error²
    w_heading: float = 5.0         # heading error²
    w_uncertainty: float = 3.0     # GP variance penalty (cautious control)
    w_steer_change: float = 1.0    # control smoothness
    k_stanley_ct: float = 1.5      # Stanley gain for warm-start
    k_stanley_soft: float = 0.3
    ff_blend: float = 0.8          # feedforward blend for warm-start
    smooth_window: int = 15
    cruise_throttle: float = 0.25
    approach_throttle: float = 0.10
    heading_saturation_rad: float = math.radians(50.0)
    slowdown_radius_m: float = 3.0


class GPMPCFullController(PathController):
    """
    Full GP-MPC: warm-started from GP-feedforward+Stanley, refined by
    scipy.optimize over a short horizon using GP-predicted trajectories.

    Each step:
      1. Build a curvature cache from the GP (one batch predict, fast).
      2. Compute the warm-start steer sequence from GP-feedforward+Stanley
         at each horizon step (what the simple controller would do).
      3. Run scipy.optimize.minimize (Powell method, no gradients needed)
         to refine the sequence, minimizing cross-track + heading error +
         GP uncertainty + control effort.
      4. Apply the first element of the optimized sequence (receding horizon).

    The optimizer starts from a good initial guess so it converges in ~10-30
    function evaluations instead of hundreds. Total compute: ~5-15ms per step.
    """

    def __init__(
        self,
        gp_model: GPDynamicsModel,
        config: Optional[GPMPCFullConfig] = None,
    ) -> None:
        self.config = config or GPMPCFullConfig()
        self.gp_model = gp_model
        self._last_steer: float = 0.0
        self._curvatures: Optional[np.ndarray] = None
        self._tangent_headings: Optional[np.ndarray] = None
        self._cache_steers: Optional[np.ndarray] = None
        self._cache_curv_means: Optional[np.ndarray] = None
        self._cache_curv_vars: Optional[np.ndarray] = None
        self._obs_since_cache: int = 0

    def observe(self, steer: float, speed: float, yaw_rate: float) -> None:
        """Feed driving data into the GP for online learning."""
        self.gp_model.observe(steer, speed, yaw_rate)
        self._obs_since_cache += 1
        if self._obs_since_cache >= 50:
            self._cache_steers = None
            self._obs_since_cache = 0

    def compute(self, rx, ry, rh, rv, path, closest_idx):
        cfg = self.config

        if self._curvatures is None:
            self._precompute_path(path)

        if not self.gp_model.fitted or abs(rv) < 0.05 or len(path) < 3:
            return self._fallback(rx, ry, rh, path, closest_idx)

        if self._cache_steers is None:
            self._build_gp_cache()

        ci, signed_ct = _signed_cross_track(rx, ry, rh, path, closest_idx)
        ct_unsigned = abs(signed_ct)

        dist_to_end = _remaining(path, ci, rx, ry)
        if dist_to_end < 1.0:
            return ControlOutput(steer=0, throttle=cfg.approach_throttle, cross_track_m=ct_unsigned)

        # Step 1: warm-start from GP-feedforward + Stanley
        warm_start = self._compute_warm_start(rx, ry, rh, rv, path, ci, cfg.horizon)

        # Step 2: optimize with scipy
        bounds = [(-cfg.max_steer_rad * 0.9, cfg.max_steer_rad * 0.9)] * cfg.horizon

        def cost_fn(steer_seq):
            return self._evaluate_trajectory(rx, ry, rh, rv, steer_seq, path, ci)

        result = minimize(
            cost_fn, warm_start,
            method="Powell",
            bounds=bounds,
            options={"maxiter": 15, "maxfev": 80, "ftol": 1e-3},
        )
        optimal = result.x

        steer = _clamp(float(optimal[0]) / cfg.heading_saturation_rad, -1.0, 1.0)
        self._last_steer = float(optimal[0])

        if dist_to_end < cfg.slowdown_radius_m:
            frac = dist_to_end / cfg.slowdown_radius_m
            throttle = cfg.approach_throttle + frac * (cfg.cruise_throttle - cfg.approach_throttle)
        else:
            throttle = cfg.cruise_throttle

        return ControlOutput(steer=steer, throttle=throttle, cross_track_m=ct_unsigned)

    # ---------- warm start ----------

    def _compute_warm_start(self, rx, ry, rh, rv, path, ci, N):
        """Compute what GP-feedforward+Stanley would do at each horizon step."""
        cfg = self.config
        x, y, h, v = rx, ry, rh, rv
        steers = []

        for k in range(N):
            look_ci = min(ci + int(k * v * cfg.dt_predict / 0.5), len(path) - 1)

            # Feedforward from GP-learned curvature
            kappa = float(self._curvatures[look_ci]) if self._curvatures is not None else 0.0
            delta_ff = cfg.ff_blend * self._gp_steer_for_curvature(kappa)

            # Stanley feedback
            tangent_h = float(self._tangent_headings[look_ci]) if self._tangent_headings is not None else 0.0
            ci_k, signed_ct_k = _signed_cross_track(x, y, h, path, look_ci)
            heading_err = _wrap(tangent_h - h)
            ct_term = math.atan2(cfg.k_stanley_ct * signed_ct_k, abs(v) + cfg.k_stanley_soft)

            delta = delta_ff + heading_err + ct_term
            delta = _clamp(delta, -cfg.max_steer_rad * 0.9, cfg.max_steer_rad * 0.9)
            steers.append(delta)

            # Forward predict for next horizon step
            curv_mean = self._cached_curvature_mean(delta)
            yr = curv_mean * v
            h_new = h + yr * cfg.dt_predict
            h_avg = 0.5 * (h + h_new)
            x += v * math.cos(h_avg) * cfg.dt_predict
            y += v * math.sin(h_avg) * cfg.dt_predict
            h = h_new

        return np.array(steers)

    # ---------- trajectory evaluation ----------

    def _evaluate_trajectory(self, rx, ry, rh, rv, steer_seq, path, ci):
        cfg = self.config
        x, y, h = rx, ry, rh
        v = rv
        cost = 0.0
        prev_s = self._last_steer

        for k, s in enumerate(steer_seq):
            curv_mean = self._cached_curvature_mean(s)
            curv_var = self._cached_curvature_var(s)

            yr = curv_mean * v
            h_new = h + yr * cfg.dt_predict
            h_avg = 0.5 * (h + h_new)
            x += v * math.cos(h_avg) * cfg.dt_predict
            y += v * math.sin(h_avg) * cfg.dt_predict
            h = h_new

            look_ci = min(ci + k + 1, len(path) - 1)
            ci_k, ct = _cross_track_at(x, y, path, look_ci)
            th = _tangent_heading(path, ci_k)
            he = _wrap(th - h)
            unc = math.sqrt(max(curv_var, 0)) * abs(v) * cfg.dt_predict

            cost += cfg.w_contour * ct * ct
            cost += cfg.w_heading * he * he
            cost += cfg.w_uncertainty * unc * unc
            cost += cfg.w_steer_change * (s - prev_s) ** 2
            prev_s = s

        return cost

    # ---------- GP cache ----------

    def _build_gp_cache(self):
        grid = np.linspace(-self.config.max_steer_rad, self.config.max_steer_rad, 50)
        means, vars_ = self.gp_model.predict_curvature_batch(grid)
        self._cache_steers = grid
        self._cache_curv_means = means
        self._cache_curv_vars = vars_

    def _cached_curvature_mean(self, steer):
        if self._cache_steers is None:
            return 0.0
        idx = np.searchsorted(self._cache_steers, steer)
        idx = max(0, min(idx, len(self._cache_steers) - 1))
        return float(self._cache_curv_means[idx])

    def _cached_curvature_var(self, steer):
        if self._cache_steers is None:
            return 1.0
        idx = np.searchsorted(self._cache_steers, steer)
        idx = max(0, min(idx, len(self._cache_steers) - 1))
        return float(self._cache_curv_vars[idx])

    def _gp_steer_for_curvature(self, target_curvature):
        """Inverse GP lookup: given a target curvature, find the steer that produces it."""
        if self._cache_steers is None or self._cache_curv_means is None:
            return 0.0
        idx = np.argmin(np.abs(self._cache_curv_means - target_curvature))
        return float(self._cache_steers[idx])

    # ---------- path precomputation ----------

    def _precompute_path(self, path):
        n = len(path)
        raw_curv = np.zeros(n)
        self._tangent_headings = np.zeros(n)
        for i in range(n):
            self._tangent_headings[i] = _tangent_heading(path, i)
            raw_curv[i] = _curvature_at(path, i)
        w = self.config.smooth_window
        if w > 1 and n > w:
            self._curvatures = np.convolve(raw_curv, np.ones(w)/w, mode='same')
        else:
            self._curvatures = raw_curv

    def _fallback(self, rx, ry, rh, path, ci):
        wp = path[min(ci + 1, len(path) - 1)]
        alpha = _wrap(math.atan2(wp.y - ry, wp.x - rx) - rh)
        steer = _clamp(alpha / self.config.heading_saturation_rad, -1, 1)
        ct = _cross_track_at(rx, ry, path, ci)[1]
        return ControlOutput(steer=steer, throttle=self.config.cruise_throttle, cross_track_m=ct)


# ---------- geometry helpers ----------

def _signed_cross_track(rx, ry, rh, path, hint):
    best_idx = hint
    best_d2 = (path[hint].x - rx)**2 + (path[hint].y - ry)**2
    for i in range(max(0, hint - 5), min(len(path), hint + 20)):
        d2 = (path[i].x - rx)**2 + (path[i].y - ry)**2
        if d2 < best_d2:
            best_d2 = d2
            best_idx = i
    cp = path[best_idx]
    th = _tangent_heading(path, best_idx)
    dx, dy = cp.x - rx, cp.y - ry
    sign = math.cos(th) * dy - math.sin(th) * dx
    return best_idx, math.copysign(math.sqrt(best_d2), sign)

def _cross_track_at(rx, ry, path, hint):
    hint = max(0, min(hint, len(path) - 1))
    best_idx, best_d = hint, math.hypot(path[hint].x - rx, path[hint].y - ry)
    for i in range(max(0, hint - 5), min(len(path), hint + 15)):
        d = math.hypot(path[i].x - rx, path[i].y - ry)
        if d < best_d:
            best_d = d; best_idx = i
    return best_idx, best_d

def _curvature_at(path, idx):
    if idx <= 0 or idx >= len(path) - 1: return 0.0
    p0, p1, p2 = path[idx-1], path[idx], path[idx+1]
    ax, ay = p1.x-p0.x, p1.y-p0.y; bx, by = p2.x-p1.x, p2.y-p1.y
    cross = ax*by - ay*bx; da, db = math.hypot(ax,ay), math.hypot(bx,by)
    if da < 1e-9 or db < 1e-9: return 0.0
    return 2.0 * cross / (da * db * (da + db))

def _tangent_heading(path, idx):
    if idx < len(path)-1: return math.atan2(path[idx+1].y-path[idx].y, path[idx+1].x-path[idx].x)
    elif idx > 0: return math.atan2(path[idx].y-path[idx-1].y, path[idx].x-path[idx-1].x)
    return 0.0

def _remaining(path, idx, rx, ry):
    if idx >= len(path)-1: return math.hypot(path[-1].x-rx, path[-1].y-ry)
    total = math.hypot(path[idx].x-rx, path[idx].y-ry)
    for i in range(idx, len(path)-1): total += math.hypot(path[i+1].x-path[i].x, path[i+1].y-path[i].y)
    return total

def _wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi
def _clamp(x, lo, hi): return max(lo, min(hi, x))
