"""GP-MPPI: Model Predictive Path Integral control with GP-learned dynamics.

MPPI (Williams et al., ICRA 2016) is a sampling-based MPC that draws K
trajectory samples from a Gaussian, rolls them out through the dynamics,
evaluates cost, and computes a cost-weighted average as the next control.

Key advantages over Powell-based GP-MPC:
  1. Handles noisy/non-convex cost landscapes (no gradient descent into local minima)
  2. Embarrassingly parallel — all K rollouts vectorize in NumPy
  3. Works naturally with GP dynamics (just uses the mean prediction)

Combined with polynomial parameterization (3 coefficients instead of 8 steer
values), the search space drops from 8D to 3D, reducing the sample count
needed for good coverage by ~7x.

Architecture:
  1. Warm-start the polynomial mean from GP-feedforward+Stanley
  2. Sample K=256 polynomial coefficient vectors around the mean
  3. Expand each to an N-step steer sequence
  4. Vectorized GP-predicted rollout of all K trajectories simultaneously
  5. Cost-weighted averaging → next polynomial mean → extract first steer
"""

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from rover_sim.missions import Waypoint
from rover_drive.controllers.base import ControlOutput, PathController
from rover_drive.learning.gp_dynamics import GPDynamicsModel


@dataclass
class GPMPPIConfig:
    horizon: int = 8
    dt_predict: float = 0.15
    n_samples: int = 256          # MPPI sample count
    poly_order: int = 2           # quadratic parameterization (3 coefficients)
    temperature: float = 0.5      # MPPI temperature (lower = more greedy)
    noise_std: float = 0.15       # std of polynomial coefficient perturbations
    max_steer_rad: float = math.radians(45.0)
    w_contour: float = 20.0
    w_heading: float = 5.0
    w_uncertainty: float = 2.0
    w_steer_change: float = 0.8
    k_stanley_ct: float = 1.5
    k_stanley_soft: float = 0.3
    ff_blend: float = 0.8
    smooth_window: int = 15
    cruise_throttle: float = 0.25
    approach_throttle: float = 0.10
    heading_saturation_rad: float = math.radians(50.0)
    slowdown_radius_m: float = 3.0


class GPMPPIController(PathController):
    """
    GP-MPPI: sampling-based MPC with GP dynamics and polynomial controls.

    Each step:
      1. Compute warm-start polynomial from GP-feedforward+Stanley
      2. Sample K=256 polynomial perturbations (3D Gaussian)
      3. Expand to K × N steer sequences
      4. Vectorized rollout through cached GP curvature model
      5. Compute per-sample cost
      6. Cost-weighted average → updated polynomial → first steer
    """

    def __init__(
        self,
        gp_model: GPDynamicsModel,
        config: Optional[GPMPPIConfig] = None,
    ) -> None:
        self.config = config or GPMPPIConfig()
        self.gp_model = gp_model
        self._last_steer: float = 0.0
        self._poly_mean: Optional[np.ndarray] = None
        self._curvatures: Optional[np.ndarray] = None
        self._tangent_headings: Optional[np.ndarray] = None
        self._cache_steers: Optional[np.ndarray] = None
        self._cache_curv_means: Optional[np.ndarray] = None
        self._cache_curv_vars: Optional[np.ndarray] = None
        self._obs_since_cache: int = 0
        self._rng = np.random.default_rng(42)

    def observe(self, steer: float, speed: float, yaw_rate: float) -> None:
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
        dist_to_end = _remaining(path, ci, rx, ry)

        # Step 1: warm-start polynomial from feedforward+Stanley
        warm_poly = self._warm_start_poly(rx, ry, rh, rv, path, ci)

        # Step 2: MPPI sampling and evaluation
        optimal_poly = self._mppi_optimize(rx, ry, rh, rv, path, ci, warm_poly)

        # Extract first steer from optimal polynomial
        steer_seq = self._poly_to_steers(optimal_poly)
        steer = _clamp(float(steer_seq[0]) / cfg.heading_saturation_rad, -1, 1)
        self._last_steer = float(steer_seq[0])
        self._poly_mean = optimal_poly

        if dist_to_end < cfg.slowdown_radius_m:
            frac = dist_to_end / cfg.slowdown_radius_m
            throttle = cfg.approach_throttle + frac * (cfg.cruise_throttle - cfg.approach_throttle)
        else:
            throttle = cfg.cruise_throttle

        return ControlOutput(steer=steer, throttle=throttle, cross_track_m=abs(signed_ct))

    # ---------- MPPI core ----------

    def _mppi_optimize(self, rx, ry, rh, rv, path, ci, warm_poly):
        cfg = self.config
        K = cfg.n_samples
        n_coeffs = cfg.poly_order + 1

        # Sample polynomial perturbations around warm start
        noise = self._rng.normal(0, cfg.noise_std, size=(K, n_coeffs))
        poly_samples = warm_poly + noise  # (K, n_coeffs)

        # Expand all samples to steer sequences: (K, N)
        all_steers = np.array([self._poly_to_steers(p) for p in poly_samples])

        # Vectorized rollout and cost evaluation
        costs = self._batch_evaluate(rx, ry, rh, rv, all_steers, path, ci)

        # Cost-weighted averaging (MPPI update rule)
        costs -= np.min(costs)  # numerical stability
        weights = np.exp(-costs / cfg.temperature)
        weights /= np.sum(weights) + 1e-10

        # Weighted average of polynomial coefficients
        optimal_poly = np.sum(weights[:, None] * poly_samples, axis=0)
        return optimal_poly

    def _batch_evaluate(self, rx, ry, rh, rv, all_steers, path, ci):
        """Evaluate cost for all K sample trajectories simultaneously."""
        cfg = self.config
        K, N = all_steers.shape

        # Initialize K parallel trajectories
        xs = np.full(K, rx)
        ys = np.full(K, ry)
        hs = np.full(K, rh)
        costs = np.zeros(K)
        prev_s = np.full(K, self._last_steer)

        for k in range(N):
            steers_k = all_steers[:, k]

            # Vectorized GP curvature lookup
            curv_means = self._batch_cached_lookup(steers_k)

            # Vectorized forward predict
            yaw_rates = curv_means * rv
            hs_new = hs + yaw_rates * cfg.dt_predict
            h_avg = 0.5 * (hs + hs_new)
            xs = xs + rv * np.cos(h_avg) * cfg.dt_predict
            ys = ys + rv * np.sin(h_avg) * cfg.dt_predict
            hs = hs_new

            # Cross-track cost (vectorized nearest-point lookup)
            look_ci = min(ci + k + 1, len(path) - 1)
            ct = np.hypot(xs - path[look_ci].x, ys - path[look_ci].y)
            th = float(self._tangent_headings[look_ci])
            he = _wrap_vec(th - hs)

            costs += cfg.w_contour * ct * ct
            costs += cfg.w_heading * he * he
            costs += cfg.w_steer_change * (steers_k - prev_s) ** 2
            prev_s = steers_k

        return costs

    # ---------- polynomial parameterization ----------

    def _poly_to_steers(self, coeffs):
        """Expand polynomial coefficients to an N-step steer sequence."""
        cfg = self.config
        N = cfg.horizon
        t = np.linspace(0, 1, N)
        steer = np.zeros(N)
        for i, c in enumerate(coeffs):
            steer += c * t ** i
        return np.clip(steer, -cfg.max_steer_rad * 0.9, cfg.max_steer_rad * 0.9)

    def _warm_start_poly(self, rx, ry, rh, rv, path, ci):
        """Compute feedforward+Stanley at each horizon step, then fit a polynomial."""
        cfg = self.config
        N = cfg.horizon
        x, y, h = rx, ry, rh
        steers = []

        for k in range(N):
            look_ci = min(ci + int(k * rv * cfg.dt_predict / 0.5), len(path) - 1)
            kappa = float(self._curvatures[look_ci])
            delta_ff = cfg.ff_blend * self._gp_inverse(kappa)
            tang_h = float(self._tangent_headings[look_ci])
            he = _wrap(tang_h - h)
            _, sct = _signed_cross_track(x, y, h, path, look_ci)
            ct_term = math.atan2(cfg.k_stanley_ct * sct, abs(rv) + cfg.k_stanley_soft)
            s = _clamp(delta_ff + he + ct_term, -cfg.max_steer_rad * 0.9, cfg.max_steer_rad * 0.9)
            steers.append(s)
            mc = self._batch_cached_lookup(np.array([s]))[0]
            yr = mc * rv; h_new = h + yr * cfg.dt_predict
            h_avg = 0.5 * (h + h_new)
            x += rv * math.cos(h_avg) * cfg.dt_predict
            y += rv * math.sin(h_avg) * cfg.dt_predict; h = h_new

        steers = np.array(steers)
        t = np.linspace(0, 1, N)
        coeffs = np.polyfit(t, steers, cfg.poly_order)[::-1]
        return coeffs

    # ---------- GP cache (vectorized) ----------

    def _build_gp_cache(self):
        grid = np.linspace(-self.config.max_steer_rad, self.config.max_steer_rad, 60)
        means, vars_ = self.gp_model.predict_curvature_batch(grid)
        self._cache_steers = grid
        self._cache_curv_means = means
        self._cache_curv_vars = vars_

    def _batch_cached_lookup(self, steers):
        """Vectorized curvature lookup for an array of steer values."""
        idxs = np.searchsorted(self._cache_steers, steers)
        idxs = np.clip(idxs, 0, len(self._cache_steers) - 1)
        return self._cache_curv_means[idxs]

    def _gp_inverse(self, target_curvature):
        if self._cache_curv_means is None: return 0.0
        idx = np.argmin(np.abs(self._cache_curv_means - target_curvature))
        return float(self._cache_steers[idx])

    # ---------- path precomputation ----------

    def _precompute_path(self, path):
        n = len(path)
        raw = np.zeros(n); self._tangent_headings = np.zeros(n)
        for i in range(n):
            self._tangent_headings[i] = _tangent_heading(path, i)
            raw[i] = _curvature_at(path, i)
        w = self.config.smooth_window
        if w > 1 and n > w:
            self._curvatures = np.convolve(raw, np.ones(w)/w, mode='same')
        else: self._curvatures = raw

    def _fallback(self, rx, ry, rh, path, ci):
        wp = path[min(ci + 1, len(path) - 1)]
        alpha = _wrap(math.atan2(wp.y - ry, wp.x - rx) - rh)
        steer = _clamp(alpha / self.config.heading_saturation_rad, -1, 1)
        ct = abs(_signed_cross_track(rx, ry, rh, path, ci)[1])
        return ControlOutput(steer=steer, throttle=self.config.cruise_throttle, cross_track_m=ct)


def _signed_cross_track(rx, ry, rh, path, hint):
    best_idx, best_d2 = hint, (path[hint].x-rx)**2+(path[hint].y-ry)**2
    for i in range(max(0,hint-5),min(len(path),hint+20)):
        d2=(path[i].x-rx)**2+(path[i].y-ry)**2
        if d2<best_d2: best_d2=d2;best_idx=i
    cp=path[best_idx];dx,dy=cp.x-rx,cp.y-ry;th=_tangent_heading(path,best_idx)
    sign=math.cos(th)*dy-math.sin(th)*dx
    return best_idx,math.copysign(math.sqrt(best_d2),sign)

def _curvature_at(path, idx):
    if idx<=0 or idx>=len(path)-1: return 0.0
    p0,p1,p2=path[idx-1],path[idx],path[idx+1]
    ax,ay=p1.x-p0.x,p1.y-p0.y;bx,by=p2.x-p1.x,p2.y-p1.y
    cross=ax*by-ay*bx;da,db=math.hypot(ax,ay),math.hypot(bx,by)
    if da<1e-9 or db<1e-9: return 0.0
    return 2.0*cross/(da*db*(da+db))

def _tangent_heading(path, idx):
    if idx<len(path)-1: return math.atan2(path[idx+1].y-path[idx].y,path[idx+1].x-path[idx].x)
    elif idx>0: return math.atan2(path[idx].y-path[idx-1].y,path[idx].x-path[idx-1].x)
    return 0.0

def _remaining(path, idx, rx, ry):
    if idx>=len(path)-1: return math.hypot(path[-1].x-rx,path[-1].y-ry)
    total=math.hypot(path[idx].x-rx,path[idx].y-ry)
    for i in range(idx,len(path)-1): total+=math.hypot(path[i+1].x-path[i].x,path[i+1].y-path[i].y)
    return total

def _wrap(a): return (a+math.pi)%(2*math.pi)-math.pi
def _wrap_vec(a): return (a+np.pi)%(2*np.pi)-np.pi
def _clamp(x,lo,hi): return max(lo,min(hi,x))
