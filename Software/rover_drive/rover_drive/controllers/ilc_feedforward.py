"""ILC + GP-Feedforward + Stanley: Iterative Learning Control that improves
with each repeat pass of the same track.

ILC stores a correction signal indexed by path station (arc-length). After
each pass, the correction is updated from the observed cross-track error:

    u_{k+1}(s) = Q * (u_k(s) + L * e_k(s))

where:
  u_k(s) = feedforward correction at station s on pass k
  e_k(s) = signed cross-track error at station s on pass k
  L      = learning gain (0.3-0.5 = conservative, converges in 3-5 passes)
  Q      = low-pass filter to avoid amplifying GNSS noise into steering chatter

The correction is ADDED to the GP-feedforward + Stanley baseline, so pass 1
performs identically to the GP-feedforward controller. Each subsequent pass
improves by learning from the previous pass's errors.

This is the theoretically optimal architecture for multipass repeat surveys:
  - ILC eliminates repeatable errors (model mismatch, terrain, consistent snow)
  - Stanley feedback handles non-repeatable disturbances (wind, GNSS noise)
  - GP feedforward handles the known path curvature

References:
  Bristow, Tharayil & Alleyne (2006). "A survey of iterative learning
  control." IEEE Control Systems Magazine.
"""

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from rover_sim.missions import Waypoint
from rover_drive.controllers.base import ControlOutput, PathController
from rover_drive.learning.gp_dynamics import GPDynamicsModel


@dataclass
class ILCFeedforwardConfig:
    # ILC parameters
    learning_gain: float = 0.4          # L: how aggressively to learn (0.3-0.5)
    q_filter_window: int = 9            # Q-filter smoothing kernel (odd number)
    station_resolution_m: float = 0.25  # correction resolution along the path

    # Base controller (GP-feedforward + Stanley)
    k_crosstrack: float = 1.5
    k_soft: float = 0.3
    ff_blend: float = 0.8
    smooth_window: int = 15
    cruise_throttle: float = 0.25
    approach_throttle: float = 0.10
    heading_saturation_rad: float = math.radians(50.0)
    slowdown_radius_m: float = 3.0


@dataclass
class ILCState:
    """Persistent ILC state that survives across passes."""
    correction: Optional[np.ndarray] = None  # u(s) indexed by station
    pass_count: int = 0
    pass_errors: list = field(default_factory=list)  # RMS per pass for tracking improvement


class ILCFeedforwardController(PathController):
    """
    ILC + GP-Feedforward + Stanley. Learns a station-indexed correction
    signal from previous passes and applies it on subsequent passes.

    Usage across multiple passes:
        controller = ILCFeedforwardController(gp_model, config)
        for pass_num in range(n_passes):
            # run the pass, calling compute() each step
            controller.end_pass()  # updates the ILC correction from this pass's errors
            controller.begin_pass()  # resets per-pass state for the next run
    """

    def __init__(
        self,
        gp_model: GPDynamicsModel,
        config: Optional[ILCFeedforwardConfig] = None,
    ) -> None:
        self.config = config or ILCFeedforwardConfig()
        self.gp_model = gp_model
        self.ilc = ILCState()

        # Path data (computed once)
        self._curvatures: Optional[np.ndarray] = None
        self._tangent_headings: Optional[np.ndarray] = None
        self._arc_lengths: Optional[np.ndarray] = None
        self._total_arc: float = 0.0
        self._n_stations: int = 0

        # GP cache
        self._cache_steers: Optional[np.ndarray] = None
        self._cache_curv_means: Optional[np.ndarray] = None
        self._obs_since_cache: int = 0

        # Per-pass tracking
        self._current_errors: list[tuple[float, float]] = []  # (station, signed_ct)

    def observe(self, steer: float, speed: float, yaw_rate: float) -> None:
        self.gp_model.observe(steer, speed, yaw_rate)
        self._obs_since_cache += 1
        if self._obs_since_cache >= 50:
            self._cache_steers = None
            self._obs_since_cache = 0

    @property
    def pass_count(self) -> int:
        return self.ilc.pass_count

    @property
    def pass_errors(self) -> list[float]:
        return self.ilc.pass_errors

    def begin_pass(self) -> None:
        """Reset per-pass state for a new pass. ILC correction persists."""
        self._current_errors.clear()

    def end_pass(self) -> None:
        """Update the ILC correction from this pass's errors. Call after each complete pass."""
        if not self._current_errors:
            return

        cfg = self.config
        n = self._n_stations
        if n == 0:
            return

        # Bin the errors by station
        error_at_station = np.zeros(n)
        count_at_station = np.zeros(n)
        for station, signed_ct in self._current_errors:
            idx = int(station / cfg.station_resolution_m)
            idx = max(0, min(idx, n - 1))
            error_at_station[idx] += signed_ct
            count_at_station[idx] += 1

        # Average errors per station (where we have data)
        mask = count_at_station > 0
        error_at_station[mask] /= count_at_station[mask]

        # Interpolate gaps
        if not np.all(mask):
            filled = np.interp(np.arange(n), np.where(mask)[0], error_at_station[mask])
            error_at_station = filled

        # ILC update: u_{k+1} = Q * (u_k + L * e_k)
        if self.ilc.correction is None:
            self.ilc.correction = np.zeros(n)

        raw_update = self.ilc.correction + cfg.learning_gain * error_at_station

        # Q-filter: smoothing to avoid noise amplification
        if cfg.q_filter_window > 1:
            kernel = np.ones(cfg.q_filter_window) / cfg.q_filter_window
            self.ilc.correction = np.convolve(raw_update, kernel, mode='same')
        else:
            self.ilc.correction = raw_update

        # Track pass performance
        rms = float(np.sqrt(np.mean(error_at_station[mask] ** 2)))
        self.ilc.pass_errors.append(rms)
        self.ilc.pass_count += 1

    def compute(self, rx, ry, rh, rv, path, closest_idx):
        cfg = self.config

        if self._curvatures is None:
            self._precompute_path(path)
        if self.gp_model.fitted and self._cache_steers is None:
            self._build_gp_cache()

        ci, signed_ct = _signed_cross_track(rx, ry, rh, path, closest_idx)

        # Record error for ILC update at end of pass
        if self._arc_lengths is not None and ci < len(self._arc_lengths):
            station = float(self._arc_lengths[ci])
            self._current_errors.append((station, signed_ct))

        # GP feedforward
        if self._cache_steers is not None:
            kappa = float(self._curvatures[ci])
            delta_ff = cfg.ff_blend * self._gp_inverse(kappa)
        else:
            delta_ff = 0.0

        # ILC correction (from previous passes)
        delta_ilc = 0.0
        if self.ilc.correction is not None and self._arc_lengths is not None:
            station = float(self._arc_lengths[ci])
            idx = int(station / cfg.station_resolution_m)
            idx = max(0, min(idx, len(self.ilc.correction) - 1))
            delta_ilc = float(self.ilc.correction[idx])

        # Stanley feedback
        tangent_h = float(self._tangent_headings[ci])
        heading_err = _wrap(tangent_h - rh)
        ct_term = math.atan2(cfg.k_crosstrack * signed_ct, abs(rv) + cfg.k_soft)

        # Combined: feedforward + ILC correction + feedback
        delta_total = delta_ff + delta_ilc + heading_err + ct_term
        steer = _clamp(delta_total / cfg.heading_saturation_rad, -1.0, 1.0)

        dist_to_end = _remaining(path, ci, rx, ry)
        if dist_to_end < cfg.slowdown_radius_m:
            frac = dist_to_end / cfg.slowdown_radius_m
            throttle = cfg.approach_throttle + frac * (cfg.cruise_throttle - cfg.approach_throttle)
        else:
            throttle = cfg.cruise_throttle

        return ControlOutput(steer=steer, throttle=throttle, cross_track_m=abs(signed_ct))

    # ---------- path + GP ----------

    def _precompute_path(self, path):
        n = len(path)
        raw_curv = np.zeros(n)
        self._tangent_headings = np.zeros(n)
        self._arc_lengths = np.zeros(n)
        for i in range(n):
            self._tangent_headings[i] = _tangent_heading(path, i)
            raw_curv[i] = _curvature_at(path, i)
            if i > 0:
                self._arc_lengths[i] = self._arc_lengths[i-1] + math.hypot(
                    path[i].x - path[i-1].x, path[i].y - path[i-1].y)
        w = self.config.smooth_window
        if w > 1 and n > w:
            self._curvatures = np.convolve(raw_curv, np.ones(w)/w, mode='same')
        else:
            self._curvatures = raw_curv
        self._total_arc = float(self._arc_lengths[-1]) if n > 0 else 0
        self._n_stations = max(1, int(self._total_arc / self.config.station_resolution_m) + 1)

    def _build_gp_cache(self):
        grid = np.linspace(-self.config.heading_saturation_rad, self.config.heading_saturation_rad, 50)
        means, _ = self.gp_model.predict_curvature_batch(grid)
        self._cache_steers = grid
        self._cache_curv_means = means

    def _gp_inverse(self, target_curv):
        if self._cache_curv_means is None: return 0.0
        idx = np.argmin(np.abs(self._cache_curv_means - target_curv))
        return float(self._cache_steers[idx])


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
def _clamp(x,lo,hi): return max(lo,min(hi,x))
