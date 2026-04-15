"""Spatial-predictive feedforward controller — uses variogram-based kriging
to predict cross-track errors ahead on the path from errors observed behind.

This is a geostatistical approach to path following: the cross-track error
field along the path has spatial correlation (nearby stations have similar
errors because snow conditions, terrain slope, and sled dynamics vary smoothly).
As the rover drives, it accumulates error observations. Using the variogram
structure, it predicts the error at upcoming stations and pre-corrects.

Unlike ILC, this does NOT require repeating the same track. It uses spatial
correlation WITHIN a single pass to look ahead. Unlike MPC, it doesn't
optimize a control sequence — it just predicts the disturbance and cancels it.

Architecture:
  δ = δ_ff (GP feedforward)
    + δ_kriging (predicted error correction from variogram)
    + heading_error + atan(k * cross_track / v) (Stanley feedback)

The kriging prediction uses a sliding window of recent error observations
and a parametric variogram (exponential, Gaussian, or Matérn) to compute
optimal interpolation weights for the prediction target ahead.
"""

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from rover_sim.surveys import Waypoint
from rover_drive.controllers.base import ControlOutput, PathController
from rover_drive.learning.gp_dynamics import GPDynamicsModel


@dataclass
class VariogramConfig:
    """Parametric variogram model: γ(h) = nugget + sill * (1 - exp(-(h/range)^p))"""
    model: str = "exponential"   # "exponential" | "gaussian" | "matern"
    sill: float = 0.1           # asymptotic variance (m²)
    range_m: float = 5.0        # correlation length (m)
    nugget: float = 0.001       # measurement noise variance (m²)


@dataclass
class SpatialPredictiveConfig:
    # Variogram
    variogram: VariogramConfig = None

    # Prediction
    prediction_distance_m: float = 3.0   # how far ahead to predict
    window_size: int = 60                 # max observations in the kriging window
    correction_gain: float = 0.5         # blend for kriging correction
    min_observations: int = 10           # need this many before predicting

    # Base controller (GP-feedforward + Stanley)
    k_crosstrack: float = 1.5
    k_soft: float = 0.3
    ff_blend: float = 0.8
    smooth_window: int = 15
    cruise_throttle: float = 0.25
    approach_throttle: float = 0.10
    heading_saturation_rad: float = math.radians(50.0)
    slowdown_radius_m: float = 3.0

    def __post_init__(self):
        if self.variogram is None:
            self.variogram = VariogramConfig()


class SpatialPredictiveController(PathController):
    """
    Kriging-based spatial prediction + GP feedforward + Stanley feedback.

    As the rover drives, cross-track errors are observed and stored indexed
    by arc-length station. At each step, the controller uses kriging with
    the configured variogram to predict the error at a station ahead of the
    current position. This predicted error is converted to a steering
    correction and added to the feedforward + feedback commands.

    The variogram captures: "if the error is +0.2m at the current station
    and the correlation length is 5m, then the error 3m ahead is probably
    ~+0.15m, so pre-correct by steering slightly right."
    """

    def __init__(
        self,
        gp_model: Optional[GPDynamicsModel] = None,
        config: Optional[SpatialPredictiveConfig] = None,
    ) -> None:
        self.config = config or SpatialPredictiveConfig()
        self.gp_model = gp_model
        self._curvatures: Optional[np.ndarray] = None
        self._tangent_headings: Optional[np.ndarray] = None
        self._arc_lengths: Optional[np.ndarray] = None
        self._cache_steers: Optional[np.ndarray] = None
        self._cache_curv_means: Optional[np.ndarray] = None
        self._obs_since_cache: int = 0

        # Kriging observation buffer: (station, signed_ct)
        self._error_obs: list[tuple[float, float]] = []
        self._prediction_log: list[tuple[float, float, float]] = []  # (station, predicted, actual)

    @property
    def prediction_log(self):
        return self._prediction_log

    @property
    def n_observations(self):
        return len(self._error_obs)

    def observe_gp(self, steer, speed, yaw_rate):
        if self.gp_model:
            self.gp_model.observe(steer, speed, yaw_rate)
            self._obs_since_cache += 1
            if self._obs_since_cache >= 50:
                self._cache_steers = None; self._obs_since_cache = 0

    def compute(self, rx, ry, rh, rv, path, closest_idx):
        cfg = self.config

        if self._curvatures is None:
            self._precompute_path(path)
        if self.gp_model and self.gp_model.fitted and self._cache_steers is None:
            self._build_gp_cache()

        ci, signed_ct = _signed_cross_track(rx, ry, rh, path, closest_idx)
        station = float(self._arc_lengths[ci]) if self._arc_lengths is not None else 0.0

        # Record error observation for kriging
        self._error_obs.append((station, signed_ct))
        if len(self._error_obs) > cfg.window_size:
            self._error_obs = self._error_obs[-cfg.window_size:]

        # GP feedforward
        delta_ff = 0.0
        if self._cache_steers is not None:
            kappa = float(self._curvatures[ci])
            delta_ff = cfg.ff_blend * self._gp_inverse(kappa)

        # Kriging prediction of error ahead
        delta_kriging = 0.0
        if len(self._error_obs) >= cfg.min_observations:
            target_station = station + cfg.prediction_distance_m
            predicted_ct, pred_var = self._kriging_predict(target_station)
            delta_kriging = -cfg.correction_gain * predicted_ct
            self._prediction_log.append((station, predicted_ct, signed_ct))

        # Stanley feedback
        tangent_h = float(self._tangent_headings[ci])
        heading_err = _wrap(tangent_h - rh)
        ct_term = math.atan2(cfg.k_crosstrack * signed_ct, abs(rv) + cfg.k_soft)

        delta_total = delta_ff + delta_kriging + heading_err + ct_term
        steer = _clamp(delta_total / cfg.heading_saturation_rad, -1.0, 1.0)

        dist_to_end = _remaining(path, ci, rx, ry)
        if dist_to_end < cfg.slowdown_radius_m:
            frac = dist_to_end / cfg.slowdown_radius_m
            throttle = cfg.approach_throttle + frac * (cfg.cruise_throttle - cfg.approach_throttle)
        else:
            throttle = cfg.cruise_throttle

        return ControlOutput(steer=steer, throttle=throttle, cross_track_m=abs(signed_ct))

    # ---------- kriging ----------

    def _kriging_predict(self, target_station: float) -> tuple[float, float]:
        """Simple kriging: predict error at target_station from nearby observations."""
        cfg = self.config
        vario = cfg.variogram
        obs = self._error_obs

        n = len(obs)
        stations = np.array([s for s, _ in obs])
        values = np.array([v for _, v in obs])

        # Build kriging matrices
        # Γ[i,j] = covariance(station_i, station_j)
        # γ[i] = covariance(station_i, target)
        C = self._covariance_matrix(stations, stations, vario)
        c = self._covariance_vector(stations, target_station, vario)

        # Add nugget to diagonal for numerical stability
        C += vario.nugget * np.eye(n)

        try:
            weights = np.linalg.solve(C, c)
        except np.linalg.LinAlgError:
            return 0.0, vario.sill

        predicted = float(weights @ values)
        pred_var = float(vario.sill + vario.nugget - weights @ c)
        return predicted, max(pred_var, 0.0)

    def _covariance_matrix(self, s1, s2, vario):
        """Build covariance matrix from the variogram: C(h) = sill - γ(h)"""
        h = np.abs(s1[:, None] - s2[None, :])
        return self._covariance_from_distance(h, vario)

    def _covariance_vector(self, stations, target, vario):
        h = np.abs(stations - target)
        return self._covariance_from_distance(h, vario)

    def _covariance_from_distance(self, h, vario):
        """Covariance = sill - variogram(h). Higher covariance at shorter distances."""
        if vario.model == "exponential":
            gamma = vario.sill * (1 - np.exp(-h / vario.range_m))
        elif vario.model == "gaussian":
            gamma = vario.sill * (1 - np.exp(-(h / vario.range_m)**2))
        else:  # matern 3/2
            scaled = np.sqrt(3) * h / vario.range_m
            gamma = vario.sill * (1 - (1 + scaled) * np.exp(-scaled))
        return vario.sill - gamma

    # ---------- path + GP ----------

    def _precompute_path(self, path):
        n = len(path)
        raw = np.zeros(n); self._tangent_headings = np.zeros(n); self._arc_lengths = np.zeros(n)
        for i in range(n):
            self._tangent_headings[i] = _tangent_heading(path, i)
            raw[i] = _curvature_at(path, i)
            if i > 0: self._arc_lengths[i] = self._arc_lengths[i-1] + math.hypot(
                path[i].x-path[i-1].x, path[i].y-path[i-1].y)
        w = self.config.smooth_window
        if w > 1 and n > w: self._curvatures = np.convolve(raw, np.ones(w)/w, mode='same')
        else: self._curvatures = raw

    def _build_gp_cache(self):
        grid = np.linspace(-self.config.heading_saturation_rad, self.config.heading_saturation_rad, 50)
        means, _ = self.gp_model.predict_curvature_batch(grid)
        self._cache_steers = grid; self._cache_curv_means = means

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
