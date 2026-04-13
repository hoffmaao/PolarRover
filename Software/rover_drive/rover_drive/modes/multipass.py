"""Multipass path-fidelity drive mode for repeat-track radar surveys."""

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from rover_sim.control import CommandBus, DirectionMode, FixType, RoverState
from rover_sim.missions import Mission, Waypoint

from rover_drive.controllers.base import PathController
from rover_drive.estimation import (
    STATE_HEADING,
    STATE_SPEED,
    STATE_X,
    STATE_Y,
    EnsembleKalmanFilter,
)


@dataclass
class MultipassControllerConfig:
    """
    Tunings for the multipass path-fidelity controller. The key difference
    from the waypoint controller is the objective: multipass minimizes
    *cross-track error* along a densely sampled reference line rather than
    seeking sparse goal waypoints. The steering uses a lookahead-based
    pure-pursuit policy so the rover tracks the path geometry, not just a
    heading toward the next point.
    """

    cruise_throttle: float = 0.25
    approach_throttle: float = 0.10
    lookahead_m: float = 3.0
    path_complete_tolerance_m: float = 1.5
    heading_saturation_rad: float = math.radians(40.0)
    slowdown_radius_m: float = 3.0


@dataclass
class MultipassMetrics:
    """Accumulated metrics computed during a multipass run."""

    cross_track_samples: list[float] = field(default_factory=list)

    @property
    def cross_track_rms_m(self) -> float:
        if not self.cross_track_samples:
            return 0.0
        return float(np.sqrt(np.mean(np.square(self.cross_track_samples))))

    @property
    def cross_track_max_m(self) -> float:
        if not self.cross_track_samples:
            return 0.0
        return float(np.max(np.abs(self.cross_track_samples)))

    @property
    def cross_track_mean_m(self) -> float:
        if not self.cross_track_samples:
            return 0.0
        return float(np.mean(np.abs(self.cross_track_samples)))


class MultipassDriver:
    """
    Path-fidelity controller for repeat-track radar surveys. The objective is
    to minimise the *cross-track error* between the rover's actual path and
    the planned base track, so that ice-penetrating radar passes on different
    dates register to each other when post-processed with PPK.

    The controller finds the closest point on the reference line to the
    rover's fused position, then places a *lookahead target* along the path
    at ``lookahead_m`` ahead of the closest point. Steering is a proportional
    heading-error correction toward the lookahead target — classic
    pure-pursuit — which naturally cuts the cross-track error because the
    target is *on the path*, not just "the next waypoint."

    Cross-track error is logged every step and exposed via ``metrics`` so the
    emulator runner can report RMS and max at the end of a run.

    The mission waypoints are expected to be densely sampled along the base
    track (the multipass GeoJSON loader's pass-expansion + a projection-
    stage densification are responsible for producing them). If the spacing
    is too coarse, the cross-track metric will under-report and the
    lookahead may snap to corners instead of following a smooth curve.
    """

    def __init__(
        self,
        config: Optional[MultipassControllerConfig] = None,
        enkf: Optional[EnsembleKalmanFilter] = None,
        path_controller: Optional[PathController] = None,
    ) -> None:
        self.config = config or MultipassControllerConfig()
        self.enkf = enkf or EnsembleKalmanFilter()
        self.path_controller = path_controller
        self.metrics = MultipassMetrics()
        self._initialized: bool = False
        self._complete: bool = False
        self._last_obs_t: float = -math.inf
        self._last_cmd: Optional[CommandBus] = None
        self._closest_idx: int = 0

    # ---------- public API ----------

    @property
    def complete(self) -> bool:
        return self._complete

    @property
    def progress_index(self) -> int:
        return self._closest_idx

    def reset(self) -> None:
        self._initialized = False
        self._complete = False
        self._last_obs_t = -math.inf
        self._last_cmd = None
        self._closest_idx = 0
        self.metrics = MultipassMetrics()
        self.enkf.reset()

    def update(
        self,
        state: RoverState,
        mission: Optional[Mission],
        dt: float,
    ) -> CommandBus:
        if mission is None or not mission.waypoints:
            raise RuntimeError("MultipassDriver requires a non-empty mission")

        self._step_filter(state, dt)
        cmd = self._compute_command(mission)
        self._last_cmd = cmd
        return cmd

    # ---------- filter ----------

    def _step_filter(self, state: RoverState, dt: float) -> None:
        if not self._initialized:
            if state.fix_type is FixType.NONE:
                return
            x0 = np.array(
                [state.x, state.y, state.heading, state.speed, state.yaw_rate],
                dtype=float,
            )
            self.enkf.initialize(x0)
            self._initialized = True
            self._last_obs_t = state.t
            return

        self.enkf.predict(self._last_cmd, dt)
        if state.fix_type is not FixType.NONE and state.t > self._last_obs_t + 1e-9:
            self.enkf.update_from_gnss(state)
            self._last_obs_t = state.t

    # ---------- control ----------

    def _compute_command(self, mission: Mission) -> CommandBus:
        if not self._initialized or self._complete:
            return _brake_cmd()

        wps = mission.waypoints
        fused = self.enkf.mean
        rx = float(fused[STATE_X])
        ry = float(fused[STATE_Y])
        rh = float(fused[STATE_HEADING])
        rv = float(fused[STATE_SPEED])

        closest_idx, _ = _find_closest(wps, rx, ry, self._closest_idx)
        self._closest_idx = closest_idx

        dist_to_end = _path_distance_from(wps, closest_idx, rx, ry)
        if dist_to_end < self.config.path_complete_tolerance_m and closest_idx >= len(wps) - 2:
            self._complete = True
            return _brake_cmd()

        # Delegate to pluggable path controller if provided
        if self.path_controller is not None:
            out = self.path_controller.compute(rx, ry, rh, rv, wps, closest_idx)
            self.metrics.cross_track_samples.append(out.cross_track_m)
            return CommandBus(
                throttle=out.throttle,
                steer=out.steer,
                direction_mode=DirectionMode.CLOSED_LOOP,
            )

        # Fallback: built-in pure pursuit (original behavior)
        ct_dist = _find_closest(wps, rx, ry, closest_idx)[1]
        self.metrics.cross_track_samples.append(ct_dist)

        lookahead_idx = _advance_along_path(wps, closest_idx, self.config.lookahead_m)
        target = wps[lookahead_idx]
        dx = target.x - rx
        dy = target.y - ry

        alpha = _wrap_angle(math.atan2(dy, dx) - rh)
        cfg = self.config
        steer = _clamp(alpha / cfg.heading_saturation_rad, -1.0, 1.0)

        if dist_to_end < cfg.slowdown_radius_m:
            frac = dist_to_end / cfg.slowdown_radius_m
            throttle = cfg.approach_throttle + frac * (cfg.cruise_throttle - cfg.approach_throttle)
        else:
            throttle = cfg.cruise_throttle

        return CommandBus(throttle=throttle, steer=steer, direction_mode=DirectionMode.CLOSED_LOOP)


# ---------- geometry helpers ----------


def _find_closest(
    wps: list[Waypoint], rx: float, ry: float, hint: int
) -> tuple[int, float]:
    """Find the closest waypoint to (rx, ry), searching forward from `hint`."""
    best_idx = hint
    best_d2 = (wps[hint].x - rx) ** 2 + (wps[hint].y - ry) ** 2
    for i in range(max(0, hint - 2), min(len(wps), hint + 20)):
        d2 = (wps[i].x - rx) ** 2 + (wps[i].y - ry) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_idx = i
    return best_idx, math.sqrt(best_d2)


def _advance_along_path(
    wps: list[Waypoint], start: int, distance: float
) -> int:
    """Walk forward from `start` along the waypoint sequence until accumulated
    arc length >= `distance`, and return the index reached."""
    remaining = distance
    idx = start
    while idx < len(wps) - 1:
        seg = math.hypot(wps[idx + 1].x - wps[idx].x, wps[idx + 1].y - wps[idx].y)
        if seg >= remaining:
            return idx + 1
        remaining -= seg
        idx += 1
    return len(wps) - 1


def _path_distance_from(
    wps: list[Waypoint], idx: int, rx: float, ry: float
) -> float:
    """Approximate remaining path length from the rover's position to the last waypoint."""
    if idx >= len(wps) - 1:
        return math.hypot(wps[-1].x - rx, wps[-1].y - ry)
    total = math.hypot(wps[idx].x - rx, wps[idx].y - ry)
    for i in range(idx, len(wps) - 1):
        total += math.hypot(wps[i + 1].x - wps[i].x, wps[i + 1].y - wps[i].y)
    return total


def _brake_cmd() -> CommandBus:
    return CommandBus(brake=0.5, direction_mode=DirectionMode.CLOSED_LOOP)


def _wrap_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))
