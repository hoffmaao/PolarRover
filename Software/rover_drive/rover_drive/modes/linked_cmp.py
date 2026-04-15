"""Linked CMP two-rover coordinated drive mode.

Common Midpoint survey: the midpoint M is FIXED for the duration of a shot.
Both rovers start near M and drive OUTWARD along the survey axis in opposite
directions, increasing their separation over time. The science: the same
subsurface reflection point is observed at progressively wider antenna
separations, giving velocity-vs-depth structure.

    t=0:   A . M . B           (rovers close together at midpoint)
    t=1:   A . . M . . B       (spreading outward)
    t=2:   A . . . M . . . B   (wider spread)
"""

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from rover_sim.control import CommandBus, DirectionMode, FixType, RoverState
from rover_sim.surveys import Survey, Waypoint

from rover_drive.estimation import (
    STATE_HEADING,
    STATE_SPEED,
    STATE_X,
    STATE_Y,
    EnsembleKalmanFilter,
)


@dataclass
class CMPFormationConfig:
    """
    Tunings for the CMP formation controller.

    The spread between the two rovers increases linearly from
    ``start_spread_m`` to ``end_spread_m`` at ``spread_rate_m_per_s``.
    Each rover drives outward from the fixed midpoint along the survey
    axis at half that rate.
    """

    start_spread_m: float = 2.0
    end_spread_m: float = 30.0
    spread_rate_m_per_s: float = 0.5
    cruise_throttle: float = 0.20
    approach_throttle: float = 0.08
    heading_saturation_rad: float = math.radians(40.0)
    target_tolerance_m: float = 1.5


@dataclass
class CMPMetrics:
    """Accumulated formation-quality metrics for a CMP run."""

    midpoint_drift_samples: list[float] = field(default_factory=list)
    spread_error_samples: list[float] = field(default_factory=list)
    target_spread_samples: list[float] = field(default_factory=list)

    @property
    def midpoint_drift_rms_m(self) -> float:
        if not self.midpoint_drift_samples:
            return 0.0
        return float(np.sqrt(np.mean(np.square(self.midpoint_drift_samples))))

    @property
    def midpoint_drift_max_m(self) -> float:
        if not self.midpoint_drift_samples:
            return 0.0
        return float(np.max(np.abs(self.midpoint_drift_samples)))

    @property
    def spread_error_rms_m(self) -> float:
        if not self.spread_error_samples:
            return 0.0
        return float(np.sqrt(np.mean(np.square(self.spread_error_samples))))

    @property
    def spread_error_max_m(self) -> float:
        if not self.spread_error_samples:
            return 0.0
        return float(np.max(np.abs(self.spread_error_samples)))


class LinkedCMPDriver:
    """
    Two-rover coordinated driver for Common Midpoint radar surveys.

    The midpoint M is fixed (the first waypoint in the mission). Both rovers
    drive outward from M along the survey axis (direction from waypoint 0 to
    waypoint 1). The target separation increases from ``start_spread_m`` to
    ``end_spread_m`` at ``spread_rate_m_per_s``.

    Each step:
      1. Two EnKFs fuse GNSS for each rover.
      2. Compute target_spread(t) = start + rate * elapsed, clamped to end.
      3. Target positions: M ± target_spread/2 * survey_direction.
      4. Each rover steers toward its target.
      5. Record midpoint drift (distance from actual midpoint to M) and
         spread error (actual - target).

    Complete when target_spread reaches end_spread and both rovers are
    within tolerance of their final targets.
    """

    def __init__(
        self,
        config: Optional[CMPFormationConfig] = None,
        enkf_a: Optional[EnsembleKalmanFilter] = None,
        enkf_b: Optional[EnsembleKalmanFilter] = None,
    ) -> None:
        self.config = config or CMPFormationConfig()
        self.enkf_a = enkf_a or EnsembleKalmanFilter()
        self.enkf_b = enkf_b or EnsembleKalmanFilter()
        self.metrics = CMPMetrics()
        self._init_a: bool = False
        self._init_b: bool = False
        self._complete: bool = False
        self._elapsed: float = 0.0
        self._both_ready: bool = False
        self._last_cmd_a: Optional[CommandBus] = None
        self._last_cmd_b: Optional[CommandBus] = None
        self._last_obs_t_a: float = -math.inf
        self._last_obs_t_b: float = -math.inf
        self._midpoint: Optional[tuple[float, float]] = None
        self._survey_dir: Optional[tuple[float, float]] = None

    # ---------- public API ----------

    @property
    def complete(self) -> bool:
        return self._complete

    @property
    def current_target_spread(self) -> float:
        cfg = self.config
        return min(cfg.start_spread_m + cfg.spread_rate_m_per_s * self._elapsed,
                   cfg.end_spread_m)

    def reset(self) -> None:
        self._init_a = self._init_b = False
        self._complete = False
        self._elapsed = 0.0
        self._both_ready = False
        self._last_cmd_a = self._last_cmd_b = None
        self._last_obs_t_a = self._last_obs_t_b = -math.inf
        self._midpoint = None
        self._survey_dir = None
        self.metrics = CMPMetrics()
        self.enkf_a.reset()
        self.enkf_b.reset()

    def update(
        self,
        state_a: RoverState,
        state_b: RoverState,
        mission: Survey,
        dt: float,
    ) -> tuple[CommandBus, CommandBus]:
        if len(mission.waypoints) < 2:
            raise RuntimeError("CMP mission needs at least 2 waypoints (midpoint + direction)")

        if self._midpoint is None:
            self._setup_geometry(mission)

        self._step_filter(
            self.enkf_a, state_a, self._last_cmd_a, dt, "_init_a", "_last_obs_t_a",
        )
        self._step_filter(
            self.enkf_b, state_b, self._last_cmd_b, dt, "_init_b", "_last_obs_t_b",
        )

        if self._init_a and self._init_b:
            if not self._both_ready:
                self._both_ready = True
            self._elapsed += dt

        cmd_a, cmd_b = self._compute_commands()
        self._last_cmd_a = cmd_a
        self._last_cmd_b = cmd_b
        return cmd_a, cmd_b

    # ---------- setup ----------

    def _setup_geometry(self, mission: Survey) -> None:
        wp0 = mission.waypoints[0]
        wp1 = mission.waypoints[1]
        self._midpoint = (wp0.x, wp0.y)
        dx = wp1.x - wp0.x
        dy = wp1.y - wp0.y
        length = max(1e-12, math.hypot(dx, dy))
        self._survey_dir = (dx / length, dy / length)

    # ---------- filter ----------

    def _step_filter(
        self,
        enkf: EnsembleKalmanFilter,
        state: RoverState,
        last_cmd: Optional[CommandBus],
        dt: float,
        init_attr: str,
        obs_t_attr: str,
    ) -> None:
        initialized = getattr(self, init_attr)
        last_obs_t = getattr(self, obs_t_attr)

        if not initialized:
            if state.fix_type is FixType.NONE:
                return
            x0 = np.array(
                [state.x, state.y, state.heading, state.speed, state.yaw_rate],
                dtype=float,
            )
            enkf.initialize(x0)
            setattr(self, init_attr, True)
            setattr(self, obs_t_attr, state.t)
            return

        enkf.predict(last_cmd, dt)
        if state.fix_type is not FixType.NONE and state.t > last_obs_t + 1e-9:
            enkf.update_from_gnss(state)
            setattr(self, obs_t_attr, state.t)

    # ---------- formation control ----------

    def _compute_commands(self) -> tuple[CommandBus, CommandBus]:
        if not self._both_ready or self._complete:
            return _brake_cmd(), _brake_cmd()

        cfg = self.config
        mx, my = self._midpoint
        sx, sy = self._survey_dir
        target_spread = self.current_target_spread
        half = target_spread / 2.0

        target_a = (mx + half * sx, my + half * sy)
        target_b = (mx - half * sx, my - half * sy)

        fused_a = self.enkf_a.mean
        fused_b = self.enkf_b.mean
        ax = float(fused_a[STATE_X])
        ay = float(fused_a[STATE_Y])
        bx = float(fused_b[STATE_X])
        by = float(fused_b[STATE_Y])

        actual_mid_x = (ax + bx) / 2.0
        actual_mid_y = (ay + by) / 2.0
        midpoint_drift = math.hypot(actual_mid_x - mx, actual_mid_y - my)

        actual_spread = math.hypot(ax - bx, ay - by)
        spread_error = actual_spread - target_spread

        self.metrics.midpoint_drift_samples.append(midpoint_drift)
        self.metrics.spread_error_samples.append(spread_error)
        self.metrics.target_spread_samples.append(target_spread)

        at_max = target_spread >= cfg.end_spread_m
        dist_a = math.hypot(ax - target_a[0], ay - target_a[1])
        dist_b = math.hypot(bx - target_b[0], by - target_b[1])
        if at_max and dist_a < cfg.target_tolerance_m and dist_b < cfg.target_tolerance_m:
            self._complete = True
            return _brake_cmd(), _brake_cmd()

        cmd_a = self._steer_toward(fused_a, target_a, dist_a)
        cmd_b = self._steer_toward(fused_b, target_b, dist_b)
        return cmd_a, cmd_b

    def _steer_toward(
        self,
        fused: np.ndarray,
        target: tuple[float, float],
        dist: float,
    ) -> CommandBus:
        rx = float(fused[STATE_X])
        ry = float(fused[STATE_Y])
        rh = float(fused[STATE_HEADING])

        dx = target[0] - rx
        dy = target[1] - ry

        target_heading = math.atan2(dy, dx)
        alpha = _wrap_angle(target_heading - rh)

        cfg = self.config
        steer = _clamp(alpha / cfg.heading_saturation_rad, -1.0, 1.0)

        if dist < 1.0:
            throttle = cfg.approach_throttle * dist
        elif dist < 3.0:
            frac = dist / 3.0
            throttle = cfg.approach_throttle + frac * (
                cfg.cruise_throttle - cfg.approach_throttle
            )
        else:
            throttle = cfg.cruise_throttle

        return CommandBus(
            throttle=throttle, steer=steer, direction_mode=DirectionMode.CLOSED_LOOP
        )


def _brake_cmd() -> CommandBus:
    return CommandBus(brake=0.5, direction_mode=DirectionMode.CLOSED_LOOP)


def _wrap_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))
