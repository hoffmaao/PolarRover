"""Autonomous waypoint-following drive mode."""

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from rover_sim.control import CommandBus, DirectionMode, FixType, RoverState
from rover_sim.surveys import Survey

from rover_drive.estimation import (
    STATE_HEADING,
    STATE_X,
    STATE_Y,
    EnsembleKalmanFilter,
)


@dataclass
class WaypointControllerConfig:
    """Tunings for the waypoint-following controller."""

    cruise_throttle: float = 0.3       # commanded throttle in steady-state cruise
    approach_throttle: float = 0.12    # minimum throttle when approaching a waypoint
    waypoint_tolerance_m: float = 1.0  # radius within which a waypoint counts as "reached"
    slowdown_radius_m: float = 3.0     # begin linear slowdown inside this radius
    heading_saturation_rad: float = math.radians(45.0)  # heading error at which steer saturates


class WaypointDriver:
    """
    Canonical autonomous drive mode: goal-seeking waypoint follower.

    On every step, the driver:
      1. Predicts the EnKF forward by dt using the last commanded action, and
         fuses any fresh GNSS observation into the ensemble.
      2. Reads the fused state estimate (x, y, heading) from the filter
         instead of the raw GNSS sample, so transient dropouts and degraded
         fixes don't destabilize the control loop.
      3. Advances through reached waypoints (those closer than
         ``waypoint_tolerance_m`` to the fused position).
      4. Computes a proportional steering command from the heading error to
         the current active waypoint, saturating at ``heading_saturation_rad``.
      5. Linearly tapers throttle from ``cruise_throttle`` down to
         ``approach_throttle`` inside ``slowdown_radius_m`` of the current
         active waypoint, so we don't overshoot.
      6. Emits the command in closed-loop direction mode so the vehicle
         steering cylinder tracks the normalized steer value directly.

    Once all waypoints are reached, the driver brakes the rover to a stop
    and flags ``complete = True``.

    This is the *goal-seeking* variant of path following — the metric of
    success is "did we visit every waypoint within tolerance?" The Phase 5
    multipass driver will use a path-fidelity controller with a different
    tuning and metric (cross-track RMS), sharing the EnKF integration.
    """

    def __init__(
        self,
        config: Optional[WaypointControllerConfig] = None,
        enkf: Optional[EnsembleKalmanFilter] = None,
    ) -> None:
        self.config = config or WaypointControllerConfig()
        self.enkf = enkf or EnsembleKalmanFilter()
        self._active: int = 0
        self._complete: bool = False
        self._initialized: bool = False
        self._last_obs_t: float = -math.inf
        self._last_cmd: Optional[CommandBus] = None

    # ---------- public API ----------

    @property
    def active_waypoint_index(self) -> int:
        return self._active

    @property
    def complete(self) -> bool:
        return self._complete

    def reset(self) -> None:
        self._active = 0
        self._complete = False
        self._initialized = False
        self._last_obs_t = -math.inf
        self._last_cmd = None
        self.enkf.reset()

    def update(
        self,
        state: RoverState,
        mission: Optional[Survey],
        dt: float,
    ) -> CommandBus:
        if mission is None:
            raise RuntimeError("WaypointDriver requires a mission with waypoints")
        if not mission.waypoints:
            raise RuntimeError("WaypointDriver requires a non-empty mission")

        self._step_filter(state, dt)
        cmd = self._compute_command(mission)
        self._last_cmd = cmd
        return cmd

    # ---------- internal ----------

    def _step_filter(self, state: RoverState, dt: float) -> None:
        if not self._initialized:
            if state.fix_type is FixType.NONE:
                return  # wait for a valid fix before initializing
            x0 = np.array(
                [state.x, state.y, state.heading, state.speed, state.yaw_rate],
                dtype=float,
            )
            self.enkf.initialize(x0)
            self._initialized = True
            self._last_obs_t = state.t
            return

        self.enkf.predict(self._last_cmd, dt)
        # Only fuse a *new* observation. The runner caches the last-good rover
        # state between GNSS ticks, so we'd otherwise double-count the same fix.
        if state.fix_type is not FixType.NONE and state.t > self._last_obs_t + 1e-9:
            self.enkf.update_from_gnss(state)
            self._last_obs_t = state.t

    def _compute_command(self, mission: Survey) -> CommandBus:
        if not self._initialized:
            return _brake_cmd()
        if self._complete:
            return _brake_cmd()

        fused = self.enkf.mean
        rx = float(fused[STATE_X])
        ry = float(fused[STATE_Y])
        rh = float(fused[STATE_HEADING])

        # Advance through any waypoints already within tolerance
        while self._active < len(mission.waypoints):
            wp = mission.waypoints[self._active]
            if math.hypot(wp.x - rx, wp.y - ry) < self.config.waypoint_tolerance_m:
                self._active += 1
            else:
                break

        if self._active >= len(mission.waypoints):
            self._complete = True
            return _brake_cmd()

        wp = mission.waypoints[self._active]
        dx = wp.x - rx
        dy = wp.y - ry
        dist = math.hypot(dx, dy)

        target_heading = math.atan2(dy, dx)
        alpha = _wrap_angle(target_heading - rh)

        cfg = self.config
        steer = _clamp(alpha / cfg.heading_saturation_rad, -1.0, 1.0)

        if dist >= cfg.slowdown_radius_m:
            throttle = cfg.cruise_throttle
        else:
            frac = dist / cfg.slowdown_radius_m
            throttle = cfg.approach_throttle + frac * (
                cfg.cruise_throttle - cfg.approach_throttle
            )

        return CommandBus(
            throttle=throttle,
            steer=steer,
            direction_mode=DirectionMode.CLOSED_LOOP,
        )


def _brake_cmd() -> CommandBus:
    return CommandBus(brake=0.5, direction_mode=DirectionMode.CLOSED_LOOP)


def _wrap_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))
