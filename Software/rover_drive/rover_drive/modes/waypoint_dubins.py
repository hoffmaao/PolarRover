"""Dubins + rosette waypoint driver — smooth kinematically feasible paths
with optional radar crossing patterns at waypoints.

The planner maintains a running "current pose" that tracks the end of the
last planned segment, ensuring continuity between segments. For rosette
mode, the rover drives THROUGH the waypoint, executes a Dubins loop, and
crosses the waypoint again on the departure heading before continuing.
"""

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from rover_sim.control import CommandBus, DirectionMode, FixType, RoverState
from rover_sim.missions import Mission, Waypoint

from rover_drive.estimation import (
    STATE_HEADING, STATE_SPEED, STATE_X, STATE_Y,
    EnsembleKalmanFilter,
)
from rover_drive.planning.dubins import dubins_path
from rover_drive.planning.rosette import rosette_waypoints


@dataclass
class DubinsWaypointConfig:
    min_turn_radius_m: float = 3.0
    approach_mode: str = "dubins"
    rosette_overshoot_m: float = 5.0
    path_spacing_m: float = 0.5
    waypoint_tolerance_m: float = 2.0
    cruise_throttle: float = 0.5
    approach_throttle: float = 0.15
    heading_saturation_rad: float = math.radians(45.0)
    slowdown_radius_m: float = 5.0
    k_crosstrack: float = 1.5
    k_soft: float = 0.3
    # Turn speed scheduling: taper speed based on upcoming path curvature
    curvature_speed_enabled: bool = True
    curvature_lookahead_m: float = 15.0     # look this far ahead for curvature
    max_comfortable_curvature: float = 0.15  # above this, start slowing (1/R_comfortable)
    min_turn_throttle: float = 0.2           # minimum throttle in tight curves


class DubinsWaypointDriver:
    """Pre-plans smooth paths through waypoints, then follows with Stanley tracking."""

    def __init__(self, config: Optional[DubinsWaypointConfig] = None,
                 enkf: Optional[EnsembleKalmanFilter] = None) -> None:
        self.config = config or DubinsWaypointConfig()
        self.enkf = enkf or EnsembleKalmanFilter()
        self._planned_path: list[tuple[float, float, float]] = []
        self._path_waypoints: list[Waypoint] = []
        self._closest_idx: int = 0
        self._complete: bool = False
        self._initialized: bool = False
        self._last_obs_t: float = -math.inf
        self._last_cmd: Optional[CommandBus] = None
        self._planned: bool = False

    @property
    def complete(self): return self._complete
    @property
    def planned_path(self): return self._planned_path
    @property
    def planned_waypoints(self): return self._path_waypoints

    def reset(self):
        self._planned_path.clear(); self._path_waypoints.clear()
        self._closest_idx = 0; self._complete = False
        self._initialized = False; self._last_obs_t = -math.inf
        self._last_cmd = None; self._planned = False; self.enkf.reset()

    def update(self, state: RoverState, mission: Optional[Mission], dt: float) -> CommandBus:
        if mission is None or not mission.waypoints:
            raise RuntimeError("DubinsWaypointDriver requires a non-empty mission")
        self._step_filter(state, dt)
        if not self._initialized: return _brake_cmd()
        if not self._planned:
            fused = self.enkf.mean
            self._plan_path((float(fused[STATE_X]), float(fused[STATE_Y]), float(fused[STATE_HEADING])), mission)
            self._planned = True
        cmd = self._track_planned_path()
        self._last_cmd = cmd
        return cmd

    # ---------- path planning with proper continuity ----------

    def _plan_path(self, start_pose: tuple[float, float, float], mission: Mission) -> None:
        cfg = self.config
        wps = mission.waypoints
        mode = mission.params.get("approach_mode", cfg.approach_mode)
        R = cfg.min_turn_radius_m
        sp = cfg.path_spacing_m

        # Compute target heading at each waypoint (direction toward next waypoint)
        wp_poses: list[tuple[float, float, float]] = []
        for i, wp in enumerate(wps):
            if i < len(wps) - 1:
                h = math.atan2(wps[i+1].y - wp.y, wps[i+1].x - wp.x)
            else:
                h = math.atan2(wp.y - wps[i-1].y, wp.x - wps[i-1].x) if i > 0 else 0.0
            wp_poses.append((wp.x, wp.y, h))

        path: list[tuple[float, float, float]] = []
        cursor = start_pose  # running end-of-last-segment pose

        for i, wp_pose in enumerate(wp_poses):
            arrival_heading = math.atan2(wp_pose[1] - cursor[1], wp_pose[0] - cursor[0])
            if i < len(wp_poses) - 1:
                departure_heading = math.atan2(wp_poses[i+1][1] - wp_pose[1], wp_poses[i+1][0] - wp_pose[0])
            else:
                departure_heading = arrival_heading

            # --- Transit leg: always a plain Dubins curve to get to the waypoint ---
            wp_arrive = (wp_pose[0], wp_pose[1], arrival_heading)
            transit = dubins_path(cursor, wp_arrive, R)
            if transit:
                pts = transit.sample(sp)
                path.extend(pts)
                cursor = pts[-1] if pts else wp_arrive
            else:
                cursor = wp_arrive

            # --- Waypoint behavior: depends on the configured approach mode ---
            # The transit leg above is mode-independent (just get there smoothly).
            # What happens AT the waypoint differs by mode:
            #   direct: nothing extra, just continue to next transit
            #   dubins: nothing extra (the transit IS the Dubins curve)
            #   rosette: drive through, loop, cross again on departure heading

            if mode == "rosette":
                # Phase 1: Drive through waypoint by overshoot distance
                overshoot_x = wp_pose[0] + cfg.rosette_overshoot_m * math.cos(arrival_heading)
                overshoot_y = wp_pose[1] + cfg.rosette_overshoot_m * math.sin(arrival_heading)
                n_ov = max(1, int(cfg.rosette_overshoot_m / sp))
                for k in range(1, n_ov + 1):
                    t = k / n_ov
                    path.append((cursor[0] + t * (overshoot_x - cursor[0]),
                                 cursor[1] + t * (overshoot_y - cursor[1]), arrival_heading))
                cursor = (overshoot_x, overshoot_y, arrival_heading)

                # Phase 2: Rosette loop — Dubins from overshoot back to approach point
                approach_x = wp_pose[0] - cfg.rosette_overshoot_m * math.cos(departure_heading)
                approach_y = wp_pose[1] - cfg.rosette_overshoot_m * math.sin(departure_heading)
                approach_pose = (approach_x, approach_y, departure_heading)
                loop = dubins_path(cursor, approach_pose, R)
                if loop:
                    pts = loop.sample(sp)
                    path.extend(pts)
                    cursor = pts[-1] if pts else approach_pose
                else:
                    cursor = approach_pose

                # Phase 3: Drive through waypoint again on departure heading
                through_x = wp_pose[0] + cfg.rosette_overshoot_m * math.cos(departure_heading)
                through_y = wp_pose[1] + cfg.rosette_overshoot_m * math.sin(departure_heading)
                n_th = max(1, int(2 * cfg.rosette_overshoot_m / sp))
                for k in range(1, n_th + 1):
                    t = k / n_th
                    path.append((cursor[0] + t * (through_x - cursor[0]),
                                 cursor[1] + t * (through_y - cursor[1]), departure_heading))
                cursor = (through_x, through_y, departure_heading)

            elif mode == "dubins":
                # Dubins mode: the transit IS the waypoint behavior — nothing extra
                pass

            else:  # direct
                # Direct mode: transit already moved cursor to waypoint
                pass

        self._planned_path = path
        self._path_waypoints = [Waypoint(x=p[0], y=p[1]) for p in path]

    # ---------- path tracking ----------

    def _track_planned_path(self) -> CommandBus:
        if self._complete or not self._path_waypoints: return _brake_cmd()
        cfg = self.config
        fused = self.enkf.mean
        rx, ry = float(fused[STATE_X]), float(fused[STATE_Y])
        rh, rv = float(fused[STATE_HEADING]), float(fused[STATE_SPEED])

        ci = self._find_closest(rx, ry)
        self._closest_idx = ci
        dist_end = self._remaining_dist(ci, rx, ry)
        if dist_end < cfg.waypoint_tolerance_m and ci >= len(self._planned_path) - 3:
            self._complete = True; return _brake_cmd()

        # Lookahead on the planned path
        look = min(ci + max(1, int(5.0 / cfg.path_spacing_m)), len(self._planned_path) - 1)

        # Feedforward: align with path heading
        path_h = self._planned_path[ci][2]
        heading_err = _wrap(path_h - rh)

        # Cross-track feedback (Stanley)
        _, signed_ct = self._signed_ct(rx, ry, rh, ci)
        ct_term = math.atan2(cfg.k_crosstrack * signed_ct, abs(rv) + cfg.k_soft)

        steer = _clamp((heading_err + ct_term) / cfg.heading_saturation_rad, -1, 1)

        # Speed scheduling: slow for upcoming curvature, slow near end
        throttle = cfg.cruise_throttle

        if cfg.curvature_speed_enabled and len(self._planned_path) > ci + 5:
            max_curv = self._lookahead_curvature(ci, cfg.curvature_lookahead_m)
            if max_curv > cfg.max_comfortable_curvature:
                curv_ratio = min(1.0, max_curv / (cfg.max_comfortable_curvature * 3))
                throttle = cfg.cruise_throttle - curv_ratio * (cfg.cruise_throttle - cfg.min_turn_throttle)

        if dist_end < cfg.slowdown_radius_m:
            frac = dist_end / cfg.slowdown_radius_m
            end_throttle = cfg.approach_throttle + frac * (cfg.cruise_throttle - cfg.approach_throttle)
            throttle = min(throttle, end_throttle)

        return CommandBus(throttle=throttle, steer=steer, direction_mode=DirectionMode.CLOSED_LOOP)

    def _find_closest(self, rx, ry):
        best = self._closest_idx
        best_d2 = (self._path_waypoints[best].x - rx)**2 + (self._path_waypoints[best].y - ry)**2
        for i in range(max(0, best - 3), min(len(self._path_waypoints), best + 30)):
            d2 = (self._path_waypoints[i].x - rx)**2 + (self._path_waypoints[i].y - ry)**2
            if d2 < best_d2: best_d2 = d2; best = i
        return best

    def _signed_ct(self, rx, ry, rh, ci):
        pp = self._planned_path; cp = pp[ci]
        dx, dy = cp[0] - rx, cp[1] - ry; th = cp[2]
        sign = math.cos(th) * dy - math.sin(th) * dx
        return ci, math.copysign(math.hypot(dx, dy), sign)

    def _remaining_dist(self, ci, rx, ry):
        wps = self._path_waypoints
        if ci >= len(wps) - 1: return math.hypot(wps[-1].x - rx, wps[-1].y - ry)
        total = math.hypot(wps[ci].x - rx, wps[ci].y - ry)
        for i in range(ci, min(ci + 200, len(wps) - 1)):
            total += math.hypot(wps[i+1].x - wps[i].x, wps[i+1].y - wps[i].y)
        return total

    def _lookahead_curvature(self, ci, distance_m):
        """Find the maximum curvature on the planned path within the lookahead distance."""
        pp = self._planned_path
        n = len(pp)
        max_curv = 0.0
        accum = 0.0
        for i in range(ci, min(ci + int(distance_m / self.config.path_spacing_m) + 1, n - 1)):
            if i > 0 and i < n - 1:
                ax = pp[i][0] - pp[i-1][0]; ay = pp[i][1] - pp[i-1][1]
                bx = pp[i+1][0] - pp[i][0]; by = pp[i+1][1] - pp[i][1]
                cross = ax * by - ay * bx
                da = math.hypot(ax, ay); db = math.hypot(bx, by)
                if da > 1e-9 and db > 1e-9:
                    curv = abs(2.0 * cross / (da * db * (da + db)))
                    max_curv = max(max_curv, curv)
            accum += math.hypot(pp[min(i+1,n-1)][0]-pp[i][0], pp[min(i+1,n-1)][1]-pp[i][1])
            if accum > distance_m: break
        return max_curv

    def _step_filter(self, state, dt):
        if not self._initialized:
            if state.fix_type is FixType.NONE: return
            self.enkf.initialize(np.array([state.x, state.y, state.heading, state.speed, state.yaw_rate], dtype=float))
            self._initialized = True; self._last_obs_t = state.t; return
        self.enkf.predict(self._last_cmd, dt)
        if state.fix_type is not FixType.NONE and state.t > self._last_obs_t + 1e-9:
            self.enkf.update_from_gnss(state); self._last_obs_t = state.t


def _brake_cmd(): return CommandBus(brake=0.5, direction_mode=DirectionMode.CLOSED_LOOP)
def _wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi
def _clamp(x, lo, hi): return max(lo, min(hi, x))
