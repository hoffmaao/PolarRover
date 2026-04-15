"""LQR path-following controller — linearized error dynamics about the reference path.

Linearizes the cross-track and heading error dynamics of the bicycle model
about the reference path, then computes optimal state-feedback gains via the
discrete algebraic Riccati equation. At runtime, the controller is just a
matrix multiply — extremely fast and well-suited for real-time operation on
an embedded companion computer.

The state is [cross_track_error, heading_error], the input is [steer].
The linearized model (at constant speed v along the path):

    e_ct_dot  =  v * sin(e_h)  ≈  v * e_h        (small angle)
    e_h_dot   =  v / L * steer  -  v * kappa      (curvature tracking)

Discretized at dt:
    e[k+1] = A e[k] + B u[k]

where A = [[1, v*dt], [0, 1]], B = [[0], [v*dt/L]], and kappa is the path
curvature at the closest point (treated as a feedforward disturbance).

Reference: Snider (2009), CMU-RI-TR-09-08, Section 4.3.
"""

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from rover_sim.surveys import Waypoint
from rover_drive.controllers.base import ControlOutput, PathController


@dataclass
class LQRConfig:
    arm_length_m: float = 2.5
    q_crosstrack: float = 15.0       # state cost: cross-track error²
    q_heading: float = 5.0           # state cost: heading error²
    r_steer: float = 1.0             # input cost: steer²
    dt_design: float = 0.05          # design time step for discretization
    cruise_throttle: float = 0.25
    approach_throttle: float = 0.10
    heading_saturation_rad: float = math.radians(50.0)
    slowdown_radius_m: float = 3.0
    design_speed_mps: float = 1.4    # nominal speed for gain computation


class LQRController(PathController):
    """
    LQR path follower. Precomputes optimal gains at construction for the
    design speed, then applies them each step as:

        steer_raw = -K @ [cross_track_error, heading_error] + feedforward
        steer = clamp(steer_raw / heading_saturation, -1, 1)

    The feedforward term compensates for path curvature so the controller
    doesn't have to "discover" turns through error accumulation.
    """

    def __init__(self, config: LQRConfig | None = None) -> None:
        self.config = config or LQRConfig()
        self._K = self._solve_gains()

    def _solve_gains(self) -> np.ndarray:
        cfg = self.config
        v = cfg.design_speed_mps
        L = cfg.arm_length_m
        dt = cfg.dt_design

        A = np.array([[1.0, v * dt],
                      [0.0, 1.0]])
        B = np.array([[0.0],
                      [v * dt / L]])

        Q = np.diag([cfg.q_crosstrack, cfg.q_heading])
        R = np.array([[cfg.r_steer]])

        P = _dare(A, B, Q, R)
        K = np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P @ A)
        return K  # shape (1, 2)

    def compute(self, rx, ry, rh, rv, path, closest_idx):
        cfg = self.config

        ci, signed_ct = _signed_cross_track(rx, ry, rh, path, closest_idx)
        tangent_h = _tangent_heading(path, ci)
        heading_err = _wrap(rh - tangent_h)
        kappa = _curvature(path, ci)

        error = np.array([signed_ct, heading_err])
        steer_raw = float(-self._K @ error)

        # Feedforward for path curvature
        if abs(rv) > 0.05:
            steer_raw += cfg.arm_length_m * kappa

        steer = _clamp(steer_raw / cfg.heading_saturation_rad, -1, 1)

        dist_to_end = _remaining(path, ci, rx, ry)
        if dist_to_end < cfg.slowdown_radius_m:
            frac = dist_to_end / cfg.slowdown_radius_m
            throttle = cfg.approach_throttle + frac * (cfg.cruise_throttle - cfg.approach_throttle)
        else:
            throttle = cfg.cruise_throttle

        return ControlOutput(steer=steer, throttle=throttle, cross_track_m=abs(signed_ct))


def _dare(A, B, Q, R, max_iter=200):
    """Solve the discrete algebraic Riccati equation via iteration."""
    P = Q.copy()
    for _ in range(max_iter):
        K = np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P @ A)
        P_new = Q + A.T @ P @ (A - B @ K)
        if np.max(np.abs(P_new - P)) < 1e-10:
            return P_new
        P = P_new
    return P


def _signed_cross_track(rx, ry, rh, path, hint):
    best_idx = hint
    best_d2 = (path[hint].x - rx)**2 + (path[hint].y - ry)**2
    for i in range(max(0, hint - 3), min(len(path), hint + 15)):
        d2 = (path[i].x - rx)**2 + (path[i].y - ry)**2
        if d2 < best_d2:
            best_d2 = d2
            best_idx = i
    cp = path[best_idx]
    dx, dy = cp.x - rx, cp.y - ry
    th = _tangent_heading(path, best_idx)
    sign = math.cos(th) * dy - math.sin(th) * dx
    return best_idx, math.copysign(math.sqrt(best_d2), sign)


def _tangent_heading(path, idx):
    if idx < len(path) - 1:
        return math.atan2(path[idx+1].y - path[idx].y, path[idx+1].x - path[idx].x)
    elif idx > 0:
        return math.atan2(path[idx].y - path[idx-1].y, path[idx].x - path[idx-1].x)
    return 0.0


def _curvature(path, idx):
    """Estimate path curvature at idx using three consecutive points."""
    if idx <= 0 or idx >= len(path) - 1:
        return 0.0
    p0, p1, p2 = path[idx-1], path[idx], path[idx+1]
    dx1, dy1 = p1.x - p0.x, p1.y - p0.y
    dx2, dy2 = p2.x - p1.x, p2.y - p1.y
    cross = dx1 * dy2 - dy1 * dx2
    d1 = math.hypot(dx1, dy1)
    d2 = math.hypot(dx2, dy2)
    if d1 < 1e-9 or d2 < 1e-9:
        return 0.0
    return 2.0 * cross / (d1 * d2 * (d1 + d2))


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
