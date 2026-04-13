"""Dubins path computation for bounded-curvature vehicles.

A Dubins path is the shortest path between two oriented poses (x, y, heading)
for a vehicle that can only move forward with a minimum turn radius R. Every
Dubins path consists of at most three segments chosen from {L, S, R}:
  L = left arc (counter-clockwise)
  S = straight
  R = right arc (clockwise)

The six possible types are: LSL, LSR, RSL, RSR, RLR, LRL.
We compute all six candidates and return the shortest feasible one.

Reference: Dubins, L.E. (1957). "On Curves of Minimal Length with a
Constraint on Average Curvature."
"""

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional

import numpy as np


class DubinsType(Enum):
    LSL = "LSL"
    LSR = "LSR"
    RSL = "RSL"
    RSR = "RSR"
    RLR = "RLR"
    LRL = "LRL"


@dataclass(frozen=True)
class DubinsPath:
    """A computed Dubins path between two poses."""

    path_type: DubinsType
    lengths: tuple[float, float, float]  # segment lengths (arc1, mid, arc2) in radians/meters
    radius: float
    start: tuple[float, float, float]    # (x, y, heading)
    end: tuple[float, float, float]
    total_length: float

    def sample(self, spacing: float = 0.5) -> list[tuple[float, float, float]]:
        """Sample the path at regular intervals, returning (x, y, heading) tuples."""
        points: list[tuple[float, float, float]] = []
        x, y, h = self.start
        R = self.radius
        seg_types = self.path_type.value  # e.g. "LSR"

        for seg_idx, (seg_char, seg_len) in enumerate(zip(seg_types, self.lengths)):
            if seg_len < 1e-9:
                continue

            if seg_char == "S":
                actual_len = seg_len * R
                n_steps = max(1, int(actual_len / spacing))
                for k in range(n_steps):
                    t = (k + 1) / n_steps * actual_len
                    px = x + t * math.cos(h)
                    py = y + t * math.sin(h)
                    points.append((px, py, h))
                x += actual_len * math.cos(h)
                y += actual_len * math.sin(h)

            elif seg_char == "L":
                cx = x - R * math.sin(h)
                cy = y + R * math.cos(h)
                n_steps = max(1, int(seg_len * R / spacing))
                for k in range(n_steps):
                    t = (k + 1) / n_steps * seg_len
                    nx = cx + R * math.sin(h + t)
                    ny = cy - R * math.cos(h + t)
                    points.append((nx, ny, _wrap(h + t)))
                h = _wrap(h + seg_len)
                x = cx + R * math.sin(h)
                y = cy - R * math.cos(h)

            elif seg_char == "R":
                cx = x + R * math.sin(h)
                cy = y - R * math.cos(h)
                n_steps = max(1, int(seg_len * R / spacing))
                for k in range(n_steps):
                    t = (k + 1) / n_steps * seg_len
                    nx = cx - R * math.sin(h - t)
                    ny = cy + R * math.cos(h - t)
                    points.append((nx, ny, _wrap(h - t)))
                h = _wrap(h - seg_len)
                x = cx - R * math.sin(h)
                y = cy + R * math.cos(h)

        return points


def dubins_path(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
) -> Optional[DubinsPath]:
    """
    Compute the shortest Dubins path from `start` to `end` with minimum
    turn radius `radius`. Each pose is (x, y, heading_rad).

    Returns the shortest feasible path, or None if no path exists (which
    shouldn't happen for valid inputs with R > 0).
    """
    if radius <= 0:
        raise ValueError(f"radius must be positive, got {radius}")

    dx = end[0] - start[0]
    dy = end[1] - start[1]
    d = math.hypot(dx, dy) / radius
    theta = math.atan2(dy, dx)
    alpha = _wrap(start[2] - theta)
    beta = _wrap(end[2] - theta)

    candidates: list[Optional[tuple[DubinsType, tuple[float, float, float]]]] = [
        _lsl(alpha, beta, d),
        _lsr(alpha, beta, d),
        _rsl(alpha, beta, d),
        _rsr(alpha, beta, d),
        _rlr(alpha, beta, d),
        _lrl(alpha, beta, d),
    ]

    best: Optional[DubinsPath] = None
    for c in candidates:
        if c is None:
            continue
        dtype, (t, p, q) = c
        total = (t + p + q) * radius
        if best is None or total < best.total_length:
            best = DubinsPath(
                path_type=dtype,
                lengths=(t, p, q),
                radius=radius,
                start=start,
                end=end,
                total_length=total,
            )
    return best


# ---------- six Dubins path type computations ----------


def _lsl(a: float, b: float, d: float):
    tmp = 2 + d * d - 2 * math.cos(a - b) + 2 * d * (math.sin(a) - math.sin(b))
    if tmp < 0:
        return None
    p = math.sqrt(tmp)
    t = _mod2pi(-a + math.atan2(math.cos(b) - math.cos(a), d + math.sin(a) - math.sin(b)))
    q = _mod2pi(b - math.atan2(math.cos(b) - math.cos(a), d + math.sin(a) - math.sin(b)))
    return (DubinsType.LSL, (t, p, q))


def _lsr(a: float, b: float, d: float):
    tmp = -2 + d * d + 2 * math.cos(a - b) + 2 * d * (math.sin(a) + math.sin(b))
    if tmp < 0:
        return None
    p = math.sqrt(tmp)
    t = _mod2pi(-a + math.atan2(-math.cos(a) - math.cos(b), d + math.sin(a) + math.sin(b)) - math.atan2(-2, p))
    q = _mod2pi(-_mod2pi(b) + math.atan2(-math.cos(a) - math.cos(b), d + math.sin(a) + math.sin(b)) - math.atan2(-2, p))
    return (DubinsType.LSR, (t, p, q))


def _rsl(a: float, b: float, d: float):
    tmp = -2 + d * d + 2 * math.cos(a - b) - 2 * d * (math.sin(a) + math.sin(b))
    if tmp < 0:
        return None
    p = math.sqrt(tmp)
    t = _mod2pi(a - math.atan2(math.cos(a) + math.cos(b), d - math.sin(a) - math.sin(b)) + math.atan2(2, p))
    q = _mod2pi(_mod2pi(b) - math.atan2(math.cos(a) + math.cos(b), d - math.sin(a) - math.sin(b)) + math.atan2(2, p))
    return (DubinsType.RSL, (t, p, q))


def _rsr(a: float, b: float, d: float):
    tmp = 2 + d * d - 2 * math.cos(a - b) - 2 * d * (math.sin(a) - math.sin(b))
    if tmp < 0:
        return None
    p = math.sqrt(tmp)
    t = _mod2pi(a - math.atan2(math.cos(a) - math.cos(b), d - math.sin(a) + math.sin(b)))
    q = _mod2pi(-b + math.atan2(math.cos(a) - math.cos(b), d - math.sin(a) + math.sin(b)))
    return (DubinsType.RSR, (t, p, q))


def _rlr(a: float, b: float, d: float):
    tmp = (6 - d * d + 2 * math.cos(a - b) + 2 * d * (math.sin(a) - math.sin(b))) / 8
    if abs(tmp) > 1:
        return None
    p = _mod2pi(2 * math.pi - math.acos(tmp))
    t = _mod2pi(a - math.atan2(math.cos(a) - math.cos(b), d - math.sin(a) + math.sin(b)) + _mod2pi(p / 2))
    q = _mod2pi(a - b - t + _mod2pi(p))
    return (DubinsType.RLR, (t, p, q))


def _lrl(a: float, b: float, d: float):
    tmp = (6 - d * d + 2 * math.cos(a - b) - 2 * d * (math.sin(a) - math.sin(b))) / 8
    if abs(tmp) > 1:
        return None
    p = _mod2pi(2 * math.pi - math.acos(tmp))
    t = _mod2pi(-a + math.atan2(-math.cos(a) + math.cos(b), d + math.sin(a) - math.sin(b)) + _mod2pi(p / 2))
    q = _mod2pi(_mod2pi(b) - a + 2 * _mod2pi(p / 2) - _mod2pi(t))
    return (DubinsType.LRL, (t, p, q))


# ---------- helpers ----------


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _mod2pi(a: float) -> float:
    return a % (2 * math.pi)
