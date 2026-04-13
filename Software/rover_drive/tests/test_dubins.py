import math

import pytest

from rover_drive.planning.dubins import DubinsPath, DubinsType, dubins_path


def _approx_pose(path_points, target, tol=0.3):
    """Check that a sampled path ends near the target pose."""
    if not path_points:
        return False
    last = path_points[-1]
    dx = abs(last[0] - target[0])
    dy = abs(last[1] - target[1])
    return dx < tol and dy < tol


# ---------- basic functionality ----------


def test_straight_line_path():
    """Start and end on the same heading, directly ahead → should be a straight line."""
    path = dubins_path((0, 0, 0), (10, 0, 0), radius=2.0)
    assert path is not None
    assert path.total_length == pytest.approx(10.0, rel=0.05)
    pts = path.sample(spacing=0.5)
    assert len(pts) > 0
    assert _approx_pose(pts, (10, 0, 0))


def test_180_degree_turn():
    """Start heading east, end at same point heading west → needs a U-turn."""
    path = dubins_path((0, 0, 0), (0, 0, math.pi), radius=2.0)
    assert path is not None
    assert path.total_length > 0
    pts = path.sample(spacing=0.5)
    assert _approx_pose(pts, (0, 0, math.pi), tol=0.5)


def test_90_degree_left_turn():
    """Heading east, target is north → left turn."""
    path = dubins_path((0, 0, 0), (5, 5, math.pi / 2), radius=2.0)
    assert path is not None
    pts = path.sample(spacing=0.3)
    assert len(pts) > 5
    assert _approx_pose(pts, (5, 5, math.pi / 2), tol=0.5)


def test_90_degree_right_turn():
    """Heading east, target is south → right turn."""
    path = dubins_path((0, 0, 0), (5, -5, -math.pi / 2), radius=2.0)
    assert path is not None
    pts = path.sample(spacing=0.3)
    assert len(pts) > 5
    assert _approx_pose(pts, (5, -5, -math.pi / 2), tol=0.5)


# ---------- radius constraints ----------


def test_smaller_radius_gives_shorter_path():
    path_tight = dubins_path((0, 0, 0), (5, 5, math.pi / 2), radius=1.0)
    path_wide = dubins_path((0, 0, 0), (5, 5, math.pi / 2), radius=3.0)
    assert path_tight is not None and path_wide is not None
    assert path_tight.total_length < path_wide.total_length


def test_zero_radius_raises():
    with pytest.raises(ValueError):
        dubins_path((0, 0, 0), (5, 0, 0), radius=0.0)


# ---------- path types ----------


def test_returns_a_valid_dubins_type():
    path = dubins_path((0, 0, 0), (10, 5, math.pi / 4), radius=2.0)
    assert path is not None
    assert path.path_type in DubinsType


# ---------- sampling ----------


def test_sampling_produces_dense_points():
    path = dubins_path((0, 0, 0), (10, 0, 0), radius=2.0)
    pts = path.sample(spacing=0.2)
    assert len(pts) >= 40  # 10 m at 0.2 m spacing


def test_sampled_points_progress_monotonically_for_straight():
    path = dubins_path((0, 0, 0), (10, 0, 0), radius=2.0)
    pts = path.sample(spacing=0.5)
    xs = [p[0] for p in pts]
    for i in range(1, len(xs)):
        assert xs[i] >= xs[i - 1] - 0.01


# ---------- symmetry ----------


def test_left_right_symmetric():
    """Mirrored targets should give mirrored path lengths."""
    left = dubins_path((0, 0, 0), (5, 5, math.pi / 2), radius=2.0)
    right = dubins_path((0, 0, 0), (5, -5, -math.pi / 2), radius=2.0)
    assert left is not None and right is not None
    assert left.total_length == pytest.approx(right.total_length, rel=0.01)
