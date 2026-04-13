import math

import pytest

from rover_drive.planning.rosette import rosette_waypoints, RosetteManeuver


def test_basic_rosette_returns_maneuver():
    m = rosette_waypoints(
        waypoint=(10, 0),
        arrival_heading=0.0,
        departure_heading=math.pi / 4,
        radius=2.0,
        overshoot_m=3.0,
    )
    assert m is not None
    assert isinstance(m, RosetteManeuver)
    assert len(m.loop_points) > 0


def test_rosette_loop_connects_overshoot_to_approach():
    wp = (10, 0)
    m = rosette_waypoints(
        waypoint=wp,
        arrival_heading=0.0,
        departure_heading=math.pi / 2,
        radius=2.0,
        overshoot_m=4.0,
    )
    assert m is not None
    # First loop point should be near the overshoot position (10+4, 0) = (14, 0)
    first = m.loop_points[0]
    assert first[0] > 12.0  # roughly east of waypoint

    # Last loop point should be near the approach position for departure heading
    # Approach = waypoint - overshoot * cos/sin(departure) = (10, 0-4) = (10, -4)
    last = m.loop_points[-1]
    assert last[1] < 0  # south of waypoint (approaching from below for northward departure)


def test_90_degree_rosette_has_reasonable_length():
    m = rosette_waypoints(
        waypoint=(0, 0),
        arrival_heading=0.0,
        departure_heading=math.pi / 2,
        radius=3.0,
        overshoot_m=5.0,
    )
    assert m is not None
    assert len(m.loop_points) > 10


def test_small_angle_rosette():
    """Even a small heading change should produce a valid rosette."""
    m = rosette_waypoints(
        waypoint=(5, 5),
        arrival_heading=0.0,
        departure_heading=math.radians(20),
        radius=2.0,
        overshoot_m=3.0,
    )
    assert m is not None
    assert len(m.loop_points) > 0


def test_180_degree_rosette():
    """A full reversal should produce a U-turn rosette."""
    m = rosette_waypoints(
        waypoint=(0, 0),
        arrival_heading=0.0,
        departure_heading=math.pi,
        radius=2.0,
        overshoot_m=3.0,
    )
    assert m is not None
    assert len(m.loop_points) > 10


def test_rosette_preserves_waypoint_and_headings():
    m = rosette_waypoints(
        waypoint=(7, 3),
        arrival_heading=0.5,
        departure_heading=1.2,
        radius=2.5,
    )
    assert m is not None
    assert m.waypoint == (7, 3)
    assert m.arrival_heading == pytest.approx(0.5)
    assert m.departure_heading == pytest.approx(1.2)
    assert m.radius == 2.5
