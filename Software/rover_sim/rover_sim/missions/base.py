from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class MissionKind(Enum):
    """Matches the `mission_kind` metadata tag in a mission.geojson file."""

    WAYPOINT = "waypoint_survey"
    MULTIPASS = "multipass_survey"
    CMP = "cmp_survey"


@dataclass(frozen=True)
class Waypoint:
    """
    A single reference point along a mission path. Units are whatever frame
    the mission was loaded into. For Phase 2 we accept raw GeoJSON longitude /
    latitude (so x=lon, y=lat); a future phase will project through ugv_common
    geodesy before the controllers see them.
    """

    x: float
    y: float
    dwell_s: float = 0.0


@dataclass
class Mission:
    """
    A loaded mission: a list of waypoints plus mission-kind-specific params.
    The controller that consumes this mission is selected from `kind`:
      WAYPOINT  → goal-seeking pure pursuit with waypoint tolerance
      MULTIPASS → path-fidelity controller minimizing cross-track RMS
      CMP       → two-rover coordinated drive around a shared midpoint
    """

    kind: MissionKind
    waypoints: list[Waypoint] = field(default_factory=list)
    params: dict[str, Any] = field(default_factory=dict)
