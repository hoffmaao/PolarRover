from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class SurveyKind(Enum):
    """Matches the `survey_kind` metadata tag in a mission.geojson file."""

    WAYPOINT = "waypoint_survey"
    MULTIPASS = "multipass_survey"
    CMP = "cmp_survey"


@dataclass(frozen=True)
class Waypoint:
    """
    A single reference point along a survey path. Units are whatever frame
    the survey was loaded into — the simulator and the field rover both
    expect coordinates already projected to the scenario CRS (EPSG:3031
    for the polar deployment), not raw WGS84 lat/lon.
    """

    x: float
    y: float
    dwell_s: float = 0.0


@dataclass
class Survey:
    """
    A loaded mission: a list of waypoints plus mission-kind-specific params.
    The controller that consumes this mission is selected from `kind`:
      WAYPOINT  → goal-seeking pure pursuit with waypoint tolerance
      MULTIPASS → path-fidelity controller minimizing cross-track RMS
      CMP       → two-rover coordinated drive around a shared midpoint
    """

    kind: SurveyKind
    waypoints: list[Waypoint] = field(default_factory=list)
    params: dict[str, Any] = field(default_factory=dict)
