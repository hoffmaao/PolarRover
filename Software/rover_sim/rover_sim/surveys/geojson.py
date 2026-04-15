"""GeoJSON mission loader and writer — QGIS/ArcGIS compatible.

Survey files are standard GeoJSON FeatureCollections that open directly in
GIS software (QGIS, ArcGIS, Google Earth). The CRS is declared via a "crs"
member (GeoJSON 2008 / QGIS convention) and all rover-specific metadata
lives in per-feature properties (visible as attribute table columns in GIS).

Coordinate system: EPSG:3031 (Antarctic Polar Stereographic).

All coordinates are in meters (easting, northing) on the Antarctic Polar
Stereographic projection. This is a conformal projection where headings,
distances, and angles are locally accurate — unlike EPSG:4326 (WGS84 lat/lon)
where spherical geometry makes straight-line headings between waypoints
inaccurate at polar latitudes. The rover's GNSS (ZED-F9P) outputs WGS84
positions which must be projected to EPSG:3031 before use.

EPSG:3031 is the standard CRS for Antarctic research used by NSIDC, BAS,
and most polar science programs. If Arctic operations are needed in the
future, EPSG:3413 (NSIDC Polar Stereographic North) uses the same approach.
"""

import json
from pathlib import Path
from typing import Any, Optional, Union

from rover_sim.surveys.base import Survey, SurveyKind, Waypoint


# ============================================================
# Loading
# ============================================================


def load_survey(path: Union[str, Path]) -> Survey:
    """Load a mission from a GeoJSON file."""
    data = json.loads(Path(path).read_text())
    return from_geojson_dict(data)


def from_geojson_dict(data: dict[str, Any]) -> Survey:
    """Parse a GeoJSON FeatureCollection into a typed Survey."""
    if data.get("type") != "FeatureCollection":
        raise ValueError(f"expected FeatureCollection, got {data.get('type')!r}")

    # Survey kind: check per-feature first (GIS-friendly), fall back to top-level
    kind_str = None
    for f in data.get("features", []):
        fk = (f.get("properties") or {}).get("survey_kind")
        if fk:
            kind_str = fk
            break
    if kind_str is None:
        kind_str = data.get("properties", {}).get("survey_kind")
    if kind_str is None:
        raise ValueError("mission.geojson missing survey_kind (in feature properties or top-level)")

    try:
        kind = SurveyKind(kind_str)
    except ValueError as e:
        raise ValueError(f"unknown survey_kind: {kind_str!r}") from e

    features = data.get("features", [])
    coords = _extract_base_line(features, kind)
    waypoints = [Waypoint(x=c[0], y=c[1]) for c in coords]

    # Collect params from features or top-level
    params: dict[str, Any] = {}
    for f in features:
        fp = (f.get("properties") or {}).get("params")
        if isinstance(fp, dict):
            params.update(fp)
    top_params = data.get("properties", {}).get("params")
    if isinstance(top_params, dict):
        params.update(top_params)

    # Check CRS — reject WGS84 lat/lon
    crs = data.get("crs", {})
    if crs:
        crs_name = crs.get("properties", {}).get("name", "")
        if "4326" in crs_name:
            raise ValueError(
                "This mission file uses EPSG:4326 (WGS84 lat/lon). Nisse "
                "requires projected coordinates in EPSG:3031 (Antarctic Polar "
                "Stereographic) because heading calculations between waypoints "
                "are inaccurate in spherical coordinates at polar latitudes. "
                "Re-project the file to EPSG:3031 using QGIS, ogr2ogr, or "
                "pyproj before loading."
            )
        params["_crs"] = crs
    else:
        # No CRS declared — check if coordinates look like lat/lon
        if waypoints and all(abs(w.x) <= 180 and abs(w.y) <= 90 for w in waypoints[:5]):
            raise ValueError(
                "Coordinates appear to be WGS84 lat/lon (values in the "
                "[-180,180] / [-90,90] range) but no CRS is declared. "
                "Nisse requires EPSG:3031 (Antarctic Polar Stereographic) "
                "coordinates in meters. Re-project and add a CRS declaration."
            )

    mission = Survey(kind=kind, waypoints=waypoints, params=params)

    if kind == SurveyKind.MULTIPASS:
        mission = _expand_multipass(mission)
    return mission


# ============================================================
# Writing (QGIS/ArcGIS compatible)
# ============================================================


def write_mission(
    path: Union[str, Path],
    kind: SurveyKind,
    coordinates: list[list[float]],
    epsg: int = 3031,
    params: Optional[dict[str, Any]] = None,
    name: str = "mission",
    description: str = "",
) -> Path:
    """
    Write a QGIS/ArcGIS-compatible mission GeoJSON file.

    Coordinates must be in EPSG:3031 Antarctic Polar Stereographic
    (easting, northing in meters). The output opens directly in GIS
    software with correct projection and useful attribute columns.

    EPSG:4326 (WGS84 lat/lon) is NOT supported because spherical
    geometry makes heading calculations between waypoints inaccurate
    at polar latitudes. Project to EPSG:3031 before calling this.
    """
    if epsg == 4326:
        raise ValueError(
            "EPSG:4326 (WGS84 lat/lon) is not supported — headings between "
            "waypoints are inaccurate in spherical coordinates at polar "
            "latitudes. Project coordinates to EPSG:3031 (Antarctic Polar "
            "Stereographic) first."
        )
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    role = _ROLE_FOR_KIND[kind]
    params = params or {}

    doc = {
        "type": "FeatureCollection",
        "name": name,
        "crs": {
            "type": "name",
            "properties": {"name": f"urn:ogc:def:crs:EPSG::{epsg}"}
        },
        "features": [
            {
                "type": "Feature",
                "geometry": {
                    "type": "LineString",
                    "coordinates": coordinates,
                },
                "properties": {
                    "name": name,
                    "description": description,
                    "role": role,
                    "survey_kind": kind.value,
                    "survey_version": 1,
                    "epsg": epsg,
                    "n_points": len(coordinates),
                    "params": params,
                },
            }
        ],
    }

    # Add waypoint markers for waypoint surveys
    if kind == SurveyKind.WAYPOINT:
        for i, coord in enumerate(coordinates):
            doc["features"].append({
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": coord},
                "properties": {
                    "name": f"WP{i + 1}",
                    "role": "waypoint",
                    "sequence": i + 1,
                    "survey_kind": kind.value,
                },
            })

    # Add midpoint marker for CMP surveys
    if kind == SurveyKind.CMP and len(coordinates) >= 1:
        doc["features"].append({
            "type": "Feature",
            "geometry": {"type": "Point", "coordinates": coordinates[0]},
            "properties": {
                "name": "Midpoint (base station)",
                "role": "midpoint",
                "survey_kind": kind.value,
            },
        })

    path.write_text(json.dumps(doc, indent=2))
    return path


# ============================================================
# Internal
# ============================================================


_ROLE_FOR_KIND: dict[SurveyKind, str] = {
    SurveyKind.WAYPOINT: "path",
    SurveyKind.MULTIPASS: "base_track",
    SurveyKind.CMP: "centerline",
}


def _extract_base_line(
    features: list[dict[str, Any]], kind: SurveyKind
) -> list[tuple[float, float]]:
    expected_role = _ROLE_FOR_KIND[kind]

    for f in features:
        role = (f.get("properties") or {}).get("role")
        if role != expected_role:
            continue
        geom = f.get("geometry") or {}
        if geom.get("type") == "LineString":
            return [(c[0], c[1]) for c in geom["coordinates"]]

    for f in features:
        geom = f.get("geometry") or {}
        if geom.get("type") == "LineString":
            return [(c[0], c[1]) for c in geom["coordinates"]]

    raise ValueError(f"no LineString feature found for {kind.value}")


def _expand_multipass(mission: Survey) -> Survey:
    """Expand a multipass base track into the full N-pass sequence."""
    n_passes = int(mission.params.get("n_passes", 1))
    direction = mission.params.get("pass_direction", "alternating")
    base = mission.waypoints
    if n_passes <= 1 or len(base) < 2:
        return mission

    sequence: list[Waypoint] = []
    for i in range(n_passes):
        if direction == "alternating" and i % 2 == 1:
            sequence.extend(reversed(base))
        else:
            sequence.extend(base)
    return Survey(kind=mission.kind, waypoints=sequence, params=dict(mission.params))
