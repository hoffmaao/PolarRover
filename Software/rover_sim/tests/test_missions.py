import json
from pathlib import Path

import pytest

from rover_sim.surveys import Survey, SurveyKind, Waypoint, from_geojson_dict, load_survey


_CRS_3031 = {"type": "name", "properties": {"name": "urn:ogc:def:crs:EPSG::3031"}}


def _waypoint_doc() -> dict:
    return {
        "type": "FeatureCollection",
        "crs": _CRS_3031,
        "properties": {
            "survey_kind": "waypoint_survey",
            "survey_version": 1,
            "params": {"speed_target_mps": 1.0, "waypoint_tolerance_m": 1.0},
        },
        "features": [
            {
                "type": "Feature",
                "geometry": {
                    "type": "LineString",
                    "coordinates": [[-250000, 500000], [-249900, 500000], [-249800, 500000]],
                },
                "properties": {"role": "path", "survey_kind": "waypoint_survey"},
            }
        ],
    }


def _multipass_doc(n_passes: int, direction: str = "alternating") -> dict:
    return {
        "type": "FeatureCollection",
        "crs": _CRS_3031,
        "properties": {
            "survey_kind": "multipass_survey",
            "survey_version": 1,
            "params": {
                "n_passes": n_passes,
                "pass_direction": direction,
                "lateral_offset_m": 0.0,
            },
        },
        "features": [
            {
                "type": "Feature",
                "geometry": {
                    "type": "LineString",
                    "coordinates": [[-250000, 500000], [-249999, 500000], [-249998, 500000]],
                },
                "properties": {"role": "base_track", "survey_kind": "multipass_survey"},
            }
        ],
    }


def _cmp_doc() -> dict:
    return {
        "type": "FeatureCollection",
        "crs": _CRS_3031,
        "properties": {
            "survey_kind": "cmp_survey",
            "survey_version": 1,
            "params": {
                "spread_mode": "constant",
                "start_spread_m": 30.0,
                "end_spread_m": 30.0,
                "baseline": "perpendicular_to_tangent",
            },
        },
        "features": [
            {
                "type": "Feature",
                "geometry": {
                    "type": "LineString",
                    "coordinates": [[-250000, 500000], [-249900, 500000]],
                },
                "properties": {"role": "centerline", "survey_kind": "cmp_survey"},
            }
        ],
    }


def test_waypoint_loader():
    m = from_geojson_dict(_waypoint_doc())
    assert m.kind is SurveyKind.WAYPOINT
    assert len(m.waypoints) == 3
    assert m.waypoints[0] == Waypoint(x=-250000, y=500000)
    assert m.params["speed_target_mps"] == 1.0


def test_multipass_alternating_reverses_odd_passes():
    m = from_geojson_dict(_multipass_doc(n_passes=2, direction="alternating"))
    assert m.kind is SurveyKind.MULTIPASS
    assert len(m.waypoints) == 6
    assert [w.x for w in m.waypoints[:3]] == [-250000, -249999, -249998]
    assert [w.x for w in m.waypoints[3:]] == [-249998, -249999, -250000]


def test_multipass_same_direction_repeats_identically():
    m = from_geojson_dict(_multipass_doc(n_passes=3, direction="same"))
    assert len(m.waypoints) == 9
    block_x = [w.x for w in m.waypoints[:3]]
    for i in range(3):
        assert [w.x for w in m.waypoints[i * 3 : (i + 1) * 3]] == block_x


def test_multipass_single_pass_passthrough():
    m = from_geojson_dict(_multipass_doc(n_passes=1))
    assert len(m.waypoints) == 3


def test_cmp_loader_keeps_centerline_only():
    m = from_geojson_dict(_cmp_doc())
    assert m.kind is SurveyKind.CMP
    assert len(m.waypoints) == 2
    assert m.params["start_spread_m"] == 30.0
    assert m.params["baseline"] == "perpendicular_to_tangent"


def test_unknown_mission_kind_raises():
    doc = _waypoint_doc()
    doc["properties"]["survey_kind"] = "bogus_survey"
    doc["features"][0]["properties"]["survey_kind"] = "bogus_survey"
    with pytest.raises(ValueError, match="unknown survey_kind"):
        from_geojson_dict(doc)


def test_missing_mission_kind_raises():
    doc = _waypoint_doc()
    del doc["properties"]["survey_kind"]
    del doc["features"][0]["properties"]["survey_kind"]
    with pytest.raises(ValueError, match="survey_kind"):
        from_geojson_dict(doc)


def test_not_a_feature_collection_raises():
    with pytest.raises(ValueError, match="FeatureCollection"):
        from_geojson_dict({"type": "Feature"})


def test_no_linestring_raises():
    doc = _waypoint_doc()
    doc["features"] = []
    with pytest.raises(ValueError, match="no LineString"):
        from_geojson_dict(doc)


def test_role_hint_prefers_matching_feature():
    doc = _waypoint_doc()
    doc["features"].insert(
        0,
        {
            "type": "Feature",
            "geometry": {"type": "LineString", "coordinates": [[-300000, 600000]]},
            "properties": {"role": "distractor", "survey_kind": "waypoint_survey"},
        },
    )
    m = from_geojson_dict(doc)
    assert m.waypoints[0].x == -250000


def test_load_mission_from_disk(tmp_path: Path):
    f = tmp_path / "mission.geojson"
    f.write_text(json.dumps(_waypoint_doc()))
    m = load_survey(f)
    assert m.kind is SurveyKind.WAYPOINT
    assert len(m.waypoints) == 3


def test_wgs84_crs_rejected():
    doc = _waypoint_doc()
    doc["crs"] = {"type": "name", "properties": {"name": "urn:ogc:def:crs:EPSG::4326"}}
    with pytest.raises(ValueError, match="EPSG:4326"):
        from_geojson_dict(doc)


def test_latlon_coordinates_without_crs_rejected():
    doc = {
        "type": "FeatureCollection",
        "features": [{
            "type": "Feature",
            "geometry": {"type": "LineString", "coordinates": [[166.67, -77.85], [166.68, -77.85]]},
            "properties": {"role": "path", "survey_kind": "waypoint_survey"},
        }],
    }
    with pytest.raises(ValueError, match="WGS84 lat/lon"):
        from_geojson_dict(doc)
