import json
import math
from pathlib import Path

import pytest

from rover_sim.config import ControllerConfig, ScenarioConfig

from rover_sim_emulator.cmp_runner import LinkedCMPRunner


def _write_cmp_mission(path: Path, midpoint=(0.0, 0.0), direction=(1.0, 0.0)) -> None:
    end = (midpoint[0] + direction[0] * 50, midpoint[1] + direction[1] * 50)
    data = {
        "type": "FeatureCollection",
        "crs": {"type": "name", "properties": {"name": "urn:ogc:def:crs:EPSG::3031"}},
        "features": [{
            "type": "Feature",
            "geometry": {"type": "LineString", "coordinates": [list(midpoint), list(end)]},
            "properties": {"role": "centerline", "survey_kind": "cmp_survey"},
        }],
    }
    path.write_text(json.dumps(data))


def test_cmp_rovers_spread_outward(tmp_path: Path):
    survey_path = tmp_path / "cmp.geojson"
    _write_cmp_mission(survey_path)

    cfg = ScenarioConfig(
        name="cmp_spread_test",
        seed=7,
        duration_s=40.0,
        dt_s=0.05,
        log_path=str(tmp_path / "cmp.jsonl"),
        survey_path=str(survey_path),
        controller=ControllerConfig(
            kind="linked_cmp",
            params={
                "start_spread_m": 2.0,
                "end_spread_m": 15.0,
                "spread_rate_m_per_s": 0.5,
                "cruise_throttle": 0.25,
                "heading_saturation_deg": 40.0,
                "enkf_seed": 7,
            },
        ),
    )
    runner = LinkedCMPRunner(cfg)
    summary = runner.run()

    assert summary["controller"] == "linked_cmp"
    assert Path(summary["log_path"]).exists()

    # Rovers should have spread outward — final spread > start spread
    assert summary["final_spread_m"] > 5.0
    # Midpoint should stay near the origin (fixed midpoint M)
    assert summary["midpoint_drift_rms_m"] < 3.0

    # Check log format
    lines = Path(summary["log_path"]).read_text().strip().split("\n")
    first = json.loads(lines[0])
    assert "rover_a" in first
    assert "formation" in first

    ax, ay = summary["rover_a_final"]
    bx, by = summary["rover_b_final"]
    # Rover A should be in +x direction, Rover B in -x direction (east survey axis)
    assert ax > 0
    assert bx < 0


def test_initial_poses_along_survey_axis(tmp_path: Path):
    survey_path = tmp_path / "cmp.geojson"
    _write_cmp_mission(survey_path, midpoint=(10, 5), direction=(1, 0))

    cfg = ScenarioConfig(
        name="pose_check",
        seed=1,
        duration_s=0.1,
        dt_s=0.05,
        log_path=str(tmp_path / "check.jsonl"),
        survey_path=str(survey_path),
        controller=ControllerConfig(
            kind="linked_cmp",
            params={"start_spread_m": 4.0, "enkf_seed": 1},
        ),
    )
    runner = LinkedCMPRunner(cfg)
    sa = runner.vehicle_a.state
    sb = runner.vehicle_b.state

    # Survey axis is east (+x). Rovers start along that axis, not perpendicular.
    # Rover A at (10+2, 5), heading east. Rover B at (10-2, 5), heading west.
    assert sa.x == pytest.approx(12.0, abs=0.01)
    assert sa.y == pytest.approx(5.0, abs=0.01)
    assert sb.x == pytest.approx(8.0, abs=0.01)
    assert sb.y == pytest.approx(5.0, abs=0.01)
    # Headings: A east (0), B west (pi)
    assert abs(sa.heading) < 0.01
    assert abs(abs(sb.heading) - math.pi) < 0.01


def test_missing_mission_raises(tmp_path: Path):
    cfg = ScenarioConfig(
        name="no_mission", duration_s=1.0, dt_s=0.05,
        log_path=str(tmp_path / "out.jsonl"),
        controller=ControllerConfig(kind="linked_cmp"),
    )
    with pytest.raises(ValueError, match="survey_path"):
        LinkedCMPRunner(cfg)


def test_dispatch_routes_to_cmp_runner(tmp_path: Path):
    survey_path = tmp_path / "cmp.geojson"
    _write_cmp_mission(survey_path)

    cfg = ScenarioConfig(
        name="dispatch_test", seed=0, duration_s=2.0, dt_s=0.05,
        log_path=str(tmp_path / "dispatch.jsonl"),
        survey_path=str(survey_path),
        controller=ControllerConfig(
            kind="linked_cmp",
            params={"start_spread_m": 2.0, "end_spread_m": 4.0, "enkf_seed": 0},
        ),
    )
    from rover_sim_emulator.dispatch import run_scenario
    summary = run_scenario(cfg)
    assert summary["controller"] == "linked_cmp"
    assert Path(summary["log_path"]).exists()
