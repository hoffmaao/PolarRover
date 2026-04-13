import json
import math
from pathlib import Path

import pytest

from rover_sim.config import ControllerConfig, ScenarioConfig, VehicleConfig

from rover_sim_emulator.runner import ScenarioRunner


def _scripted_forward_scenario(tmp_path: Path, duration_s: float = 2.0) -> ScenarioConfig:
    return ScenarioConfig(
        name="scripted_forward",
        duration_s=duration_s,
        dt_s=0.05,
        log_path=str(tmp_path / "scripted_forward.jsonl"),
        controller=ControllerConfig(
            kind="teleop",
            params={
                "source": "scripted_inline",
                "commands": [{"t": 0.0, "throttle": 0.5, "direction_mode": "closed"}],
            },
        ),
    )


def test_scripted_teleop_moves_rover_forward(tmp_path: Path):
    cfg = _scripted_forward_scenario(tmp_path, duration_s=3.0)
    runner = ScenarioRunner(cfg)
    summary = runner.run()

    assert summary["controller"] == "teleop"
    # Forward throttle for 3 s at 50% of 5.56 m/s — account for acceleration ramp
    assert summary["final_x"] > 2.0
    assert abs(summary["final_y"]) < 0.5


def test_idle_teleop_leaves_rover_stationary(tmp_path: Path):
    cfg = ScenarioConfig(
        name="idle_demo",
        duration_s=0.5,
        dt_s=0.05,
        log_path=str(tmp_path / "idle.jsonl"),
        # ControllerConfig default: teleop with no source → IdleCommandSource
    )
    runner = ScenarioRunner(cfg)
    summary = runner.run()
    assert summary["controller"] == "teleop"
    assert abs(summary["final_x"]) < 0.01
    assert abs(summary["final_y"]) < 0.01
    assert abs(summary["final_speed_mps"]) < 0.01


def test_log_file_has_one_record_per_step(tmp_path: Path):
    duration_s = 1.0
    dt_s = 0.05
    cfg = _scripted_forward_scenario(tmp_path, duration_s=duration_s)
    cfg.dt_s = dt_s
    runner = ScenarioRunner(cfg)
    runner.run()

    log_path = Path(cfg.log_path)
    assert log_path.exists()
    lines = log_path.read_text().strip().split("\n")
    assert len(lines) == int(round(duration_s / dt_s))

    # Each line is a self-contained JSON record with the expected keys
    first = json.loads(lines[0])
    assert set(first) == {"t", "truth", "rover_state", "cmd"}
    assert "x" in first["truth"]
    assert "fix_type" in first["rover_state"]
    assert "throttle" in first["cmd"]


def test_yaml_roundtrip_via_runner(tmp_path: Path):
    yaml_path = tmp_path / "scenario.yaml"
    log_path = tmp_path / "run.jsonl"
    yaml_path.write_text(
        f"""
name: yaml_demo
duration_s: 1.0
dt_s: 0.05
log_path: {log_path}
controller:
  kind: teleop
  params:
    source: scripted_inline
    commands:
      - t: 0.0
        throttle: 0.3
        direction_mode: closed
"""
    )
    cfg = ScenarioConfig.from_yaml(yaml_path)
    runner = ScenarioRunner(cfg)
    summary = runner.run()

    assert summary["name"] == "yaml_demo"
    assert log_path.exists()
    assert summary["final_x"] > 0.0


def test_unknown_controller_kind_raises(tmp_path: Path):
    cfg = ScenarioConfig(
        name="bogus",
        duration_s=0.1,
        dt_s=0.05,
        log_path=str(tmp_path / "bogus.jsonl"),
        controller=ControllerConfig(kind="nonsense"),
    )
    with pytest.raises(ValueError, match="unknown controller kind"):
        ScenarioRunner(cfg)


def test_unknown_vehicle_type_raises(tmp_path: Path):
    cfg = ScenarioConfig(
        name="bogus_vehicle",
        duration_s=0.1,
        dt_s=0.05,
        log_path=str(tmp_path / "bogus.jsonl"),
        vehicle=VehicleConfig(type="imaginary_vehicle"),
    )
    with pytest.raises(ValueError, match="unknown vehicle type"):
        ScenarioRunner(cfg)


def _write_waypoint_mission(path: Path, *coords: tuple[float, float]) -> None:
    data = {
        "type": "FeatureCollection",
        "crs": {"type": "name", "properties": {"name": "urn:ogc:def:crs:EPSG::3031"}},
        "features": [
            {
                "type": "Feature",
                "geometry": {"type": "LineString", "coordinates": [list(c) for c in coords]},
                "properties": {"role": "path", "mission_kind": "waypoint_survey"},
            }
        ],
    }
    path.write_text(json.dumps(data))


def test_waypoint_scenario_reaches_all_waypoints(tmp_path: Path):
    mission_path = tmp_path / "mission.geojson"
    _write_waypoint_mission(mission_path, (5.0, 0.0), (10.0, 5.0), (15.0, 0.0))

    cfg = ScenarioConfig(
        name="waypoint_e2e",
        seed=42,
        duration_s=90.0,
        dt_s=0.05,
        log_path=str(tmp_path / "waypoint.jsonl"),
        mission_path=str(mission_path),
        controller=ControllerConfig(
            kind="waypoint",
            params={
                "cruise_throttle": 0.4,
                "waypoint_tolerance_m": 1.0,
                "slowdown_radius_m": 2.0,
                "heading_saturation_deg": 45.0,
                "enkf_seed": 1,
            },
        ),
    )
    runner = ScenarioRunner(cfg)
    summary = runner.run()

    # The rover should visit all three waypoints within the time budget
    assert runner.driver.complete, (
        f"driver did not complete: active_idx={runner.driver.active_waypoint_index}, "
        f"final=({summary['final_x']:.2f}, {summary['final_y']:.2f})"
    )
    # And end roughly near the last waypoint (it'll overshoot a little, then brake)
    assert math.hypot(summary["final_x"] - 15.0, summary["final_y"] - 0.0) < 3.0


def test_multipass_scenario_reports_cross_track(tmp_path: Path):
    mission_path = tmp_path / "multipass.geojson"
    _write_waypoint_mission(mission_path, *[(float(x), 0.0) for x in range(0, 25)])
    # Re-tag as multipass
    import json as _json

    data = _json.loads(mission_path.read_text())
    data["features"][0]["properties"]["role"] = "base_track"
    data["features"][0]["properties"]["mission_kind"] = "multipass_survey"
    mission_path.write_text(_json.dumps(data))

    cfg = ScenarioConfig(
        name="multipass_e2e",
        seed=7,
        duration_s=60.0,
        dt_s=0.05,
        log_path=str(tmp_path / "multipass.jsonl"),
        mission_path=str(mission_path),
        controller=ControllerConfig(
            kind="multipass",
            params={
                "cruise_throttle": 0.3,
                "lookahead_m": 3.0,
                "heading_saturation_deg": 40.0,
                "enkf_seed": 7,
            },
        ),
    )
    runner = ScenarioRunner(cfg)
    summary = runner.run()

    metrics = runner.driver.metrics
    assert len(metrics.cross_track_samples) > 0
    assert metrics.cross_track_rms_m < 2.0


def test_waypoint_without_mission_raises(tmp_path: Path):
    cfg = ScenarioConfig(
        name="waypoint_no_mission",
        duration_s=1.0,
        dt_s=0.05,
        log_path=str(tmp_path / "no_mission.jsonl"),
        # mission_path not set
        controller=ControllerConfig(kind="waypoint", params={}),
    )
    with pytest.raises(ValueError, match="mission_path"):
        ScenarioRunner(cfg)


def test_cli_demo_command(tmp_path: Path, monkeypatch, capsys):
    # Redirect the ./runs default path into tmp_path for the duration of the test
    monkeypatch.chdir(tmp_path)
    from rover_sim_emulator.cli import main

    rc = main(["demo"])
    assert rc == 0
    out = capsys.readouterr().out
    assert "demo_teleop" in out
    assert "teleop" in out

    # The demo writes to ./runs/demo_teleop.jsonl under the cwd
    log = Path("runs/demo_teleop.jsonl")
    assert log.exists()
    assert len(log.read_text().strip().split("\n")) > 0
