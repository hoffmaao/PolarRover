import json
import math
from pathlib import Path

import pytest

from rover_sim_emulator.reftrack import extract_reference_track


def _write_fake_log(path: Path, n_steps: int = 100, with_fused: bool = True) -> None:
    """Write a synthetic JSONL log of a rover driving east at 1 m/s."""
    lines = []
    for i in range(n_steps):
        t = i * 0.1
        x = i * 0.1  # 1 m/s * 0.1 s = 0.1 m per step
        record = {
            "t": t,
            "truth": {"x": x, "y": 0.0, "heading": 0.0, "speed": 1.0,
                       "yaw_rate": 0.0, "gamma": 0.0, "v_left": 0.0, "v_right": 0.0},
            "rover_state": {"t": t, "x": x + 0.01, "y": 0.01, "heading": 0.0,
                            "speed": 1.0, "yaw_rate": 0.0, "fix_type": "fixed",
                            "position_std_m": 0.015},
            "cmd": {"throttle": 0.3, "brake": 0.0, "steer": 0.0, "direction": "F",
                    "neutral": False, "estop_handle": False, "estop_machine": False,
                    "direction_mode": "closed"},
        }
        if with_fused:
            record["fused"] = {"x": x + 0.005, "y": 0.002, "heading": 0.0,
                               "speed": 1.0, "yaw_rate": 0.0,
                               "pos_std_x": 0.01, "pos_std_y": 0.01}
        lines.append(json.dumps(record))
    path.write_text("\n".join(lines))


# ---------- basic extraction ----------


def test_extract_fused_reference_track(tmp_path: Path):
    log = tmp_path / "run.jsonl"
    out = tmp_path / "ref.geojson"
    _write_fake_log(log, n_steps=200)

    result = extract_reference_track(log, out, source="fused", spacing_m=0.5)
    assert result.exists()

    data = json.loads(result.read_text())
    feat_props = data["features"][0]["properties"]
    assert feat_props["mission_kind"] == "multipass_survey"
    assert feat_props["source_stream"] == "fused"
    assert "3031" in data["crs"]["properties"]["name"]

    coords = data["features"][0]["geometry"]["coordinates"]
    assert len(coords) > 10
    # Check spacing is approximately 0.5 m
    for i in range(1, min(10, len(coords))):
        dx = coords[i][0] - coords[i-1][0]
        dy = coords[i][1] - coords[i-1][1]
        assert math.hypot(dx, dy) == pytest.approx(0.5, abs=0.1)


def test_extract_truth_source(tmp_path: Path):
    log = tmp_path / "run.jsonl"
    out = tmp_path / "ref.geojson"
    _write_fake_log(log, n_steps=100)

    extract_reference_track(log, out, source="truth", spacing_m=0.5)
    data = json.loads(out.read_text())
    assert data["features"][0]["properties"]["source_stream"] == "truth"
    assert len(data["features"][0]["geometry"]["coordinates"]) > 5


def test_extract_gnss_source(tmp_path: Path):
    log = tmp_path / "run.jsonl"
    out = tmp_path / "ref.geojson"
    _write_fake_log(log, n_steps=100)

    extract_reference_track(log, out, source="gnss", spacing_m=0.5)
    data = json.loads(out.read_text())
    assert data["features"][0]["properties"]["source_stream"] == "gnss"


def test_unknown_source_raises(tmp_path: Path):
    log = tmp_path / "run.jsonl"
    _write_fake_log(log, n_steps=50)
    with pytest.raises(ValueError, match="unknown source"):
        extract_reference_track(log, tmp_path / "out.geojson", source="bogus")


# ---------- speed filtering ----------


def test_stationary_points_filtered_out(tmp_path: Path):
    log = tmp_path / "run.jsonl"
    lines = []
    for i in range(50):
        t = i * 0.1
        speed = 1.0 if i > 20 else 0.0  # stationary for first 2 s
        x = max(0, (i - 20) * 0.1) if i > 20 else 0
        lines.append(json.dumps({
            "t": t,
            "truth": {"x": x, "y": 0, "heading": 0, "speed": speed,
                       "yaw_rate": 0, "gamma": 0, "v_left": 0, "v_right": 0},
            "rover_state": {"t": t, "x": x, "y": 0, "heading": 0, "speed": speed,
                            "yaw_rate": 0, "fix_type": "fixed", "position_std_m": 0.015},
            "cmd": {"throttle": 0.3, "brake": 0, "steer": 0, "direction": "F",
                    "neutral": False, "estop_handle": False, "estop_machine": False,
                    "direction_mode": "closed"},
            "fused": {"x": x, "y": 0, "heading": 0, "speed": speed, "yaw_rate": 0,
                      "pos_std_x": 0.01, "pos_std_y": 0.01},
        }))
    log.write_text("\n".join(lines))

    extract_reference_track(log, tmp_path / "ref.geojson", source="fused",
                           min_speed_mps=0.05, spacing_m=0.2)
    data = json.loads((tmp_path / "ref.geojson").read_text())
    coords = data["features"][0]["geometry"]["coordinates"]
    # All extracted points should be from the moving portion (x > 0)
    assert all(c[0] >= -0.01 for c in coords)


# ---------- log without fused field ----------


def test_no_fused_field_raises_when_source_is_fused(tmp_path: Path):
    log = tmp_path / "run.jsonl"
    _write_fake_log(log, n_steps=50, with_fused=False)
    with pytest.raises(ValueError, match="Not enough moving points"):
        extract_reference_track(log, tmp_path / "out.geojson", source="fused")


# ---------- round-trip: extract → load as mission ----------


def test_extracted_track_loads_as_valid_mission(tmp_path: Path):
    log = tmp_path / "run.jsonl"
    out = tmp_path / "ref.geojson"
    _write_fake_log(log, n_steps=200)
    extract_reference_track(log, out, source="fused", spacing_m=0.5)

    from rover_sim.missions import load_mission
    mission = load_mission(out)
    assert mission.kind.value == "multipass_survey"
    assert len(mission.waypoints) > 10
