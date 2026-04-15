import json
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from rover_sim_startup.main import create_app


@pytest.fixture
def client(tmp_path: Path):
    app = create_app(scenarios_dir=tmp_path)
    return TestClient(app)


def test_index_page_loads(client):
    resp = client.get("/")
    assert resp.status_code == 200
    assert "Scenarios" in resp.text


def test_new_scenario_page_loads(client):
    resp = client.get("/scenarios/new")
    assert resp.status_code == 200
    assert "New Scenario" in resp.text


def test_log_viewer_page_loads(client):
    resp = client.get("/log-viewer")
    assert resp.status_code == 200
    assert "Log Viewer" in resp.text


def test_json_schema_endpoint(client):
    resp = client.get("/api/schema")
    assert resp.status_code == 200
    schema = resp.json()
    assert "properties" in schema
    assert "name" in schema["properties"]


def test_scenario_crud_lifecycle(client, tmp_path: Path):
    scenario = {
        "name": "test_crud",
        "description": "CRUD test scenario",
        "duration_s": 5.0,
        "dt_s": 0.05,
        "controller": {"kind": "teleop"},
    }

    resp = client.post("/api/scenarios", json=scenario)
    assert resp.status_code == 200
    assert resp.json()["status"] == "created"

    resp = client.get("/api/scenarios")
    assert resp.status_code == 200
    names = [s["name"] for s in resp.json()]
    assert "test_crud" in names

    resp = client.get("/api/scenarios/test_crud")
    assert resp.status_code == 200
    assert resp.json()["name"] == "test_crud"

    resp = client.get("/scenarios/test_crud/edit")
    assert resp.status_code == 200
    assert "test_crud" in resp.text

    resp = client.delete("/api/scenarios/test_crud")
    assert resp.status_code == 200

    resp = client.get("/api/scenarios")
    names = [s["name"] for s in resp.json()]
    assert "test_crud" not in names


def test_mission_upload(client, tmp_path: Path):
    client.post("/api/scenarios", json={"name": "mission_test", "duration_s": 1.0})

    geojson = {
        "type": "FeatureCollection",
        "properties": {"survey_kind": "waypoint_survey", "params": {}},
        "features": [
            {
                "type": "Feature",
                "geometry": {"type": "LineString", "coordinates": [[0, 0], [10, 0]]},
                "properties": {"role": "path"},
            }
        ],
    }
    resp = client.post(
        "/api/scenarios/mission_test/mission",
        files={"file": ("mission.geojson", json.dumps(geojson).encode(), "application/json")},
    )
    assert resp.status_code == 200
    assert resp.json()["status"] == "uploaded"
    assert (tmp_path / "mission_test_mission.geojson").exists()


def test_log_analysis_single_rover(client):
    log_lines = [
        json.dumps({
            "t": i * 0.1,
            "truth": {"x": i * 0.5, "y": 0, "heading": 0, "speed": 0.5, "yaw_rate": 0, "gamma": 0, "v_left": 0, "v_right": 0},
            "rover_state": {"t": i * 0.1, "x": i * 0.5 + 0.01, "y": 0.01, "heading": 0, "speed": 0.5, "yaw_rate": 0, "fix_type": "fixed", "position_std_m": 0.015},
            "cmd": {"throttle": 0.5, "brake": 0, "steer": 0, "direction": "F", "neutral": False, "estop_handle": False, "estop_machine": False, "direction_mode": "closed"},
        })
        for i in range(20)
    ]
    log_content = "\n".join(log_lines)
    resp = client.post(
        "/api/log/analyze",
        files={"file": ("run.jsonl", log_content.encode(), "text/plain")},
    )
    assert resp.status_code == 200
    data = resp.json()
    assert data["mode"] == "single"
    assert data["n_steps"] == 20
    assert data["total_distance_m"] > 0
    assert "trajectory" in data
    assert len(data["trajectory"]["x"]) == 20


def test_404_for_missing_scenario(client):
    resp = client.get("/scenarios/nonexistent/edit")
    assert resp.status_code == 404
