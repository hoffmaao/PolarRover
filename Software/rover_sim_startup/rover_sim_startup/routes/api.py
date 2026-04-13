"""JSON API routes."""

import json
from typing import Any

from fastapi import APIRouter, Request, UploadFile, File
from fastapi.responses import JSONResponse

from rover_sim.config import ScenarioConfig

from rover_sim_startup.storage import (
    delete_scenario,
    list_scenarios,
    load_scenario,
    save_mission,
    save_scenario,
)

router = APIRouter()


@router.get("/schema")
async def json_schema():
    return ScenarioConfig.model_json_schema()


@router.get("/scenarios")
async def api_list_scenarios(request: Request):
    return list_scenarios(request.app.state.scenarios_dir)


@router.post("/scenarios")
async def api_create_scenario(request: Request):
    body = await request.json()
    config = ScenarioConfig.model_validate(body)
    path = save_scenario(request.app.state.scenarios_dir, config)
    return {"status": "created", "name": config.name, "path": str(path)}


@router.get("/scenarios/{name}")
async def api_get_scenario(request: Request, name: str):
    try:
        cfg = load_scenario(request.app.state.scenarios_dir, name)
    except FileNotFoundError:
        return JSONResponse({"error": f"not found: {name}"}, status_code=404)
    return cfg.model_dump(mode="json")


@router.put("/scenarios/{name}")
async def api_update_scenario(request: Request, name: str):
    body = await request.json()
    config = ScenarioConfig.model_validate(body)
    if config.name != name:
        delete_scenario(request.app.state.scenarios_dir, name)
    save_scenario(request.app.state.scenarios_dir, config)
    return {"status": "updated", "name": config.name}


@router.delete("/scenarios/{name}")
async def api_delete_scenario(request: Request, name: str):
    delete_scenario(request.app.state.scenarios_dir, name)
    return {"status": "deleted", "name": name}


@router.post("/scenarios/{name}/mission")
async def api_upload_mission(request: Request, name: str, file: UploadFile = File(...)):
    content = await file.read()
    sdir = request.app.state.scenarios_dir
    path = save_mission(sdir, name, content)
    try:
        cfg = load_scenario(sdir, name)
        cfg.mission_path = str(path)
        save_scenario(sdir, cfg)
    except FileNotFoundError:
        pass
    return {"status": "uploaded", "path": str(path)}


@router.post("/log/analyze")
async def api_analyze_log(file: UploadFile = File(...)):
    content = await file.read()
    lines = content.decode().strip().split("\n")
    records = [json.loads(line) for line in lines if line.strip()]
    if not records:
        return {"error": "empty log file"}
    return _analyze_records(records)


def _analyze_records(records: list[dict[str, Any]]) -> dict[str, Any]:
    import math

    is_cmp = "rover_a" in records[0]
    duration = records[-1]["t"] if records else 0.0

    if is_cmp:
        return _analyze_cmp(records, duration)
    return _analyze_single(records, duration)


def _analyze_single(records: list[dict], duration: float) -> dict:
    import math

    truths = [r["truth"] for r in records]
    states = [r["rover_state"] for r in records]
    cmds = [r["cmd"] for r in records]

    xs = [t["x"] for t in truths]
    ys = [t["y"] for t in truths]
    total_dist = sum(
        math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]) for i in range(1, len(xs))
    )
    speeds = [t.get("speed", 0) for t in truths]
    throttles = [c["throttle"] for c in cmds]

    fix_counts: dict[str, int] = {}
    for s in states:
        ft = s.get("fix_type", "unknown")
        fix_counts[ft] = fix_counts.get(ft, 0) + 1

    return {
        "mode": "single",
        "duration_s": duration,
        "n_steps": len(records),
        "total_distance_m": total_dist,
        "max_speed_mps": max(abs(s) for s in speeds) if speeds else 0,
        "mean_throttle": sum(throttles) / len(throttles) if throttles else 0,
        "fix_distribution": fix_counts,
        "trajectory": {"x": xs, "y": ys},
        "noisy_trajectory": {"x": [s["x"] for s in states], "y": [s["y"] for s in states]},
    }


def _analyze_cmp(records: list[dict], duration: float) -> dict:
    import math

    formations = [r.get("formation", {}) for r in records]
    rover_a = [r["rover_a"]["truth"] for r in records]
    rover_b = [r["rover_b"]["truth"] for r in records]

    spreads = [f.get("actual_spread_m", 0) for f in formations]
    midpoints_x = [f.get("midpoint_x", 0) for f in formations]
    midpoints_y = [f.get("midpoint_y", 0) for f in formations]

    return {
        "mode": "linked_cmp",
        "duration_s": duration,
        "n_steps": len(records),
        "mean_spread_m": sum(spreads) / len(spreads) if spreads else 0,
        "trajectory_a": {"x": [t["x"] for t in rover_a], "y": [t["y"] for t in rover_a]},
        "trajectory_b": {"x": [t["x"] for t in rover_b], "y": [t["y"] for t in rover_b]},
        "midpoint_trajectory": {"x": midpoints_x, "y": midpoints_y},
    }
