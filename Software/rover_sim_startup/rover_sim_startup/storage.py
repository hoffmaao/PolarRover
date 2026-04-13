"""Scenario persistence — YAML files on disk."""

import shutil
from pathlib import Path
from typing import Optional

from rover_sim.config import ScenarioConfig


def list_scenarios(directory: Path) -> list[dict]:
    if not directory.exists():
        return []
    result = []
    for f in sorted(directory.glob("*.yaml")):
        try:
            cfg = ScenarioConfig.from_yaml(f)
            result.append({
                "name": cfg.name,
                "description": cfg.description,
                "controller": cfg.controller.kind,
                "duration_s": cfg.duration_s,
                "has_mission": cfg.mission_path is not None,
            })
        except Exception:
            result.append({"name": f.stem, "description": "(invalid YAML)", "controller": "?",
                           "duration_s": 0, "has_mission": False})
    return result


def load_scenario(directory: Path, name: str) -> ScenarioConfig:
    path = directory / f"{name}.yaml"
    if not path.exists():
        raise FileNotFoundError(f"scenario {name!r} not found in {directory}")
    return ScenarioConfig.from_yaml(path)


def save_scenario(directory: Path, config: ScenarioConfig) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{config.name}.yaml"
    path.write_text(config.to_yaml())
    return path


def delete_scenario(directory: Path, name: str) -> None:
    path = directory / f"{name}.yaml"
    if path.exists():
        path.unlink()
    mission_path = directory / f"{name}_mission.geojson"
    if mission_path.exists():
        mission_path.unlink()


def save_mission(directory: Path, scenario_name: str, geojson_bytes: bytes) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{scenario_name}_mission.geojson"
    path.write_bytes(geojson_bytes)
    return path
