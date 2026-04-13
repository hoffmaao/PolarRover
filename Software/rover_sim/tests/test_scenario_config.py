from pathlib import Path

import pytest
import yaml

from rover_sim.config import ControllerConfig, ScenarioConfig, VehicleConfig


def test_default_controller_is_teleop():
    cfg = ScenarioConfig()
    assert cfg.controller.kind == "teleop"
    assert cfg.controller.params == {}


def test_default_vehicle_type_is_single_track():
    cfg = ScenarioConfig()
    assert cfg.vehicle.type == "single_track"
    assert cfg.vehicle.direction_mode == "closed"


def test_defaults_reflect_mtt154_hardware_caps():
    cfg = ScenarioConfig()
    # 20 km/h hardware cap
    assert cfg.vehicle.kinematics.max_speed_mps == pytest.approx(5.56)
    # 15-second auto-neutral timeout from the owner's manual
    assert cfg.vehicle.safety.auto_neutral_timeout_s == 15.0
    # 10 Hz GNSS matches ZED-F9P typical rate
    assert cfg.vehicle.gnss.rate_hz == 10.0


def test_yaml_roundtrip(tmp_path: Path):
    original = ScenarioConfig(
        name="roundtrip_test",
        description="yaml round-trip",
        duration_s=42.0,
        dt_s=0.1,
    )
    yaml_text = original.to_yaml()
    reloaded = ScenarioConfig.model_validate(yaml.safe_load(yaml_text))
    assert reloaded.name == "roundtrip_test"
    assert reloaded.duration_s == 42.0
    assert reloaded.dt_s == 0.1
    assert reloaded.controller.kind == "teleop"


def test_from_yaml_file(tmp_path: Path):
    path = tmp_path / "scenario.yaml"
    path.write_text(
        """
name: from_yaml_test
duration_s: 5.0
dt_s: 0.05
vehicle:
  type: single_track
  kinematics:
    arm_length_m: 3.2
controller:
  kind: teleop
  params:
    source: scripted_inline
    commands:
      - t: 0.0
        throttle: 0.5
"""
    )
    cfg = ScenarioConfig.from_yaml(path)
    assert cfg.name == "from_yaml_test"
    assert cfg.vehicle.kinematics.arm_length_m == 3.2
    assert cfg.controller.params["source"] == "scripted_inline"
    assert len(cfg.controller.params["commands"]) == 1


def test_empty_yaml_uses_full_defaults(tmp_path: Path):
    path = tmp_path / "empty.yaml"
    path.write_text("")
    cfg = ScenarioConfig.from_yaml(path)
    assert cfg.name == "unnamed"
    assert cfg.controller.kind == "teleop"


def test_partial_vehicle_section_keeps_other_defaults():
    cfg = ScenarioConfig.model_validate(
        {"vehicle": {"kinematics": {"arm_length_m": 4.0}}}
    )
    assert cfg.vehicle.kinematics.arm_length_m == 4.0
    # Other kinematics fields should remain at defaults
    assert cfg.vehicle.kinematics.max_speed_mps == pytest.approx(5.56)
    assert cfg.vehicle.type == "single_track"
