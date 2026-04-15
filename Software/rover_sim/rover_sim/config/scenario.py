"""Pydantic scenario configuration models — the single source of truth for scenario.yaml schema."""

from pathlib import Path
from typing import Any, Optional

import yaml
from pydantic import BaseModel, Field


class InitialPose(BaseModel):
    """Starting pose of a vehicle at t=0. ENU meters + heading in radians."""

    x: float = 0.0
    y: float = 0.0
    heading_rad: float = 0.0


class KinematicsConfig(BaseModel):
    """
    Kinematic parameters for the vehicle. Most apply to SingleTrackArticulated;
    `track_width_m` is the side-by-side tank-mode track width. All are used by
    the runner to construct the underlying rover_sim vehicle config.
    """

    arm_length_m: float = 2.5
    max_speed_mps: float = 5.56  # 20 km/h hardware cap
    max_gamma_deg: float = 45.0
    accel_mps2: float = 1.5
    brake_decel_mps2: float = 4.0
    track_width_m: float = 0.584  # only used by side_by_side_* vehicle types


class SafetyConfigModel(BaseModel):
    """Safety interlock parameters (F/R lock, learning mode, auto-neutral)."""

    max_speed_mps: float = 5.56
    learning_max_speed_mps: float = 1.11  # 4 km/h
    learning_mode: bool = False
    auto_neutral_timeout_s: float = 15.0
    f_r_stop_eps_mps: float = 0.05


class GnssConfigModel(BaseModel):
    """u-blox ZED-F9P GNSS sensor model parameters."""

    rate_hz: float = 10.0
    default_fix: str = "fixed"  # fixed | float | dgps | spp | none
    prob_dropout: float = 0.0
    prob_downgrade: float = 0.0
    prob_upgrade: float = 0.0


class VehicleConfig(BaseModel):
    """Per-vehicle configuration — vehicle type + kinematics + sensors + safety."""

    id: str = "rover"
    type: str = "single_track"  # single_track | side_by_side_left | side_by_side_right
    direction_mode: str = "closed"  # open | closed — matches MTT direction-mode menu
    initial_pose: InitialPose = Field(default_factory=InitialPose)
    kinematics: KinematicsConfig = Field(default_factory=KinematicsConfig)
    safety: SafetyConfigModel = Field(default_factory=SafetyConfigModel)
    gnss: GnssConfigModel = Field(default_factory=GnssConfigModel)


class ControllerConfig(BaseModel):
    """
    Which Driver to run and its parameters.

    `kind` is one of:
      - ``teleop``       — manual / scripted command replay (the *default*).
      - ``waypoint``     — autonomous waypoint following (Phase 4).
      - ``multipass``    — path-fidelity controller for repeat surveys (Phase 5).
      - ``linked_cmp``   — two-rover coordinated CMP drive (Phase 6).

    Teleop is the default because the rover should boot into a safe manual
    state; autonomous modes are opt-in via an explicit scenario config.
    """

    kind: str = "teleop"
    params: dict[str, Any] = Field(default_factory=dict)


class ScenarioConfig(BaseModel):
    """Top-level scenario description consumed by the emulator runner."""

    name: str = "unnamed"
    description: str = ""
    seed: int = 0
    duration_s: float = 60.0
    dt_s: float = 0.05
    log_path: Optional[str] = None  # default resolves to ./runs/<name>.jsonl

    vehicle: VehicleConfig = Field(default_factory=VehicleConfig)
    controller: ControllerConfig = Field(default_factory=ControllerConfig)
    survey_path: Optional[str] = None

    @classmethod
    def from_yaml(cls, path: str | Path) -> "ScenarioConfig":
        data = yaml.safe_load(Path(path).read_text()) or {}
        return cls.model_validate(data)

    def to_yaml(self) -> str:
        return yaml.safe_dump(
            self.model_dump(mode="python", exclude_none=False),
            sort_keys=False,
        )
