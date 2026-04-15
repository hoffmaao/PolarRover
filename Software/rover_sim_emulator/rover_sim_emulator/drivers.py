"""Driver factory — maps controller config into concrete Driver instances."""

import math
from typing import Any, Optional

from rover_sim.control import Driver
from rover_sim.surveys import Survey

from rover_drive.estimation import EnKFConfig, EnsembleKalmanFilter
from rover_drive.modes import (
    IdleCommandSource,
    MultipassControllerConfig,
    MultipassDriver,
    ScriptedCommandSource,
    TeleopDriver,
    WaypointControllerConfig,
    WaypointDriver,
)


_AVAILABLE_KINDS = ["teleop", "waypoint", "multipass"]


def create_driver(
    kind: str,
    params: dict[str, Any],
    mission: Optional[Survey],
) -> Driver:
    """
    Resolve a ``(kind, params)`` pair from the scenario config into a
    concrete Driver instance. Unknown kinds raise ValueError with a list of
    what's currently available — each phase unlocks a new option here.
    """
    if kind == "teleop":
        return _make_teleop(params)
    if kind == "waypoint":
        return _make_waypoint(params, mission)
    if kind == "multipass":
        return _make_multipass(params, mission)

    raise ValueError(
        f"unknown controller kind {kind!r}. Available in this build: {_AVAILABLE_KINDS}"
    )


def _make_teleop(params: dict[str, Any]) -> TeleopDriver:
    source_kind = params.get("source", "idle")
    if source_kind == "idle":
        return TeleopDriver(IdleCommandSource())
    if source_kind == "scripted_inline":
        commands = params.get("commands", [])
        return TeleopDriver(ScriptedCommandSource.from_inline(commands))
    if source_kind == "scripted":
        script_path = params.get("script_path")
        if not script_path:
            raise ValueError("teleop source=scripted requires params.script_path")
        return TeleopDriver(ScriptedCommandSource.from_yaml(script_path))
    raise ValueError(
        f"unknown teleop source {source_kind!r}. "
        f"Available: idle, scripted_inline, scripted"
    )


def _make_waypoint(
    params: dict[str, Any], mission: Optional[Survey]
) -> WaypointDriver:
    if mission is None:
        raise ValueError(
            "waypoint controller requires a survey_path in the scenario config"
        )

    controller_config = WaypointControllerConfig()
    _copy_if_present(params, controller_config, "cruise_throttle")
    _copy_if_present(params, controller_config, "approach_throttle")
    _copy_if_present(params, controller_config, "waypoint_tolerance_m")
    _copy_if_present(params, controller_config, "slowdown_radius_m")
    if "heading_saturation_deg" in params:
        controller_config.heading_saturation_rad = math.radians(
            float(params["heading_saturation_deg"])
        )

    return WaypointDriver(
        config=controller_config, enkf=_make_enkf(params)
    )


def _make_multipass(
    params: dict[str, Any], mission: Optional[Survey]
) -> MultipassDriver:
    if mission is None:
        raise ValueError(
            "multipass controller requires a survey_path in the scenario config"
        )

    controller_config = MultipassControllerConfig()
    _copy_if_present(params, controller_config, "cruise_throttle")
    _copy_if_present(params, controller_config, "approach_throttle")
    _copy_if_present(params, controller_config, "lookahead_m")
    _copy_if_present(params, controller_config, "path_complete_tolerance_m")
    _copy_if_present(params, controller_config, "slowdown_radius_m")
    if "heading_saturation_deg" in params:
        controller_config.heading_saturation_rad = math.radians(
            float(params["heading_saturation_deg"])
        )

    return MultipassDriver(
        config=controller_config, enkf=_make_enkf(params)
    )


def _make_enkf(params: dict[str, Any]) -> Optional[EnsembleKalmanFilter]:
    enkf_seed = params.get("enkf_seed")
    if enkf_seed is not None:
        return EnsembleKalmanFilter(EnKFConfig(seed=enkf_seed))
    return None


def _copy_if_present(
    params: dict[str, Any], target: Any, key: str
) -> None:
    if key in params:
        setattr(target, key, float(params[key]))
