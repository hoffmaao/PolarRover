"""Survey launcher — map a SurveyHandle to a concrete run.

The launcher builds a ScenarioConfig from a survey handle and the runtime
knobs the operator passes on the command line, then dispatches through
the existing `rover_sim_emulator.dispatch.run_scenario`. That means the
first cut of this package runs through the simulator substrate even on
the real rover — the hardware backend layer from `rover_hardware` plugs
in later as a replacement for the sim Vehicle step, without changing any
of the code here.
"""

from pathlib import Path
from typing import Any, Optional

from rover_sim.config import (
    ControllerConfig,
    ScenarioConfig,
    VehicleConfig,
)
from rover_sim_emulator.dispatch import run_scenario

from rover_field_boot.discovery import SurveyHandle


# Survey kinds map to controller kinds one-for-one at this level — the
# runner-internal mapping from controller kind to actual Driver class
# already lives in rover_sim_emulator.drivers.create_driver.
_KIND_TO_CONTROLLER = {
    "waypoint_survey": "waypoint",
    "multipass_survey": "multipass",
    "cmp_survey": "linked_cmp",
    "calibration_survey": "calibration",
}


def launch_survey(
    handle: SurveyHandle,
    *,
    vehicle_type: str = "single_track",
    duration_s: float = 600.0,
    dt_s: float = 0.05,
    log_path: Optional[Path] = None,
    controller_params: Optional[dict[str, Any]] = None,
) -> dict[str, Any]:
    """Build a scenario for the given survey and run it end to end.

    Returns the summary dict produced by the scenario runner (name,
    controller, duration, log path, final pose, and any metric fields
    the runner chose to report).
    """
    controller_kind = _KIND_TO_CONTROLLER.get(handle.kind)
    if controller_kind is None:
        raise ValueError(
            f"no controller mapped for survey kind {handle.kind!r} "
            f"(survey: {handle.path})"
        )

    cfg = ScenarioConfig(
        name=handle.name or handle.path.stem,
        description=handle.description or f"Field run of {handle.path.name}",
        seed=0,
        duration_s=duration_s,
        dt_s=dt_s,
        log_path=str(log_path) if log_path is not None else None,
        survey_path=str(handle.path),
        vehicle=VehicleConfig(type=vehicle_type),
        controller=ControllerConfig(
            kind=controller_kind,
            params=controller_params or {},
        ),
    )

    return run_scenario(cfg)
