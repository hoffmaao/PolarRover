"""Top-level scenario dispatch — picks the right runner class for the controller kind."""

from typing import Any

from rover_sim.config import ScenarioConfig


def run_scenario(config: ScenarioConfig) -> dict[str, Any]:
    """
    Instantiate the right runner for the scenario's controller kind and
    execute the sim to completion. Returns the summary dict.

    - ``teleop`` / ``waypoint`` / ``multipass`` → single-vehicle ScenarioRunner
    - ``linked_cmp`` → dual-vehicle LinkedCMPRunner
    """
    if config.controller.kind == "linked_cmp":
        from rover_sim_emulator.cmp_runner import LinkedCMPRunner

        runner = LinkedCMPRunner(config)
    else:
        from rover_sim_emulator.runner import ScenarioRunner

        runner = ScenarioRunner(config)
    return runner.run()
