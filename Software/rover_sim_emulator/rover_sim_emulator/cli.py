"""CLI for `rover-sim-emu`."""

import sys
from pathlib import Path
from typing import Optional


def main(argv: Optional[list[str]] = None) -> int:
    argv = argv if argv is not None else sys.argv[1:]
    if not argv or argv[0] in {"-h", "--help", "help"}:
        _print_help()
        return 0

    cmd = argv[0]
    rest = argv[1:]
    if cmd == "run":
        return _cmd_run(rest)
    if cmd == "demo":
        return _cmd_demo(rest)
    if cmd == "extract-track":
        return _cmd_extract_track(rest)

    print(f"rover-sim-emu: unknown command {cmd!r}", file=sys.stderr)
    _print_help()
    return 2


# ---------- commands ----------


def _cmd_run(args: list[str]) -> int:
    if not args:
        print("usage: rover-sim-emu run <scenario.yaml>", file=sys.stderr)
        return 2

    from rover_sim_emulator.dispatch import run_scenario

    path = Path(args[0])
    if not path.exists():
        print(f"scenario file not found: {path}", file=sys.stderr)
        return 1

    from rover_sim.config import ScenarioConfig

    config = ScenarioConfig.from_yaml(path)
    summary = run_scenario(config)
    _print_summary(summary)
    return 0


def _cmd_demo(args: list[str]) -> int:
    from rover_sim_emulator.runner import ScenarioRunner

    config = _build_demo_config()
    runner = ScenarioRunner(config)
    summary = runner.run()
    print("demo: canned teleop scenario — forward for 3 s, then brake to stop.")
    _print_summary(summary)
    return 0


# ---------- demo scenario ----------


def _build_demo_config():
    """Minimal teleop demo scenario built programmatically — no YAML file needed."""
    from rover_sim.config import (
        ControllerConfig,
        ScenarioConfig,
        VehicleConfig,
    )

    return ScenarioConfig(
        name="demo_teleop",
        description="Canned teleop demo: forward for 3 s, then brake to a stop.",
        seed=42,
        duration_s=5.0,
        dt_s=0.05,
        vehicle=VehicleConfig(),  # defaults: single_track, 20 km/h cap, ZED-F9P fixed
        controller=ControllerConfig(
            kind="teleop",
            params={
                "source": "scripted_inline",
                "commands": [
                    {"t": 0.0, "throttle": 0.5, "direction_mode": "closed"},
                    {"t": 3.0, "throttle": 0.0, "brake": 0.8},
                ],
            },
        ),
    )


# ---------- helpers ----------


def _print_summary(summary: dict) -> None:
    print(f"scenario:     {summary['name']}")
    print(f"controller:   {summary['controller']}")
    print(f"duration:     {summary['duration_s']:.2f} s")

    if "final_x" in summary:
        print(
            f"final pose:   ({summary['final_x']:.2f}, {summary['final_y']:.2f}) "
            f"heading={summary['final_heading_rad']:.3f} rad"
        )
        print(f"final speed:  {summary['final_speed_mps']:.2f} m/s")
    elif "rover_a_final" in summary:
        ax, ay = summary["rover_a_final"]
        bx, by = summary["rover_b_final"]
        print(f"rover A:      ({ax:.2f}, {ay:.2f})")
        print(f"rover B:      ({bx:.2f}, {by:.2f})")
        print(f"midpoint drift: {summary.get('midpoint_drift_rms_m', 0):.3f} m RMS (max {summary.get('midpoint_drift_max_m', 0):.3f})")
        print(f"spread err:     {summary.get('spread_error_rms_m', 0):.3f} m RMS")
        print(f"final spread:   {summary.get('final_spread_m', 0):.2f} m (target {summary.get('target_spread_m', 0):.2f})")
        print(f"complete:       {summary.get('complete', False)}")

    print(f"log:          {summary['log_path']}")


def _cmd_extract_track(args: list[str]) -> int:
    if len(args) < 2:
        print("usage: rover-sim-emu extract-track <log.jsonl> <output.geojson> [--source fused|truth|gnss] [--spacing 0.5]",
              file=sys.stderr)
        return 2

    from rover_sim_emulator.reftrack import extract_reference_track

    log_path = Path(args[0])
    out_path = Path(args[1])
    source = "fused"
    spacing = 0.5

    i = 2
    while i < len(args):
        if args[i] == "--source" and i + 1 < len(args):
            source = args[i + 1]
            i += 2
        elif args[i] == "--spacing" and i + 1 < len(args):
            spacing = float(args[i + 1])
            i += 2
        else:
            print(f"unknown option: {args[i]}", file=sys.stderr)
            return 2

    if not log_path.exists():
        print(f"log file not found: {log_path}", file=sys.stderr)
        return 1

    result = extract_reference_track(log_path, out_path, source=source, spacing_m=spacing)
    import json
    data = json.loads(result.read_text())
    n_pts = len(data["features"][0]["geometry"]["coordinates"])
    print(f"Extracted {n_pts} reference points from {source} stream")
    print(f"Spacing: {spacing} m")
    print(f"Output:  {result}")
    return 0


def _print_help() -> None:
    print("rover-sim-emu — Nisse simulation emulator")
    print()
    print("Usage: rover-sim-emu <command> [args]")
    print()
    print("Commands:")
    print("  run <scenario.yaml>          run a scenario to completion")
    print("  demo                         run a canned teleop demo")
    print("  extract-track <log> <out>    extract EnKF fused track as multipass GeoJSON")
    print("  help                         show this help")
    print()
    print("Default controller in any scenario is 'teleop' — the rover boots")
    print("into manual control and autonomous modes are opt-in via the scenario.")


if __name__ == "__main__":
    raise SystemExit(main())
