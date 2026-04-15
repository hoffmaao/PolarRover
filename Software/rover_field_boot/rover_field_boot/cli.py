"""CLI entry point for `rover-field-boot`."""

import argparse
import os
import sys
from pathlib import Path
from typing import Optional

from rover_field_boot.discovery import (
    SURVEY_KINDS,
    SurveySource,
    load_surveys,
)
from rover_field_boot.launcher import launch_survey
from rover_field_boot.selector import pick_survey


# Where to look for local disk surveys when no --surveys-dir is given. In
# order of preference — first existing directory wins. On the repo this
# resolves to `surveys/` relative to the cwd; on a deployed rover it's
# expected to live under /opt/nisse/surveys/ or in the user's home.
DEFAULT_LOCAL_ROOTS: tuple[Path, ...] = (
    Path.cwd() / "surveys",
    Path("/opt/nisse/surveys"),
    Path.home() / ".nisse" / "surveys",
)


def main(argv: Optional[list[str]] = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    local_root = _resolve_local_root(args.surveys_dir)
    card_candidates = [Path(p) for p in args.card_candidates] if args.card_candidates else None

    surveys_by_kind, card_root = load_surveys(local_root, card_candidates)

    total = sum(len(h) for h in surveys_by_kind.values())
    if card_root is None:
        print(f"rover-field-boot: no card detected; loaded {total} surveys from {local_root}")
    else:
        print(f"rover-field-boot: card detected at {card_root}; loaded {total} surveys (card + local)")

    if args.list_only:
        _print_listing(surveys_by_kind)
        return 0

    handle = pick_survey(
        surveys_by_kind,
        mode=args.select,
        active_file=Path(args.active_file) if args.active_file else None,
    )
    if handle is None:
        print("rover-field-boot: no survey selected", file=sys.stderr)
        return 1

    source_tag = "card" if handle.source is SurveySource.CARD else "disk"
    print(f"rover-field-boot: running {handle.name} ({handle.kind}, {source_tag}) from {handle.path}")

    summary = launch_survey(
        handle,
        vehicle_type=args.vehicle_type,
        duration_s=args.duration_s,
        dt_s=args.dt_s,
        log_path=Path(args.log_path) if args.log_path else None,
    )
    _print_summary(summary)
    return 0


# ---------- helpers ----------


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="rover-field-boot",
        description="Load surveys from a removable card or local disk and run the selected one.",
    )
    p.add_argument(
        "--surveys-dir",
        type=str,
        default=None,
        help="Local surveys/ directory. Defaults to the first existing path in "
             "./surveys, /opt/nisse/surveys, ~/.nisse/surveys.",
    )
    p.add_argument(
        "--card-candidates",
        nargs="+",
        default=None,
        help="Mount points to search for a removable survey card. Defaults to "
             "the standard udisks2 locations.",
    )
    p.add_argument(
        "--select",
        choices=["console", "active_file"],
        default="console",
        help="Selection mode. 'active_file' reads a survey name from a plain "
             "text file (typically active.txt on the card) and is the intended "
             "headless mode for the field rover.",
    )
    p.add_argument(
        "--active-file",
        type=str,
        default=None,
        help="Path to the active.txt selector file (used with --select active_file).",
    )
    p.add_argument(
        "--list-only",
        action="store_true",
        help="Print the discovered surveys and exit without running anything.",
    )
    p.add_argument(
        "--vehicle-type",
        type=str,
        default="single_track",
        choices=["single_track", "side_by_side_left", "side_by_side_right"],
        help="Vehicle configuration. Defaults to single_track.",
    )
    p.add_argument("--duration-s", type=float, default=600.0, help="Run duration in seconds.")
    p.add_argument("--dt-s", type=float, default=0.05, help="Control loop period in seconds.")
    p.add_argument("--log-path", type=str, default=None, help="Destination for the telemetry JSONL.")
    return p


def _resolve_local_root(explicit: Optional[str]) -> Path:
    if explicit is not None:
        return Path(explicit)
    for candidate in DEFAULT_LOCAL_ROOTS:
        if candidate.is_dir():
            return candidate
    # Fall back to the first candidate even if it doesn't exist yet, so
    # error messages point at a consistent default.
    return DEFAULT_LOCAL_ROOTS[0]


def _print_listing(surveys_by_kind: dict) -> None:
    for kind in SURVEY_KINDS:
        handles = surveys_by_kind.get(kind, [])
        print(f"\n[{kind}]  ({len(handles)} surveys)")
        for h in handles:
            src = "card" if h.source is SurveySource.CARD else "disk"
            print(f"  {h.name:<30}  {src:<4}  {h.n_points:>5} pts  {h.path}")


def _print_summary(summary: dict) -> None:
    print()
    for k, v in summary.items():
        print(f"  {k}: {v}")


if __name__ == "__main__":
    raise SystemExit(main())
