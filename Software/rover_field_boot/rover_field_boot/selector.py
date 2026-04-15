"""Survey selection — pick which survey to run from the discovered set.

Two selection modes are supported. The ``active_file`` mode reads a plain
text file (typically ``active.txt`` on the card) that names the survey to
run — this is the non-interactive mode the field rover boots into by
default, since it needs no screen and no keyboard. The ``console`` mode
is an interactive picker that numbers the discovered surveys and reads
a choice from stdin; it's useful for bench testing and for operators
with a debug screen.
"""

from pathlib import Path
from typing import Literal, Optional

from rover_field_boot.discovery import SURVEY_KINDS, SurveyHandle, SurveySource


def pick_survey(
    surveys_by_kind: dict[str, list[SurveyHandle]],
    mode: Literal["console", "active_file"] = "console",
    active_file: Optional[Path] = None,
) -> Optional[SurveyHandle]:
    """Return the chosen survey, or None if no valid selection is made."""
    if mode == "active_file":
        return _pick_from_active_file(surveys_by_kind, active_file)
    if mode == "console":
        return _pick_from_console(surveys_by_kind)
    raise ValueError(f"unknown selector mode: {mode!r}")


def _pick_from_active_file(
    surveys_by_kind: dict[str, list[SurveyHandle]],
    active_file: Optional[Path],
) -> Optional[SurveyHandle]:
    if active_file is None or not active_file.is_file():
        return None

    requested = active_file.read_text().strip()
    if not requested:
        return None

    # Match by path (absolute or relative) or by survey name.
    for handles in surveys_by_kind.values():
        for h in handles:
            if str(h.path) == requested or h.path.name == requested or h.name == requested:
                return h
    return None


def _pick_from_console(
    surveys_by_kind: dict[str, list[SurveyHandle]],
) -> Optional[SurveyHandle]:
    flat: list[SurveyHandle] = []
    print()
    print("Available surveys")
    print("=" * 64)
    for kind in SURVEY_KINDS:
        handles = surveys_by_kind.get(kind, [])
        if not handles:
            continue
        print(f"\n  [{_kind_label(kind)}]")
        for h in handles:
            flat.append(h)
            source_tag = "card" if h.source is SurveySource.CARD else "disk"
            print(f"    {len(flat):2d}. {h.name}  ({source_tag}, {h.n_points} pts)")

    if not flat:
        print("\n  (no surveys found)")
        return None

    print()
    choice = input("Select a survey number (or press Enter to cancel): ").strip()
    if not choice:
        return None
    try:
        idx = int(choice) - 1
        return flat[idx]
    except (ValueError, IndexError):
        print(f"invalid selection: {choice!r}")
        return None


_KIND_LABELS = {
    "waypoint_survey": "Waypoint navigation",
    "multipass_survey": "Multipass repeat-track",
    "cmp_survey": "Linked CMP spread",
    "calibration_survey": "Calibration",
}


def _kind_label(kind: str) -> str:
    return _KIND_LABELS.get(kind, kind)
