"""Survey discovery — find a removable card, list the surveys on it, and
merge with local-disk surveys in the default folder."""

import json
import os
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Iterable, Optional


# The four kinds the field rover knows how to run. The order here is the
# order the boot CLI lists them in, so the most frequently used kinds
# come first.
SURVEY_KINDS: tuple[str, ...] = (
    "waypoint_survey",
    "multipass_survey",
    "cmp_survey",
    "calibration_survey",
)


# Default mount points where we look for a removable survey card. These
# are checked in order; the first directory that exists AND contains a
# `surveys/` subtree wins. The list is deliberately conservative — it
# matches the paths that udisks2 + automount create on a stock Jetson/
# Ubuntu box. Pass a custom list into find_card() if the deployment
# uses something different.
DEFAULT_CARD_MOUNT_CANDIDATES: tuple[Path, ...] = (
    Path("/media/rover"),
    Path("/media/nvidia"),
    Path("/media/nisse"),
    Path("/mnt/nisse"),
)


class SurveySource(str, Enum):
    """Where a survey was loaded from — used by the selector UI and the
    telemetry log so it's unambiguous which file actually ran."""

    CARD = "card"
    LOCAL = "local"


@dataclass(frozen=True)
class SurveyHandle:
    """A pointer to a survey file on disk, with enough metadata for a
    selector UI to show a useful line per survey without re-opening each
    file. All fields are populated from the GeoJSON at discovery time."""

    name: str
    kind: str
    path: Path
    source: SurveySource
    n_points: int = 0
    description: str = ""


# ---------------------------------------------------------------------------
# Card detection
# ---------------------------------------------------------------------------


def find_card(
    candidates: Optional[Iterable[Path]] = None,
) -> Optional[Path]:
    """Return the first mount point under which a `surveys/` subtree exists.

    The search is non-recursive at the mount-candidate level: we expect a
    removable card to be mounted at `/media/rover/<LABEL>/` and to carry
    its surveys at `/media/rover/<LABEL>/surveys/`. If `candidates` is
    None the DEFAULT_CARD_MOUNT_CANDIDATES list is used.

    Returns the `surveys/` directory (not the mount point) so callers
    can pass it straight to discover_surveys.
    """
    candidates = list(candidates) if candidates is not None else list(DEFAULT_CARD_MOUNT_CANDIDATES)

    for root in candidates:
        if not root.is_dir():
            continue
        for entry in sorted(root.iterdir()):
            surveys_dir = entry / "surveys"
            if surveys_dir.is_dir():
                return surveys_dir
    return None


# ---------------------------------------------------------------------------
# Survey listing
# ---------------------------------------------------------------------------


def discover_surveys(
    root: Path,
    source: SurveySource,
) -> dict[str, list[SurveyHandle]]:
    """Walk a `surveys/` tree and return handles grouped by kind.

    The kind key comes from the `survey_kind` property inside the file,
    not from the folder path — a file mis-filed under the wrong folder
    still ends up in the right kind bucket. Files that fail to parse are
    skipped with a warning on stderr; the caller gets an empty list for
    that kind rather than an exception.
    """
    buckets: dict[str, list[SurveyHandle]] = {k: [] for k in SURVEY_KINDS}
    if not root.is_dir():
        return buckets

    for path in sorted(root.rglob("*.geojson")):
        handle = _read_handle(path, source)
        if handle is None:
            continue
        if handle.kind not in buckets:
            # Unknown kind — add the bucket so the caller at least sees
            # the file in its listing rather than silently dropping it.
            buckets[handle.kind] = []
        buckets[handle.kind].append(handle)

    return buckets


def merge_sources(
    card: Optional[dict[str, list[SurveyHandle]]],
    local: dict[str, list[SurveyHandle]],
) -> dict[str, list[SurveyHandle]]:
    """Combine card and local survey buckets.

    Card surveys come first within each kind so they take visual
    precedence in the selector, and they keep their `SurveySource.CARD`
    tag so telemetry is unambiguous about which source actually ran.
    Local surveys follow, without deduplication against the card — the
    operator can see both and pick.
    """
    merged: dict[str, list[SurveyHandle]] = {k: [] for k in SURVEY_KINDS}
    if card:
        for kind, handles in card.items():
            merged.setdefault(kind, []).extend(handles)
    for kind, handles in local.items():
        merged.setdefault(kind, []).extend(handles)
    return merged


def load_surveys(
    local_root: Path,
    card_candidates: Optional[Iterable[Path]] = None,
) -> tuple[dict[str, list[SurveyHandle]], Optional[Path]]:
    """High-level entry point: check for a card, fall back to disk, and
    return the merged bucket plus the card path (or None) so the caller
    can report which source was in play."""
    card_root = find_card(card_candidates)
    card_buckets = discover_surveys(card_root, SurveySource.CARD) if card_root else None
    local_buckets = discover_surveys(local_root, SurveySource.LOCAL)
    return merge_sources(card_buckets, local_buckets), card_root


# ---------------------------------------------------------------------------
# File parsing
# ---------------------------------------------------------------------------


def _read_handle(path: Path, source: SurveySource) -> Optional[SurveyHandle]:
    try:
        data = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as e:
        _warn(f"skipping {path}: {e}")
        return None

    kind = None
    n_points = 0
    description = ""
    name = path.stem

    for feat in data.get("features", []):
        props = feat.get("properties", {}) or {}
        kind = kind or props.get("survey_kind")
        description = description or props.get("description", "")
        name = props.get("name", name)
        geom = feat.get("geometry") or {}
        if geom.get("type") == "LineString":
            coords = geom.get("coordinates") or []
            n_points = max(n_points, len(coords))

    if kind is None:
        _warn(f"skipping {path}: no survey_kind property in any feature")
        return None

    return SurveyHandle(
        name=str(name),
        kind=str(kind),
        path=path,
        source=source,
        n_points=n_points,
        description=str(description),
    )


def _warn(message: str) -> None:
    # Small wrapper so tests can monkey-patch stderr output if needed.
    import sys
    print(f"rover-field-boot: {message}", file=sys.stderr)
