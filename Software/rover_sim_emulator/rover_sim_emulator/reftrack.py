"""Reference track extraction — converts a waypoint run's EnKF fused
positions into a densely sampled multipass reference track GeoJSON.

This is the pipeline that connects a first-pass waypoint survey to all
subsequent repeat-track multipass surveys: drive once with waypoints, extract
the fused track, use it as the reference that future passes try to follow.

The fused positions (not raw GNSS, not sim ground truth) are the right source
because they represent the best real-time estimate of where the rover actually
was. On real hardware, the equivalent is the PPK post-processed track.
"""

import json
import math
from pathlib import Path
from typing import Any, Optional


def extract_reference_track(
    log_path: str | Path,
    output_path: str | Path,
    source: str = "fused",
    min_speed_mps: float = 0.1,
    spacing_m: float = 0.5,
    mission_kind: str = "multipass_survey",
) -> Path:
    """
    Read a JSONL telemetry log and extract a densely sampled reference track
    as a multipass-ready GeoJSON file.

    Parameters
    ----------
    log_path : path to the JSONL log from a previous run
    output_path : path to write the output GeoJSON
    source : which position stream to use:
        ``"fused"`` — EnKF fused estimate (recommended, best available)
        ``"truth"`` — sim ground truth (only available in sim, for comparison)
        ``"gnss"``  — raw GNSS observations (noisy, not recommended)
    min_speed_mps : exclude points where the rover was slower than this
        (filters out the startup and braking phases)
    spacing_m : resample the track to approximately this spacing in meters
    mission_kind : the ``mission_kind`` tag for the output GeoJSON
        (``"multipass_survey"`` by default)

    Returns
    -------
    Path to the written GeoJSON file.
    """
    log_path = Path(log_path)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    records = [json.loads(line) for line in log_path.read_text().strip().split("\n") if line.strip()]

    raw_coords = _extract_coords(records, source, min_speed_mps)
    if len(raw_coords) < 2:
        raise ValueError(
            f"Not enough moving points in log (got {len(raw_coords)}). "
            f"Check source={source!r} and min_speed_mps={min_speed_mps}"
        )

    resampled = _resample(raw_coords, spacing_m)

    geojson = {
        "type": "FeatureCollection",
        "crs": {"type": "name", "properties": {"name": "urn:ogc:def:crs:EPSG::3031"}},
        "features": [
            {
                "type": "Feature",
                "geometry": {
                    "type": "LineString",
                    "coordinates": resampled,
                },
                "properties": {
                    "role": "base_track",
                    "mission_kind": mission_kind,
                    "source_log": str(log_path.name),
                    "source_stream": source,
                    "spacing_m": spacing_m,
                    "n_passes": 1,
                    "pass_direction": "same",
                },
            }
        ],
    }

    output_path.write_text(json.dumps(geojson, indent=2))
    return output_path


def _extract_coords(
    records: list[dict[str, Any]],
    source: str,
    min_speed: float,
) -> list[tuple[float, float]]:
    """Pull (x, y) pairs from the specified source stream, filtering by speed."""
    coords: list[tuple[float, float]] = []

    for r in records:
        speed = abs(r.get("truth", {}).get("speed", 0))
        if speed < min_speed:
            continue

        if source == "fused":
            fused = r.get("fused")
            if fused is None:
                continue
            coords.append((fused["x"], fused["y"]))

        elif source == "truth":
            t = r["truth"]
            coords.append((t["x"], t["y"]))

        elif source == "gnss":
            s = r["rover_state"]
            if s.get("fix_type") == "none":
                continue
            coords.append((s["x"], s["y"]))

        else:
            raise ValueError(f"unknown source {source!r}; use 'fused', 'truth', or 'gnss'")

    return coords


def _resample(
    coords: list[tuple[float, float]],
    spacing: float,
) -> list[list[float]]:
    """Resample a polyline to approximately uniform spacing."""
    if spacing <= 0 or len(coords) < 2:
        return [[x, y] for x, y in coords]

    out: list[list[float]] = [[coords[0][0], coords[0][1]]]
    accum = 0.0
    last_x, last_y = coords[0]

    for x, y in coords[1:]:
        dx = x - last_x
        dy = y - last_y
        seg = math.hypot(dx, dy)
        if seg < 1e-12:
            continue

        accum += seg
        while accum >= spacing:
            frac = (accum - spacing) / seg
            ix = x - frac * dx
            iy = y - frac * dy
            out.append([ix, iy])
            accum -= spacing

        last_x, last_y = x, y

    last_out = out[-1]
    if math.hypot(last_x - last_out[0], last_y - last_out[1]) > spacing * 0.3:
        out.append([last_x, last_y])

    return out
