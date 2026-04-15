"""Animated video comparing an original waypoint track with its multipass repeat."""

import json
import math
from pathlib import Path
from typing import Any, Optional

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
import numpy as np


def render_multipass_comparison_video(
    original_log_path: str | Path,
    repeat_log_path: str | Path,
    output_path: str | Path,
    survey_path: Optional[str | Path] = None,
    fps: int = 20,
    dpi: int = 100,
) -> Path:
    """
    Render a video showing:
      - The original waypoint track (faded, already driven)
      - The multipass repeat track being driven in real-time
      - GNSS scatter for the repeat pass
      - EnKF fused estimate for the repeat pass
      - The reference base track (the dense sampling of the original)
      - Cross-track error readout
    """
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    orig = _load_log(Path(original_log_path))
    repeat = _load_log(Path(repeat_log_path))
    base_track = _load_waypoints(survey_path) if survey_path else []

    orig_x = [r["truth"]["x"] for r in orig]
    orig_y = [r["truth"]["y"] for r in orig]

    rep_truth_x = [r["truth"]["x"] for r in repeat]
    rep_truth_y = [r["truth"]["y"] for r in repeat]
    rep_gnss_x = [r["rover_state"]["x"] for r in repeat]
    rep_gnss_y = [r["rover_state"]["y"] for r in repeat]
    rep_times = [r["t"] for r in repeat]
    rep_speeds = [r["truth"].get("speed", 0) for r in repeat]

    enkf_x, enkf_y = _compute_ema(rep_gnss_x, rep_gnss_y, alpha=0.3)

    cross_track = _compute_cross_track(rep_truth_x, rep_truth_y, base_track)

    fig, (ax_map, ax_ct) = plt.subplots(
        2, 1, figsize=(10, 9), facecolor="white",
        gridspec_kw={"height_ratios": [3, 1]},
    )

    ax_map.set_facecolor("white")
    ax_map.set_aspect("equal")
    ax_map.grid(True, alpha=0.3, color="#ccc")
    ax_map.set_xlabel("East (m)")
    ax_map.set_ylabel("North (m)")
    ax_map.set_title("Multipass Repeat of Waypoint Track", fontweight="bold")

    pad = 3
    all_x = orig_x + rep_truth_x + [w[0] for w in base_track]
    all_y = orig_y + rep_truth_y + [w[1] for w in base_track]
    ax_map.set_xlim(min(all_x) - pad, max(all_x) + pad)
    ax_map.set_ylim(min(all_y) - pad, max(all_y) + pad)

    ax_map.plot(orig_x, orig_y, "-", color="#a8dadc", linewidth=3, alpha=0.5, label="Original waypoint track", zorder=1)

    if base_track:
        bx = [w[0] for w in base_track]
        by = [w[1] for w in base_track]
        ax_map.plot(bx, by, ":", color="#457b9d", linewidth=1, alpha=0.6, label="Reference (dense)", zorder=1)

    rep_trail, = ax_map.plot([], [], "-", color="#e63946", linewidth=2, label="Repeat pass (truth)", zorder=3)
    gnss_scatter = ax_map.scatter([], [], s=10, c="#e76f51", alpha=0.35, label="GNSS", zorder=2)
    enkf_trail, = ax_map.plot([], [], "-", color="#264653", linewidth=1.5, alpha=0.8, label="EnKF estimate", zorder=4)
    rover_dot, = ax_map.plot([], [], "o", color="#e63946", markersize=8, zorder=5)

    time_text = ax_map.text(0.02, 0.98, "", transform=ax_map.transAxes, fontsize=9,
                            verticalalignment="top", fontfamily="monospace",
                            bbox=dict(boxstyle="round,pad=0.3", facecolor="white", edgecolor="#ccc"))

    ax_map.legend(loc="upper right", fontsize=7, framealpha=0.9)

    ax_ct.set_facecolor("white")
    ax_ct.set_xlabel("Time (s)")
    ax_ct.set_ylabel("Cross-track error (m)")
    ax_ct.set_title("Position Misfit from Reference Track", fontsize=10)
    ax_ct.grid(True, alpha=0.3, color="#ccc")
    ax_ct.set_xlim(0, rep_times[-1] if rep_times else 1)
    max_ct = max(cross_track) if cross_track else 1
    ax_ct.set_ylim(0, min(max_ct * 1.5, 5.0))

    ct_line, = ax_ct.plot([], [], "-", color="#e63946", linewidth=1.5)
    rms_line = ax_ct.axhline(y=0, color="#264653", linewidth=1, linestyle="--", alpha=0.6)
    rms_text = ax_ct.text(0.98, 0.95, "", transform=ax_ct.transAxes, fontsize=9,
                          ha="right", va="top", fontfamily="monospace",
                          bbox=dict(boxstyle="round,pad=0.3", facecolor="white", edgecolor="#ccc"))

    fig.tight_layout()

    subsample = max(1, len(repeat) // (fps * 40))
    trail_len = 300

    def update(frame_idx):
        i = min(frame_idx * subsample, len(repeat) - 1)
        start = max(0, i - trail_len)

        rep_trail.set_data(rep_truth_x[start:i + 1], rep_truth_y[start:i + 1])
        enkf_trail.set_data(enkf_x[start:i + 1], enkf_y[start:i + 1])

        gx = rep_gnss_x[max(0, i - 80):i + 1]
        gy = rep_gnss_y[max(0, i - 80):i + 1]
        gnss_scatter.set_offsets(np.column_stack([gx, gy]) if gx else np.empty((0, 2)))

        rover_dot.set_data([rep_truth_x[i]], [rep_truth_y[i]])

        ct_line.set_data(rep_times[:i + 1], cross_track[:i + 1])

        ct_so_far = cross_track[:i + 1]
        rms = (sum(c ** 2 for c in ct_so_far) / len(ct_so_far)) ** 0.5 if ct_so_far else 0
        rms_line.set_ydata([rms])
        rms_text.set_text(f"CT RMS: {rms:.3f} m\nCT max: {max(ct_so_far):.3f} m")

        time_text.set_text(
            f"t = {rep_times[i]:.1f} s\n"
            f"speed = {abs(rep_speeds[i]):.2f} m/s\n"
            f"cross-track = {cross_track[i]:.3f} m"
        )
        return rep_trail, gnss_scatter, enkf_trail, rover_dot, ct_line, rms_line, rms_text, time_text

    n_frames = len(repeat) // subsample
    anim = FuncAnimation(fig, update, frames=n_frames, interval=1000 // fps, blit=False)
    writer = FFMpegWriter(fps=fps, metadata={"title": "Nisse Multipass Repeat"}, bitrate=2500)
    anim.save(str(output_path), writer=writer, dpi=dpi)
    plt.close(fig)
    return output_path


def _load_log(path: Path) -> list[dict[str, Any]]:
    return [json.loads(l) for l in path.read_text().strip().split("\n") if l.strip()]


def _load_waypoints(path: Optional[str | Path]) -> list[tuple[float, float]]:
    if path is None:
        return []
    path = Path(path)
    if not path.exists():
        return []
    data = json.loads(path.read_text())
    for feat in data.get("features", []):
        geom = feat.get("geometry", {})
        if geom.get("type") == "LineString":
            return [(c[0], c[1]) for c in geom["coordinates"]]
    return []


def _compute_ema(x, y, alpha=0.3):
    ex, ey = [x[0]], [y[0]]
    for i in range(1, len(x)):
        ex.append(alpha * x[i] + (1 - alpha) * ex[-1])
        ey.append(alpha * y[i] + (1 - alpha) * ey[-1])
    return ex, ey


def _compute_cross_track(
    truth_x: list[float],
    truth_y: list[float],
    ref: list[tuple[float, float]],
) -> list[float]:
    """For each truth point, find the minimum distance to any reference point."""
    if not ref:
        return [0.0] * len(truth_x)
    ref_arr = np.array(ref)
    result = []
    for tx, ty in zip(truth_x, truth_y):
        dists = np.hypot(ref_arr[:, 0] - tx, ref_arr[:, 1] - ty)
        result.append(float(np.min(dists)))
    return result
