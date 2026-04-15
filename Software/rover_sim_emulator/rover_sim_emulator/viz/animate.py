"""Animated scenario video renderer — white background, GNSS scatter,
EnKF estimate trail, ground truth, waypoints, and velocity vector."""

import json
import math
from pathlib import Path
from typing import Any, Optional

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation, FFMpegWriter
import numpy as np


def render_single_rover_video(
    log_path: str | Path,
    output_path: str | Path,
    survey_path: Optional[str | Path] = None,
    fps: int = 20,
    trail_len: int = 200,
    dpi: int = 100,
    persistent_path: bool = True,
) -> Path:
    """Render an animated video of a single-rover scenario run.

    The ground-truth and EnKF trails draw the full run history from start
    to the current frame when ``persistent_path`` is True (the default),
    so the full rover path stays visible on screen for the length of the
    video. Set ``persistent_path=False`` to fall back to a sliding window
    of ``trail_len`` samples — useful for very long missions where the
    earliest history is no longer interesting. The GNSS scatter always
    uses the sliding window to keep the fix cloud readable.
    """
    log_path = Path(log_path)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    records = _load_log(log_path)
    waypoints = _load_waypoints(survey_path) if survey_path else []

    truth_x = [r["truth"]["x"] for r in records]
    truth_y = [r["truth"]["y"] for r in records]
    gnss_x = [r["rover_state"]["x"] for r in records]
    gnss_y = [r["rover_state"]["y"] for r in records]
    times = [r["t"] for r in records]
    speeds = [r["truth"].get("speed", 0) for r in records]
    headings = [r["truth"].get("heading", 0) for r in records]

    fig, ax = plt.subplots(figsize=(10, 8), facecolor="white")
    ax.set_facecolor("white")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3, color="#ccc")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")

    pad = 3
    all_x = truth_x + gnss_x + [w[0] for w in waypoints]
    all_y = truth_y + gnss_y + [w[1] for w in waypoints]
    ax.set_xlim(min(all_x) - pad, max(all_x) + pad)
    ax.set_ylim(min(all_y) - pad, max(all_y) + pad)

    if waypoints:
        wx = [w[0] for w in waypoints]
        wy = [w[1] for w in waypoints]
        ax.plot(wx, wy, "s--", color="#457b9d", markersize=8, linewidth=1, alpha=0.5, label="Waypoints", zorder=1)
        for i, (x, y) in enumerate(waypoints):
            ax.annotate(f"WP{i}", (x, y), textcoords="offset points", xytext=(5, 5), fontsize=7, color="#457b9d")

    truth_trail, = ax.plot([], [], "-", color="#2a9d8f", linewidth=2, label="Ground truth", zorder=3)
    gnss_scatter = ax.scatter([], [], s=12, c="#e76f51", alpha=0.4, label="GNSS fixes", zorder=2)
    enkf_trail, = ax.plot([], [], "-", color="#264653", linewidth=1.5, alpha=0.8, label="EnKF estimate", zorder=4)
    rover_dot, = ax.plot([], [], "o", color="#e63946", markersize=8, zorder=5)
    vel_arrow = ax.annotate("", xy=(0, 0), xytext=(0, 0),
                            arrowprops=dict(arrowstyle="->", color="#e63946", lw=2), zorder=6)
    time_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, fontsize=10,
                        verticalalignment="top", fontfamily="monospace",
                        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", edgecolor="#ccc"))

    ax.legend(loc="upper right", fontsize=8, framealpha=0.9)
    ax.set_title("Nisse Simulation", fontweight="bold")

    # For the EnKF trail we'll use a running average as a proxy (since the log
    # doesn't store the filter mean directly — it stores noisy observations).
    # A simple exponential moving average approximates the filter output.
    enkf_x, enkf_y = _compute_ema(gnss_x, gnss_y, alpha=0.3)

    subsample = max(1, len(records) // (fps * 30))

    def init():
        return truth_trail, gnss_scatter, enkf_trail, rover_dot, time_text

    def update(frame_idx):
        i = min(frame_idx * subsample, len(records) - 1)
        # GNSS stays windowed so the fix cloud doesn't smother the plot;
        # the ground-truth and EnKF trails either track the whole run
        # from zero or window with everything else.
        window_start = max(0, i - trail_len)
        path_start = 0 if persistent_path else window_start

        truth_trail.set_data(truth_x[path_start:i + 1], truth_y[path_start:i + 1])

        gx = gnss_x[window_start:i + 1]
        gy = gnss_y[window_start:i + 1]
        gnss_scatter.set_offsets(np.column_stack([gx, gy]) if gx else np.empty((0, 2)))

        enkf_trail.set_data(enkf_x[path_start:i + 1], enkf_y[path_start:i + 1])

        rover_dot.set_data([truth_x[i]], [truth_y[i]])

        v = speeds[i]
        h = headings[i]
        scale = min(2.0, abs(v)) * 0.8
        vel_arrow.xy = (truth_x[i] + scale * math.cos(h), truth_y[i] + scale * math.sin(h))
        vel_arrow.set_position((truth_x[i], truth_y[i]))

        fix = records[i]["rover_state"].get("fix_type", "?")
        time_text.set_text(
            f"t = {times[i]:.1f} s\n"
            f"speed = {abs(v):.2f} m/s\n"
            f"GNSS: {fix}"
        )
        return truth_trail, gnss_scatter, enkf_trail, rover_dot, time_text

    n_frames = len(records) // subsample
    anim = FuncAnimation(fig, update, init_func=init, frames=n_frames, interval=1000 // fps, blit=False)
    writer = FFMpegWriter(fps=fps, metadata={"title": "Nisse Sim"}, bitrate=2000)
    anim.save(str(output_path), writer=writer, dpi=dpi)
    plt.close(fig)
    return output_path


def render_cmp_video(
    log_path: str | Path,
    output_path: str | Path,
    survey_path: Optional[str | Path] = None,
    fps: int = 20,
    trail_len: int = 200,
    dpi: int = 100,
    persistent_path: bool = True,
) -> Path:
    """Render an animated video of a linked CMP dual-rover scenario.

    Both rover trails and the midpoint trace default to the full history
    (``persistent_path=True``) so the spread geometry is visible end to
    end. The per-rover GNSS scatter stays windowed to keep the fix cloud
    readable.
    """
    log_path = Path(log_path)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    records = _load_log(log_path)
    centerline = _load_waypoints(survey_path) if survey_path else []

    a_x = [r["rover_a"]["truth"]["x"] for r in records]
    a_y = [r["rover_a"]["truth"]["y"] for r in records]
    b_x = [r["rover_b"]["truth"]["x"] for r in records]
    b_y = [r["rover_b"]["truth"]["y"] for r in records]
    a_gx = [r["rover_a"]["rover_state"]["x"] for r in records]
    a_gy = [r["rover_a"]["rover_state"]["y"] for r in records]
    b_gx = [r["rover_b"]["rover_state"]["x"] for r in records]
    b_gy = [r["rover_b"]["rover_state"]["y"] for r in records]
    mid_x = [r["formation"]["midpoint_x"] for r in records]
    mid_y = [r["formation"]["midpoint_y"] for r in records]
    spreads = [r["formation"]["actual_spread_m"] for r in records]
    times = [r["t"] for r in records]

    fig, ax = plt.subplots(figsize=(12, 8), facecolor="white")
    ax.set_facecolor("white")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3, color="#ccc")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")

    pad = 5
    all_x = a_x + b_x + [c[0] for c in centerline]
    all_y = a_y + b_y + [c[1] for c in centerline]
    if all_x:
        ax.set_xlim(min(all_x) - pad, max(all_x) + pad)
        ax.set_ylim(min(all_y) - pad, max(all_y) + pad)

    if centerline:
        cx = [c[0] for c in centerline]
        cy = [c[1] for c in centerline]
        ax.plot(cx, cy, "--", color="#a8dadc", linewidth=2, alpha=0.6, label="Centerline", zorder=1)

    trail_a, = ax.plot([], [], "-", color="#2a9d8f", linewidth=2, label="Rover A", zorder=3)
    trail_b, = ax.plot([], [], "-", color="#e63946", linewidth=2, label="Rover B", zorder=3)
    trail_mid, = ax.plot([], [], "-", color="#457b9d", linewidth=1.5, alpha=0.7, label="Midpoint", zorder=2)
    scatter_a = ax.scatter([], [], s=8, c="#2a9d8f", alpha=0.3, zorder=2)
    scatter_b = ax.scatter([], [], s=8, c="#e63946", alpha=0.3, zorder=2)
    dot_a, = ax.plot([], [], "o", color="#2a9d8f", markersize=10, zorder=5)
    dot_b, = ax.plot([], [], "o", color="#e63946", markersize=10, zorder=5)
    baseline, = ax.plot([], [], "-", color="#264653", linewidth=1.5, alpha=0.6, zorder=4)

    time_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, fontsize=10,
                        verticalalignment="top", fontfamily="monospace",
                        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", edgecolor="#ccc"))

    ax.legend(loc="upper right", fontsize=8, framealpha=0.9)
    ax.set_title("Nisse CMP Survey Simulation", fontweight="bold")

    subsample = max(1, len(records) // (fps * 30))

    def update(frame_idx):
        i = min(frame_idx * subsample, len(records) - 1)
        window_start = max(0, i - trail_len)
        path_start = 0 if persistent_path else window_start

        trail_a.set_data(a_x[path_start:i + 1], a_y[path_start:i + 1])
        trail_b.set_data(b_x[path_start:i + 1], b_y[path_start:i + 1])
        trail_mid.set_data(mid_x[path_start:i + 1], mid_y[path_start:i + 1])

        sa = np.column_stack([a_gx[max(0, i - 50):i + 1], a_gy[max(0, i - 50):i + 1]])
        sb = np.column_stack([b_gx[max(0, i - 50):i + 1], b_gy[max(0, i - 50):i + 1]])
        scatter_a.set_offsets(sa if len(sa) else np.empty((0, 2)))
        scatter_b.set_offsets(sb if len(sb) else np.empty((0, 2)))

        dot_a.set_data([a_x[i]], [a_y[i]])
        dot_b.set_data([b_x[i]], [b_y[i]])
        baseline.set_data([a_x[i], b_x[i]], [a_y[i], b_y[i]])

        time_text.set_text(
            f"t = {times[i]:.1f} s\n"
            f"spread = {spreads[i]:.2f} m\n"
            f"midpoint = ({mid_x[i]:.1f}, {mid_y[i]:.1f})"
        )
        return trail_a, trail_b, trail_mid, dot_a, dot_b, baseline, time_text

    n_frames = len(records) // subsample
    anim = FuncAnimation(fig, update, frames=n_frames, interval=1000 // fps, blit=False)
    writer = FFMpegWriter(fps=fps, metadata={"title": "Nisse CMP Sim"}, bitrate=2000)
    anim.save(str(output_path), writer=writer, dpi=dpi)
    plt.close(fig)
    return output_path


# ---------- helpers ----------


def _load_log(path: Path) -> list[dict[str, Any]]:
    lines = path.read_text().strip().split("\n")
    return [json.loads(line) for line in lines if line.strip()]


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


def _compute_ema(
    x: list[float], y: list[float], alpha: float = 0.3
) -> tuple[list[float], list[float]]:
    """Exponential moving average as a simple proxy for the EnKF fused estimate."""
    ex, ey = [x[0]], [y[0]]
    for i in range(1, len(x)):
        ex.append(alpha * x[i] + (1 - alpha) * ex[-1])
        ey.append(alpha * y[i] + (1 - alpha) * ey[-1])
    return ex, ey
