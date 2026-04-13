"""JSONL telemetry logger for scenario runs."""

import json
from dataclasses import asdict, is_dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Optional, TextIO

from rover_sim.control import CommandBus, RoverState
from rover_sim.vehicle.base import VehicleState


class TelemetryLogger:
    """
    Append-only JSONL telemetry logger. One record per simulation step, each
    line a self-contained JSON object with:

      - t            — simulation time (s)
      - truth        — ground-truth VehicleState from the sim
      - rover_state  — noisy RoverState the Driver saw
      - cmd          — CommandBus the Driver produced (post-safety-filter)

    JSONL is chosen for simplicity, line-oriented tooling compatibility, and
    because it's the easiest format for the post-run log viewer in the
    startup web app to read incrementally. Swap for Parquet later if volumes
    justify it.
    """

    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        self._fh: Optional[TextIO] = None

    def start(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = self.path.open("w")

    def finish(self) -> None:
        if self._fh is not None:
            self._fh.close()
            self._fh = None

    def log(
        self,
        t: float,
        truth: VehicleState,
        rover_state: RoverState,
        cmd: CommandBus,
        fused: Optional[dict[str, float]] = None,
    ) -> None:
        if self._fh is None:
            raise RuntimeError(
                "TelemetryLogger: start() must be called before log()"
            )
        record = {
            "t": t,
            "truth": _to_jsonable(truth),
            "rover_state": _to_jsonable(rover_state),
            "cmd": _to_jsonable(cmd),
        }
        if fused is not None:
            record["fused"] = fused
        self._fh.write(json.dumps(record) + "\n")

    def log_record(self, record: dict[str, Any]) -> None:
        """Write an arbitrary dict as one JSONL line (used by dual-vehicle CMP runner)."""
        if self._fh is None:
            raise RuntimeError(
                "TelemetryLogger: start() must be called before log_record()"
            )
        self._fh.write(json.dumps(record, default=_json_default) + "\n")

    def __enter__(self) -> "TelemetryLogger":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.finish()


def _json_default(obj: Any) -> Any:
    """json.dumps default handler for dataclasses and enums."""
    if is_dataclass(obj):
        return {k: v for k, v in asdict(obj).items()}
    if isinstance(obj, Enum):
        return obj.value
    raise TypeError(f"Object of type {type(obj)} is not JSON serializable")


def _to_jsonable(obj: Any) -> Any:
    """Recursively convert dataclasses and enums to JSON-serializable primitives."""
    if is_dataclass(obj):
        return {k: _to_jsonable(v) for k, v in asdict(obj).items()}
    if isinstance(obj, Enum):
        return obj.value
    if isinstance(obj, dict):
        return {k: _to_jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_to_jsonable(v) for v in obj]
    return obj
