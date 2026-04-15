"""Typed snapshot of everything the operator-facing UI and TTS layer need to
know about the rover at a given instant. The drive loop writes one of these
into the StatusBroker each tick, and readers (web UI, OLED, audio events)
pull the latest copy without racing against the writer.

Keep this dataclass small and cheap to copy — it's serialized to JSON for the
WebSocket push and copied many times per second. Don't attach callables or
live handles; this is data only.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


class Mode(str, Enum):
    """High-level rover mode the operator sees on the status panel."""

    IDLE = "idle"
    TELEOP = "teleop"
    WAYPOINT = "waypoint"
    MULTIPASS = "multipass"
    LINKED_CMP = "linked_cmp"
    CALIBRATION = "calibration"
    FAULT = "fault"


@dataclass(frozen=True)
class GnssStatus:
    """Subset of the GNSS receiver's current solution for the status panel."""

    fix_type: str = "none"
    latitude_deg: Optional[float] = None
    longitude_deg: Optional[float] = None
    hdop: Optional[float] = None
    num_satellites: Optional[int] = None
    last_fix_age_s: Optional[float] = None


@dataclass(frozen=True)
class BatteryStatus:
    """Decoded from the BMS feedback frames when the hardware backend is live."""

    soc_percent: Optional[float] = None
    voltage_v: Optional[float] = None
    current_a: Optional[float] = None
    charge_time_remaining_s: Optional[float] = None
    heatpad_a_on: bool = False
    heatpad_b_on: bool = False


@dataclass(frozen=True)
class DriveStatus:
    """What the drive loop is currently commanding and observing."""

    commanded_throttle: float = 0.0
    commanded_steer: float = 0.0
    commanded_brake: float = 0.0
    safety_unlocked: bool = False
    actual_speed_mps: Optional[float] = None
    heading_rad: Optional[float] = None
    yaw_rate_rad_s: Optional[float] = None
    ecu_fault: bool = False


@dataclass(frozen=True)
class SurveyStatus:
    """Progress through the currently loaded survey."""

    name: Optional[str] = None
    kind: Optional[str] = None
    path: Optional[str] = None
    waypoint_index: Optional[int] = None
    waypoint_count: Optional[int] = None
    cross_track_rms_m: Optional[float] = None
    spread_m: Optional[float] = None
    elapsed_s: float = 0.0


@dataclass(frozen=True)
class RoverStatusSnapshot:
    """Everything the operator-facing UI reads. Single source of truth for
    what the rover is doing at a given moment."""

    rover_name: str = "rover"
    mode: Mode = Mode.IDLE
    position_x_m: Optional[float] = None
    position_y_m: Optional[float] = None
    gnss: GnssStatus = field(default_factory=GnssStatus)
    battery: BatteryStatus = field(default_factory=BatteryStatus)
    drive: DriveStatus = field(default_factory=DriveStatus)
    survey: SurveyStatus = field(default_factory=SurveyStatus)
    timestamp: float = 0.0

    def to_dict(self) -> dict:
        """JSON-serializable dict form for the WebSocket push."""
        return {
            "rover_name": self.rover_name,
            "mode": self.mode.value,
            "position_x_m": self.position_x_m,
            "position_y_m": self.position_y_m,
            "gnss": self.gnss.__dict__,
            "battery": self.battery.__dict__,
            "drive": self.drive.__dict__,
            "survey": self.survey.__dict__,
            "timestamp": self.timestamp,
        }
