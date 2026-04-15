"""Events the rover speaks aloud, and the logic that derives them from
changes in the RoverStatusSnapshot."""

from dataclasses import dataclass
from enum import Enum
from typing import Any, Iterable

from rover_onboard.status.snapshot import Mode, RoverStatusSnapshot


class EventType(str, Enum):
    MISSION_STARTED = "mission_started"
    WAYPOINT_REACHED = "waypoint_reached"
    MISSION_COMPLETE = "mission_complete"
    GNSS_FIX_LOST = "gnss_fix_lost"
    GNSS_FIX_RECOVERED = "gnss_fix_recovered"
    BATTERY_LOW = "battery_low"
    ESTOP_ENGAGED = "estop_engaged"
    ESTOP_CLEARED = "estop_cleared"
    OBSTACLE_AHEAD = "obstacle_ahead"
    CARD_DETECTED = "card_detected"
    NO_SURVEY = "no_survey"
    MODE_CHANGED = "mode_changed"


@dataclass(frozen=True)
class AudioEvent:
    type: EventType
    payload: dict[str, Any]


# Fix types that count as "having a fix" for gain/loss transitions. SPP
# is not here intentionally — in polar deployments we treat anything
# below DGPS as effectively no fix for science-grade positioning.
_USABLE_FIX_TYPES: frozenset[str] = frozenset({"fixed", "float", "dgps"})

# SOC thresholds at which we announce a low-battery warning. Each
# threshold fires once per crossing (from above to below) — we don't
# want the rover repeating "battery at 20 percent" every tick.
_BATTERY_THRESHOLDS_PCT: tuple[float, ...] = (30.0, 20.0, 10.0)


def events_from_state_change(
    prev: RoverStatusSnapshot,
    curr: RoverStatusSnapshot,
) -> Iterable[AudioEvent]:
    """Yield AudioEvents for every operator-relevant transition between two
    consecutive snapshots. The StatusBroker's subscribe() hook wires this
    to a TTS backend to produce spoken callouts."""
    yield from _fix_events(prev, curr)
    yield from _battery_events(prev, curr)
    yield from _mode_events(prev, curr)
    yield from _survey_events(prev, curr)
    yield from _estop_events(prev, curr)


# ---- per-field derivers ----


def _fix_events(prev: RoverStatusSnapshot, curr: RoverStatusSnapshot) -> Iterable[AudioEvent]:
    had = prev.gnss.fix_type in _USABLE_FIX_TYPES
    has = curr.gnss.fix_type in _USABLE_FIX_TYPES
    if had and not has:
        yield AudioEvent(EventType.GNSS_FIX_LOST, {"fix_type": curr.gnss.fix_type})
    elif has and not had:
        yield AudioEvent(EventType.GNSS_FIX_RECOVERED, {"fix_type": curr.gnss.fix_type})


def _battery_events(prev: RoverStatusSnapshot, curr: RoverStatusSnapshot) -> Iterable[AudioEvent]:
    a = prev.battery.soc_percent
    b = curr.battery.soc_percent
    if a is None or b is None:
        return
    for threshold in _BATTERY_THRESHOLDS_PCT:
        if a > threshold >= b:
            yield AudioEvent(EventType.BATTERY_LOW, {"soc_percent": int(round(b))})


def _mode_events(prev: RoverStatusSnapshot, curr: RoverStatusSnapshot) -> Iterable[AudioEvent]:
    if prev.mode == curr.mode:
        return
    yield AudioEvent(
        EventType.MODE_CHANGED,
        {"from": prev.mode.value, "to": curr.mode.value},
    )
    # Starting or finishing a mission is worth announcing on its own.
    if prev.mode in {Mode.IDLE, Mode.TELEOP} and curr.mode not in {Mode.IDLE, Mode.TELEOP, Mode.FAULT}:
        yield AudioEvent(
            EventType.MISSION_STARTED,
            {"rover_name": curr.rover_name, "kind": curr.mode.value},
        )


def _survey_events(prev: RoverStatusSnapshot, curr: RoverStatusSnapshot) -> Iterable[AudioEvent]:
    # Waypoint progress.
    a = prev.survey.waypoint_index
    b = curr.survey.waypoint_index
    total = curr.survey.waypoint_count
    if a is not None and b is not None and b > a:
        yield AudioEvent(
            EventType.WAYPOINT_REACHED,
            {"index": b, "total": total},
        )

    # Mission completion — detected when the mode returns to idle/teleop
    # after running one of the autonomous modes, and a survey was active.
    if (
        prev.mode not in {Mode.IDLE, Mode.TELEOP, Mode.FAULT}
        and curr.mode in {Mode.IDLE, Mode.TELEOP}
        and prev.survey.name
    ):
        yield AudioEvent(
            EventType.MISSION_COMPLETE,
            {"name": prev.survey.name},
        )


def _estop_events(prev: RoverStatusSnapshot, curr: RoverStatusSnapshot) -> Iterable[AudioEvent]:
    was_locked = not prev.drive.safety_unlocked
    is_locked = not curr.drive.safety_unlocked
    if not was_locked and is_locked:
        yield AudioEvent(EventType.ESTOP_ENGAGED, {})
    elif was_locked and not is_locked:
        yield AudioEvent(EventType.ESTOP_CLEARED, {})
