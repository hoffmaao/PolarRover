"""Tests for event derivation and phrasebook rendering."""

import pytest

from rover_onboard.audio.events import (
    AudioEvent,
    EventType,
    events_from_state_change,
)
from rover_onboard.audio.phrasebook import phrase_for
from rover_onboard.status.snapshot import (
    BatteryStatus,
    DriveStatus,
    GnssStatus,
    Mode,
    RoverStatusSnapshot,
    SurveyStatus,
)


def _snap(**overrides) -> RoverStatusSnapshot:
    defaults = {
        "rover_name": "nisse-en",
        "mode": Mode.IDLE,
        "gnss": GnssStatus(fix_type="none"),
        "battery": BatteryStatus(soc_percent=80.0),
        "drive": DriveStatus(safety_unlocked=True),
        "survey": SurveyStatus(),
    }
    defaults.update(overrides)
    return RoverStatusSnapshot(**defaults)


# ---- GNSS transitions ----


def test_fix_lost_when_dropping_below_usable():
    prev = _snap(gnss=GnssStatus(fix_type="fixed"))
    curr = _snap(gnss=GnssStatus(fix_type="spp"))
    events = list(events_from_state_change(prev, curr))
    assert any(e.type is EventType.GNSS_FIX_LOST for e in events)


def test_fix_recovered_when_rising_to_usable():
    prev = _snap(gnss=GnssStatus(fix_type="none"))
    curr = _snap(gnss=GnssStatus(fix_type="float"))
    events = list(events_from_state_change(prev, curr))
    assert any(e.type is EventType.GNSS_FIX_RECOVERED for e in events)


def test_fix_transition_within_usable_is_silent():
    prev = _snap(gnss=GnssStatus(fix_type="float"))
    curr = _snap(gnss=GnssStatus(fix_type="fixed"))
    events = list(events_from_state_change(prev, curr))
    assert all(e.type is not EventType.GNSS_FIX_LOST for e in events)
    assert all(e.type is not EventType.GNSS_FIX_RECOVERED for e in events)


# ---- Battery thresholds ----


def test_battery_low_fires_once_per_threshold():
    prev = _snap(battery=BatteryStatus(soc_percent=31.0))
    curr = _snap(battery=BatteryStatus(soc_percent=29.0))
    events = list(events_from_state_change(prev, curr))
    lows = [e for e in events if e.type is EventType.BATTERY_LOW]
    assert len(lows) == 1
    assert lows[0].payload["soc_percent"] == 29


def test_battery_crossing_multiple_thresholds_fires_once_each():
    prev = _snap(battery=BatteryStatus(soc_percent=35.0))
    curr = _snap(battery=BatteryStatus(soc_percent=9.0))
    events = list(events_from_state_change(prev, curr))
    thresholds = [e.payload["soc_percent"] for e in events if e.type is EventType.BATTERY_LOW]
    # Crossed 30, 20, and 10 — one event per threshold with the final SOC.
    assert len(thresholds) == 3


def test_battery_above_highest_threshold_silent():
    prev = _snap(battery=BatteryStatus(soc_percent=95.0))
    curr = _snap(battery=BatteryStatus(soc_percent=50.0))
    events = list(events_from_state_change(prev, curr))
    assert not any(e.type is EventType.BATTERY_LOW for e in events)


# ---- Mode + mission ----


def test_mode_change_announced():
    prev = _snap(mode=Mode.TELEOP)
    curr = _snap(mode=Mode.WAYPOINT)
    events = list(events_from_state_change(prev, curr))
    types = {e.type for e in events}
    assert EventType.MODE_CHANGED in types
    assert EventType.MISSION_STARTED in types


def test_mission_complete_fires_on_return_to_idle():
    prev = _snap(mode=Mode.WAYPOINT, survey=SurveyStatus(name="ross_ice_01"))
    curr = _snap(mode=Mode.IDLE, survey=SurveyStatus(name="ross_ice_01"))
    events = list(events_from_state_change(prev, curr))
    assert any(e.type is EventType.MISSION_COMPLETE for e in events)


def test_waypoint_reached_on_index_advance():
    prev = _snap(survey=SurveyStatus(waypoint_index=2, waypoint_count=5))
    curr = _snap(survey=SurveyStatus(waypoint_index=3, waypoint_count=5))
    events = list(events_from_state_change(prev, curr))
    matches = [e for e in events if e.type is EventType.WAYPOINT_REACHED]
    assert matches
    assert matches[0].payload["index"] == 3


# ---- E-stop ----


def test_estop_engaged_when_locked():
    prev = _snap(drive=DriveStatus(safety_unlocked=True))
    curr = _snap(drive=DriveStatus(safety_unlocked=False))
    events = list(events_from_state_change(prev, curr))
    assert any(e.type is EventType.ESTOP_ENGAGED for e in events)


def test_estop_cleared_when_unlocked():
    prev = _snap(drive=DriveStatus(safety_unlocked=False))
    curr = _snap(drive=DriveStatus(safety_unlocked=True))
    events = list(events_from_state_change(prev, curr))
    assert any(e.type is EventType.ESTOP_CLEARED for e in events)


# ---- Phrasebook ----


def test_phrase_for_battery_low():
    e = AudioEvent(EventType.BATTERY_LOW, {"soc_percent": 20})
    assert phrase_for(e) == "Battery at 20 percent."


def test_phrase_for_mission_started_uses_rover_name():
    e = AudioEvent(EventType.MISSION_STARTED, {"rover_name": "nisse-en", "kind": "waypoint"})
    assert "nisse-en" in phrase_for(e)


def test_phrase_for_missing_payload_does_not_raise():
    e = AudioEvent(EventType.BATTERY_LOW, {})
    # Should fall back to the template without raising.
    phrase_for(e)
