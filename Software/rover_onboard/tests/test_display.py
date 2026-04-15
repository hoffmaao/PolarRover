"""Tests for the OLED layout via NullDisplay."""

from rover_onboard.display.oled import NullDisplay, _format_lines
from rover_onboard.status.snapshot import (
    BatteryStatus,
    DriveStatus,
    GnssStatus,
    Mode,
    RoverStatusSnapshot,
    SurveyStatus,
)


def test_null_display_captures_frames():
    d = NullDisplay()
    d.render(RoverStatusSnapshot(rover_name="nisse-en"))
    d.render(RoverStatusSnapshot(rover_name="nisse-to"))
    assert len(d.frames) == 2
    assert d.last_frame[0].startswith("nisse-to")


def test_format_lines_shows_mode_and_fix():
    snap = RoverStatusSnapshot(
        rover_name="nisse-en",
        mode=Mode.WAYPOINT,
        gnss=GnssStatus(fix_type="fixed", num_satellites=15),
        battery=BatteryStatus(soc_percent=82.0, voltage_v=48.1),
        drive=DriveStatus(actual_speed_mps=1.75),
        survey=SurveyStatus(name="ross_ice_01"),
    )
    lines = _format_lines(snap)
    assert lines[0] == "nisse-en"
    assert "waypoint" in lines[1]
    assert "FIXED" in lines[2]
    assert "sat:15" in lines[2]
    assert "82%" in lines[3]
    assert "48.1V" in lines[3]
    assert "1.75" in lines[4]
    assert "ross_ice_01" in lines[5]


def test_format_lines_handles_missing_values():
    snap = RoverStatusSnapshot()  # all defaults / None
    lines = _format_lines(snap)
    assert len(lines) == 6
    # No exceptions from None values; placeholders fill the gaps.
    assert "-%" in lines[3] or "-" in lines[3]
