"""Tests for the status broker — publish/latest, patch, subscriptions."""

import threading
import time

from rover_onboard.status.broker import StatusBroker
from rover_onboard.status.snapshot import (
    BatteryStatus,
    DriveStatus,
    GnssStatus,
    Mode,
    RoverStatusSnapshot,
    SurveyStatus,
)


def test_latest_returns_initial_snapshot():
    initial = RoverStatusSnapshot(rover_name="nisse-en", mode=Mode.TELEOP)
    broker = StatusBroker(initial=initial)
    assert broker.latest().rover_name == "nisse-en"
    assert broker.latest().mode is Mode.TELEOP


def test_publish_replaces_snapshot():
    broker = StatusBroker()
    new = RoverStatusSnapshot(rover_name="nisse-to", mode=Mode.WAYPOINT)
    broker.publish(new)
    assert broker.latest().mode is Mode.WAYPOINT
    assert broker.latest().rover_name == "nisse-to"


def test_publish_stamps_timestamp_when_missing():
    broker = StatusBroker()
    broker.publish(RoverStatusSnapshot())
    assert broker.latest().timestamp > 0


def test_patch_applies_partial_update():
    broker = StatusBroker(initial=RoverStatusSnapshot(rover_name="a", mode=Mode.IDLE))
    broker.patch(mode=Mode.WAYPOINT, gnss=GnssStatus(fix_type="fixed", num_satellites=12))
    snap = broker.latest()
    assert snap.mode is Mode.WAYPOINT
    assert snap.gnss.fix_type == "fixed"
    assert snap.rover_name == "a"  # preserved


def test_subscribe_receives_prev_and_curr():
    broker = StatusBroker()
    calls = []

    def _capture(prev, curr):
        calls.append((prev.mode, curr.mode))

    broker.subscribe(_capture)
    broker.publish(RoverStatusSnapshot(mode=Mode.TELEOP))
    broker.publish(RoverStatusSnapshot(mode=Mode.WAYPOINT))

    assert calls == [(Mode.IDLE, Mode.TELEOP), (Mode.TELEOP, Mode.WAYPOINT)]


def test_unsubscribe_stops_callbacks():
    broker = StatusBroker()
    count = [0]

    def _inc(prev, curr):
        count[0] += 1

    unsub = broker.subscribe(_inc)
    broker.publish(RoverStatusSnapshot(mode=Mode.TELEOP))
    unsub()
    broker.publish(RoverStatusSnapshot(mode=Mode.WAYPOINT))

    assert count[0] == 1


def test_subscriber_exception_does_not_stall_writer(capsys):
    broker = StatusBroker()

    def _boom(prev, curr):
        raise RuntimeError("kaboom")

    def _ok(prev, curr):
        _ok.calls = getattr(_ok, "calls", 0) + 1

    broker.subscribe(_boom)
    broker.subscribe(_ok)
    broker.publish(RoverStatusSnapshot(mode=Mode.WAYPOINT))

    assert getattr(_ok, "calls", 0) == 1
    assert "kaboom" in capsys.readouterr().err


def test_concurrent_publish_and_read_is_safe():
    broker = StatusBroker()
    stop = threading.Event()
    seen = []

    def _writer():
        i = 0
        while not stop.is_set():
            broker.publish(RoverStatusSnapshot(rover_name=f"r{i}"))
            i += 1
            time.sleep(0.0005)

    def _reader():
        while not stop.is_set():
            seen.append(broker.latest().rover_name)
            time.sleep(0.0005)

    w = threading.Thread(target=_writer)
    r = threading.Thread(target=_reader)
    w.start(); r.start()
    time.sleep(0.05)
    stop.set()
    w.join(); r.join()

    assert len(seen) > 0
    assert all(name.startswith("r") for name in seen if name)


def test_snapshot_to_dict_is_json_ready():
    snap = RoverStatusSnapshot(
        rover_name="nisse-en",
        mode=Mode.WAYPOINT,
        gnss=GnssStatus(fix_type="fixed", num_satellites=15),
        battery=BatteryStatus(soc_percent=82.5, voltage_v=48.1),
        drive=DriveStatus(commanded_throttle=0.3, actual_speed_mps=1.8),
        survey=SurveyStatus(name="ross_ice_01", waypoint_index=3, waypoint_count=14),
    )
    d = snap.to_dict()
    import json
    blob = json.dumps(d)  # must round-trip without errors
    assert "nisse-en" in blob
    assert "waypoint" in blob
    assert d["gnss"]["num_satellites"] == 15
    assert d["battery"]["soc_percent"] == 82.5
