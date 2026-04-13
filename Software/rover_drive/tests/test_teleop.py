from pathlib import Path

import pytest

from rover_sim.control import CommandBus, Direction, DirectionMode, RoverState

from rover_drive.modes import (
    IdleCommandSource,
    ScriptedCommandSource,
    TeleopDriver,
)


def test_idle_source_always_zero():
    src = IdleCommandSource()
    for t in [0.0, 1.0, 100.0]:
        cmd = src.command_at(t)
        assert cmd.throttle == 0.0
        assert cmd.brake == 0.0
        assert cmd.steer == 0.0


def test_scripted_source_returns_latest_applicable_command():
    src = ScriptedCommandSource.from_inline(
        [
            {"t": 0.0, "throttle": 0.3},
            {"t": 2.0, "throttle": 0.5},
            {"t": 4.0, "throttle": 0.0, "brake": 0.5},
        ]
    )
    # Before first entry: zero
    assert src.command_at(-0.5).throttle == 0.0
    # At t=0, first entry applies
    assert src.command_at(0.0).throttle == pytest.approx(0.3)
    # Between entries, hold most recent
    assert src.command_at(1.9).throttle == pytest.approx(0.3)
    # At next boundary
    assert src.command_at(2.0).throttle == pytest.approx(0.5)
    # Later
    cmd_late = src.command_at(5.0)
    assert cmd_late.throttle == 0.0
    assert cmd_late.brake == pytest.approx(0.5)


def test_scripted_source_coerces_enum_strings():
    src = ScriptedCommandSource.from_inline(
        [{"t": 0.0, "direction": "R", "direction_mode": "closed"}]
    )
    cmd = src.command_at(0.0)
    assert cmd.direction is Direction.REVERSE
    assert cmd.direction_mode is DirectionMode.CLOSED_LOOP


def test_scripted_source_rejects_entry_missing_time():
    with pytest.raises(ValueError, match="missing 't'"):
        ScriptedCommandSource.from_inline([{"throttle": 0.5}])


def test_scripted_source_sorts_out_of_order_entries():
    src = ScriptedCommandSource.from_inline(
        [
            {"t": 2.0, "throttle": 0.8},
            {"t": 0.0, "throttle": 0.1},
            {"t": 1.0, "throttle": 0.4},
        ]
    )
    assert src.command_at(0.5).throttle == pytest.approx(0.1)
    assert src.command_at(1.5).throttle == pytest.approx(0.4)
    assert src.command_at(2.5).throttle == pytest.approx(0.8)


def test_scripted_source_from_yaml(tmp_path: Path):
    path = tmp_path / "script.yaml"
    path.write_text(
        """
commands:
  - t: 0.0
    throttle: 0.5
  - t: 3.0
    throttle: 0.0
    brake: 0.7
"""
    )
    src = ScriptedCommandSource.from_yaml(path)
    assert src.command_at(0.0).throttle == pytest.approx(0.5)
    assert src.command_at(3.5).brake == pytest.approx(0.7)


def test_teleop_driver_advances_its_internal_clock():
    src = ScriptedCommandSource.from_inline(
        [
            {"t": 0.0, "throttle": 0.2},
            {"t": 1.0, "throttle": 0.8},
        ]
    )
    driver = TeleopDriver(src)
    state = RoverState()

    cmd0 = driver.update(state, mission=None, dt=0.5)
    assert cmd0.throttle == pytest.approx(0.2)

    cmd1 = driver.update(state, mission=None, dt=0.5)
    assert cmd1.throttle == pytest.approx(0.2)  # still at t=0.5

    cmd2 = driver.update(state, mission=None, dt=0.5)
    assert cmd2.throttle == pytest.approx(0.8)  # now past t=1.0


def test_teleop_driver_default_source_is_idle():
    driver = TeleopDriver()
    cmd = driver.update(RoverState(), mission=None, dt=0.1)
    assert cmd.throttle == 0.0


def test_teleop_driver_reset_rewinds_clock():
    src = ScriptedCommandSource.from_inline(
        [
            {"t": 0.0, "throttle": 0.1},
            {"t": 5.0, "throttle": 0.9},
        ]
    )
    driver = TeleopDriver(src)
    for _ in range(50):
        driver.update(RoverState(), mission=None, dt=0.2)  # advance to t=10
    assert driver.update(RoverState(), mission=None, dt=0.0).throttle == pytest.approx(0.9)

    driver.reset()
    cmd = driver.update(RoverState(), mission=None, dt=0.0)
    assert cmd.throttle == pytest.approx(0.1)
