"""Teleop driver — the default manual-control mode for PolarRover."""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional, Protocol, runtime_checkable

import yaml

from rover_sim.control import CommandBus, Direction, DirectionMode, RoverState
from rover_sim.missions import Mission


@runtime_checkable
class CommandSource(Protocol):
    """
    A CommandSource produces a CommandBus for a given elapsed time. The
    TeleopDriver holds one and polls it each simulation step. Different
    concrete implementations let the same driver work for:
      - CI / deterministic tests (ScriptedCommandSource)
      - safe-default fallback (IdleCommandSource)
      - live interactive driving (a future JoystickCommandSource using pygame
        or evdev, intentionally kept out of the required dependency set)
    """

    def command_at(self, t: float) -> CommandBus: ...


class IdleCommandSource:
    """Always returns a zero CommandBus. Used as the safe default fallback."""

    def command_at(self, t: float) -> CommandBus:
        return CommandBus.zero()


@dataclass
class ScriptedCommandSource:
    """
    Plays back a time-stamped sequence of CommandBus values. The script is a
    list of ``(t_seconds, command_kwargs)`` tuples in ascending time order;
    the source returns the most recent command whose time is <= the query
    time. Before the first command, it returns a zero CommandBus.

    The command kwargs mirror the CommandBus constructor verbatim, so the
    D1 / D2 / D3 / D4 / D9 ergonomics of the OEM MTT joystick carry through:
    a script entry like ``{"t": 2.5, "throttle": 0.5, "steer": -0.2,
    "direction": "F", "direction_mode": "closed"}`` is literally the same set
    of knobs the OEM controller exposes.
    """

    commands: list[tuple[float, dict[str, Any]]] = field(default_factory=list)

    def command_at(self, t: float) -> CommandBus:
        latest: Optional[dict[str, Any]] = None
        for time_t, kwargs in self.commands:
            if time_t > t:
                break
            latest = kwargs
        if latest is None:
            return CommandBus.zero()
        return _construct_cmd(latest)

    @classmethod
    def from_inline(
        cls, commands: list[dict[str, Any]]
    ) -> "ScriptedCommandSource":
        """
        Build from an inline Python list of dicts, each containing a ``t`` key
        and any CommandBus fields. Ignores unknown keys.
        """
        parsed: list[tuple[float, dict[str, Any]]] = []
        for entry in commands:
            entry = dict(entry)
            if "t" not in entry:
                raise ValueError(f"scripted command entry missing 't': {entry!r}")
            t_val = float(entry.pop("t"))
            parsed.append((t_val, entry))
        parsed.sort(key=lambda pair: pair[0])
        return cls(commands=parsed)

    @classmethod
    def from_yaml(cls, path: str | Path) -> "ScriptedCommandSource":
        """
        Load from a YAML file of the form::

            commands:
              - t: 0.0
                throttle: 0.5
              - t: 5.0
                throttle: 0.0
                brake: 0.5
        """
        data = yaml.safe_load(Path(path).read_text()) or {}
        commands = data.get("commands", [])
        return cls.from_inline(commands)


class TeleopDriver:
    """
    Default Driver implementation — manual / scripted teleoperation.

    This driver treats whatever CommandSource it was given as authoritative:
    it polls the source once per simulation step and forwards the resulting
    CommandBus. It does *not* plan, it does *not* consume a mission, and it
    does *not* use the EnKF — manual teleop is a human (or recorded script)
    reacting to reality, not to a filtered estimate.

    On real hardware, teleop is the state where the OEM MTT wireless joystick
    drives the main module directly over its own CAN frame (0x001), and the
    companion computer's CommandBus is quiescent (IdleCommandSource). In the
    sim, TeleopDriver produces the command stream that the OEM joystick would
    normally emit; the ScriptedCommandSource is the usual choice for CI and
    canned demos because it keeps the full loop deterministic and testable.
    """

    def __init__(self, source: Optional[CommandSource] = None) -> None:
        self._source: CommandSource = source or IdleCommandSource()
        self._t: float = 0.0

    @property
    def source(self) -> CommandSource:
        return self._source

    def update(
        self,
        state: RoverState,
        mission: Optional[Mission],
        dt: float,
    ) -> CommandBus:
        cmd = self._source.command_at(self._t)
        self._t += dt
        return cmd

    def reset(self) -> None:
        self._t = 0.0


# ---------- internal ----------


def _construct_cmd(kwargs: dict[str, Any]) -> CommandBus:
    """Construct a CommandBus from plain-dict kwargs, coercing enum strings."""
    kwargs = dict(kwargs)
    if "direction" in kwargs and isinstance(kwargs["direction"], str):
        kwargs["direction"] = Direction(kwargs["direction"])
    if "direction_mode" in kwargs and isinstance(kwargs["direction_mode"], str):
        kwargs["direction_mode"] = DirectionMode(kwargs["direction_mode"])
    return CommandBus(**kwargs)
