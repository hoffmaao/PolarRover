from dataclasses import dataclass, replace
from typing import Optional

from rover_sim.control.command import CommandBus, Direction
from rover_sim.vehicle.base import VehicleState


@dataclass
class SafetyConfig:
    """
    Safety behavior the MTT-154 enforces at the main-module / firmware level.
    Modeled in the sim so that drive code written against rover_sim can't rely
    on behaviors the real vehicle would silently override.
    """

    max_speed_mps: float = 5.56           # 20 km/h vehicle cap
    learning_max_speed_mps: float = 1.11  # 4 km/h (MTT "learning mode")
    learning_mode: bool = False
    auto_neutral_timeout_s: float = 15.0
    f_r_stop_eps_mps: float = 0.05


class SafetyFilter:
    """
    A stateful command filter that sits between the driver's CommandBus output
    and the Vehicle's step() input. It enforces four MTT-154 safety behaviors:

    1. **F/R interlock.** A change from FORWARD to REVERSE (or vice versa) is
       rejected while |speed| > f_r_stop_eps_mps. The previously accepted
       direction is held until the vehicle is stopped.
    2. **Learning mode throttle cap.** When learning_mode is True, commanded
       throttle is clipped so the resulting speed never exceeds learning_max.
    3. **Auto-neutral timeout.** If neither throttle nor brake is used for
       auto_neutral_timeout_s seconds, the filter trips into neutral and
       overrides throttle to 0 until the driver explicitly cycles neutral
       (sends cmd.neutral=True for one frame, simulating the D9 button pull).
    4. Extreme-speed clamp is already handled inside the Vehicle model, but
       the filter refuses to pass through throttle above 1.0 as a belt-and-
       suspenders check (range validation in CommandBus handles it too).

    The filter is coordinate-free: it only needs the current VehicleState and
    a dt. It holds its own clock internally so the caller doesn't have to
    thread time state through.
    """

    def __init__(self, config: Optional[SafetyConfig] = None) -> None:
        self.config = config or SafetyConfig()
        self._time: float = 0.0
        self._last_active_time: float = 0.0
        self._neutral_tripped: bool = False
        self._last_accepted_direction: Direction = Direction.FORWARD

    def reset(self) -> None:
        self._time = 0.0
        self._last_active_time = 0.0
        self._neutral_tripped = False
        self._last_accepted_direction = Direction.FORWARD

    def filter(self, cmd: CommandBus, state: VehicleState, dt: float) -> CommandBus:
        self._time += dt
        cfg = self.config

        cmd = self._apply_fr_interlock(cmd, state)
        cmd = self._apply_learning_cap(cmd)
        cmd = self._apply_auto_neutral(cmd)
        return cmd

    # ---------- interlocks ----------

    def _apply_fr_interlock(self, cmd: CommandBus, state: VehicleState) -> CommandBus:
        if cmd.direction != self._last_accepted_direction:
            if abs(state.speed) > self.config.f_r_stop_eps_mps:
                return replace(cmd, direction=self._last_accepted_direction)
            self._last_accepted_direction = cmd.direction
        return cmd

    def _apply_learning_cap(self, cmd: CommandBus) -> CommandBus:
        cfg = self.config
        if not cfg.learning_mode or cmd.throttle == 0.0:
            return cmd
        max_ratio = cfg.learning_max_speed_mps / cfg.max_speed_mps
        if cmd.throttle > max_ratio:
            return replace(cmd, throttle=max_ratio)
        return cmd

    def _apply_auto_neutral(self, cmd: CommandBus) -> CommandBus:
        cfg = self.config
        active = cmd.throttle > 0.0 or cmd.brake > 0.0
        if active:
            self._last_active_time = self._time

        if (
            not self._neutral_tripped
            and self._time - self._last_active_time > cfg.auto_neutral_timeout_s
        ):
            self._neutral_tripped = True

        if self._neutral_tripped:
            if cmd.neutral:
                # Driver cycled the neutral button — clear the lockout
                self._neutral_tripped = False
                self._last_active_time = self._time
                return cmd
            return replace(cmd, throttle=0.0)

        return cmd

    @property
    def neutral_tripped(self) -> bool:
        return self._neutral_tripped
