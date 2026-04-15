from typing import TYPE_CHECKING, Optional, Protocol, runtime_checkable

from rover_sim.control.command import CommandBus
from rover_sim.control.state import RoverState

if TYPE_CHECKING:
    from rover_sim.surveys.base import Survey


@runtime_checkable
class Driver(Protocol):
    """
    A Driver consumes RoverState (plus an optional Survey) and produces
    CommandBus outputs. This is the single seam between the drive code
    (rover_drive) and the execution substrate (rover_sim for emulator runs,
    CAN-side backend for the real rover).

    Survey is Optional because teleop drivers — the default mode that the
    rover boots into — run without a loaded mission. Autonomous drivers
    (waypoint / multipass / linked-CMP) require one and should raise on None.

    A Driver instance lives for one scenario run; it may hold internal state
    (waypoint index, EnKF ensemble, filter taps, scripted command pointer, ...).
    Call reset() before a new scenario or when restarting a mission.
    """

    def update(
        self,
        state: RoverState,
        mission: Optional["Survey"],
        dt: float,
    ) -> CommandBus: ...

    def reset(self) -> None: ...
