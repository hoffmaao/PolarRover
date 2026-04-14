"""Hardware backend abstraction — consumes CommandBus, pushes to the wire."""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional

from rover_sim.control.command import CommandBus


@dataclass(frozen=True)
class VehicleFeedback:
    """
    Decoded ECU feedback from a single MTT-154 drive unit. The ECU publishes
    its own feedback frames on the CAN bus; the FrameCoder parses them into
    this neutral representation for consumption by the state estimator.

    Fields that a particular ECU does not report are left at their defaults.
    """

    speed_mps: float = 0.0
    articulation_rad: float = 0.0
    battery_voltage_v: float = 0.0
    ecu_fault: bool = False
    raw: dict = field(default_factory=dict)


class HardwareBackend(ABC):
    """
    A HardwareBackend is the bridge between the high-level CommandBus
    (joystick-level API produced by a Driver) and the physical wire protocol
    on the MTT-154 (CAN frames to/from the ECU).

    The backend holds the CAN interface handle, the send cadence, and the
    FrameCoder that knows how to pack the 8-byte command payload. Concrete
    subclasses differ in how they interpret the CommandBus for the vehicle
    configuration they target — a single articulated drive, two MTTs coupled
    as tank tracks, and so on.
    """

    @abstractmethod
    def send(self, cmd: CommandBus) -> None:
        """Transmit a command frame representing the given CommandBus."""

    @abstractmethod
    def read(self) -> Optional[VehicleFeedback]:
        """Poll for a feedback frame. Returns None if no frame is available."""

    def open(self) -> None:
        """Open the CAN interface. Default is a no-op so test backends work unchanged."""

    def close(self) -> None:
        """Close the CAN interface. Default is a no-op."""

    def __enter__(self) -> "HardwareBackend":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
