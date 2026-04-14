"""Single MTT-154 articulated drive over one CAN bus."""

from dataclasses import dataclass
from typing import Optional

from rover_sim.control.command import CommandBus

from rover_hardware.backend import HardwareBackend, VehicleFeedback
from rover_hardware.frame_coder import FrameCoder


@dataclass
class CANInterfaceConfig:
    """python-can Bus configuration. `interface` is the driver name
    (socketcan, slcan, pcan, kvaser, virtual, ...) and `channel` is the
    adapter-specific handle (e.g., "can0", "/dev/ttyACM0")."""

    interface: str = "socketcan"
    channel: str = "can0"
    bitrate: int = 250_000
    command_id: int = 0x100
    feedback_id: Optional[int] = None  # if set, filter read() to only this ID


class SingleTrackCANBackend(HardwareBackend):
    """
    Drives a single MTT-154 in its articulated configuration. The CommandBus
    is packed into one 8-byte frame at the command arbitration ID and sent
    over the CAN bus. Feedback frames are read non-blocking; the FrameCoder
    decodes them into VehicleFeedback.

    The bus handle is created lazily in open() so that constructing the
    backend has no side effects and tests can run without python-can
    installed or hardware present.
    """

    def __init__(self, coder: FrameCoder, config: CANInterfaceConfig) -> None:
        self.coder = coder
        self.config = config
        self._bus = None

    def open(self) -> None:
        import can  # noqa: local import — hardware extras are optional

        self._bus = can.Bus(
            interface=self.config.interface,
            channel=self.config.channel,
            bitrate=self.config.bitrate,
        )

    def close(self) -> None:
        if self._bus is not None:
            self._bus.shutdown()
            self._bus = None

    def send(self, cmd: CommandBus) -> None:
        import can

        if self._bus is None:
            raise RuntimeError("CAN bus not open — call open() or use as context manager")
        payload = self.coder.encode_command(cmd)
        msg = can.Message(
            arbitration_id=self.config.command_id,
            data=payload,
            is_extended_id=False,
        )
        self._bus.send(msg)

    def read(self) -> Optional[VehicleFeedback]:
        if self._bus is None:
            raise RuntimeError("CAN bus not open — call open() or use as context manager")
        msg = self._bus.recv(timeout=0.0)
        if msg is None:
            return None
        if self.config.feedback_id is not None and msg.arbitration_id != self.config.feedback_id:
            return None
        return self.coder.decode_feedback(msg.arbitration_id, bytes(msg.data))
