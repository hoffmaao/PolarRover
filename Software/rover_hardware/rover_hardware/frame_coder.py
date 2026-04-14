"""FrameCoder protocol — the contract the vendor drivers must implement."""

from typing import Protocol, runtime_checkable

from rover_sim.control.command import CommandBus

from rover_hardware.backend import VehicleFeedback


@runtime_checkable
class FrameCoder(Protocol):
    """
    Packs CommandBus instances into 8-byte CAN payloads and unpacks ECU
    feedback frames into VehicleFeedback objects. The concrete implementation
    lives in the vendor driver (gitignored) so the confidential wire format
    never enters the public repository.

    A HardwareBackend takes a FrameCoder instance at construction time — the
    backend holds the CAN interface and cadence, the coder holds the byte
    layout. This separation means the public code is fully testable with a
    fake coder, and the vendor drivers can be updated without touching the
    bridge logic.
    """

    def encode_command(self, cmd: CommandBus) -> bytes:
        """Return the 8-byte payload for the CommandBus."""
        ...

    def decode_feedback(self, frame_id: int, payload: bytes) -> VehicleFeedback:
        """Update internal state from an ECU feedback frame and return the
        current accumulated VehicleFeedback. The MTT publishes several
        feedback frame types (main module, BMS, revisions), each contributing
        different fields; the coder merges them as they arrive. Frame IDs the
        coder doesn't recognize are silently ignored and the last-known
        VehicleFeedback is returned unchanged."""
        ...
