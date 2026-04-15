"""Frame capture from the drive-system cameras.

The `FrameSource` protocol is the shared API the rest of the vision stack
consumes — segmentation, MJPEG streaming, LiDAR fusion — so the upstream
camera implementation is swappable without touching any consumer. The
concrete backends (OAK-D Pro via depthai, a generic V4L2 webcam via
OpenCV, a recorded-file replay) all implement this interface.

This module only defines the interface and a `NullFrameSource` for tests.
Hardware-backed backends (OAKDProSource, LidarScanSource) live in
submodules and require the 'vision' extra.
"""

from dataclasses import dataclass
from typing import Optional, Protocol, runtime_checkable


@dataclass(frozen=True)
class CameraFrame:
    """One frame from a camera. `image` is a 3-channel uint8 HxWx3 array;
    `depth` is an optional float32 HxW array in meters (populated only by
    stereo/depth sources like the OAK-D Pro)."""

    timestamp_s: float
    image: "object"  # numpy array when available; not typed here to avoid import
    width: int
    height: int
    depth: Optional["object"] = None
    source: str = ""


@runtime_checkable
class FrameSource(Protocol):
    """Minimal capture API. Consumers call next_frame() in a loop and get
    None when the source is exhausted or disconnected."""

    def open(self) -> None: ...
    def close(self) -> None: ...
    def next_frame(self) -> Optional[CameraFrame]: ...


class NullFrameSource:
    """Deterministic fake source for tests — yields a configurable sequence
    of frames without needing any hardware."""

    def __init__(self, frames: Optional[list[CameraFrame]] = None) -> None:
        self._frames = list(frames or [])
        self._idx = 0

    def open(self) -> None:
        self._idx = 0

    def close(self) -> None:
        pass

    def next_frame(self) -> Optional[CameraFrame]:
        if self._idx >= len(self._frames):
            return None
        frame = self._frames[self._idx]
        self._idx += 1
        return frame
