"""Vision pipeline for the polar rover.

The top-level modules here are the stable API. Concrete implementations
that depend on external libraries (depthai for OAK-D Pro, opencv for
LiDAR processing, torch for segmentation) are imported lazily to keep
the package installable in environments without those heavy deps.
"""

from rover_onboard.vision.capture import (
    CameraFrame,
    FrameSource,
    NullFrameSource,
)
from rover_onboard.vision.segmentation import (
    Detection,
    NullSegmenter,
    Segmenter,
)

__all__ = [
    "CameraFrame",
    "Detection",
    "FrameSource",
    "NullFrameSource",
    "NullSegmenter",
    "Segmenter",
]
