"""Feature segmentation — person, vehicle, structure, hazard detection.

The architecture is inherited from the pipeline used on the bench platform
(OpenCV + MobileNet-SSD / MediaPipe). The weights there were trained on
general-purpose data (ImageNet / COCO / Open Images) and will not give
reliable results on polar imagery — snow fields, polar clothing, other
MTTs, scientific equipment, crevasses, and sastrugi are all
under-represented or absent in those datasets.

The migration plan is to keep the inference shape here stable (one
Segmenter instance, `segment(frame)` returns a list of Detection objects)
and swap the underlying weights for a model fine-tuned on polar data
collected during the outdoor-ground test phase. The class list is
deliberately shorter than COCO so the planner can act on it directly.

This module defines the interface and a NullSegmenter for tests. Real
backends (COCO baseline, polar fine-tune v1, ...) live in submodules and
require the 'vision' extra.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Protocol, runtime_checkable

from rover_onboard.vision.capture import CameraFrame


class DetectionClass(str, Enum):
    """Navigation-relevant classes. Shorter than COCO; every class here
    has a direct action implication for the obstacle layer or the audio
    events."""

    PERSON = "person"
    VEHICLE = "vehicle"          # other rovers, snowmobiles
    STAKE = "stake"              # flags, waypoint markers
    STRUCTURE = "structure"      # tents, science equipment
    HAZARD = "hazard"            # crevasse entries, pressure ridges
    UNKNOWN = "unknown"


@dataclass(frozen=True)
class Detection:
    cls: DetectionClass
    confidence: float
    # Bounding box in image pixel coordinates: (x, y, width, height).
    bbox: tuple[int, int, int, int]
    # Optional distance from camera in meters when stereo depth is available.
    distance_m: Optional[float] = None


@runtime_checkable
class Segmenter(Protocol):
    """Anything that maps a CameraFrame to a list of Detections."""

    def segment(self, frame: CameraFrame) -> list[Detection]: ...
    def close(self) -> None: ...


class NullSegmenter:
    """No-op segmenter — returns whatever list of Detections the caller
    seeds it with, once per call. Tests use this to simulate detection
    sequences without running a real model."""

    def __init__(
        self, scripted_detections: Optional[list[list[Detection]]] = None
    ) -> None:
        self._scripted = list(scripted_detections or [])
        self._idx = 0

    def segment(self, frame: CameraFrame) -> list[Detection]:
        if not self._scripted:
            return []
        dets = self._scripted[self._idx % len(self._scripted)]
        self._idx += 1
        return list(dets)

    def close(self) -> None:
        pass
