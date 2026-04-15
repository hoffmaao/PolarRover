"""Tests for the vision protocols — NullFrameSource and NullSegmenter."""

from rover_onboard.vision.capture import CameraFrame, NullFrameSource
from rover_onboard.vision.segmentation import (
    Detection,
    DetectionClass,
    NullSegmenter,
)


def test_null_frame_source_yields_then_none():
    frames = [
        CameraFrame(timestamp_s=1.0, image=None, width=640, height=480),
        CameraFrame(timestamp_s=2.0, image=None, width=640, height=480),
    ]
    src = NullFrameSource(frames)
    src.open()
    assert src.next_frame().timestamp_s == 1.0
    assert src.next_frame().timestamp_s == 2.0
    assert src.next_frame() is None


def test_null_segmenter_returns_scripted_detections():
    det = Detection(cls=DetectionClass.PERSON, confidence=0.9, bbox=(0, 0, 100, 100))
    seg = NullSegmenter([[det]])
    frame = CameraFrame(timestamp_s=0, image=None, width=640, height=480)
    assert seg.segment(frame) == [det]


def test_null_segmenter_cycles_scripted_sequence():
    a = Detection(DetectionClass.PERSON, 0.9, (0, 0, 10, 10))
    b = Detection(DetectionClass.VEHICLE, 0.8, (0, 0, 20, 20))
    seg = NullSegmenter([[a], [b]])
    frame = CameraFrame(timestamp_s=0, image=None, width=640, height=480)
    assert seg.segment(frame) == [a]
    assert seg.segment(frame) == [b]
    assert seg.segment(frame) == [a]  # cycles
