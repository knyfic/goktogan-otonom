import cv2
import numpy as np

from models import TargetColor
from vision.geometry import CameraModel
from vision.payload_detector import PayloadDetector
from vision.target_detector import TargetDetector


def test_blue_hexagon_detection():
    img = np.zeros((720, 1280, 3), dtype=np.uint8)
    pts = np.array(
        [
            [560, 250], [720, 250], [800, 360],
            [720, 470], [560, 470], [480, 360],
        ],
        dtype=np.int32,
    )
    cv2.fillPoly(img, [pts], (255, 0, 0))

    detections = TargetDetector().detect(img)
    assert any(
        d.color == TargetColor.BLUE and d.shape == "HEXAGON"
        for d in detections
    )


def test_red_triangle_detection():
    img = np.zeros((720, 1280, 3), dtype=np.uint8)
    pts = np.array(
        [[640, 230], [500, 470], [780, 470]],
        dtype=np.int32,
    )
    cv2.fillPoly(img, [pts], (0, 0, 255))

    detections = TargetDetector().detect(img)
    assert any(
        d.color == TargetColor.RED and d.shape == "TRIANGLE"
        for d in detections
    )


def test_payload_detector_constructs():
    camera = CameraModel(1280, 720, 78, 60)
    detector = PayloadDetector(camera)
    img = np.zeros((720, 1280, 3), dtype=np.uint8)
    cv2.rectangle(img, (620, 350), (700, 390), (0, 0, 255), -1)

    results = detector.detect(img, 1.0, TargetColor.RED)
    assert isinstance(results, list)
