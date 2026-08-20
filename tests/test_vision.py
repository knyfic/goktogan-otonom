import cv2
import numpy as np

from models import TargetColor
from vision.geometry import CameraModel
from vision.payload_detector import PayloadDetector
from vision.target_detector import TargetDetector

# Test frame çözünürlüğü: gerçek kamera ile eşleşmeli
W, H = 1920, 1080


def test_blue_hexagon_detection():
    img = np.zeros((H, W, 3), dtype=np.uint8)
    # Düzgün (eşkenar) altıgen: kenar 240px, 1920×1080'de ortalanmış
    pts = np.array(
        [[720, 540], [840, 332], [1080, 332], [1200, 540], [1080, 748], [840, 748]],
        dtype=np.int32,
    )
    cv2.fillPoly(img, [pts], (255, 0, 0))

    detections = TargetDetector().detect(img)
    assert any(
        d.color == TargetColor.BLUE and d.shape == "HEXAGON"
        for d in detections
    )


def test_red_triangle_detection():
    img = np.zeros((H, W, 3), dtype=np.uint8)
    # Triangle: 1280×720'deki koordinatlar 1920×1080'e ölçeklenmiş
    pts = np.array(
        [[960, 345], [750, 705], [1170, 705]],
        dtype=np.int32,
    )
    cv2.fillPoly(img, [pts], (0, 0, 255))

    detections = TargetDetector().detect(img)
    assert any(
        d.color == TargetColor.RED and d.shape == "TRIANGLE"
        for d in detections
    )


def test_payload_detector_constructs():
    camera = CameraModel(1920, 1080, 78, 46.4)
    detector = PayloadDetector(camera)
    img = np.zeros((H, W, 3), dtype=np.uint8)
    # Kırmızı dikdörtgen: 1920×1080 merkezine yakın
    cv2.rectangle(img, (930, 525), (1050, 585), (0, 0, 255), -1)

    results = detector.detect(img, 1.0, TargetColor.RED)
    assert isinstance(results, list)
