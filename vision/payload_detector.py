from dataclasses import dataclass

import cv2
import numpy as np

from config import MIN_PAYLOAD_AREA_PX, PAYLOAD_HEIGHT, PAYLOAD_WIDTH
from models import TargetColor
from vision.geometry import CameraModel, expected_payload_pixels


@dataclass
class PayloadDetection:
    color: TargetColor
    center_x: float
    center_y: float
    width_px: float
    height_px: float
    area_px: float
    confidence: float


class PayloadDetector:
    """Küçük dikdörtgen payload için ayrı detector."""

    def __init__(self, camera: CameraModel):
        self.camera = camera

    @staticmethod
    def _mask(hsv, color):
        if color == TargetColor.BLUE:
            return cv2.inRange(
                hsv,
                np.array([90, 80, 50], dtype=np.uint8),
                np.array([135, 255, 255], dtype=np.uint8),
            )

        m_low = cv2.inRange(
            hsv,
            np.array([0, 80, 50], dtype=np.uint8),
            np.array([10, 255, 255], dtype=np.uint8),
        )
        m_high = cv2.inRange(
            hsv,
            np.array([170, 80, 50], dtype=np.uint8),
            np.array([179, 255, 255], dtype=np.uint8),
        )
        return cv2.bitwise_or(m_low, m_high)

    def detect(self, frame_bgr, altitude_m, color):
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = self._mask(hsv, color)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        expected_w, expected_h = expected_payload_pixels(
            PAYLOAD_WIDTH, PAYLOAD_HEIGHT, altitude_m, self.camera
        )

        results = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < MIN_PAYLOAD_AREA_PX:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if w <= 0 or h <= 0:
                continue

            aspect = max(w, h) / max(1.0, min(w, h))
            expected_aspect = max(PAYLOAD_WIDTH, PAYLOAD_HEIGHT) / min(
                PAYLOAD_WIDTH, PAYLOAD_HEIGHT
            )

            size_error = (
                abs(w - expected_w) / max(expected_w, 1.0)
                + abs(h - expected_h) / max(expected_h, 1.0)
            )
            aspect_error = abs(aspect - expected_aspect) / expected_aspect

            size_score = max(0.0, 1.0 - min(1.0, size_error / 2.0))
            aspect_score = max(0.0, 1.0 - min(1.0, aspect_error))
            area_score = min(
                1.0,
                area / max(1.0, expected_w * expected_h),
            )

            confidence = (
                0.45 * size_score
                + 0.35 * aspect_score
                + 0.20 * area_score
            )

            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue

            results.append(
                PayloadDetection(
                    color=color,
                    center_x=moments["m10"] / moments["m00"],
                    center_y=moments["m01"] / moments["m00"],
                    width_px=float(w),
                    height_px=float(h),
                    area_px=float(area),
                    confidence=float(confidence),
                )
            )

        return sorted(results, key=lambda d: d.confidence, reverse=True)
