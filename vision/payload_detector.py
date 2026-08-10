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
    """Küçük dikdörtgen payload için ayrı detector.

    Hedeflerden (üçgen/altıgen) farklı olarak payload için:
      - LAB-CLAHE ışık normalizasyonu (RPi Cam3 için)
      - Elips kernel + çoklu iterasyon morfoloji
      - Solidity (doluluk) + rectangularity (dikdörtgensellik) doğrulaması
      - Beklenen 2:1 aspect ratio'ya yakınlık zorunlu (PAYLOAD_WIDTH / PAYLOAD_HEIGHT = 2)
    """

    def __init__(self, camera: CameraModel):
        self.camera = camera

    # ──────────────────────────────────────────────────────────────
    # Ön işleme & renk maskesi
    # ──────────────────────────────────────────────────────────────

    @staticmethod
    def _preprocess_for_hsv(frame_bgr):
        """LAB uzayında CLAHE ile kontrast eşitle → HSV."""
        lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))
        l = clahe.apply(l)
        equalized = cv2.cvtColor(cv2.merge((l, a, b)), cv2.COLOR_LAB2BGR)
        return cv2.cvtColor(equalized, cv2.COLOR_BGR2HSV)

    @staticmethod
    def _mask(hsv, color):
        if color == TargetColor.BLUE:
            return cv2.inRange(
                hsv,
                np.array([95, 90, 65], dtype=np.uint8),
                np.array([130, 255, 255], dtype=np.uint8),
            )

        m_low = cv2.inRange(
            hsv,
            np.array([0, 100, 65], dtype=np.uint8),
            np.array([10, 255, 255], dtype=np.uint8),
        )
        m_high = cv2.inRange(
            hsv,
            np.array([170, 100, 65], dtype=np.uint8),
            np.array([179, 255, 255], dtype=np.uint8),
        )
        return cv2.bitwise_or(m_low, m_high)

    @staticmethod
    def _clean_mask(mask, kernel_size=5):
        """Elips kernel: önce 2x kapat (delikleri doldur) sonra 1x aç (gürültüyü at)."""
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        return mask

    # ──────────────────────────────────────────────────────────────
    # Dikdörtgen şekil doğrulaması
    # ──────────────────────────────────────────────────────────────

    @staticmethod
    def _is_valid_rectangle(contour, area) -> bool:
        """Payload dikdörtgen olduğu için üç doğrulama:
        1. Solidity (doluluk) — içi boş/kırıksı konturları atar
        2. Rectangularity (dikdörtgensellik) — contour alanı / bbox alanı
        3. Aspect ratio — PAYLOAD 20cm × 10cm yani 2:1 oranı beklenir
        """
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        if hull_area == 0:
            return False
        solidity = float(area) / hull_area
        if solidity < 0.82:
            return False

        x, y, w, h = cv2.boundingRect(contour)
        if w == 0 or h == 0:
            return False

        bbox_area = float(w) * float(h)
        if bbox_area == 0:
            return False
        rectangularity = float(area) / bbox_area
        if rectangularity < 0.70:
            return False

        aspect = float(w) / float(h)
        expected_aspect = max(PAYLOAD_WIDTH, PAYLOAD_HEIGHT) / min(
            PAYLOAD_WIDTH, PAYLOAD_HEIGHT
        )
        aspect_max = max(aspect, 1.0 / max(aspect, 1e-9))
        expected_max = max(expected_aspect, 1.0 / expected_aspect)
        if aspect_max > expected_max * 1.6 or aspect_max < expected_max * 0.6:
            return False

        return True

    # ──────────────────────────────────────────────────────────────
    # Renk doğrulama (ROI histogramı)
    # ──────────────────────────────────────────────────────────────

    @classmethod
    def _verify_color_roi(cls, hsv, contour, expected_color) -> bool:
        """Bounding rect ROI'sindeki HSV ortalaması ile rengi doğrula.

        Kontur doğru renkte değilse (örn. mavi maskeye rağmen yeşil bir
        nesne yanlışlıkla maskeye girdiyse) tespiti reddeder.
        """
        x, y, w, h = cv2.boundingRect(contour)
        if w <= 2 or h <= 2:
            return False
        roi = hsv[y:y + h, x:x + w]
        if roi.size == 0:
            return False

        sat_mask = cv2.inRange(
            roi,
            np.array([0, 70, 55], dtype=np.uint8),
            np.array([179, 255, 255], dtype=np.uint8),
        )
        if cv2.countNonZero(sat_mask) < max(1, int(roi.shape[0] * roi.shape[1] * 0.25)):
            return False

        mean_hsv = cv2.mean(roi, mask=sat_mask)
        hue, sat, val = int(mean_hsv[0]), int(mean_hsv[1]), int(mean_hsv[2])
        if sat < 70 or val < 55:
            return False

        if expected_color == TargetColor.BLUE:
            return 95 <= hue <= 130
        return hue <= 10 or hue >= 170

    # ──────────────────────────────────────────────────────────────
    # Ana detect() metodu
    # ──────────────────────────────────────────────────────────────

    def detect(self, frame_bgr, altitude_m, color):
        hsv = self._preprocess_for_hsv(frame_bgr)
        mask = self._mask(hsv, color)
        mask = self._clean_mask(mask)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        expected_w, expected_h = expected_payload_pixels(
            PAYLOAD_WIDTH, PAYLOAD_HEIGHT, altitude_m, self.camera
        )

        expected_aspect = max(PAYLOAD_WIDTH, PAYLOAD_HEIGHT) / min(
            PAYLOAD_WIDTH, PAYLOAD_HEIGHT
        )

        results = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < MIN_PAYLOAD_AREA_PX:
                continue

            if not self._is_valid_rectangle(contour, area):
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if w <= 0 or h <= 0:
                continue

            if not self._verify_color_roi(hsv, contour, color):
                continue

            aspect = max(w, h) / max(1.0, min(w, h))

            size_error = (
                abs(w - expected_w) / max(expected_w, 1.0)
                + abs(h - expected_h) / max(expected_h, 1.0)
            ) / 2.0
            aspect_error = abs(aspect - expected_aspect) / expected_aspect

            size_score = max(0.0, 1.0 - min(1.0, size_error / 1.5))
            aspect_score = max(0.0, 1.0 - min(1.0, aspect_error / 0.7))
            area_score = min(
                1.0,
                area / max(1.0, expected_w * expected_h),
            )

            confidence = (
                0.40 * size_score
                + 0.35 * aspect_score
                + 0.25 * area_score
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
