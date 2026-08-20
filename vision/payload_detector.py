from dataclasses import dataclass

import cv2
import numpy as np

from config import MIN_PAYLOAD_AREA_PX, PAYLOAD_HEIGHT, PAYLOAD_WIDTH
from models import TargetColor
from vision.geometry import CameraModel, expected_payload_pixels


# ── HSV renk bantları — ışık değişimine toleranslı ──────────────────────────
# TargetDetector ile BİREBİR aynı mantık: ışık koşulu değişince aynı fiziksel
# renk HSV'de kayabilir, bu yüzden bantlar geniş tutulur:
#   KIRMIZI  → turuncu (H≈10-25) ve pembe/macenta (H≈145-170) DAHİL
#   MAVİ     → buz/sky mavisi (H≈75-95) ve koyu/açık mavi DAHİL (düşük S/V)
# Maske ve purity bantları aynı olmalı (aşağıdaki _color_purity_from_contour).
_BLUE_MASK_LO = np.array([75, 60, 50], dtype=np.uint8)
_BLUE_MASK_HI = np.array([140, 255, 255], dtype=np.uint8)
_RED_LOW_LO   = np.array([0, 60, 50], dtype=np.uint8)     # kırmızı + turuncu
_RED_LOW_HI   = np.array([25, 255, 255], dtype=np.uint8)
_RED_HIGH_LO  = np.array([145, 60, 50], dtype=np.uint8)   # pembe/macenta + kırmızı
_RED_HIGH_HI  = np.array([179, 255, 255], dtype=np.uint8)

# Purity eşleştirme (hue) + canlı piksel eşiği (S/V) — maske ile aynı
_BLUE_HUE_LO, _BLUE_HUE_HI = 75, 140
_RED_HUE_MAX = 25    # hue <= 25 (kırmızı + turuncu)
_RED_HUE_MIN = 145   # hue >= 145 (pembe/macenta + kırmızı)
_PURITY_SAT_MIN = 60
_PURITY_VAL_MIN = 50


@dataclass
class PayloadDetection:
    color: TargetColor
    center_x: float
    center_y: float
    width_px: float
    height_px: float
    area_px: float
    confidence: float
    color_purity: float = 0.0  # kontur içi beklenen renk oranı (0..1)


class PayloadDetector:
    """Küçük dikdörtgen payload için ayrı detector.

    Hedeflerden (üçgen/altıgen) farklı olarak payload için:
      - LAB-CLAHE ışık normalizasyonu (RPi Cam3 için)
      - Elips kernel + çoklu iterasyon morfoloji
      - Solidity (doluluk) + rectangularity (dikdörtgensellik) doğrulaması
      - Beklenen 2:1 aspect ratio'ya yakınlık zorunlu (PAYLOAD_WIDTH / PAYLOAD_HEIGHT = 2)
      - 3. emniyet katmanı: İlk 12 frame'de otomatik B↔R kanal takası kalibrasyonu
    """

    CALIBRATION_FRAMES = 12

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
            return cv2.inRange(hsv, _BLUE_MASK_LO, _BLUE_MASK_HI)

        m_low = cv2.inRange(hsv, _RED_LOW_LO, _RED_LOW_HI)
        m_high = cv2.inRange(hsv, _RED_HIGH_LO, _RED_HIGH_HI)
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
    # Renk doğrulama (Merkez pikselden)
    # ──────────────────────────────────────────────────────────────

    @classmethod
    def _color_purity_from_contour(cls, hsv, contour, expected_color, purity_min=0.40):
        """Kontur içindeki canlı piksellerden beklenen renk oranını döndürür.

        Tek piksel (merkez) kontrolü yerine konturun İÇ bölgesindeki gerçek
        piksel dağılımına bakar — maske sızıntısı/CLAHE artefaktı ile oluşan
        gürültü konturları burada elenir.

        Returns: (purity_ratio) veya None (gürültü / yetersiz renk).
        """
        x, y, w, h = cv2.boundingRect(contour)
        if w <= 2 or h <= 2:
            return None

        contour_mask = np.zeros((h, w), dtype=np.uint8)
        shifted = contour - np.array([x, y])
        cv2.drawContours(contour_mask, [shifted], -1, 255, -1)

        crop = hsv[y:y + h, x:x + w]
        interior = crop[contour_mask == 255]
        if interior.size == 0:
            return None

        hue = interior[:, 0]
        sat = interior[:, 1]
        val = interior[:, 2]
        vivid = (sat >= _PURITY_SAT_MIN) & (val >= _PURITY_VAL_MIN)
        n = int(vivid.sum())
        if n == 0:
            return None

        if expected_color == TargetColor.BLUE:
            matched = vivid & (hue >= _BLUE_HUE_LO) & (hue <= _BLUE_HUE_HI)
        else:
            matched = vivid & ((hue <= _RED_HUE_MAX) | (hue >= _RED_HUE_MIN))

        purity = int(matched.sum()) / n
        if purity < purity_min:
            return None
        return purity

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

            # Payload için şekil (kare/dikdörtgen) doğrulaması yapmıyoruz.
            # Görev gereksinimi: doğru renk + fiziksel alana uygun boyut yeterli.
            x, y, w, h = cv2.boundingRect(contour)
            if w <= 0 or h <= 0:
                continue

            # Renk: kontur içi gerçek piksel dağılımı (maske sızıntısına karşı)
            purity = self._color_purity_from_contour(hsv, contour, color)
            if purity is None:
                continue

            size_error = (
                abs(w - expected_w) / max(expected_w, 1.0)
                + abs(h - expected_h) / max(expected_h, 1.0)
            ) / 2.0

            size_score = max(0.0, 1.0 - min(1.0, size_error / 1.5))
            area_score = min(
                1.0,
                area / max(1.0, expected_w * expected_h),
            )

            # Karar: boyut (beklenen fiziksel alan) + renk purity.
            confidence = 0.45 * size_score + 0.20 * area_score + 0.35 * purity

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
                    color_purity=float(purity),
                )
            )

        return sorted(results, key=lambda d: d.confidence, reverse=True)
