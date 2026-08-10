import time
from collections import Counter, deque
from typing import Optional

import cv2
import numpy as np

from config import (
    DETECTION_HISTORY_SIZE,
    MIN_DETECTION_CONFIDENCE,
    MIN_STABLE_DETECTIONS,
    MIN_TARGET_AREA_PX,
    TARGET_LOCK_DURATION_S,
)
from models import Detection, TargetColor


_KAPAMA_KERNEL_Buyuk = 7
_KAPAMA_KERNEL_Kucuk = 3


class TargetDetector:
    """Mavi altıgen + kırmızı üçgen detector.

    İki aşamalı yaklaşımla çok daha az yanlış pozitif:
    1. LAB-CLAHE ile ışık normalizasyonu → HSV renk maskesi
    2. Renk maskesi başarısız olursa yedek: adaptive threshold + kenar tabanlı

    Her kontur için solidity (doluluk) ve aspect ratio doğrulaması +
    çoklu epsilon (0.03 / 0.04 / 0.05) ile çoğunluklu köşe sayımı.
    """

    CALIBRATION_FRAMES = 12   # İlk N frame boyunca renk oranı kalibrasyonu yap
    CALIBRATION_SWAP_RATIO = 4.0  # (kırmızı_piksel / mavi_piksel) veya tersi ≥ bu ise → takas kararı

    def __init__(self):
        self.history: deque = deque(maxlen=DETECTION_HISTORY_SIZE)
        self._first_seen: dict[tuple, float] = {}
        self.time_locked_color: Optional[TargetColor] = None
        self.time_locked_shape: Optional[str] = None

        # ⚠ Defansif otomatik B ↔ R kanal takası (IMX708 sorunu için 3. emniyet katmanı)
        #   None → kalibrasyon devam ediyor
        #   True → sürekli B ↔ R takası uygula
        #   False → takas YOK (kalibrasyon tamamlandı, düzgün)
        self._auto_swap: Optional[bool] = None
        self._calib_count: int = 0
        # SUM metriği sayaçları
        self._calib_sum_noswap: int = 0
        self._calib_sum_swap:   int = 0
        # Homojenlik metriği (tek renkli sahneler için)
        self._calib_homo_noswap: float = 0.0
        self._calib_homo_swap:   float = 0.0
        # MAX bant metriği
        self._calib_max_noswap:  int = 0
        self._calib_max_swap:    int = 0
        # M4: Çapraz Oran metriği için ayrı ayrı bant toplamları
        self._calib_band_blue_noswap: int = 0
        self._calib_band_red_noswap:  int = 0
        self._calib_band_blue_swap:   int = 0
        self._calib_band_red_swap:    int = 0

    # ──────────────────────────────────────────────────────────────
    # DEFANSİF: B ↔ R kanalları takası gerekiyor mu? Otomatik kalibrasyon
    # ──────────────────────────────────────────────────────────────

    def _maybe_auto_swap(self, frame_bgr: np.ndarray) -> np.ndarray:
        """AutoSwap V3 — İlk CALIBRATION_FRAMES frame'de iki hipotezi (takas vs
        takasız) ÜÇ METRİK ile karşılaştır, KALICI karar ver:

        (M1) SUM metriği  = mavi_piksel + kırmızı_piksel (toplam anlamlı piksel
        (M2) HOMOJENLİK = max(baskın_bant, min_bant) / (min(bant) + eps
                            (tek renkli sahnelerde belirleyici)
        (M3) MAXBANT    = max(mavi_piksel, kırmızı_piksel)

        Her metrik için (SWAP versiyonu / NOSWAP versiyonu) oranları alınır.
        Karar: 3 metrikten ≥ 2 tanesi SWAP lehinde ≥ 1.35x ÜSTÜNLÜK göstermiyse
        VEYA herhangi biri ≥ 2.5x AÇIK üstünlük göstermiyorsa — takas AÇILIR.

        Bu metrik üçlüsü:
          • Tek renkli (sadece mavi altıgen, sadece kırmızı üçgen) testler
          • Karışık renkli (iki nesneli GERÇEK yarışma sahneleri
          • Boş / az renkli / loş ışık sahneleri
        HEPSİNDE doğru karar verir.
        """
        h, w = frame_bgr.shape[:2]

        if self._auto_swap is True:
            return frame_bgr[:, :, ::-1].copy()
        if self._auto_swap is False:
            return frame_bgr

        scale = max(1, min(w, h) // 480)
        small = frame_bgr[::scale, ::scale]
        small_swapped = small[:, :, ::-1].copy()

        def _split_counts(bgr) -> tuple[int, int]:
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            m_blue = cv2.inRange(
                hsv,
                np.array([95, 70, 60], dtype=np.uint8),
                np.array([130, 255, 255], dtype=np.uint8),
            )
            m_rl = cv2.inRange(
                hsv,
                np.array([0, 70, 60], dtype=np.uint8),
                np.array([10, 255, 255], dtype=np.uint8),
            )
            m_rh = cv2.inRange(
                hsv,
                np.array([170, 70, 60], dtype=np.uint8),
                np.array([179, 255, 255], dtype=np.uint8),
            )
            return (
                int(cv2.countNonZero(m_blue)),
                int(cv2.countNonZero(m_rl) + cv2.countNonZero(m_rh)),
            )

        b_blue, b_red = _split_counts(small)
        s_blue, s_red = _split_counts(small_swapped)
        eps = 20

        # Aggregate sayaçları güncelle
        self._calib_sum_noswap += (b_blue + b_red)
        self._calib_sum_swap   += (s_blue + s_red)
        self._calib_max_noswap  += max(b_blue, b_red)
        self._calib_max_swap    += max(s_blue, s_red)
        self._calib_homo_noswap += (max(b_blue, b_red) + eps) / (min(b_blue, b_red) + eps)
        self._calib_homo_swap   += (max(s_blue, s_red) + eps) / (min(s_blue, s_red) + eps)
        self._calib_count += 1

        if self._calib_count < self.CALIBRATION_FRAMES:
            return frame_bgr

        # --- Kalibrasyon tamam — karar verme aşaması
        eps_m = 50
        r_sum  = (self._calib_sum_swap    + eps_m) / (self._calib_sum_noswap  + eps_m)
        r_homo = (self._calib_homo_swap   + eps_m) / (self._calib_homo_noswap + eps_m)
        r_max  = (self._calib_max_swap    + eps_m) / (self._calib_max_noswap + eps_m)

        swap_wins = (
            (1 if r_sum  >= 1.35 else 0)
            + (1 if r_homo >= 1.35 else 0)
            + (1 if r_max  >= 1.35 else 0)
        )
        any_clear = max(r_sum, r_homo, r_max) >= 2.5

        self._auto_swap = bool(swap_wins >= 2 or any_clear)

        import logging
        log = logging.getLogger("TargetDetector")
        if self._auto_swap:
            log.warning(
                "[AutoSwap-V3] ⚠ BAYT SIRASI TERS (RGB/BGR uyumsuz) — KALICI B↔R takası AÇILDI. "
                "Metrikler (swap/noswap): SUM=%.1fx  HOMO=%.1fx  MAXBANT=%.1fx  "
                "swap_wins=%d/3  clear_win_2.5x=%s",
                r_sum, r_homo, r_max, swap_wins, any_clear,
            )
        else:
            log.info(
                "[AutoSwap-V3] ✓ Renk kanalları DÜZGÜN — takas YOK. "
                "Metrikler (swap/noswap): SUM=%.1fx  HOMO=%.1fx  MAXBANT=%.1fx  "
                "swap_wins=%d/3",
                r_sum, r_homo, r_max, swap_wins,
            )

        # Tamponları sıfırla (gelecekte tekrar kalibrasyon yapmasın — tek seferlik)
        self._calib_sum_noswap = 0
        self._calib_sum_swap = 0
        self._calib_homo_noswap = 0.0
        self._calib_homo_swap = 0.0
        self._calib_max_noswap = 0
        self._calib_max_swap = 0
        self._calib_count = 0

        return frame_bgr

    # ──────────────────────────────────────────────────────────────
    # Renk ve ön işleme yardımcıları
    # ──────────────────────────────────────────────────────────────

    @staticmethod
    def _preprocess_for_hsv(frame_bgr):
        """LAB uzayında CLAHE ile kontrast eşitle → HSV'ye dön.

        RPi Camera Module 3 (IMX708) ve değişken dış ışık koşullarında
        renk maskesini çok daha kararlı hale getirir.
        """
        lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))
        l = clahe.apply(l)
        equalized = cv2.cvtColor(cv2.merge((l, a, b)), cv2.COLOR_LAB2BGR)
        return cv2.cvtColor(equalized, cv2.COLOR_BGR2HSV)

    @staticmethod
    def _color_mask(hsv, color):
        if color == TargetColor.BLUE:
            return cv2.inRange(
                hsv,
                np.array([95, 90, 70], dtype=np.uint8),
                np.array([130, 255, 255], dtype=np.uint8),
            )

        mask_low = cv2.inRange(
            hsv,
            np.array([0, 100, 70], dtype=np.uint8),
            np.array([10, 255, 255], dtype=np.uint8),
        )
        mask_high = cv2.inRange(
            hsv,
            np.array([170, 100, 70], dtype=np.uint8),
            np.array([179, 255, 255], dtype=np.uint8),
        )
        return cv2.bitwise_or(mask_low, mask_high)

    @staticmethod
    def _clean_mask(mask, kernel_size=_KAPAMA_KERNEL_Buyuk):
        """Elips kernel ile kapat (delikleri doldur) sonra aç (küçük gürültüyü at)."""
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        return mask

    # ──────────────────────────────────────────────────────────────
    # Şekil doğrulama ve köşe sayımı
    # ──────────────────────────────────────────────────────────────

    @staticmethod
    def _is_valid_shape(contour, area) -> bool:
        """Solidity (doluluk) ve aspect ratio ile şekil doğruluğu kontrolü.

        Yanlış pozitifleri (kırıksı, U şekilli, çok uzun/dar konturlar)
        maskeleyerek yüzde onlarca yanlış tespiti engeller.
        """
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        if hull_area == 0:
            return False
        solidity = float(area) / hull_area
        if solidity < 0.85:
            return False

        _, _, w, h = cv2.boundingRect(contour)
        if w == 0 or h == 0:
            return False
        aspect_ratio = float(w) / h
        if aspect_ratio < 0.5 or aspect_ratio > 2.0:
            return False
        return True

    @staticmethod
    def _count_vertices_robust(contour) -> int:
        """3 farklı epsilon ile approxPolyDP → çoğunluklu (most-common) sonuç.

        Tek epsilon (0.035) kullanmak köşe sayımını çok hassas yapıyordu;
        0.03 / 0.04 / 0.05 çoğunlukluğu gerçek şekle çok daha yakın olur.
        """
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            return 0
        votes = []
        for eps in (0.03, 0.04, 0.05):
            approx = cv2.approxPolyDP(contour, eps * perimeter, True)
            votes.append(len(approx))
        return Counter(votes).most_common(1)[0][0]

    @staticmethod
    def _classify_shape(vertices: int) -> Optional[str]:
        if vertices == 3:
            return "TRIANGLE"
        if 5 <= vertices <= 7:
            return "HEXAGON"
        return None

    # ──────────────────────────────────────────────────────────────
    # Yedek (fallback) kontur çıkarımı: gri + adaptive threshold
    # ──────────────────────────────────────────────────────────────

    @staticmethod
    def _fallback_contours(frame_bgr):
        """Renk maskesi başarısız/parçalı olduğunda gri tonlamalı yöntem.

        Adaptive threshold → kenar tabanlı kontur çıkarımı. Referans kodun
        on_isleme() mantığının doğrudan uyarlaması.
        """
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.adaptiveThreshold(
            blurred,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            11,
            2,
        )
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=1)
        contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        return contours

    # ──────────────────────────────────────────────────────────────
    # Yardımcı: kontur listesinden tespit çıkarımı
    # ──────────────────────────────────────────────────────────────

    def _detections_from_contours(
        self, contours, frame_hsv, mask_hint=None, color_override=None
    ):
        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < MIN_TARGET_AREA_PX:
                continue

            if not self._is_valid_shape(contour, area):
                continue

            vertices = self._count_vertices_robust(contour)
            shape = self._classify_shape(vertices)
            if shape is None:
                continue

            expected_vertices = 6 if shape == "HEXAGON" else 3
            shape_score = max(0.0, 1.0 - abs(vertices - expected_vertices) / 4.0)
            area_score = min(1.0, area / 15000.0)

            if color_override is not None:
                color = color_override
            else:
                color = self._classify_color_from_mask(frame_hsv, contour)
                if color is None:
                    # Fallback modda renk maskesi olmadan kontur gelmiş olabilir;
                    # bu durumda atla (renksiz şekil = muhtemelen gürültü)
                    if mask_hint == "color":
                        continue
                    # Gri modda renk kararı contour bounding rect'ten yapılır
                    color = self._classify_color_from_roi(frame_hsv, contour)
                    if color is None:
                        continue

            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue
            cx = moments["m10"] / moments["m00"]
            cy = moments["m01"] / moments["m00"]
            x, y, w, h = cv2.boundingRect(contour)

            confidence = 0.65 * shape_score + 0.35 * area_score
            if confidence < MIN_DETECTION_CONFIDENCE:
                continue

            detections.append(
                Detection(
                    color=color,
                    shape=shape,
                    center_x=cx,
                    center_y=cy,
                    area_px=area,
                    bbox_width_px=w,
                    bbox_height_px=h,
                    confidence=confidence,
                )
            )
        return detections

    @staticmethod
    def _classify_color_from_mask(hsv, contour):
        """Konturun merkez pikselinden HSV ile renk kararını doğrula."""
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return None
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
        h, w = hsv.shape[:2]
        if not (0 <= cx < w and 0 <= cy < h):
            return None

        px = hsv[cy, cx]
        hue, sat, val = int(px[0]), int(px[1]), int(px[2])
        if sat < 70 or val < 60:
            return None
        if 95 <= hue <= 130:
            return TargetColor.BLUE
        if (hue <= 10 or hue >= 170):
            return TargetColor.RED
        return None

    @classmethod
    def _classify_color_from_roi(cls, hsv, contour):
        """Bounding rect içindeki HSV histogramı ile renk sınıflandırması."""
        x, y, w, h = cv2.boundingRect(contour)
        if w <= 2 or h <= 2:
            return None
        roi = hsv[y:y + h, x:x + w]
        if roi.size == 0:
            return None
        mask = cv2.inRange(
            roi,
            np.array([0, 70, 60], dtype=np.uint8),
            np.array([179, 255, 255], dtype=np.uint8),
        )
        if cv2.countNonZero(mask) == 0:
            return None
        mean_hsv = cv2.mean(roi, mask=mask)
        hue, sat, val = int(mean_hsv[0]), int(mean_hsv[1]), int(mean_hsv[2])
        if sat < 70 or val < 60:
            return None
        if 95 <= hue <= 130:
            return TargetColor.BLUE
        if hue <= 10 or hue >= 170:
            return TargetColor.RED
        return None

    # ──────────────────────────────────────────────────────────────
    # Ana detect() metodu
    # ──────────────────────────────────────────────────────────────

    def _detect_noswap(self, frame_bgr):
        """Detect helper that assumes given frame is correctly BGR (does NOT call _maybe_auto_swap)."""
        hsv = self._preprocess_for_hsv(frame_bgr)
        all_detections = []

        for color in (TargetColor.BLUE, TargetColor.RED):
            mask = self._color_mask(hsv, color)
            mask = self._clean_mask(mask)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            dets = self._detections_from_contours(
                contours, hsv, mask_hint="color", color_override=color
            )
            all_detections.extend(dets)

        if all_detections:
            return sorted(all_detections, key=lambda d: d.confidence, reverse=True)

        fallback_contours = self._fallback_contours(frame_bgr)
        fallback_dets = self._detections_from_contours(
            fallback_contours, hsv, mask_hint="gray"
        )
        return sorted(fallback_dets, key=lambda d: d.confidence, reverse=True)

    def detect(self, frame_bgr):
        # If calibration hasn't decided, try a quick dual-pass heuristic to decide swap
        if self._auto_swap is None:
            # run noswap and swapped detectors and compare best confidences
            det_n = self._detect_noswap(frame_bgr)
            det_s = self._detect_noswap(frame_bgr[:, :, ::-1].copy())
            best_n = det_n[0].confidence if det_n else 0.0
            best_s = det_s[0].confidence if det_s else 0.0
            # Prefer the view with significantly higher top confidence
            if best_s > max(0.0, best_n * 1.10) and best_s >= 0.35:
                self._auto_swap = True
            elif best_n > max(0.0, best_s * 1.10) and best_n >= 0.35:
                self._auto_swap = False
            # else leave as None and fall back to metric-based decision in _maybe_auto_swap

        # 3. emniyet katmanı — Defansif B↔R kanalı otomatik kalibrasyon
        frame_bgr = self._maybe_auto_swap(frame_bgr)

        hsv = self._preprocess_for_hsv(frame_bgr)
        all_detections = []

        for color in (TargetColor.BLUE, TargetColor.RED):
            mask = self._color_mask(hsv, color)
            mask = self._clean_mask(mask)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            dets = self._detections_from_contours(
                contours, hsv, mask_hint="color", color_override=color
            )
            all_detections.extend(dets)

        if all_detections:
            return sorted(all_detections, key=lambda d: d.confidence, reverse=True)

        fallback_contours = self._fallback_contours(frame_bgr)
        fallback_dets = self._detections_from_contours(
            fallback_contours, hsv, mask_hint="gray"
        )
        return sorted(fallback_dets, key=lambda d: d.confidence, reverse=True)

    # ──────────────────────────────────────────────────────────────
    # Zaman kilidi, history, draw — davranış aynı kalıyor
    # ──────────────────────────────────────────────────────────────

    def update_time_lock(self, detections: list) -> None:
        if self.time_locked_color is not None:
            return

        now = time.monotonic()
        seen_types = {(d.color, d.shape) for d in detections}

        for key in list(self._first_seen.keys()):
            if key not in seen_types:
                del self._first_seen[key]

        for key in seen_types:
            if key not in self._first_seen:
                self._first_seen[key] = now

        for (color, shape), first_t in self._first_seen.items():
            if now - first_t >= TARGET_LOCK_DURATION_S:
                self.time_locked_color = color
                self.time_locked_shape = shape
                return

    def reset_time_lock(self) -> None:
        self.time_locked_color = None
        self.time_locked_shape = None
        self._first_seen.clear()

    @staticmethod
    def draw_detections(frame_bgr, detections):
        out = frame_bgr.copy()
        h, w = out.shape[:2]
        cx, cy = w // 2, h // 2
        cv2.drawMarker(out, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 20, 2)

        for det in detections:
            color_bgr = (255, 80, 0) if det.color.value == "BLUE" else (0, 60, 255)
            tx = int(det.center_x)
            ty = int(det.center_y)

            cv2.circle(out, (tx, ty), 6, color_bgr, -1)
            cv2.line(out, (cx, cy), (tx, ty), color_bgr, 1)

            bw = int(det.bbox_width_px)
            bh = int(det.bbox_height_px)
            x1 = tx - bw // 2
            y1 = ty - bh // 2
            cv2.rectangle(out, (x1, y1), (x1 + bw, y1 + bh), color_bgr, 2)

            label = f"{det.color.value} {det.shape} {det.confidence:.2f}"
            cv2.putText(
                out, label, (x1, max(y1 - 8, 12)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color_bgr, 2,
            )

        cv2.putText(
            out, f"Det: {len(detections)}",
            (10, frame_bgr.shape[0] - 12),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
        )
        return out

    def stable_detection(
        self,
        detections,
        locked_color: Optional["TargetColor"] = None,
        locked_shape: Optional[str] = None,
        search_mode: bool = False,
    ) -> Optional[Detection]:
        if not detections:
            self.history.clear()
            return None

        effective_color = locked_color
        effective_shape = locked_shape

        if effective_color is None and self.time_locked_color is not None:
            effective_color = self.time_locked_color
            effective_shape = self.time_locked_shape

        if effective_color is not None and effective_shape is not None:
            filtered = [
                d for d in detections
                if d.color == effective_color and d.shape == effective_shape
            ]
            if not filtered:
                return None
            candidate = filtered[0]
        else:
            if len(self.history) == 0:
                candidate = detections[0]
            else:
                last = self.history[-1]
                same_type = [
                    d for d in detections
                    if d.color == last.color and d.shape == last.shape
                ]
                candidate = (
                    max(same_type, key=lambda d: d.confidence)
                    if same_type
                    else detections[0]
                )

        self.history.append(candidate)

        if search_mode:
            compatible = [
                item
                for item in self.history
                if item.color == candidate.color and item.shape == candidate.shape
            ]
        else:
            compatible = [
                item
                for item in self.history
                if item.color == candidate.color
                and item.shape == candidate.shape
                and abs(item.center_x - candidate.center_x) < 80
                and abs(item.center_y - candidate.center_y) < 80
            ]

        if len(compatible) < MIN_STABLE_DETECTIONS:
            return None

        return max(compatible, key=lambda d: d.confidence)
