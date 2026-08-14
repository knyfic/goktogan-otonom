import logging
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
    TARGET_DETECTOR_AUTO_SWAP,
    TARGET_LOCK_DURATION_S,
)
from models import Detection, TargetColor


_KAPAMA_KERNEL_Buyuk = 7
_KAPAMA_KERNEL_Kucuk = 3

# ── Tespit sağlamlığı sabitleri (algoritma davranışı korunarak sıkılaştırma) ──
# Kontur içi baskın renk oranı eşiği. Renk maskesinden gelen konturlarda bile
# gerçek renk iç-bölgeden doğrulanır; altında kalanlar GÜRÜLTÜ sayılır.
_MIN_COLOR_PURITY = 0.50
# Gri fallback yolu (renk maskesi yok) için biraz gevşek eşik — kontur içine
# arka plan sızabilir.
_FALLBACK_MIN_COLOR_PURITY = 0.40
# Köşe sayımı tutarlılığı: 3 epsilon'dan en az 2'si aynı köşe sayısını vermeli.
# [3,5,6] gibi dağınık oylar şekil kabul edilmez.
_MIN_VERTEX_AGREEMENT = 2
# Şekle özgü extent (alan/bbox_alan) bantları: üçgen ≈ 0.50, altıgen ≈ 0.75.
_TRIANGLE_EXTENT = (0.35, 0.70)
_HEXAGON_EXTENT = (0.55, 0.95)
# area_score referansı (büyük hedef alanı)
_AREA_REF_PX = 15000.0

# Görev hedefleri — yarışma kuralı: yalnızca MAVİ ALTIGEN ve KIRMIZI ÜÇGEN.
# Bu ikisi dışındaki renk+şekil kombinasyonları (örn. BLUE+TRIANGLE) detector
# tarafından ÜRETİLMEZ; aksi halde state machine sürekli "Geçersiz kombinasyon"
# deyip SEARCHING'e döner → sonsuz döngü.
_VALID_TARGET_COMBINATIONS = (
    (TargetColor.BLUE, "HEXAGON"),
    (TargetColor.RED, "TRIANGLE"),
)


class TargetDetector:
    """Mavi altıgen + kırmızı üçgen detector.

    İki aşamalı yaklaşımla çok daha az yanlış pozitif:
    1. LAB-CLAHE ile ışık normalizasyonu → HSV renk maskesi
    2. Renk maskesi başarısız olursa yedek: adaptive threshold + kenar tabanlı

    Her kontur için solidity (doluluk) ve aspect ratio doğrulaması +
    çoklu epsilon (0.03 / 0.04 / 0.05) ile çoğunluklu köşe sayımı.
    """

    CALIBRATION_FRAMES = 12   # İlk N frame boyunca renk oranı kalibrasyonu yap

    def __init__(self):
        self.history: deque = deque(maxlen=DETECTION_HISTORY_SIZE)
        self._first_seen: dict[tuple, float] = {}
        self.time_locked_color: Optional[TargetColor] = None
        self.time_locked_shape: Optional[str] = None
        self._log = logging.getLogger("TargetDetector")

        # ⚠ Defansif otomatik B ↔ R kanal takası (yalnızca TARGET_DETECTOR_AUTO_SWAP=True ise)
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

    # ──────────────────────────────────────────────────────────────
    # DEFANSİF: B ↔ R kanalları takası gerekiyor mu? Otomatik kalibrasyon
    # ──────────────────────────────────────────────────────────────

    def _maybe_auto_swap(self, frame_bgr: np.ndarray) -> np.ndarray:
        """AutoSwap V3 — yalnızca TARGET_DETECTOR_AUTO_SWAP=True iken aktif.

        Varsayılan KAPALI: kanal takasının sahibi kamera katmanıdır
        (RPiCam3 — IMX708 için sabit B↔R takası). Detektör seviyesi
        kalibrasyon, saf mavi+kırmızı sahnelerde simetrik olduğundan gerçek
        bir ters kamerayı ayırt edemez ve karışık renkli sahnelerde
        YANLIŞLIKLA takas açabilir (sistematik renk inversiyonu).

        Bu metrik üçlüsü yalnızca bayrak açıkken güvenlik ağı olarak çalışır:
        (M1) SUM = mavi+kırmızı piksel, (M2) HOMO = max/min bant,
        (M3) MAXBANT = max(mavi, kırmızı). Karar: 3 metrikten ≥2'si
        ≥1.35x SWAP lehinde VEYA biri ≥2.5x açık üstünlük → takas açılır.
        """
        if not TARGET_DETECTOR_AUTO_SWAP:
            return frame_bgr
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

        if self._auto_swap:
            self._log.warning(
                "[AutoSwap-V3] ⚠ BAYT SIRASI TERS (RGB/BGR uyumsuz) — KALICI B↔R takası AÇILDI. "
                "Metrikler (swap/noswap): SUM=%.1fx  HOMO=%.1fx  MAXBANT=%.1fx  "
                "swap_wins=%d/3  clear_win_2.5x=%s",
                r_sum, r_homo, r_max, swap_wins, any_clear,
            )
        else:
            self._log.info(
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
    def _count_vertices_robust(contour) -> tuple[int, int]:
        """3 farklı epsilon ile approxPolyDP → (köşe_sayısı, tutarlılık).

        Tutarlılık = en çok oy alan köşe sayısının oy miktarı (1..3).
        [6,6,5] → (6, 2): kabul edilebilir. [5,6,7] → (5, 1): dağınık → reddet.
        """
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            return 0, 0
        votes = []
        for eps in (0.03, 0.04, 0.05):
            approx = cv2.approxPolyDP(contour, eps * perimeter, True)
            votes.append(len(approx))
        (count, agreement) = Counter(votes).most_common(1)[0]
        return count, agreement

    @staticmethod
    def _classify_shape(count: int, agreement: int) -> Optional[str]:
        """Köşe sayısı + tutarlılıkla şekil sınıflandırması.

        En az _MIN_VERTEX_AGREEMENT oy tutarlılığı zorunludur — dağınık köşe
        sayıları (örn. [3,5,6]) gürültüdür, şekil kabul edilmez.
        """
        if agreement < _MIN_VERTEX_AGREEMENT:
            return None
        if count == 3:
            return "TRIANGLE"
        if 5 <= count <= 7:
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

    def _detections_from_contours(self, contours, frame_hsv, purity_min=_MIN_COLOR_PURITY):
        """Kontur listesinden güvenilir tespitler üretir.

        Renk, konturun İÇİNDEKİ gerçek HSV piksel dağılımından (color purity)
        belirlenir — maskeden çıktı diye rengi varsaymaz (false positive kaynağı).
        Şekil; köşe sayısı tutarlılığı + extent (alan/bbox) ile doğrulanır.
        """
        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < MIN_TARGET_AREA_PX:
                continue

            if not self._is_valid_shape(contour, area):
                self._log.debug("REJECT: invalid shape (solidity/aspect) area=%.0f", area)
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if w <= 2 or h <= 2:
                continue

            count, agreement = self._count_vertices_robust(contour)
            shape = self._classify_shape(count, agreement)
            if shape is None:
                self._log.debug(
                    "REJECT: unstable vertex count votes=(%d,%d) area=%.0f",
                    count, agreement, area,
                )
                continue

            # Şekle özgü extent doğrulaması (üçgen ≈ 0.50, altıgen ≈ 0.75)
            extent = float(area) / float(w * h)
            if shape == "TRIANGLE":
                expected_vertices, expected_extent = 3, 0.50
                ext_lo, ext_hi = _TRIANGLE_EXTENT
            else:
                expected_vertices, expected_extent = 6, 0.75
                ext_lo, ext_hi = _HEXAGON_EXTENT
            if not (ext_lo <= extent <= ext_hi):
                self._log.debug(
                    "REJECT: extent out of range shape=%s extent=%.2f area=%.0f",
                    shape, extent, area,
                )
                continue

            # Renk: kontur içi gerçek piksel dağılımı (color purity)
            color_info = self._color_purity_from_contour(frame_hsv, contour, purity_min)
            if color_info is None:
                self._log.debug(
                    "REJECT: color purity too low area=%.0f", area,
                )
                continue
            color, purity = color_info

            # Yalnızca görev hedefleri: mavi altıgen + kırmızı üçgen.
            if (color, shape) not in _VALID_TARGET_COMBINATIONS:
                self._log.debug(
                    "REJECT: görev dışı kombinasyon %s+%s", color.value, shape,
                )
                continue

            vertex_score = 1.0 - min(1.0, abs(count - expected_vertices) / 2.0)
            extent_score = 1.0 - min(1.0, abs(extent - expected_extent) / 0.25)
            shape_score = 0.7 * vertex_score + 0.3 * extent_score
            area_score = min(1.0, area / _AREA_REF_PX)
            confidence = 0.45 * shape_score + 0.20 * area_score + 0.35 * purity
            if confidence < MIN_DETECTION_CONFIDENCE:
                self._log.debug(
                    "REJECT: confidence too low %.2f shape=%s purity=%.2f area=%.0f",
                    confidence, shape, purity, area,
                )
                continue

            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue
            cx = moments["m10"] / moments["m00"]
            cy = moments["m01"] / moments["m00"]

            self._log.debug(
                "DETECT %s %s conf=%.2f purity=%.2f vertices=%d area=%.0f "
                "solidity/aspect ok extent=%.2f center=(%.0f,%.0f)",
                color.value, shape, confidence, purity, count, area,
                extent, cx, cy,
            )

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
                    color_purity=purity,
                )
            )
        return detections

    @staticmethod
    def _color_purity_from_contour(hsv, contour, purity_min: float):
        """Kontur içindeki canlı piksellerden baskın renk ve oranını döndürür.

        BGR maskesinden gelen konturlarda bile rengi körlemesine kabul etmek
        yerine, konturun İÇİNDEKİ piksellerin ne kadarı gerçekten mavi/kırmızı
        olduğuna bakar. Baskın renk oranı purity_min altındaysa None (gürültü).

        Returns: (TargetColor, purity_ratio) veya None.
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
        vivid = (sat >= 70) & (val >= 60)
        n = int(vivid.sum())
        if n == 0:
            return None

        blue = vivid & (hue >= 95) & (hue <= 130)
        red = vivid & ((hue <= 10) | (hue >= 170))
        blue_ratio = int(blue.sum()) / n
        red_ratio = int(red.sum()) / n

        # Baskın renk: eşiğin üstünde OLMALI ve diğer renge açıkça üstün olmalı
        # (≥1.3x). 50/50 veya yakın karışık blob'lar kabul edilmez.
        if blue_ratio >= red_ratio:
            if blue_ratio < purity_min or blue_ratio < red_ratio * 1.3:
                return None
            return TargetColor.BLUE, blue_ratio
        if red_ratio < purity_min or red_ratio < blue_ratio * 1.3:
            return None
        return TargetColor.RED, red_ratio

    # ──────────────────────────────────────────────────────────────
    # Ana detect() metodu
    # ──────────────────────────────────────────────────────────────

    def detect(self, frame_bgr):
        """Ana tespit girişi (API korunur).

        Akış:
          1. _maybe_auto_swap — yalnızca TARGET_DETECTOR_AUTO_SWAP=True iken
             kanal takası uygular (varsayılan kapalı — kamera katmanı sahibidir).
          2. LAB-CLAHE → HSV → mavi/kırmızı maskeler → konturlar →
             _detections_from_contours (renk purity + şekil doğrulaması).
          3. Renk tespiti yoksa gri fallback (adaptive threshold).
        """
        frame_bgr = self._maybe_auto_swap(frame_bgr)

        hsv = self._preprocess_for_hsv(frame_bgr)
        all_detections = []

        for color in (TargetColor.BLUE, TargetColor.RED):
            mask = self._color_mask(hsv, color)
            mask = self._clean_mask(mask)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            all_detections.extend(self._detections_from_contours(contours, hsv))

        if all_detections:
            return sorted(all_detections, key=lambda d: d.confidence, reverse=True)

        fallback_contours = self._fallback_contours(frame_bgr)
        fallback_dets = self._detections_from_contours(
            fallback_contours, hsv, purity_min=_FALLBACK_MIN_COLOR_PURITY
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
        # Çözünürlüğe bağlı ölçek — üst HUD ile aynı (640 bazlı): 1920px'de ~3x.
        # Sabit boyutlar küçültme sonrası okunmuyordu; artık HUD ile tutarlı.
        ui_scale = w / 640.0
        font_scale = 0.65 * ui_scale
        thickness  = max(1, int(2 * ui_scale))
        marker_size = int(28 * ui_scale)
        dot_r = int(10 * ui_scale)

        cv2.drawMarker(out, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS,
                       marker_size, thickness)

        for det in detections:
            color_bgr = (255, 80, 0) if det.color.value == "BLUE" else (0, 60, 255)
            tx = int(det.center_x)
            ty = int(det.center_y)

            cv2.circle(out, (tx, ty), dot_r, color_bgr, -1)
            cv2.line(out, (cx, cy), (tx, ty), color_bgr, max(1, int(ui_scale)))

            bw = int(det.bbox_width_px)
            bh = int(det.bbox_height_px)
            x1 = tx - bw // 2
            y1 = ty - bh // 2
            cv2.rectangle(out, (x1, y1), (x1 + bw, y1 + bh), color_bgr, thickness)

            # Etiket bbox üstünde; üst HUD bandının altına düşecek şekilde
            # sınırlanır (bbox kadraja yakınsa etiket bandın altında kalır).
            label = f"{det.color.value} {det.shape} {det.confidence:.2f}"
            label_y = max(y1 - int(10 * ui_scale), int(70 * ui_scale))
            cv2.putText(
                out, label, (x1, label_y),
                cv2.FONT_HERSHEY_SIMPLEX, font_scale, color_bgr, thickness, cv2.LINE_AA,
            )

        cv2.putText(
            out, f"Det: {len(detections)}",
            (int(14 * ui_scale), frame_bgr.shape[0] - int(14 * ui_scale)),
            cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 255, 0), thickness, cv2.LINE_AA,
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

        # Tip değişimi guard'ı: history'yi farklı tiple kirletme.
        # Salınım (A,B,A,B,...) hiçbir zaman stabilize olamaz; gerçek tip
        # değişimi ise yeni tipin MIN_STABLE_DETECTIONS ardışık frame'ini
        # gerektirir (kilitli modda zaten tip filtresi var — bu yol yalnızca
        # kilitsiz aramada çalışır).
        if self.history:
            last = self.history[-1]
            if (candidate.color, candidate.shape) != (last.color, last.shape):
                self.history.clear()

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
