"""
TargetDetector V2 — hızlı + hareket/titreşim toleranslı klasik CV detector.

PIPELINE (detection ~960×352'de çalışır — VISION_DETECT_WIDTH_PX):

    BGR frame (1920×704, kadraj kırpılmış)
      → downscale (max 960 px genişlik — INTER_AREA, 4x daha az piksel)
      → HSV (tek cvtColor; LAB-CLAHE YOK — Pi'de pahalı darboğazdı)
      → renk maskeleri (kırmızı: hue wrap-around iki bant; mavi: tek bant)
      → hafif CLOSE morphology (delik doldurma; ağır median/OPEN yok)
      → findContours (RETR_EXTERNAL)
      → her kontur: GEOMETRİK SKORLAMA
          • köşe sayısı çoğunluğu (3 epsilon approxPolyDP — toleranslı)
          • dairesellik (4πA/P²) üçgen≈0.60 / altıgen≈0.83
          • extent (alan/bbox) üçgen≈0.50 / altıgen≈0.75
          • solidity + aspect ratio
        → triangle_score / hexagon_score ∈ [0,1] (tek kritere bağımlı DEĞİL)
      → ROI-based RENK PURITY (kontur içi canlı piksel dağılımı:
        baskın renk + oran + 1.3x üstünlük — gürültüyü eler)
      → confidence = şekil + renk + boyut
      → TEMPORAL RECOVERY: kısa süreli detection kaybında (blur/kısmi
        örtünme) son 5 frame'de güçlü eşleşme varsa zayıf aday kabul edilir
      → koordinatlar ORİJİNAL frame çözünürlüğüne geri ölçeklenir

DIŞ INTERFACE (korunur — state machine / stream / record buna bağlı):
    detect(frame_bgr) → list[Detection]   (koordinatlar giriş frame'i ile AYNI)
    stable_detection / update_time_lock / reset_time_lock / draw_detections
    .history, .time_locked_color, .time_locked_shape

YALNIZCA görev hedefleri üretilir: (BLUE, HEXAGON) ve (RED, TRIANGLE).
"""
import logging
import math
import time
from collections import Counter, deque
from typing import Any, Optional

import cv2
import numpy as np

from config import (
    DETECTION_HISTORY_SIZE,
    MIN_DETECTION_CONFIDENCE,
    MIN_STABLE_DETECTIONS,
    MIN_TARGET_AREA_PX,
    TARGET_DETECTOR_AUTO_SWAP,
    TARGET_LOCK_DURATION_S,
    VISION_DETECT_WIDTH_PX,
)
from models import Detection, TargetColor

# ── Geometrik skorlama sabitleri ────────────────────────────────────────────
# Dairesellik merkezleri: düzgün üçgen ≈ 0.60, düzgün altıgen ≈ 0.83.
_TRI_CIRC_MU = 0.60
_HEX_CIRC_MU = 0.83
# Extent merkezleri: üçgen ≈ 0.50, altıgen ≈ 0.75.
_TRI_EXT_MU = 0.50
_HEX_EXT_MU = 0.75
# Gaussian yakınlık bant genişlikleri (sigma) — blur/küçük hedef toleransı.
_CIRC_SIGMA = 0.10
_EXT_SIGMA = 0.13
# Köşe sayısı → şekil skoru katkısı (küçük hedefte köşe sayısı güvenilmez;
# dairesellik+extent ağırlığı devreye girer).
_TRI_VERT_MAP = {3: 1.0, 4: 0.55, 5: 0.30, 6: 0.15}
_HEX_VERT_MAP = {6: 1.0, 5: 0.80, 7: 0.80, 4: 0.45, 8: 0.45, 3: 0.20}
# Şekil skoru ağırlıkları.
_W_VERT, _W_CIRC, _W_EXT, _W_SOL = 0.35, 0.30, 0.25, 0.10

# Şekil skoru kabul eşiği (altında aday kesin elenir — temporal'e gitmez).
_SHAPE_MIN = 0.45
# Confidence temporal recovery alt eşiği: [this, MIN_DETECTION_CONFIDENCE)
# aralığındaki zayıf adaylar geçmiş güçlü eşleşmeyle kabul edilebilir.
_CONF_RECOVER_MIN = 0.42
# Temporal bellek: son N frame'in güçlü detection'ları (kısa kayıp toleransı).
_TRACK_FRAMES = 5

# ── Renk eşikleri (HSV) ─────────────────────────────────────────────────────
# Orta-geniş bantlar: dış ışık değişimine dayanıklı; purity doğrulaması zaten
# iç bölge dağılımını kontrol eder (bant gevşekliği yanlış pozitif üretmez).
_BLUE_H_LO, _BLUE_H_HI = 90, 130
_RED_H_LO1, _RED_H_HI1 = 0, 12
_RED_H_LO2, _RED_H_HI2 = 168, 179
_SAT_MIN, _VAL_MIN = 60, 50
# Purity içinde "canlı" piksel eşikleri (kontur içi renk doğrulaması).
_VIVID_SAT, _VIVID_VAL = 60, 50

# Renk purity eşikleri — kontur içi baskın renk oranı ve diğerine üstünlük.
_MIN_COLOR_PURITY = 0.50
_DOMINANCE_RATIO = 1.3

# Görev hedefleri — yarışma kuralı: yalnızca MAVİ ALTIGEN ve KIRMIZI ÜÇGEN.
# Bu ikisi dışındaki renk+şekil kombinasyonları (örn. BLUE+TRIANGLE) detector
# tarafından ÜRETİLMEZ; aksi halde state machine sürekli "Geçersiz kombinasyon"
# deyip SEARCHING'e döner → sonsuz döngü.
_VALID_TARGET_COMBINATIONS = (
    (TargetColor.BLUE, "HEXAGON"),
    (TargetColor.RED, "TRIANGLE"),
)

# Boyut skoru referansı — 1920 genişlik ölçeğinde; çalışma çözünürlüğüne
# ölçek² ile çevrilir (örn. 960'ta 15000/4 = 3750 px²).
_AREA_REF_PX = 15000.0
# Kontur çevresi karelemesi öncesi asgari çevre (bölme koruması).
_MIN_PERIMETER = 5.0
# Kadrajın %75'inden büyük blob = hedef değil (kamera hedefe çok yakın /
# tamamen dolu kadraj — centroid sinyali güvenilmez).
_MAX_BLOB_FRACTION = 0.75


def _gauss(x: float, mu: float, sigma: float) -> float:
    """Gaussian yakınlık: x mu'ya ne kadar yakınsa 1'e o kadar yakın."""
    d = (x - mu) / sigma
    return math.exp(-d * d)


class TargetDetector:
    """Mavi altıgen + kırmızı üçgen detector (V2).

    Renk ve geometri İKİ BAĞIMSIZ KANIT olarak ele alınır:
      1. Renk segmentation (HSV mask) → aday konturlar
      2. Geometrik skorlama → üçgen/altıgen skoru (köşe + dairesellik +
         extent + solidity; tek kritere bağımlı değil)
      3. ROI-based renk purity → adayın gerçekten kırmızı/mavi olduğunun
         iç bölge doğrulaması
      4. Temporal recovery → kısa süreli detection kaybında geçmiş güçlü
         eşleşmeyle zayıf aday kabulü (motion blur / kısmi örtünme)
    """

    CALIBRATION_FRAMES = 12  # (AutoSwap bayrağı için — korunur)

    def __init__(self):
        self.history: deque[Detection] = deque(maxlen=DETECTION_HISTORY_SIZE)
        self._first_seen: dict[tuple[TargetColor, str], float] = {}
        self.time_locked_color: Optional[TargetColor] = None
        self.time_locked_shape: Optional[str] = None
        self._log = logging.getLogger("TargetDetector")

        # ⚠ Defansif otomatik B ↔ R kanal takası (yalnızca TARGET_DETECTOR_AUTO_SWAP=True ise)
        self._auto_swap: Optional[bool] = None
        self._calib_count: int = 0
        self._calib_sum_noswap: int = 0
        self._calib_sum_swap:   int = 0
        self._calib_homo_noswap: float = 0.0
        self._calib_homo_swap:   float = 0.0
        self._calib_max_noswap:  int = 0
        self._calib_max_swap:    int = 0

        # Temporal recovery belleği: son _TRACK_FRAMES frame'in güçlü
        # detection'ları (renk, şekil, merkez, bbox, confidence).
        self._track: deque[dict[str, Any]] = deque(maxlen=_TRACK_FRAMES)

    # ──────────────────────────────────────────────────────────────
    # DEFANSİF: B ↔ R kanalları takası (yalnızca TARGET_DETECTOR_AUTO_SWAP=True)
    # ──────────────────────────────────────────────────────────────

    def _maybe_auto_swap(self, frame_bgr: np.ndarray) -> np.ndarray:
        """AutoSwap V3 — yalnızca TARGET_DETECTOR_AUTO_SWAP=True iken aktif.

        Varsayılan KAPALI: kanal takasının sahibi kamera katmanıdır
        (RPiCam3 — IMX708 için sabit B↔R takası). Bu metrik üçlüsü yalnızca
        bayrak açıkken güvenlik ağı olarak çalışır.
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

        self._calib_sum_noswap += (b_blue + b_red)
        self._calib_sum_swap   += (s_blue + s_red)
        self._calib_max_noswap  += max(b_blue, b_red)
        self._calib_max_swap    += max(s_blue, s_red)
        self._calib_homo_noswap += (max(b_blue, b_red) + eps) / (min(b_blue, b_red) + eps)
        self._calib_homo_swap   += (max(s_blue, s_red) + eps) / (min(s_blue, s_red) + eps)
        self._calib_count += 1

        if self._calib_count < self.CALIBRATION_FRAMES:
            return frame_bgr

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

        self._calib_sum_noswap = 0
        self._calib_sum_swap = 0
        self._calib_homo_noswap = 0.0
        self._calib_homo_swap = 0.0
        self._calib_max_noswap = 0
        self._calib_max_swap = 0
        self._calib_count = 0

        return frame_bgr

    # ──────────────────────────────────────────────────────────────
    # Renk segmentation (hızlı — CLAHE yok)
    # ──────────────────────────────────────────────────────────────

    @staticmethod
    def _color_masks(hsv: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Kırmızı (hue wrap-around iki bant) + mavi maskelerini üretir."""
        low = np.array([_BLUE_H_LO, _SAT_MIN, _VAL_MIN], dtype=np.uint8)
        high = np.array([_BLUE_H_HI, 255, 255], dtype=np.uint8)
        mask_blue = cv2.inRange(hsv, low, high)

        mask_red_lo = cv2.inRange(
            hsv,
            np.array([_RED_H_LO1, _SAT_MIN, _VAL_MIN], dtype=np.uint8),
            np.array([_RED_H_HI1, 255, 255], dtype=np.uint8),
        )
        mask_red_hi = cv2.inRange(
            hsv,
            np.array([_RED_H_LO2, _SAT_MIN, _VAL_MIN], dtype=np.uint8),
            np.array([_RED_H_HI2, 255, 255], dtype=np.uint8),
        )
        return mask_blue, cv2.bitwise_or(mask_red_lo, mask_red_hi)

    @staticmethod
    def _clean_mask(mask: np.ndarray) -> np.ndarray:
        """Hafif CLOSE: delikleri doldur; ağır median/OPEN Pi'de pahalıydı.

        Küçük gürültü parçacıkları alan eşiği (MIN_TARGET_AREA_PX ölçekli)
        tarafından elenir — morphology'ye gerek yok.
        """
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    # ──────────────────────────────────────────────────────────────
    # Geometrik skorlama
    # ──────────────────────────────────────────────────────────────

    @classmethod
    def _shape_scores(
        cls, contour: np.ndarray, area: float
    ) -> Optional[tuple[float, float]]:
        """Kontur için (triangle_score, hexagon_score) üretir; None = elenmeli.

        Skorlar birden çok bağımsız geometrik özelliğin birleşimidir:
          • köşe sayısı çoğunluğu (3 epsilon ile approxPolyDP — blur'da bile
            düzenli şekiller tutarlı köşe sayısı verir)
          • dairesellik (4πA/P²) — üçgen ≈ 0.60, altıgen ≈ 0.83
          • extent (alan/bbox_alanı) — üçgen ≈ 0.50, altıgen ≈ 0.75
          • solidity (alan/hull_alanı) — düzgün şekillerde yüksek
        Hiçbir TEK kriter kararı belirlemez — tırtıklı/uzun/karmaşık konturlar
        bütün skorlarda birden düşer ve elenir.
        """
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        if hull_area <= 0 or area <= 0:
            return None
        solidity = float(area) / hull_area
        if solidity < 0.55:
            return None  # güçlü iç bükeylik: şekil değil, leke

        perimeter = cv2.arcLength(contour, True)
        if perimeter < _MIN_PERIMETER:
            return None
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        # Sert üst sınır: düzgün altıgen circ ≈ 0.907 (blur'da DÜŞER), daire/elips
        # 0.95+. Circ 0.94'ü aşan kontur hiçbir hedef şekle uymaz.
        if circularity > 0.94:
            return None

        x, y, w, h = cv2.boundingRect(contour)
        if w <= 2 or h <= 2 or w * h == 0:
            return None
        extent = float(area) / float(w * h)
        aspect = float(w) / h if h >= w else float(h) / w
        if aspect > 3.0:
            return None  # aşırı uzun/dar blob şekil olamaz

        # Şekle özgü SERT extent sınırları: düzgün altıgen bbox'un en fazla
        # ~0.83'ünü doldurur (blur'da ~0.88), üçgen ~0.50 (blur'da ~0.60).
        # Kare/dikdörtgen (extent → 1.0) böylece şekil olarak ELENİR —
        # skorlama ne kadar cömert olursa olsun yanlış pozitif üretilmez.
        if extent > 0.92:
            return None  # ne üçgen ne altıgen olabilir
        tri_allowed = extent <= 0.85
        hex_allowed = extent <= 0.92  # üst sınır zaten kontrol edildi

        # Köşe sayısı çoğunluğu (3 epsilon — uygun maliyetli).
        votes = []
        for eps in (0.02, 0.04, 0.06):
            approx = cv2.approxPolyDP(contour, eps * perimeter, True)
            votes.append(len(approx))
        count, _agreement = Counter(votes).most_common(1)[0]

        # Sert köşe sınırı: üçgen/altıgen blur'da bile 3-7 köşe üretir
        # (ölçüm: 20-100px altıgenlerde 5-7). 8+ köşe = eğri kontur
        # (daire/elips) — hiçbir hedef şekle uymaz.
        if count >= 8:
            return None

        sol_s = max(0.0, min(1.0, (solidity - 0.70) / 0.30))

        tri_vert = _TRI_VERT_MAP.get(count, 0.05) if tri_allowed else 0.0
        tri_circ = _gauss(circularity, _TRI_CIRC_MU, _CIRC_SIGMA) if tri_allowed else 0.0
        tri_ext = _gauss(extent, _TRI_EXT_MU, _EXT_SIGMA) if tri_allowed else 0.0
        tri_score = (
            _W_VERT * tri_vert + _W_CIRC * tri_circ
            + _W_EXT * tri_ext + _W_SOL * sol_s
        )

        hex_vert = _HEX_VERT_MAP.get(count, 0.05) if hex_allowed else 0.0
        hex_circ = _gauss(circularity, _HEX_CIRC_MU, _CIRC_SIGMA) if hex_allowed else 0.0
        hex_ext = _gauss(extent, _HEX_EXT_MU, _EXT_SIGMA) if hex_allowed else 0.0
        hex_score = (
            _W_VERT * hex_vert + _W_CIRC * hex_circ
            + _W_EXT * hex_ext + _W_SOL * sol_s
        )

        return tri_score, hex_score

    # ──────────────────────────────────────────────────────────────
    # ROI-based renk purity (kontur içi doğrulama)
    # ──────────────────────────────────────────────────────────────

    @staticmethod
    def _color_purity(hsv: np.ndarray, contour: np.ndarray) -> Optional[tuple[TargetColor, float]]:
        """Kontur İÇİNDEKİ canlı piksellerden baskın renk ve oranını döndürür.

        Maskeden çıktı diye rengi körlemesine kabul etmez — konturun içindeki
        piksellerin ne kadarı gerçekten mavi/kırmızı olduğuna bakar. Baskın
        renk oranı _MIN_COLOR_PURITY altındaysa veya diğer renge 1.3x üstünlük
        yoksa None (gürültü / karışık blob).

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
        vivid = (sat >= _VIVID_SAT) & (val >= _VIVID_VAL)
        n = int(vivid.sum())
        if n == 0:
            return None

        blue = vivid & (hue >= _BLUE_H_LO) & (hue <= _BLUE_H_HI)
        red = vivid & ((hue <= _RED_H_HI1) | (hue >= _RED_H_LO2))
        blue_ratio = int(blue.sum()) / n
        red_ratio = int(red.sum()) / n

        if blue_ratio >= red_ratio:
            if blue_ratio < _MIN_COLOR_PURITY or blue_ratio < red_ratio * _DOMINANCE_RATIO:
                return None
            return TargetColor.BLUE, blue_ratio
        if red_ratio < _MIN_COLOR_PURITY or red_ratio < blue_ratio * _DOMINANCE_RATIO:
            return None
        return TargetColor.RED, red_ratio

    # ──────────────────────────────────────────────────────────────
    # Ana detect() — V2 pipeline
    # ──────────────────────────────────────────────────────────────

    def detect(self, frame_bgr):
        """Ana tespit girişi (API korunur — koordinatlar giriş frame'i ile AYNI).

        Akış:
          1. _maybe_auto_swap (yalnızca TARGET_DETECTOR_AUTO_SWAP=True iken)
          2. VISION_DETECT_WIDTH_PX'e downscale (INTER_AREA)
          3. HSV → kırmızı/mavi maskeler → hafif CLOSE → konturlar
          4. Geometrik skorlama + renk purity + confidence
          5. Temporal recovery (kısa süreli kayıpta geçmiş güçlü eşleşme)
          6. Çıktı koordinatları ORİJİNAL çözünürlüğe geri ölçeklenir
        """
        frame_bgr = self._maybe_auto_swap(frame_bgr)

        work, scale = self._downscale(frame_bgr)
        hsv = cv2.cvtColor(work, cv2.COLOR_BGR2HSV)

        strong: list[Detection] = []
        weak: list[Detection] = []

        # Bileşik mask (mavi | kırmızı): karışık renkli blob'lar tek kontur
        # olur → purity içinde hiçbir renk 1.3x baskın olamaz → elenir.
        # Ayrık hedefler kendi saf konturlarını üretir (tek findContours — hızlı).
        mask_blue, mask_red = self._color_masks(hsv)
        combined = cv2.bitwise_or(mask_blue, mask_red)
        combined = self._clean_mask(combined)
        contours, _ = cv2.findContours(
            combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        dets = self._analyze_contours(contours, hsv, scale)
        for d, confidence in dets:
            if confidence >= MIN_DETECTION_CONFIDENCE:
                strong.append(d)
            elif confidence >= _CONF_RECOVER_MIN:
                weak.append(d)

        # Temporal recovery: zayıf aday, son _TRACK_FRAMES frame'de güçlü
        # eşleşmeyle (aynı renk+şekil, bbox ölçeğinde yakın merkez) kabul edilir.
        recovered = self._recover_weak(weak)

        # Temporal bellek: yalnızca güçlü detection'lar track'i besler.
        self._update_track(strong)

        all_dets = strong + recovered
        if not all_dets:
            return []
        return sorted(all_dets, key=lambda d: d.confidence, reverse=True)

    @staticmethod
    def _downscale(frame: np.ndarray) -> tuple[np.ndarray, float]:
        """Frame'i max VISION_DETECT_WIDTH_PX genişliğe ölçekler.

        Returns: (çalışma frame'i, ölçek) — ölçek ≤ 1; çıktı koordinatları
        geri ölçekleme için 1/scale ile çarpılır.
        """
        h, w = frame.shape[:2]
        if w <= VISION_DETECT_WIDTH_PX:
            return frame, 1.0
        scale = VISION_DETECT_WIDTH_PX / w
        nw = VISION_DETECT_WIDTH_PX
        nh = max(1, int(round(h * scale)))
        return cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_AREA), scale

    def _analyze_contours(
        self, contours, hsv: np.ndarray, scale: float
    ) -> list[tuple[Detection, float]]:
        """Kontur listesinden (Detection, confidence) adayları üretir.

        Çıktı koordinatları ORİJİNAL çözünürlükte (1/scale ile geri ölçeklenir);
        area_px da orijinal ölçeğe çevrilir (bölge ölçek² ile küçülür).
        """
        h_work, w_work = hsv.shape[:2]
        max_area = _MAX_BLOB_FRACTION * w_work * h_work
        min_area = MIN_TARGET_AREA_PX * scale * scale

        candidates: list[tuple[Detection, float]] = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area or area > max_area:
                continue

            scores = self._shape_scores(contour, area)
            if scores is None:
                continue
            tri_score, hex_score = scores
            shape_score = max(tri_score, hex_score)
            if shape_score < _SHAPE_MIN:
                continue
            shape = "TRIANGLE" if tri_score >= hex_score else "HEXAGON"

            # Renk: kontur içi gerçek piksel dağılımı (ROI-based purity).
            color_info = self._color_purity(hsv, contour)
            if color_info is None:
                continue
            color, purity = color_info

            # Yalnızca görev hedefleri: mavi altıgen + kırmızı üçgen.
            if (color, shape) not in _VALID_TARGET_COMBINATIONS:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue
            cx = moments["m10"] / moments["m00"]
            cy = moments["m01"] / moments["m00"]

            area_ref = _AREA_REF_PX * scale * scale
            size_score = min(1.0, area / area_ref)
            confidence = 0.45 * shape_score + 0.35 * purity + 0.20 * size_score

            det = Detection(
                color=color,
                shape=shape,
                center_x=cx / scale,
                center_y=cy / scale,
                area_px=area / (scale * scale),
                bbox_width_px=w / scale,
                bbox_height_px=h / scale,
                confidence=confidence,
                color_purity=purity,
            )
            candidates.append((det, confidence))

            self._log.debug(
                "ADAY %s %s shape=%.2f purity=%.2f conf=%.2f area=%.0f "
                "center=(%.0f,%.0f)",
                color.value, shape, shape_score, purity, confidence, area, cx, cy,
            )
        return candidates

    # ──────────────────────────────────────────────────────────────
    # Temporal recovery (kısa süreli detection kaybı toleransı)
    # ──────────────────────────────────────────────────────────────

    def _recover_weak(self, weak: list[Detection]) -> list[Detection]:
        """Zayıf adayları, track'teki güçlü geçmişle eşleşirse kabul eder.

        Motion blur / kısmi örtünme / düşük ışıkta bir hedef birkaç frame
        zayıf görünebilir. Aynı (renk, şekil) ve bbox ölçeğinde yakın merkezli
        güçlü bir geçmiş varsa aday güvenilir sayılır (confidence geçmişle
        birleştirilir). Track'te eşleşme YOKSA zayıf aday ATILIR — yeni/rastgele
        blob'lar temporal sayesinde kabul edilmez (false positive kontrolü).
        """
        if not weak or not self._track:
            return []

        recovered: list[Detection] = []
        for cand in weak:
            best = None
            best_conf = 0.0
            for tr in self._track:
                if tr["color"] != cand.color or tr["shape"] != cand.shape:
                    continue
                tw = float(tr["w"])
                th = float(tr["h"])
                tcx = float(tr["cx"])
                tcy = float(tr["cy"])
                tconf = float(tr["confidence"])
                if abs(tcx - cand.center_x) > max(tw, cand.bbox_width_px) * 0.7 + 15:
                    continue
                if abs(tcy - cand.center_y) > max(th, cand.bbox_height_px) * 0.7 + 15:
                    continue
                if tconf > best_conf:
                    best_conf = tconf
                    best = tr
            if best is not None:
                # Geçmiş güvenilirlikle birleştir: en az MIN_DETECTION_CONFIDENCE'e
                # çıkar (track'teki güçlü tespit bu hedefin gerçek olduğunu söyler).
                cand.confidence = max(cand.confidence, 0.85 * best_conf)
                recovered.append(cand)
                self._log.debug(
                    "TEMPORAL RECOVERY: %s %s conf=%.2f (geçmiş %.2f)",
                    cand.color.value, cand.shape, cand.confidence, best_conf,
                )
        return recovered

    def _update_track(self, strong: list[Detection]) -> None:
        """Güçlü detection'ları temporal belleğe ekler (max _TRACK_FRAMES)."""
        for d in strong:
            self._track.append(
                {
                    "color": d.color,
                    "shape": d.shape,
                    "cx": d.center_x,
                    "cy": d.center_y,
                    "w": d.bbox_width_px,
                    "h": d.bbox_height_px,
                    "confidence": d.confidence,
                }
            )

    # ──────────────────────────────────────────────────────────────
    # Zaman kilidi, history, draw — davranış aynı kalıyor
    # ──────────────────────────────────────────────────────────────

    def update_time_lock(self, detections: list[Detection]) -> None:
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

        mature = [
            (color, shape)
            for (color, shape), first_t in self._first_seen.items()
            if now - first_t >= TARGET_LOCK_DURATION_S
        ]
        if not mature:
            return

        if len(mature) > 1:
            # Her iki tür de aynı anda görünürlük eşiğini aştıysa EN SAĞLIKLI
            # (en yüksek güven) olan birinci hedef olarak kilitlenir; diğeri
            # ikinci hedef kalır. (Tek tür varsa ilk olgunlaşan kilitlenir.)
            best = max(
                (d for d in detections if (d.color, d.shape) in mature),
                key=lambda d: d.confidence,
                default=None,
            )
            if best is not None:
                self.time_locked_color = best.color
                self.time_locked_shape = best.shape
                return

        color, shape = mature[0]
        self.time_locked_color = color
        self.time_locked_shape = shape

    def reset_time_lock(self) -> None:
        self.time_locked_color = None
        self.time_locked_shape = None
        self._first_seen.clear()
        # Yeni hedef aramasına dönerken eski hedefin temporal belleği de
        # temizlenir — geçmiş blob recovery ile yanlış pozitif üretmesin.
        self._track.clear()

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
