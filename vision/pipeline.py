"""
Ana görüntü işleme pipeline'ı.

Sıra (mantıksal bağımlılıklara göre):

    frame_gate (gyro — opsiyonel)
        ↓
    downscale (VISION_DETECT_WIDTH_PX'e)
        ↓
    HSV (tek dönüşüm — tüm hedefler paylaşır)
        ↓
    renk maskeleri (her hedef ayrı config; bileşik maske OR)
        ↓
    kontur çıkarımı (tek findContours — verimli)
        ↓
    her kontur: renk purity → şekil skoru → boyut filtresi
        ↓
    geo_projection (pose verildiyse → dünya koordinatı)
        ↓
    temporal_validator (N frame tutarlılık)
        ↓
    Detection listesi (koordinatlar ORİJİNAL çözünürlüğe ölçeklenir)

- Her hedef tipi (red_triangle / blue_hexagon) ayrı config bloğuna sahiptir
  ve bağımsız değerlendirilir.
- İrtifa/pose verilmediyse sistem ÇALIŞMAYA DEVAM eder: boyut filtresi
  mutlak eşiklere düşer (eski davranış), temporal piksel-uzay fallback
  kullanır, dünya koordinatı üretilmez.
- Tarama rotası / lawnmower planner BU MODÜLÜN SORUMLULUĞU DEĞİLDİR.
"""
from __future__ import annotations

import time
from typing import Any, Optional

import cv2
import numpy as np

from models import Detection, TargetColor
from vision.color_mask import ColorMask
from vision.frame_gate import FrameGate
from vision.geo_projection import GeoProjection, VehiclePose
from vision.shape_detector import ShapeCandidate, ShapeDetector
from vision.size_filter import SizeFilter
from vision.temporal_validator import TemporalEntry, TemporalValidator

# Çalışma çözünürlüğü için hedef renk → TargetColor eşlemesi.
_COLOR_MAP = {
    "RED": TargetColor.RED,
    "BLUE": TargetColor.BLUE,
}


class VisionPipeline:
    """Config-bundle ile kurulan modüler pipeline."""

    def __init__(
        self,
        targets_cfg: dict[str, Any],
        morphology_cfg: dict[str, Any],
        scoring_cfg: dict[str, Any],
        vertex_epsilons: list[float],
        size_filter_cfg: dict[str, Any],
        geometry_cfg: dict[str, Any],
        frame_gate_cfg: dict[str, Any],
        geo_cfg: dict[str, Any],
        temporal_cfg: dict[str, Any],
        purity_cfg: dict[str, Any],
        confidence_weights: dict[str, Any],
        work_width_px: int,
        min_detection_confidence: float,
    ) -> None:
        self._work_width = int(work_width_px)
        self._min_conf = float(min_detection_confidence)
        self._purity_min = float(purity_cfg["min_ratio"])
        self._dominance = float(purity_cfg["dominance_ratio"])
        self._vivid_s = int(purity_cfg["vivid_s_min"])
        self._vivid_v = int(purity_cfg["vivid_v_min"])
        self._w_shape = float(confidence_weights["shape"])
        self._w_color = float(confidence_weights["color"])
        self._w_size = float(confidence_weights["size"])
        self._w_temporal = float(confidence_weights["temporal"])

        self._frame_gate = FrameGate(frame_gate_cfg)
        self._geo = GeoProjection(geo_cfg, geometry_cfg)
        self._temporal = TemporalValidator(temporal_cfg)
        self._size_filter = SizeFilter(geometry_cfg, size_filter_cfg)
        self._geometry = geometry_cfg

        # Hedefler: (color_key, shape_name, config, masker, shape_detector)
        self._targets: list[tuple[str, str, dict[str, Any], ColorMask, ShapeDetector]] = []
        for tcfg in targets_cfg.values():
            color_key = str(tcfg["color"])
            shape_name = str(tcfg["shape"])
            masker = ColorMask(tcfg["hsv_ranges"], morphology_cfg)
            shape_det = ShapeDetector(
                vertex_epsilons, scoring_cfg, tcfg, shape_name=shape_name,
            )
            self._targets.append((color_key, shape_name, tcfg, masker, shape_det))

        # İşleme süresi ölçümü (FPS takibi için).
        self.last_frame_ms: float = 0.0

    # ── Public API ───────────────────────────────────────────────────────── #
    def process(
        self,
        frame_bgr: np.ndarray,
        altitude_m: Optional[float] = None,
        pose: Optional[VehiclePose] = None,
        gyro_deg_s: Optional[float] = None,
    ) -> list[Detection]:
        """Tek frame işleme → Detection listesi (orijinal çözünürlükte)."""
        t0 = time.perf_counter()
        if not self._frame_gate.accept(gyro_deg_s):
            self.last_frame_ms = (time.perf_counter() - t0) * 1000.0
            return []

        orig_w = frame_bgr.shape[1]
        work, scale = self._downscale(frame_bgr)
        hsv = cv2.cvtColor(work, cv2.COLOR_BGR2HSV)

        # Bileşik maske: tüm hedef maskeleri OR → tek findContours (verimli).
        combined = None
        maskers = []
        for color_key, _shape, _tcfg, masker, _det in self._targets:
            mask = masker.apply(hsv)
            maskers.append((color_key, mask))
            combined = mask if combined is None else cv2.bitwise_or(combined, mask)
        if combined is None:
            combined = np.zeros(work.shape[:2], dtype=np.uint8)

        contours, _ = cv2.findContours(
            combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        now = time.time()
        entries: list[tuple[Detection, TemporalEntry]] = []

        for contour in contours:
            cand = self._classify_contour(contour, hsv, scale, now, altitude_m, pose)
            if cand is not None:
                entries.append(cand)

        # Temporal doğrulama: etkinse doğrulanan adaylara temporal katkı ekle,
        # doğrulanmayanları ele. Etkin değilse tüm adaylar geçer (state machine
        # stable_detection zaten temporal tutarlılık sağlar).
        if self._temporal.enabled:
            validated_entries = self._temporal.validate([e for _, e in entries])
            validated_ids = {id(e) for e in validated_entries}
            results: list[Detection] = []
            for detection, entry in entries:
                if id(entry) not in validated_ids:
                    continue
                detection.temporal_score = 1.0
                detection.confidence = min(1.0, detection.confidence + self._w_temporal)
                if detection.confidence >= self._min_conf:
                    results.append(detection)
        else:
            results = [det for det, _ in entries]
        self.last_frame_ms = (time.perf_counter() - t0) * 1000.0
        return sorted(results, key=lambda d: d.confidence, reverse=True)

    def reset_temporal(self) -> None:
        """Yeni hedef aramasında temporal belleği temizle."""
        self._temporal.reset()

    @property
    def gsd_ref_m_per_px(self) -> float:
        return float(self._geometry["gsd_ref_m_per_px"])

    # ── Yardımcılar ──────────────────────────────────────────────────────── #
    def _downscale(self, frame: np.ndarray) -> tuple[np.ndarray, float]:
        """Max VISION_DETECT_WIDTH_PX genişliğe ölçekler. Ölçek ≤ 1."""
        h, w = frame.shape[:2]
        if w <= self._work_width:
            return frame, 1.0
        scale = self._work_width / w
        nw = self._work_width
        nh = max(1, int(round(h * scale)))
        return cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_AREA), scale

    def _classify_contour(
        self,
        contour: np.ndarray,
        hsv: np.ndarray,
        scale: float,
        now: float,
        altitude_m: Optional[float],
        pose: Optional[VehiclePose],
    ) -> Optional[tuple[Detection, TemporalEntry]]:
        """Tek konturu hedef adayına dönüştürür (None = elendi)."""
        # 1) Renk purity: kontur içindeki baskın canlı renk (karışık blob ele).
        purity = self._color_purity(hsv, contour)
        if purity is None:
            return None
        color_key, color_score = purity

        # 2) O renge ait hedefle değerlendir (şekil + boyut).
        for t_color, t_shape, tcfg, _masker, shape_det in self._targets:
            if t_color != color_key:
                continue
            cand = self._shape_candidate(shape_det, contour)
            if cand is None or cand.shape != t_shape:
                continue

            area_tol = (float(tcfg["area_tolerance"][0]), float(tcfg["area_tolerance"][1]))
            frame_area = hsv.shape[0] * hsv.shape[1]
            if not self._size_filter.accept(
                cand.area_px, cand.shape, float(tcfg["real_edge_m"]),
                altitude_m, frame_area, area_tol,
            ):
                continue

            # 3) Confidence: şekil + renk + boyut (temporal katkı doğrulama
            #    sonrası eklenir — pipeline.process içinde).
            size_score = self._size_score(cand.area_px, tcfg, altitude_m)
            confidence = (
                self._w_shape * cand.shape_score
                + self._w_color * color_score
                + self._w_size * size_score
            )
            if confidence < self._min_conf:
                continue

            # 4) Dünya koordinatı (pose verildiyse).
            world_lat = world_lon = None
            if pose is not None and altitude_m is not None:
                res = self._geo.project(
                    cand.centroid_x, cand.centroid_y,
                    hsv.shape[1], hsv.shape[0], altitude_m, pose,
                )
                if res.valid:
                    world_lat, world_lon = res.latitude, res.longitude

            det = Detection(
                color=_COLOR_MAP[color_key],
                shape=t_shape,
                center_x=cand.centroid_x / scale,
                center_y=cand.centroid_y / scale,
                area_px=cand.area_px / (scale * scale),
                bbox_width_px=cand.bbox_w / scale,
                bbox_height_px=cand.bbox_h / scale,
                confidence=confidence,
                color_purity=color_score,
                world_lat=world_lat,
                world_lon=world_lon,
            )
            entry = TemporalEntry(
                color=color_key, shape=t_shape,
                cx=cand.centroid_x, cy=cand.centroid_y,
                frame_t=now, world_lat=world_lat, world_lon=world_lon,
            )
            return det, entry
        return None

    def _shape_candidate(
        self, shape_det: ShapeDetector, contour: np.ndarray
    ) -> Optional[ShapeCandidate]:
        return shape_det.classify_contour(contour)

    def _color_purity(
        self, hsv: np.ndarray, contour: np.ndarray
    ) -> Optional[tuple[str, float]]:
        """Kontur içi canlı piksel dağılımından baskın renk + oranı.

        Karışık blob (yarım kırmızı yarım mavi) hiçbir renk 1.3x üstünlük
        sağlayamadığı için elenir. Düşük saturation (asfalt/beton) vivid
        sayılmaz. Eşikler config'ten.
        """
        x, y, w, h = cv2.boundingRect(contour)
        if w <= 2 or h <= 2:
            return None
        contour_mask = np.zeros((h, w), dtype=np.uint8)
        shifted = contour - np.array([x, y])
        cv2.drawContours(contour_mask, [shifted], -1, 255, -1)
        interior = hsv[y:y + h, x:x + w][contour_mask == 255]
        if interior.size == 0:
            return None

        hue = interior[:, 0]
        sat = interior[:, 1]
        val = interior[:, 2]
        vivid = (sat >= self._vivid_s) & (val >= self._vivid_v)
        n = int(vivid.sum())
        if n == 0:
            return None

        red = vivid & ((hue <= 10) | (hue >= 160))
        blue = vivid & (hue >= 90) & (hue <= 130)
        red_ratio = int(red.sum()) / n
        blue_ratio = int(blue.sum()) / n

        if red_ratio >= blue_ratio:
            if red_ratio < self._purity_min or red_ratio < blue_ratio * self._dominance:
                return None
            return "RED", red_ratio
        if blue_ratio < self._purity_min or blue_ratio < red_ratio * self._dominance:
            return None
        return "BLUE", blue_ratio

    def _size_score(
        self, area_px: float, tcfg: dict[str, Any], altitude_m: Optional[float]
    ) -> float:
        """Boyut kanıtı: beklenen alana yakınlık (altitude varsa) veya büyüklük."""
        if altitude_m is None:
            return min(1.0, area_px / 4000.0)  # 960 ölçeğinde doygunluk
        gsd = self._size_filter.gsd_at(altitude_m)
        expected = SizeFilter.expected_area_px(
            str(tcfg["shape"]), float(tcfg["real_edge_m"]), gsd
        )
        if expected <= 0:
            return 0.0
        ratio = area_px / expected
        lo, hi = (float(v) for v in tcfg["area_tolerance"])
        if lo <= ratio <= hi:
            return 1.0
        # Tolerans dışı ama kabul edildiyse (mutlak mod): düşük skor.
        return max(0.0, 1.0 - abs(ratio - (lo + hi) / 2.0))
