"""
TargetDetector — mevcut state machine interface'ini koruyan FACADE.

Mevcut sistemle (state machine, stream, record) BİREBİR UYUMLUDUR:

    detector.detect(frame)              → list[Detection]  (eski API)
    detector.stable_detection(...)      → history tabanlı kararlı tespit
    detector.update_time_lock(...)      → zaman bazlı hedef kilidi
    detector.reset_time_lock()          → kilit + temporal sıfırlama
    TargetDetector.draw_detections(...) → overlay çizimi
    .history / .time_locked_color / .time_locked_shape

İçeride yeni modüler pipeline (vision/pipeline.py) çalışır:
    frame_gate → color_mask → shape_detector → size_filter →
    geo_projection → temporal_validator

detect() TÜM ek parametreleri OPSİYONELDİR (state machine `detect(frame)`
çağırır → sistem eskisi gibi çalışır). İrtifa/pose/gyro verilirse:
  - boyut filtresi GSD tabanlı fiziksel alan eşleşmesi yapar,
  - dünya koordinatı üretilir,
  - gyro frame kapısı devreye girer.
"""
from __future__ import annotations

import logging
import time
from collections import deque
from typing import Optional

import cv2
import numpy as np

from config import (
    DETECTION_HISTORY_SIZE,
    MIN_DETECTION_CONFIDENCE,
    MIN_STABLE_DETECTIONS,
    TARGET_DETECTOR_AUTO_SWAP,
    TARGET_LOCK_DURATION_S,
    VISION_CAMERA_GEOMETRY,
    VISION_COLOR_PURITY,
    VISION_CONFIDENCE_WEIGHTS,
    VISION_DECOY_REJECTION,
    VISION_DETECT_WIDTH_PX,
    VISION_FRAME_GATE,
    VISION_GEO_PROJECTION,
    VISION_MIN_PROCESSING_FPS,
    VISION_MORPHOLOGY,
    VISION_SHAPE_SCORING,
    VISION_SIZE_FILTER,
    VISION_SQUARE,
    VISION_TEMPORAL,
    VISION_TARGETS,
    VISION_VERTEX_EPSILONS,
)
from models import Detection, TargetColor
from vision.geo_projection import VehiclePose
from vision.pipeline import VisionPipeline


class TargetDetector:
    """State machine uyumlu hedef detektörü (modüler pipeline üzerinde)."""

    CALIBRATION_FRAMES = 12  # (AutoSwap bayrağı için — korunur)

    def __init__(self) -> None:
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

        # Modüler pipeline (tüm eşikler config'ten).
        self._pipeline = VisionPipeline(
            targets_cfg=VISION_TARGETS,
            morphology_cfg=VISION_MORPHOLOGY,
            scoring_cfg=VISION_SHAPE_SCORING,
            vertex_epsilons=VISION_VERTEX_EPSILONS,
            size_filter_cfg=VISION_SIZE_FILTER,
            geometry_cfg=VISION_CAMERA_GEOMETRY,
            frame_gate_cfg=VISION_FRAME_GATE,
            geo_cfg=VISION_GEO_PROJECTION,
            temporal_cfg=VISION_TEMPORAL,
            purity_cfg=VISION_COLOR_PURITY,
            confidence_weights=VISION_CONFIDENCE_WEIGHTS,
            square_cfg=VISION_SQUARE,
            decoy_cfg=VISION_DECOY_REJECTION,
            work_width_px=VISION_DETECT_WIDTH_PX,
            min_detection_confidence=MIN_DETECTION_CONFIDENCE,
        )

        # Araç durumu — detect()'e verilmezse kullanılır (None = bilinmiyor).
        self._altitude_m: Optional[float] = None
        self._yaw_deg: Optional[float] = None
        self._pitch_deg: Optional[float] = None
        self._roll_deg: Optional[float] = None
        self._gyro_deg_s: Optional[float] = None
        self._latitude: Optional[float] = None
        self._longitude: Optional[float] = None

        # İşleme FPS takibi (min eşik uyarısı).
        self._fps_ema: float = 0.0
        self._last_t: float = time.monotonic()
        self._last_fps_warn_t: float = 0.0

    # ──────────────────────────────────────────────────────────────
    # Araç durumu (opsiyonel — state machine ileride çağırabilir)
    # ──────────────────────────────────────────────────────────────

    def set_vehicle_state(
        self,
        altitude_m: Optional[float] = None,
        yaw_deg: Optional[float] = None,
        pitch_deg: Optional[float] = None,
        roll_deg: Optional[float] = None,
        gyro_deg_s: Optional[float] = None,
        latitude: Optional[float] = None,
        longitude: Optional[float] = None,
    ) -> None:
        """Flight controller verilerini detektöre iletir (hepsi opsiyonel)."""
        if altitude_m is not None:
            self._altitude_m = float(altitude_m)
        if yaw_deg is not None:
            self._yaw_deg = float(yaw_deg)
        if pitch_deg is not None:
            self._pitch_deg = float(pitch_deg)
        if roll_deg is not None:
            self._roll_deg = float(roll_deg)
        if gyro_deg_s is not None:
            self._gyro_deg_s = float(gyro_deg_s)
        if latitude is not None:
            self._latitude = float(latitude)
        if longitude is not None:
            self._longitude = float(longitude)

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
                "[AutoSwap-V3] ⚠ BAYT SIRASI TERS — KALICI B↔R takası AÇILDI "
                "(SUM=%.1fx HOMO=%.1fx MAXBANT=%.1fx)",
                r_sum, r_homo, r_max,
            )
        else:
            self._log.info(
                "[AutoSwap-V3] ✓ Renk kanalları DÜZGÜN — takas YOK.",
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
    # Ana API — detect (eski imza + opsiyonel vehicle verileri)
    # ──────────────────────────────────────────────────────────────

    def detect(
        self,
        frame_bgr,
        altitude_m: Optional[float] = None,
        yaw_deg: Optional[float] = None,
        pitch_deg: Optional[float] = None,
        roll_deg: Optional[float] = None,
        gyro_deg_s: Optional[float] = None,
        latitude: Optional[float] = None,
        longitude: Optional[float] = None,
    ) -> list:
        """Ana tespit girişi — state machine uyumlu (detect(frame) çalışır).

        Tüm ek parametreler opsiyoneldir; verilenler set_vehicle_state
        değerlerini override eder. Çıktı koordinatları GİRİŞ frame'i ile
        aynı çözünürlüktedir (align/meters_per_pixel uyumu).
        """
        frame_bgr = self._maybe_auto_swap(frame_bgr)

        altitude = altitude_m if altitude_m is not None else self._altitude_m
        yaw = yaw_deg if yaw_deg is not None else self._yaw_deg
        pitch = pitch_deg if pitch_deg is not None else self._pitch_deg
        roll = roll_deg if roll_deg is not None else self._roll_deg
        gyro = gyro_deg_s if gyro_deg_s is not None else self._gyro_deg_s
        lat = latitude if latitude is not None else self._latitude
        lon = longitude if longitude is not None else self._longitude

        pose: Optional[VehiclePose] = None
        if lat is not None and lon is not None:
            pose = VehiclePose(
                latitude=lat, longitude=lon,
                yaw_deg=yaw or 0.0, pitch_deg=pitch or 0.0, roll_deg=roll or 0.0,
            )

        detections = self._pipeline.process(
            frame_bgr, altitude_m=altitude, pose=pose, gyro_deg_s=gyro,
        )
        self._track_fps()
        return detections

    def _track_fps(self) -> None:
        """Pipeline işleme süresi üzerinden FPS takibi — min eşik uyarısı.

        Uyarı, döngü gecikmesine değil pipeline'ın gerçek frame süresine
        bağlıdır (warmup/ilk-frame gecikmeleri yanlış uyarı üretmez).
        """
        now = time.monotonic()
        frame_ms = self._pipeline.last_frame_ms
        if frame_ms > 0:
            self._fps_ema = 1000.0 / max(frame_ms, 1.0)
        if (
            self._fps_ema > 0
            and self._fps_ema < VISION_MIN_PROCESSING_FPS
            and now - self._last_fps_warn_t > 5.0
        ):
            self._last_fps_warn_t = now
            self._log.warning(
                "İşleme FPS düşük: %.1f (min %.1f) — pipeline %.0f ms/frame",
                self._fps_ema, VISION_MIN_PROCESSING_FPS, frame_ms,
            )

    # ──────────────────────────────────────────────────────────────
    # Zaman kilidi, history, draw — ESKİ davranış aynen korunur
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

        mature = [
            (color, shape)
            for (color, shape), first_t in self._first_seen.items()
            if now - first_t >= TARGET_LOCK_DURATION_S
        ]
        if not mature:
            return

        if len(mature) > 1:
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
        self._pipeline.reset_temporal()

    @staticmethod
    def draw_detections(frame_bgr, detections):
        out = frame_bgr.copy()
        h, w = out.shape[:2]
        cx, cy = w // 2, h // 2
        # Çözünürlüğe bağlı ölçek — üst HUD ile aynı (640 bazlı): 1920px'de ~3x.
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
