import time
from collections import deque
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


class TargetDetector:
    """Mavi altıgen + kırmızı üçgen detector."""

    def __init__(self):
        self.history: deque = deque(maxlen=DETECTION_HISTORY_SIZE)
        # Zaman bazlı kilit: {(color, shape): first_seen_timestamp}
        # İlk görüldüğü andan itibaren TARGET_LOCK_DURATION_S geçerse
        # o hedef için zaman kilidi açılır (time_locked_type set edilir).
        self._first_seen: dict[tuple, float] = {}
        self.time_locked_color: Optional[TargetColor] = None
        self.time_locked_shape: Optional[str] = None

    @staticmethod
    def _color_mask(hsv, color):
        if color == TargetColor.BLUE:
            return cv2.inRange(
                hsv,
                np.array([90, 80, 60], dtype=np.uint8),
                np.array([135, 255, 255], dtype=np.uint8),
            )

        mask_low = cv2.inRange(
            hsv,
            np.array([0, 90, 60], dtype=np.uint8),
            np.array([10, 255, 255], dtype=np.uint8),
        )
        mask_high = cv2.inRange(
            hsv,
            np.array([170, 90, 60], dtype=np.uint8),
            np.array([179, 255, 255], dtype=np.uint8),
        )
        return cv2.bitwise_or(mask_low, mask_high)

    @staticmethod
    def _shape(vertices: int) -> Optional[str]:
        if vertices == 3:
            return "TRIANGLE"
        if 5 <= vertices <= 7:
            return "HEXAGON"
        return None

    def detect(self, frame_bgr):
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        kernel = np.ones((5, 5), np.uint8)
        detections = []

        for color in (TargetColor.BLUE, TargetColor.RED):
            mask = self._color_mask(hsv, color)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for contour in contours:
                area = cv2.contourArea(contour)
                if area < MIN_TARGET_AREA_PX:
                    continue

                perimeter = cv2.arcLength(contour, True)
                if perimeter <= 0:
                    continue

                approx = cv2.approxPolyDP(contour, 0.035 * perimeter, True)
                shape = self._shape(len(approx))
                if shape is None:
                    continue

                moments = cv2.moments(contour)
                if moments["m00"] == 0:
                    continue

                cx = moments["m10"] / moments["m00"]
                cy = moments["m01"] / moments["m00"]
                x, y, w, h = cv2.boundingRect(contour)

                expected_vertices = 6 if shape == "HEXAGON" else 3
                shape_score = max(
                    0.0,
                    1.0 - abs(len(approx) - expected_vertices) / 4.0,
                )
                area_score = min(1.0, area / 10000.0)

                confidence = 0.70 * shape_score + 0.30 * area_score
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

        return sorted(detections, key=lambda d: d.confidence, reverse=True)

    def update_time_lock(self, detections: list) -> None:
        """
        Zaman bazlı kilit güncelleme.
        Her çağrıda görüntedeki tespitlerin first_seen zamanını güncelle.
        Herhangi bir tip TARGET_LOCK_DURATION_S süre boyunca görünür kaldıysa
        self.time_locked_color / time_locked_shape set edilir.

        Birden fazla tip aynı anda süreyi doldurursa ilk dolan kazanır
        (zaten kilit set edilmişse değiştirilmez).
        """
        if self.time_locked_color is not None:
            return  # kilit zaten set, değiştirme

        now = time.monotonic()
        seen_types = {(d.color, d.shape) for d in detections}

        # Artık görünmeyen tipleri temizle
        for key in list(self._first_seen.keys()):
            if key not in seen_types:
                del self._first_seen[key]

        # Yeni görünenleri kaydet
        for key in seen_types:
            if key not in self._first_seen:
                self._first_seen[key] = now

        # Süresi dolmuş var mı?
        for (color, shape), first_t in self._first_seen.items():
            if now - first_t >= TARGET_LOCK_DURATION_S:
                self.time_locked_color = color
                self.time_locked_shape = shape
                return  # ilk dolan kazanır

    def reset_time_lock(self) -> None:
        """Bir sonraki hedef arayışı için zaman kilidini ve first_seen'i sıfırla."""
        self.time_locked_color = None
        self.time_locked_shape = None
        self._first_seen.clear()

    @staticmethod
    def draw_detections(frame_bgr, detections):
        """Tespit edilen hedefleri frame üzerine çizer ve görselleştirilmiş frame döndürür."""
        out = frame_bgr.copy()
        h, w = out.shape[:2]
        # Kamera merkezi
        cx, cy = w // 2, h // 2
        cv2.drawMarker(out, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 20, 2)

        for det in detections:
            color_bgr = (255, 80, 0) if det.color.value == "BLUE" else (0, 60, 255)
            tx = int(det.center_x)
            ty = int(det.center_y)

            # Merkez noktası
            cv2.circle(out, (tx, ty), 6, color_bgr, -1)

            # Kamera merkezinden hedefe çizgi
            cv2.line(out, (cx, cy), (tx, ty), color_bgr, 1)

            # Bounding box
            bw = int(det.bbox_width_px)
            bh = int(det.bbox_height_px)
            x1 = tx - bw // 2
            y1 = ty - bh // 2
            cv2.rectangle(out, (x1, y1), (x1 + bw, y1 + bh), color_bgr, 2)

            # Etiket: renk + şekil + confidence
            label = f"{det.color.value} {det.shape} {det.confidence:.2f}"
            cv2.putText(
                out, label, (x1, max(y1 - 8, 12)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color_bgr, 2,
            )

        # "Detections" sayacı — STATE yazısıyla çakışmaması için alt satıra
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
        """
        Kararlı tespit döndürür.

        Öncelik sırası:
          1. locked_color/locked_shape (state machine'den gelen explicit kilit)
          2. time_locked_color/time_locked_shape (1 sn görünürlük kilidi)
          3. History ile tutarlı olan tip
          4. History boşsa gelen ilk tip (ilk gören kazanır)

        search_mode=True: piksel konumu tutarlılığı aranmaz (İHA hareket eder).
        search_mode=False: piksel konumu da tutarlı olmalı (hizalama modu).
        """
        if not detections:
            self.history.clear()
            return None

        # 1. Explicit kilit (state machine)
        effective_color = locked_color
        effective_shape = locked_shape

        # 2. Zaman kilidi (1 sn görünürlük) — explicit kilit yoksa devreye girer
        if effective_color is None and self.time_locked_color is not None:
            effective_color = self.time_locked_color
            effective_shape = self.time_locked_shape

        # Kilit varsa filtrele
        if effective_color is not None and effective_shape is not None:
            filtered = [
                d for d in detections
                if d.color == effective_color and d.shape == effective_shape
            ]
            if not filtered:
                # Kilitlenen hedef bu frame'de yok; geçici kayıp tolere et
                return None
            candidate = filtered[0]
        else:
            # Kilit yok: history boşsa gelen ilk tip kazanır;
            # doluysa history ile tutarlı olanı tercih et.
            if len(self.history) == 0:
                # İki tip varsa ilk frame'de gelen ilk tip seçilir
                # (update_time_lock ile zamanla doğal seçim yapılır)
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
