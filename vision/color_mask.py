"""
Renk maskesi — HSV dönüşümü + morphology (tamamen config-driven).

Sorumluluk: BGR frame + hedef config → ikili renk maskesi.

- Kırmızı: hue wrap-around nedeniyle İKİ AYRI aralık OR'lanır
  (H:[0,10] ∪ H:[160,179]) — tek geniş [0,179] aralığı KULLANILMAZ.
- Mavi: tek aralık.
- S_min / V_min config'ten gelir; düşük saturation (asfalt/beton/gri zemin)
  maskeye girmemesi için S_min yüksek tutulur.
- MORPH_OPEN + MORPH_CLOSE: kernel boyutu ve iteration'lar config'ten.

Bu modül magic number İÇERMEZ — tüm değerler yapılandırmadan okunur.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable

import cv2
import numpy as np


@dataclass(frozen=True)
class HsvRange:
    """Tek bir HSV aralığı: [H_lo, H_hi] + [S_min, V_min] eşikleri."""

    h_lo: int
    h_hi: int
    s_min: int
    v_min: int


class ColorMask:
    """Config'ten okunan HSV aralıklarıyla renk maskesi üretir."""

    def __init__(self, hsv_ranges: Iterable[Iterable[int]], morphology_cfg: dict[str, Any]) -> None:
        """
        hsv_ranges: [H_lo, H_hi, S_min, V_min] listesi (çoklu aralık OR'lanır).
        morphology_cfg: {"kernel_size": int, "open_iterations": int,
                         "close_iterations": int}
        """
        self._ranges: list[HsvRange] = [
            HsvRange(h_lo=int(r[0]), h_hi=int(r[1]), s_min=int(r[2]), v_min=int(r[3]))
            for r in hsv_ranges
        ]
        self._kernel_size = max(int(morphology_cfg["kernel_size"]), 1)
        self._open_iters = max(int(morphology_cfg["open_iterations"]), 0)
        self._close_iters = max(int(morphology_cfg["close_iterations"]), 0)
        self._kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (self._kernel_size, self._kernel_size)
        )

    def apply(self, hsv: np.ndarray) -> np.ndarray:
        """HSV frame'den maskeyi üretir (aralıklar OR + morphology).

        hsv: BGR → HSV dönüştürülmüş frame (cv2.COLOR_BGR2HSV çıktısı).
        """
        mask = None
        for r in self._ranges:
            band = cv2.inRange(
                hsv,
                np.array([r.h_lo, r.s_min, r.v_min], dtype=np.uint8),
                np.array([r.h_hi, 255, 255], dtype=np.uint8),
            )
            mask = band if mask is None else cv2.bitwise_or(mask, band)
        if mask is None:
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        return self._morphology(mask)

    def _morphology(self, mask: np.ndarray) -> np.ndarray:
        """OPEN (gürültü) + CLOSE (delik) — iteration'lar config'ten."""
        if self._open_iters > 0:
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_OPEN, self._kernel, iterations=self._open_iters
            )
        if self._close_iters > 0:
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_CLOSE, self._kernel, iterations=self._close_iters
            )
        return mask
