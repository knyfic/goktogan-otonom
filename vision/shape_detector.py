"""
Şekil tespiti — kontur analizi + skor tabanlı sınıflandırma.

Sorumluluk: renk maskesi → aday konturlar → geometrik özellikler → şekil skoru.

- findContours(RETR_EXTERNAL) ile maskeden konturlar çıkarılır.
- Her kontur için: area, perimeter, circularity (4πA/P²), solidity (A/hull_A),
  convex hull, approxPolyDP köşe sayısı (çoklu epsilon — çoğunluk oyu),
  bounding box, centroid hesaplanır.
- Şekil sınıflandırması YALNIZCA köşe sayısına dayanmaz: circularity +
  solidity + vertex_count ağırlıklı birleşim (ağırlıklar config'ten).
- Hiçbir tek özellik tek başına "hedef" kararı veremez.

Sert elenme kuralları (yanlış pozitif kontrolü — config'ten):
- circularity üst sınırı (daire/elips)
- solidity alt sınırı (düzensiz blob)
- aşırı uzun/dar kontur (aspect ratio)
"""
from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass
from typing import Any, Optional

import cv2
import numpy as np

# Matematiksel sabitler — algoritmanın geometrik tanımının parçası.
_PI = math.pi
_SQRT3 = math.sqrt(3.0)

# Köşe sayısı çoğunluk oyunda eşik: 3 farklı epsilon'dan en az 2'si aynı
# köşe sayısını vermeli (daire/elips gibi eğri konturlar dağılır).
_VERTEX_AGREEMENT_MIN = 2


@dataclass
class ShapeCandidate:
    """Bir konturdan çıkarılan geometrik aday."""

    contour: np.ndarray
    area_px: float
    perimeter_px: float
    circularity: float
    solidity: float
    vertex_count: int
    vertex_agreement: int
    bbox_x: int
    bbox_y: int
    bbox_w: int
    bbox_h: int
    extent: float
    aspect_ratio: float
    centroid_x: float
    centroid_y: float
    shape: Optional[str] = None      # "TRIANGLE" | "HEXAGON" | None
    shape_score: float = 0.0         # [0,1] — config ağırlıklarıyla


class ShapeDetector:
    """Config'ten okunan şekil eşikleriyle kontur → aday üretir."""

    def __init__(
        self,
        vertex_epsilons: list[float],
        scoring_cfg: dict[str, Any],
        target_cfg: dict[str, Any],
        shape_name: str,
    ) -> None:
        """
        vertex_epsilons: approxPolyDP epsilon oranları (örn. [0.02, 0.04, 0.06]).
        scoring_cfg: {"circularity_weight", "solidity_weight", "vertex_weight",
                      "min_shape_score"}.
        target_cfg: hedefe özgü {"circularity_range", "solidity_min",
                     "vertex_count_range"}.
        shape_name: üretilen şeklin adı ("TRIANGLE" / "HEXAGON").
        """
        self._shape_name = shape_name
        self._epsilons = [float(e) for e in vertex_epsilons]
        self._w_circ = float(scoring_cfg["circularity_weight"])
        self._w_sol = float(scoring_cfg["solidity_weight"])
        self._w_vert = float(scoring_cfg["vertex_weight"])
        self._min_shape_score = float(scoring_cfg["min_shape_score"])

        self._circ_lo, self._circ_hi = (float(v) for v in target_cfg["circularity_range"])
        self._solidity_min = float(target_cfg["solidity_min"])
        self._max_extent = float(target_cfg["max_extent"])
        v_lo, v_hi = target_cfg["vertex_count_range"]
        self._vert_lo, self._vert_hi = int(v_lo), int(v_hi)

        # Daire/elips üst sınırı: hedef circ aralığının üstü + tolerans.
        self._circ_hard_max = self._circ_hi + 0.05

    def detect(self, mask: np.ndarray) -> list[ShapeCandidate]:
        """Maskeden aday konturları çıkarır (geometrik analiz — sınıflandırma yok)."""
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        candidates: list[ShapeCandidate] = []
        for contour in contours:
            cand = self._analyze_contour(contour)
            if cand is not None:
                candidates.append(cand)
        return candidates

    def _analyze_contour(self, contour: np.ndarray) -> Optional[ShapeCandidate]:
        area = cv2.contourArea(contour)
        if area <= 0:
            return None

        perimeter = cv2.arcLength(contour, True)
        if perimeter < 5.0:
            return None
        circularity = 4.0 * _PI * area / (perimeter * perimeter)
        if circularity > self._circ_hard_max:
            return None  # daire/elips hedef şekle uymaz

        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        if hull_area <= 0:
            return None
        solidity = float(area) / hull_area
        if solidity < self._solidity_min:
            return None  # güçlü iç bükeylik: düzensiz blob

        x, y, w, h = cv2.boundingRect(contour)
        if w <= 2 or h <= 2 or w * h == 0:
            return None
        extent = float(area) / float(w * h)
        if extent > self._max_extent:
            return None  # kare/dikdörtgen gibi dolu bbox şekiller hedef olamaz
        aspect = float(w) / h if h >= w else float(h) / w
        if aspect > 3.0:
            return None  # aşırı uzun/dar

        count, agreement = self._count_vertices(contour, perimeter)
        # Sert köşe üst sınırı: hedef aralığın üstündeki köşe sayısı eğri
        # kontur demektir (daire/elips 8+ köşe üretir; altıgen blur'da bile ≤ 7).
        if count > self._vert_hi:
            return None

        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return None
        cx = moments["m10"] / moments["m00"]
        cy = moments["m01"] / moments["m00"]

        return ShapeCandidate(
            contour=contour,
            area_px=area,
            perimeter_px=perimeter,
            circularity=circularity,
            solidity=solidity,
            vertex_count=count,
            vertex_agreement=agreement,
            bbox_x=x, bbox_y=y, bbox_w=w, bbox_h=h,
            extent=extent,
            aspect_ratio=aspect,
            centroid_x=cx, centroid_y=cy,
        )

    def _count_vertices(self, contour: np.ndarray, perimeter: float) -> tuple[int, int]:
        """Çoklu epsilon ile köşe sayısı çoğunluk oyu."""
        votes = [
            len(cv2.approxPolyDP(contour, eps * perimeter, True))
            for eps in self._epsilons
        ]
        count, agreement = Counter(votes).most_common(1)[0]
        return count, agreement

    def classify(self, cand: ShapeCandidate) -> ShapeCandidate:
        """Adaya şekil + skor atar; skor eşiğin altındaysa shape=None kalır.

        Skor = w_circ·circ_score + w_sol·solidity_score + w_vert·vertex_score
        Her bileşen [0,1]'e normalize edilir:
          - circ_score: circularity hedef aralığa ne kadar yakın (trapez).
          - solidity_score: solidity_min üstünde 1'e doğru doğrusal.
          - vertex_score: vertex_count hedef aralıkta → 1, dışına düştükçe azalır.
        """
        circ_score = self._circularity_score(cand.circularity)
        sol_score = self._solidity_score(cand.solidity)
        vert_score = self._vertex_score(cand.vertex_count)
        score = (
            self._w_circ * circ_score
            + self._w_sol * sol_score
            + self._w_vert * vert_score
        )
        cand.shape_score = score
        if score >= self._min_shape_score:
            cand.shape = self._shape_name
        return cand

    def classify_contour(self, contour: np.ndarray) -> Optional[ShapeCandidate]:
        """Tek kontur için analiz + sınıflandırma (pipeline dışı çağrılar için)."""
        cand = self._analyze_contour(contour)
        if cand is None:
            return None
        return self.classify(cand)

    def _circularity_score(self, circularity: float) -> float:
        """Hedef circ aralığı içinde 1, dışına doğrusal düşüş (bant genişliği 0.1)."""
        if self._circ_lo <= circularity <= self._circ_hi:
            return 1.0
        if circularity < self._circ_lo:
            return max(0.0, 1.0 - (self._circ_lo - circularity) / 0.10)
        return max(0.0, 1.0 - (circularity - self._circ_hi) / 0.10)

    def _solidity_score(self, solidity: float) -> float:
        """solidity_min üstünde doğrusal olarak 1'e yaklaşır."""
        if solidity <= self._solidity_min:
            return 0.0
        return min(1.0, (solidity - self._solidity_min) / 0.08)

    def _vertex_score(self, count: int) -> float:
        """Hedef köşe aralığında 1; dışına her köşe için düşüş."""
        if self._vert_lo <= count <= self._vert_hi:
            return 1.0
        if count < self._vert_lo:
            return max(0.0, 1.0 - (self._vert_lo - count) * 0.35)
        return max(0.0, 1.0 - (count - self._vert_hi) * 0.35)
