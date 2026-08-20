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
from dataclasses import dataclass
from typing import Any, Optional

import cv2
import numpy as np

# Matematiksel sabitler — algoritmanın geometrik tanımının parçası.
_PI = math.pi
_SQRT3 = math.sqrt(3.0)


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
    corner_angles: Optional[list[float]] = None  # approxPolyDP iç açıları (derece)
    min_rect_aspect: float = 0.0       # minAreaRect w/h (≥1) — rotated şekillerde sağlam
    edge_equality: float = 0.0         # [0,1] — kenar uzunluklarının eşitliği (eşkenarlık)
    shape: Optional[str] = None      # "TRIANGLE" | "HEXAGON" | "SQUARE" | None
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
                      "angle_weight", "angle_tolerance_deg", "min_shape_score"}.
        target_cfg: hedefe özgü {"circularity_range", "solidity_min",
                     "vertex_count_range", "max_extent", "angle_target_deg"}.
        shape_name: üretilen şeklin adı ("TRIANGLE" / "HEXAGON" / "SQUARE").
        """
        self._shape_name = shape_name
        self._epsilons = [float(e) for e in vertex_epsilons]
        self._w_circ = float(scoring_cfg["circularity_weight"])
        self._w_sol = float(scoring_cfg["solidity_weight"])
        self._w_vert = float(scoring_cfg["vertex_weight"])
        self._w_angle = float(scoring_cfg["angle_weight"])
        self._angle_tolerance = float(scoring_cfg["angle_tolerance_deg"])
        self._merge_angle_deg = float(scoring_cfg["collinear_merge_deg"])
        self._min_shape_score = float(scoring_cfg["min_shape_score"])

        self._circ_lo, self._circ_hi = (float(v) for v in target_cfg["circularity_range"])
        self._solidity_min = float(target_cfg["solidity_min"])
        self._max_extent = float(target_cfg["max_extent"])
        self._angle_target = float(target_cfg["angle_target_deg"])
        self._min_edge_equality = float(target_cfg["min_edge_equality"])
        self._max_angle_deviation = float(target_cfg["max_angle_deviation_deg"])
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

        x, y, w, h = cv2.boundingRect(contour)
        if w <= 2 or h <= 2 or w * h == 0:
            return None
        extent = float(area) / float(w * h)
        if extent > self._max_extent:
            return None  # kare/dikdörtgen gibi dolu bbox şekiller hedef olamaz
        aspect = float(w) / h if h >= w else float(h) / w
        if aspect > 3.0:
            return None  # aşırı uzun/dar

        count, raw_count, poly, corner_angles = self._best_polygon(contour, perimeter)
        if poly is None:
            return None  # hiçbir epsilon hedef köşe aralığına uymadı
        # Geometri TEMİZLENMİŞ poligondan hesaplanır — ham kontur jagged
        # (testere dişi) olduğunda circularity/perimeter çöker; poligon düz
        # kenarlarla "ideal şekil" ölçüsü verir.
        poly_area = cv2.contourArea(poly)
        poly_per = cv2.arcLength(poly, True)
        if poly_area <= 0 or poly_per < 5.0:
            return None
        circularity = 4.0 * _PI * poly_area / (poly_per * poly_per)
        if circularity > self._circ_hard_max:
            return None  # daire/elips hedef şekle uymaz

        # Solidity: içbükeylik ölçüsü (poligon vs hull) — jaggedness DEĞİL.
        hull_poly = cv2.convexHull(poly)
        hull_poly_area = cv2.contourArea(hull_poly)
        solidity = float(poly_area) / hull_poly_area if hull_poly_area > 0 else 0.0
        if solidity < self._solidity_min:
            return None  # güçlü iç bükeylik: düzensiz blob

        # Köşe iç açıları + kenar eşitliği (temizlenmiş poligondan).
        edge_equality = _edge_equality_score(poly)
        # Eşkenarlık: kenar uzunlukları birbirine yeterince eşit olmalı
        # (eşkenar üçgen / düzgün altıgen). Yamuk ve eşkenar olmayan üçgenler
        # sert elenir — "hedef şekle yakın" olmak yetmez, DÜZGÜN olmalıdır.
        if edge_equality < self._min_edge_equality:
            return None

        # Köşe açıları hedef açıya (60°/90°/120°) yeterince yakın olmalı —
        # ikinci eşkenarlık kanıtı. Kenarları eşit ama açıları bozuk şekiller
        # (uzamış altıgen vb.) bu kapıda elenir. Tolerans config'ten.
        if corner_angles and self._max_angle_deviation > 0:
            worst_dev = max(abs(a - self._angle_target) for a in corner_angles)
            if worst_dev > self._max_angle_deviation:
                return None

        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return None
        cx = moments["m10"] / moments["m00"]
        cy = moments["m01"] / moments["m00"]

        # minAreaRect en-boy oranı — rotated (dönük) şekillerde axis-aligned
        # bbox'tan daha sağlamdır (kare her yönde ~1.0).
        min_rect_aspect = 0.0
        try:
            rect_w, rect_h = cv2.minAreaRect(contour)[1]
            if min(rect_w, rect_h) > 0:
                min_rect_aspect = max(rect_w, rect_h) / min(rect_w, rect_h)
        except Exception:
            min_rect_aspect = 0.0

        return ShapeCandidate(
            contour=contour,
            area_px=area,
            perimeter_px=perimeter,
            circularity=circularity,
            solidity=solidity,
            vertex_count=count,
            vertex_agreement=raw_count,
            bbox_x=x, bbox_y=y, bbox_w=w, bbox_h=h,
            extent=extent,
            aspect_ratio=aspect,
            centroid_x=cx, centroid_y=cy,
            corner_angles=corner_angles,
            min_rect_aspect=min_rect_aspect,
            edge_equality=edge_equality,
        )

    def _best_polygon(
        self, contour: np.ndarray, perimeter: float
    ) -> tuple[int, int, Optional[np.ndarray], list[float]]:
        """Hedefe en uygun temizlenmiş poligonu seçer.

        Tüm epsilon'larda approxPolyDP + doğrusal köşe birleştirme yapılır;
        hedef köşe aralığına uyan poligonlar arasından köşe açıları hedef
        açıya (60°/90°/120°) EN İYİ uyan seçilir.

        Neden tek epsilon değil?
          - Küçük gerçek üçgen: eps 0.02 morpholojinin yuvarlattığı tepeyi
            2 köşeye böler (4 köşe çıkar); 0.04/0.06 temiz 3 köşe verir.
          - Jagged (jitter'lı) altıgen: eps 0.04/0.06 komşu köşeleri aşırı
            birleştirip 4 köşeye düşürür; 0.02 doğru 6 köşeyi verir.
        Hangisinin doğru olduğunu köşe sayısı TEK BAŞINA söyleyemez; açı
        uyumu (eşkenar hedef için tüm açılar hedef açıya yakın) karar verir.

        Dönen: (köşe sayısı, ham approx köşe sayısı, poligon, iç açılar).
        Poligon yoksa (None, ...) → aday elenir.
        """
        best_count = 0
        best_raw = 0
        best_poly: Optional[np.ndarray] = None
        best_angles: list[float] = []
        best_dev = float("inf")
        for eps in self._epsilons:
            approx = cv2.approxPolyDP(contour, eps * perimeter, True)
            raw_count = len(approx)
            poly = _merge_collinear(approx, self._merge_angle_deg)
            count = len(poly)
            if count < self._vert_lo or count > self._vert_hi:
                continue
            angles = _corner_angles(poly)
            dev = max(abs(a - self._angle_target) for a in angles)
            if dev < best_dev:
                best_dev = dev
                best_count = count
                best_raw = raw_count
                best_poly = poly
                best_angles = angles
        return best_count, best_raw, best_poly, best_angles

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
        angle_score = self._angle_score(cand)
        score = self._combine(circ_score, sol_score, vert_score, angle_score)
        cand.shape_score = score
        if score >= self._min_shape_score:
            cand.shape = self._shape_name
        return cand

    def _combine(
        self, circ_score: float, sol_score: float, vert_score: float, angle_score: float
    ) -> float:
        """Ağırlıklı birleşim (config ağırlıkları). Alt sınıflar override edebilir."""
        return (
            self._w_circ * circ_score
            + self._w_sol * sol_score
            + self._w_vert * vert_score
            + self._w_angle * angle_score
        )

    def _angle_score(self, cand: ShapeCandidate) -> float:
        """Köşe iç açılarının hedef açıya (örn. 60°/120°) yakınlığı [0,1].

        Her köşenin |açı - hedef|/tolerans oranı 1'e clamp edilir; ortalama alınır.
        Eğri konturlarda (köşe yok) skor 0 — hedef şekil düz kenar ister.
        """
        if not cand.corner_angles:
            return 0.0
        total = 0.0
        for angle in cand.corner_angles:
            total += max(0.0, 1.0 - abs(angle - self._angle_target) / self._angle_tolerance)
        return total / len(cand.corner_angles)

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


class SquareDetector(ShapeDetector):
    """KARE DECOY rakip skoru — gerçek hedefleri kandırmak için görev alanındaki
    karelerin (mavi 2m / kırmızı 1m) ne kadar "kareye benzediğini" ölçer.

    Pipeline her aday için target_score vs square_score karşılaştırması yapar
    (VISION_DECOY_REJECTION). Kare skoru yalnızca vertex'e dayanmaz:
      vertex + circularity + solidity + angle(90°) + bbox aspect + minAreaRect aspect
    Tüm eşikler VISION_SQUARE config'inden gelir.
    """

    def __init__(
        self, vertex_epsilons: list[float], scoring_cfg: dict[str, Any], square_cfg: dict[str, Any]
    ) -> None:
        # ShapeDetector ortak geometriyi (analyze) sağlar; skor kareye özgüdür.
        super().__init__(vertex_epsilons, scoring_cfg, square_cfg, shape_name="SQUARE")
        self._square_min_score = float(square_cfg["minimum_score"])
        self._aspect_target = float(square_cfg["aspect_target"])
        self._aspect_tol = float(square_cfg["aspect_tolerance"])
        self._min_rect_tol = float(square_cfg["min_rect_aspect_tolerance"])
        # Kareye özgü dar angle toleransı (altıgenin 120° köşeleri uzak kalır).
        self._angle_tolerance = float(square_cfg["angle_tolerance_deg"])
        # Kareye özgü köşe skoru: 4 ideal; 5-6 (altıgen!) hızla düşer.
        self._vert_map = {
            int(k): float(v) for k, v in square_cfg["vertex_score_map"].items()
        }
        w = square_cfg["weights"]
        self._sq_w_vert = float(w["vertex"])
        self._sq_w_circ = float(w["circularity"])
        self._sq_w_sol = float(w["solidity"])
        self._sq_w_angle = float(w["angle"])
        self._sq_w_aspect = float(w["aspect"])
        self._sq_w_minrect = float(w["min_rect_aspect"])

    def _vertex_score(self, count: int) -> float:
        """Kareye özgü: 4 köşe ideal; 5-6 köşe (altıgen) güçlü şekilde düşer."""
        return self._vert_map.get(count, 0.0)

    def classify(self, cand: ShapeCandidate) -> ShapeCandidate:
        """Kare skorunu hesaplar (aspect/minRect dahil) ve eşiği uygular."""
        circ_score = self._circularity_score(cand.circularity)
        sol_score = self._solidity_score(cand.solidity)
        vert_score = self._vertex_score(cand.vertex_count)
        angle_score = self._angle_score(cand)
        aspect_score = self._aspect_score(cand.aspect_ratio)
        min_rect_score = self._min_rect_score(cand.min_rect_aspect)
        score = (
            self._sq_w_vert * vert_score
            + self._sq_w_circ * circ_score
            + self._sq_w_sol * sol_score
            + self._sq_w_angle * angle_score
            + self._sq_w_aspect * aspect_score
            + self._sq_w_minrect * min_rect_score
        )
        cand.shape_score = score
        if score >= self._square_min_score:
            cand.shape = "SQUARE"
        return cand

    def _aspect_score(self, aspect_ratio: float) -> float:
        """Axis-aligned bbox en-boy oranının 1.0'a yakınlığı."""
        if self._aspect_tol <= 0 or aspect_ratio <= 0:
            return 0.0
        return max(0.0, 1.0 - abs(self._aspect_target - aspect_ratio) / self._aspect_tol)

    def _min_rect_score(self, min_rect_aspect: float) -> float:
        """minAreaRect en-boy oranının 1.0'a yakınlığı (rotated kare sağlam)."""
        if self._min_rect_tol <= 0 or min_rect_aspect <= 0:
            return 0.0
        return max(0.0, 1.0 - abs(self._aspect_target - min_rect_aspect) / self._min_rect_tol)


def _corner_angles(approx: np.ndarray) -> list[float]:
    """Çokgen köşelerindeki İÇ açıları derece cinsinden döndürür."""
    n = len(approx)
    angles: list[float] = []
    pts = approx.reshape(n, 2).astype(np.float64)
    for i in range(n):
        p_prev = pts[(i - 1) % n]
        p_cur = pts[i]
        p_next = pts[(i + 1) % n]
        v1 = p_prev - p_cur
        v2 = p_next - p_cur
        n1 = math.hypot(v1[0], v1[1])
        n2 = math.hypot(v2[0], v2[1])
        if n1 < 1e-6 or n2 < 1e-6:
            continue
        cos_a = float(np.clip((v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2), -1.0, 1.0))
        angles.append(math.degrees(math.acos(cos_a)))
    return angles


def _edge_equality_score(approx: np.ndarray) -> float:
    """Kenar uzunluklarının eşitliği (eşkenarlık) — [0,1].

    Ardışık köşeler arası uzaklıkların varyasyon katsayısına (cv = std/mean)
    dayanır: eşkenar üçgen / düzgün altıgen cv ≈ 0.02-0.06 → skor ~0.9;
    yamuk / eşkenar olmayan üçgen cv ≈ 0.25+ → skor düşer.
    """
    n = len(approx)
    if n < 3:
        return 0.0
    pts = approx.reshape(n, 2).astype(np.float64)
    edges = [
        math.hypot(*(pts[i] - pts[(i + 1) % n]))
        for i in range(n)
    ]
    mean = sum(edges) / n
    if mean <= 1e-6:
        return 0.0
    var = sum((e - mean) ** 2 for e in edges) / n
    cv = math.sqrt(var) / mean
    return max(0.0, 1.0 - cv / 0.5)


def _merge_collinear(approx: np.ndarray, merge_deg: float) -> np.ndarray:
    """İç açısı 180°'ye çok yakın (≈ doğrusal) köşeleri birleştirir.

    Blur/jagged maskelerde kenar üzerinde oluşan gürültü köşelerini temizler:
    gerçek köşeler (üçgen 60°, kare 90°, altıgen 120°) korunur, kenar
    üzerindeki ~175°'lik sahte köşeler atılır. `merge_deg`: 180°'den sapma
    toleransı (örn. 20° → açısı > 160° olan köşe doğrusal sayılır).
    """
    n = len(approx)
    if n < 3:
        return approx
    pts = approx.reshape(n, 2).astype(np.float64)
    threshold = 180.0 - merge_deg
    keep = []
    for i in range(n):
        p_prev = pts[(i - 1) % n]
        p_cur = pts[i]
        p_next = pts[(i + 1) % n]
        v1 = p_prev - p_cur
        v2 = p_next - p_cur
        n1 = math.hypot(v1[0], v1[1])
        n2 = math.hypot(v2[0], v2[1])
        if n1 < 1e-6 or n2 < 1e-6:
            continue  # çift köşe → birleştir
        cos_a = float(np.clip((v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2), -1.0, 1.0))
        angle = math.degrees(math.acos(cos_a))
        if angle <= threshold:
            keep.append(i)
    if len(keep) < 3:
        return approx  # güvenli geri dönüş: aşırı agresif birleştirme olmaz
    return approx[keep].reshape(-1, 1, 2).astype(np.int32)
