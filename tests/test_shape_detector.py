"""
shape_detector modülü testleri — kontur geometrisi + şekil skoru.

Senaryolar:
  * ideal üçgen
  * ideal altıgen
  * daire (eğri kontur → elenir)
  * düzensiz blob (düşük solidity → elenir)
  * solidity / circularity / vertex count hesapları
  * shape score ağırlıklı birleşim
  * kare/dikdörtgen (extent → elenir)
"""
import numpy as np
import pytest

from config import (
    VISION_SHAPE_SCORING,
    VISION_TARGETS,
    VISION_VERTEX_EPSILONS,
)
from vision.shape_detector import ShapeDetector

_TRI_CFG = VISION_TARGETS["RED_TRIANGLE"]
_HEX_CFG = VISION_TARGETS["BLUE_HEXAGON"]


def _tri_detector() -> ShapeDetector:
    return ShapeDetector(VISION_VERTEX_EPSILONS, VISION_SHAPE_SCORING, _TRI_CFG,
                         shape_name="TRIANGLE")


def _hex_detector() -> ShapeDetector:
    return ShapeDetector(VISION_VERTEX_EPSILONS, VISION_SHAPE_SCORING, _HEX_CFG,
                         shape_name="HEXAGON")


def _tri_pts(cx=480, top=20, base=280, half=150):
    # Eşkenar üçgen: taban 2·half, yükseklik half·√3 ≈ 260 → circ ≈ 0.60.
    return np.array([[cx, top], [cx - half, base], [cx + half, base]], dtype=np.int32)


def _hex_pts(cx=480, cy=240, s=120):
    return np.array(
        [
            [cx - s, cy], [cx, cy - int(s * 0.866)], [cx + s, cy],
            [cx + s, cy + int(s * 0.866)], [cx, cy + 2 * int(s * 0.866)],
            [cx - s, cy + int(s * 0.866)],
        ],
        dtype=np.int32,
    )


def _mask_from_polygon(pts, w=960, h=352) -> np.ndarray:
    import cv2
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(mask, [pts], 255)
    return mask


def _first_candidate(detector, mask):
    cands = detector.detect(mask)
    assert len(cands) >= 1, "aday üretilmeli"
    return cands[0]


# ── ideal şekiller ────────────────────────────────────────────────────────── #

def test_ideal_triangle_geometry():
    d = _tri_detector()
    cand = _first_candidate(d, _mask_from_polygon(_tri_pts()))
    assert cand.vertex_count == 3
    lo, hi = (float(v) for v in _TRI_CFG["circularity_range"])
    assert lo - 0.02 <= cand.circularity <= hi + 0.02, \
        f"circ={cand.circularity:.3f} config bandı [{lo}, {hi}] dışında"
    assert cand.solidity > 0.9
    assert abs(cand.extent - 0.5) < 0.05


def test_ideal_hexagon_geometry():
    d = _hex_detector()
    cand = _first_candidate(d, _mask_from_polygon(_hex_pts()))
    # Küçük altıgenlerde köşe sayısı 5-7 arasında dalgalanabilir (blur toleransı).
    lo, hi = (int(v) for v in _HEX_CFG["vertex_count_range"])
    assert lo <= cand.vertex_count <= hi
    c_lo, c_hi = (float(v) for v in _HEX_CFG["circularity_range"])
    assert c_lo - 0.02 <= cand.circularity <= c_hi + 0.02
    assert cand.solidity > 0.9


def test_triangle_classified_as_triangle():
    cand = _tri_detector().classify_contour(
        _tri_detector().detect(_mask_from_polygon(_tri_pts()))[0].contour
    )
    assert cand.shape == "TRIANGLE"
    assert cand.shape_score >= VISION_SHAPE_SCORING["min_shape_score"]


def test_hexagon_classified_as_hexagon():
    d = _hex_detector()
    cand = d.classify_contour(d.detect(_mask_from_polygon(_hex_pts()))[0].contour)
    assert cand.shape == "HEXAGON"
    assert cand.shape_score >= VISION_SHAPE_SCORING["min_shape_score"]


# ── yanlış şekiller ───────────────────────────────────────────────────────── #

def test_circle_rejected():
    """Daire: 8+ köşe üretir → sert elenir (hiçbir hedef şekle uymaz)."""
    import cv2
    mask = np.zeros((352, 960), dtype=np.uint8)
    cv2.circle(mask, (480, 200), 100, 255, -1)
    assert _tri_detector().detect(mask) == []
    assert _hex_detector().detect(mask) == []


def test_square_rejected_by_extent():
    """Kare: extent 1.0 → max_extent sınırıyla elenir."""
    import cv2
    mask = np.zeros((352, 960), dtype=np.uint8)
    cv2.rectangle(mask, (380, 100), (580, 300), 255, -1)
    assert _tri_detector().detect(mask) == []
    assert _hex_detector().detect(mask) == []


def test_irregular_blob_rejected_by_solidity():
    """İç bükey düzensiz blob: düşük solidity → elenir."""
    blob = np.array(
        [[400, 200], [450, 175], [510, 185], [540, 220], [500, 250],
         [430, 240], [410, 225]],
        dtype=np.int32,
    )
    cands = _tri_detector().detect(_mask_from_polygon(blob))
    # Düzensiz blob ya elenir ya da şekil skoru eşiğin altında kalır.
    for cand in cands:
        classified = _tri_detector().classify(cand)
        assert classified.shape is None, "düzensiz blob üçgen olarak sınıflanmamalı"


def test_shape_score_weights_from_config():
    """Skor ağırlıkları config'ten okunur (sıfır magic number)."""
    d = _tri_detector()
    assert d._w_circ == VISION_SHAPE_SCORING["circularity_weight"]
    assert d._w_sol == VISION_SHAPE_SCORING["solidity_weight"]
    assert d._w_vert == VISION_SHAPE_SCORING["vertex_weight"]
    assert d._min_shape_score == VISION_SHAPE_SCORING["min_shape_score"]


def test_shape_score_is_weighted_combination():
    """Skor = ağırlıklı birleşim (circ + solidity + vertex + angle);
    tek özellik tek başına karar veremez."""
    d = _tri_detector()
    raw = d.detect(_mask_from_polygon(_tri_pts()))[0]
    cand = d.classify(raw)  # skor atanır
    circ_s = d._circularity_score(cand.circularity)
    sol_s = d._solidity_score(cand.solidity)
    vert_s = d._vertex_score(cand.vertex_count)
    angle_s = d._angle_score(cand)
    expected = (
        d._w_circ * circ_s + d._w_sol * sol_s
        + d._w_vert * vert_s + d._w_angle * angle_s
    )
    assert cand.shape_score == pytest.approx(expected)


def test_triangle_angle_score_high():
    """Eşkenar üçgenin köşe açıları 60°'ye yakın → angle skoru yüksek."""
    d = _tri_detector()
    raw = d.detect(_mask_from_polygon(_tri_pts()))[0]
    assert d._angle_score(raw) > 0.7


def test_square_detector_scores_high_for_square():
    """Kare, SquareDetector'dan YÜKSEK skor alır (decoy tespiti)."""
    import cv2
    from config import VISION_SQUARE
    from vision.shape_detector import SquareDetector

    mask = np.zeros((352, 960), dtype=np.uint8)
    cv2.rectangle(mask, (380, 100), (580, 300), 255, -1)
    sd = SquareDetector(VISION_VERTEX_EPSILONS, VISION_SHAPE_SCORING, VISION_SQUARE)
    cand = sd.classify(sd.detect(mask)[0])  # kare analyze'den geçmeli
    assert cand.shape_score >= VISION_SQUARE["minimum_score"]


def test_square_detector_low_score_for_hexagon():
    """Altıgen SquareDetector'dan DÜŞÜK skor alır (decoy skoru düşük kalmalı)."""
    from config import VISION_SQUARE
    from vision.shape_detector import SquareDetector

    sd = SquareDetector(VISION_VERTEX_EPSILONS, VISION_SHAPE_SCORING, VISION_SQUARE)
    raw = sd.detect(_mask_from_polygon(_hex_pts()))[0]
    cand = sd.classify(raw)
    assert cand.shape_score < 0.85, "altıgen kare gibi skorlanmamalı"


def test_square_angle_score_high():
    """Karenin köşe açıları 90°'ye yakın → square angle skoru yüksek."""
    import cv2
    from config import VISION_SQUARE
    from vision.shape_detector import SquareDetector

    mask = np.zeros((352, 960), dtype=np.uint8)
    cv2.rectangle(mask, (380, 100), (580, 300), 255, -1)
    sd = SquareDetector(VISION_VERTEX_EPSILONS, VISION_SHAPE_SCORING, VISION_SQUARE)
    cand = sd.detect(mask)[0]
    assert sd._angle_score(cand) > 0.7
