"""
DECOY (kare) reddi testleri — görevin EN KRİTİK kabul kriteri.

Görev alanındaki şekiller:
  🔵 Mavi altıgen (2m) — gerçek hedef
  🔴 Kırmızı üçgen (1m) — gerçek hedef
  🔵 Mavi kare (2m) — DECOY (kandırmaca)
  🔴 Kırmızı kare (1m) — DECOY

MUTLAK KURALLAR:
  * Mavi kare ASLA mavi altıgen olarak algılanmamalı.
  * Kırmızı kare ASLA kırmızı üçgen olarak algılanmamalı.
  * Gerçek hedefler başarıyla bulunmalı (decoy savunması recall'ı öldürmemeli).

Decoy'lar ideal geometride olmayabilir → blur, rotasyon, perspektif ve gürültü
varyantları da test edilir.

Gerçek boyutlar (11.5m'de, GSD ≈ 0.01555 m/px):
  * 2m altıgen kenarı ≈ 129px @960 = 258px @1920
  * 2m kare kenarı ≈ 129px @960 = 258px @1920
  * 1m üçgen kenarı ≈ 64px @960 = 128px @1920
  * 1m kare kenarı ≈ 64px @960 = 128px @1920
"""
import cv2
import numpy as np
import pytest

from models import TargetColor
from vision.target_detector import TargetDetector

H, W = 704, 1920
_BLUE = (255, 0, 0)
_RED = (0, 0, 255)


def _mk() -> np.ndarray:
    return np.zeros((H, W, 3), dtype=np.uint8)


def _square(cx=960, cy=420, side=258, color=_BLUE, angle=0.0) -> np.ndarray:
    """İsteğe bağlı rotasyonlu kare (side px kenar @1920)."""
    img = _mk()
    half = side // 2
    pts = np.array(
        [[cx - half, cy - half], [cx + half, cy - half],
         [cx + half, cy + half], [cx - half, cy + half]],
        dtype=np.float32,
    )
    if angle:
        m = cv2.getRotationMatrix2D((cx, cy), angle, 1.0)
        pts = cv2.transform(pts.reshape(1, 4, 2), m).reshape(4, 2)
    cv2.fillPoly(img, [pts.astype(np.int32)], color)
    return img


def _hexagon(cx=960, cy=420, s=258, color=_BLUE) -> np.ndarray:
    """Düzgün (eşkenar) altıgen — tüm kenarlar s."""
    img = _mk()
    cv2.fillPoly(
        img,
        [np.array(
            [[cx - s, cy], [cx - s // 2, cy - int(s * 0.866)],
             [cx + s // 2, cy - int(s * 0.866)], [cx + s, cy],
             [cx + s // 2, cy + int(s * 0.866)], [cx - s // 2, cy + int(s * 0.866)]],
            dtype=np.int32,
        )],
        color,
    )
    return img


def _triangle(cx=960, cy=420, half=64, color=_RED) -> np.ndarray:
    img = _mk()
    cv2.fillPoly(
        img,
        [np.array(
            [[cx, cy - int(half * 1.732)], [cx - half, cy], [cx + half, cy]],
            dtype=np.int32,
        )],
        color,
    )
    return img


def _perspective(img, src, dst) -> np.ndarray:
    m = cv2.getPerspectiveTransform(
        np.array(src, dtype=np.float32), np.array(dst, dtype=np.float32)
    )
    return cv2.warpPerspective(img, m, (W, H), borderValue=(0, 0, 0))


# ── EN KRİTİK: kareler ASLA hedef olmamalı ───────────────────────────────── #

@pytest.mark.parametrize("side", [258, 128, 64])
@pytest.mark.parametrize("color,shape", [(_BLUE, "HEXAGON"), (_RED, "TRIANGLE")])
def test_square_never_classified_as_target(side, color, shape):
    """Farklı boyutlardaki kareler hiçbir koşulda hedef şekle dönüşmemeli."""
    img = _square(side=side, color=color)
    dets = TargetDetector().detect(img)
    assert not any(d.shape == shape for d in dets), \
        f"{'mavi' if color == _BLUE else 'kırmızı'} kare {shape} olarak algılandı!"


def test_blue_square_not_hexagon():
    assert TargetDetector().detect(_square(side=258, color=_BLUE)) == []


def test_red_square_not_triangle():
    assert TargetDetector().detect(_square(side=128, color=_RED)) == []


# ── Gerçek hedefler HÂLÂ bulunmalı ────────────────────────────────────────── #

def test_real_blue_hexagon_detected():
    dets = TargetDetector().detect(_hexagon(s=258, color=_BLUE))
    assert any(d.color == TargetColor.BLUE and d.shape == "HEXAGON" for d in dets)


def test_real_red_triangle_detected():
    dets = TargetDetector().detect(_triangle(half=64, color=_RED))
    assert any(d.color == TargetColor.RED and d.shape == "TRIANGLE" for d in dets)


# ── Decoy varyantları ─────────────────────────────────────────────────────── #

@pytest.mark.parametrize("k", [5, 9])
@pytest.mark.parametrize("color,shape", [(_BLUE, "HEXAGON"), (_RED, "TRIANGLE")])
def test_blurred_square_rejected(k, color, shape):
    """Bulanık kare decoy olarak elenir (blur köşe sayısını bozabilir)."""
    img = cv2.GaussianBlur(
        _square(side=258 if color == _BLUE else 128, color=color), (k, k), 0
    )
    dets = TargetDetector().detect(img)
    assert not any(d.shape == shape for d in dets), f"blur k={k} kare kaçtı"


@pytest.mark.parametrize("color,shape", [(_BLUE, "HEXAGON"), (_RED, "TRIANGLE")])
def test_rotated_square_rejected(color, shape):
    """45° dönük kare: minAreaRect kareyi sağlam yakalar, köşe açıları 90° →
    square skoru yüksek → hedef reddedilir."""
    img = _square(side=258 if color == _BLUE else 128, color=color, angle=45.0)
    dets = TargetDetector().detect(img)
    assert not any(d.shape == shape for d in dets), "45° dönük kare kaçtı"


@pytest.mark.parametrize("color,shape", [(_BLUE, "HEXAGON"), (_RED, "TRIANGLE")])
def test_perspective_distorted_square_rejected(color, shape):
    """Hafif perspektif bozulma (pitch/roll etkisi) kareyi hedef yapmamalı."""
    side = 258 if color == _BLUE else 128
    img = _square(side=side, color=color)
    c = 960
    half = side // 2
    src = [[c - half, 420 - half], [c + half, 420 - half],
           [c + half, 420 + half], [c - half, 420 + half]]
    dst = [[c - half + 18, 420 - half + 20], [c + half - 20, 420 - half - 10],
           [c + half + 20, 420 + half + 20], [c - half - 15, 420 + half + 10]]
    dets = TargetDetector().detect(_perspective(img, src, dst))
    assert not any(d.shape == shape for d in dets), "perspektif kare kaçtı"


@pytest.mark.parametrize("color,shape", [(_BLUE, "HEXAGON"), (_RED, "TRIANGLE")])
def test_noisy_square_rejected(color, shape):
    """Gürültülü kare decoy olarak elenir."""
    rng = np.random.default_rng(11)
    noise = rng.integers(0, 40, (H, W, 3), dtype=np.uint8)
    img = cv2.add(
        _square(side=258 if color == _BLUE else 128, color=color), noise
    )
    dets = TargetDetector().detect(img)
    assert not any(d.shape == shape for d in dets), "gürültülü kare kaçtı"


# ── Gerçek hedef + decoy aynı frame'de ────────────────────────────────────── #

def test_hexagon_with_blue_square_same_frame():
    """Aynı frame'de gerçek altıgen + mavi kare → YALNIZCA altıgen bulunur."""
    img = _mk()
    cv2.fillPoly(
        img,
        [np.array(
            [[680 - 180, 420], [680 - 90, 420 - int(180 * 0.866)],
             [680 + 90, 420 - int(180 * 0.866)], [680 + 180, 420],
             [680 + 90, 420 + int(180 * 0.866)], [680 - 90, 420 + int(180 * 0.866)]],
            dtype=np.int32)],
        _BLUE,
    )
    cv2.fillPoly(
        img,
        [np.array(
            [[1350 - 129, 420 - 129], [1350 + 129, 420 - 129],
             [1350 + 129, 420 + 129], [1350 - 129, 420 + 129]], dtype=np.int32)],
        _BLUE,
    )
    dets = TargetDetector().detect(img)
    assert any(d.shape == "HEXAGON" for d in dets), "gerçek altıgen bulunmalı"
    assert not any(d.shape == "SQUARE" for d in dets), "decoy square üretilmemeli"


# ── EŞKENAR OLMAYAN / YAMUK ŞEKİLLER — kullanıcı gereksinimi ────────────── #
# Gerçek hedefler STANDART (eşkenar) üçgen ve DÜZGÜN altıgendir. 4 köşeli
# yamuklar, eşkenar olmayan üçgenler ve düzgün olmayan 6-köşeli şekiller
# ASLA hedef sayılmamalı (vertex + eşkenarlık + açı kapıları).


def test_trapezoid_never_triangle():
    """4 köşeli yamuk ASLA üçgen sayılmaz (sert köşe kapısı [3,3])."""
    img = _mk()
    cv2.fillPoly(
        img, [np.array([[760, 300], [1160, 300], [1260, 480], [660, 480]])], _RED
    )
    assert TargetDetector().detect(img) == []


@pytest.mark.parametrize("hgt", [200, 260, 300, 400])
def test_isosceles_triangle_never_triangle(hgt):
    """Eşkenar OLMAYAN (ikizkenar) üçgen ASLA üçgen sayılmaz.

    Taban 600px sabit, yükseklik değişir; gerçek eşkenar yüksekliği ~520px.
    Eşkenarlık (kenar eşitliği) + açı (60°) kapıları bu üçgenleri eler.
    """
    img = _mk()
    cv2.fillPoly(
        img,
        [np.array([[960, 352 - hgt], [660, 352], [1260, 352]], dtype=np.int32)],
        _RED,
    )
    assert TargetDetector().detect(img) == [], f"ikizkenar h={hgt} üçgen sayıldı!"


def test_scalene_triangle_never_triangle():
    """Dik üçgen (30-60-90) — kenarlar eşit değil, açılar 60°'den uzak → elenir."""
    img = _mk()
    cv2.fillPoly(
        img,
        [np.array([[600, 352], [1400, 352], [1400, 120]], dtype=np.int32)],
        _RED,
    )
    assert TargetDetector().detect(img) == []


def test_blue_trapezoid_never_hexagon():
    """4 köşeli yamuk ASLA altıgen sayılmaz (köşe kapısı [5,7])."""
    img = _mk()
    cv2.fillPoly(
        img, [np.array([[700, 320], [1220, 320], [1320, 480], [600, 480]])], _BLUE
    )
    assert TargetDetector().detect(img) == []


def test_irregular_hexagon_rejected():
    """6 köşeli ama DÜZGÜN olmayan şekil ASLA altıgen sayılmaz (eşkenarlık kapısı)."""
    img = _mk()
    cv2.fillPoly(
        img,
        [np.array(
            [[830, 352], [900, 290], [1050, 300], [1100, 360],
             [1000, 420], [880, 400]], dtype=np.int32)],
        _BLUE,
    )
    assert TargetDetector().detect(img) == []


def test_stretched_hexagon_rejected():
    """Dikey uzatılmış (kenar eşitliği bozuk) altıgen benzeri → elenir."""
    img = _mk()
    cv2.fillPoly(
        img,
        [np.array(
            [[840, 352], [920, 300], [1030, 290], [1100, 340],
             [1060, 410], [930, 420]], dtype=np.int32)],
        _BLUE,
    )
    assert TargetDetector().detect(img) == []


# ── Modül seviyesi: decoy rejection kuralları ─────────────────────────────── #

def test_decoy_margin_config_available():
    """Decoy rejection eşikleri config'ten gelir (sıfır magic number)."""
    from config import VISION_DECOY_REJECTION
    assert VISION_DECOY_REJECTION["minimum_target_score"] > 0
    assert VISION_DECOY_REJECTION["minimum_margin"] > 0


def test_uncertain_margin_defined():
    """UNCERTAIN bölgesi config'te tanımlıdır (margin < uncertain → belirsiz)."""
    from config import VISION_DECOY_REJECTION
    assert VISION_DECOY_REJECTION["uncertain_margin"] < \
        VISION_DECOY_REJECTION["minimum_margin"]
