"""
Görev hedef kuralları — yalnızca MAVİ ALTIGEN + KIRMIZI ÜÇGEN.

Detector, görev dışı kombinasyonları (BLUE+TRIANGLE, RED+HEXAGON) ÜRETMEMELİ;
aksi halde state machine "Geçersiz kombinasyon" deyip SEARCHING'e döner ve
sonsuz döngüye girer (gözlenen loop).
"""
import cv2
import numpy as np

from models import TargetColor
from vision.target_detector import TargetDetector

W, H = 1920, 704


def _hexagon(cx, cy, r):
    pts = []
    for i in range(6):
        ang = np.pi / 3 * i - np.pi / 2
        pts.append([cx + r * np.cos(ang), cy + r * np.sin(ang)])
    return np.array(pts, dtype=np.int32)


def _triangle(cx, cy, r):
    pts = []
    for i in range(3):
        ang = np.pi / 2 + 2 * np.pi / 3 * i  # tepe yukarı
        pts.append([cx + r * np.cos(ang), cy + r * np.sin(ang)])
    return np.array(pts, dtype=np.int32)


def _detect(color_bgr, pts):
    img = np.zeros((H, W, 3), dtype=np.uint8)
    cv2.fillPoly(img, [pts], color_bgr)
    return TargetDetector().detect(img)


def test_blue_hexagon_detected():
    dets = _detect((255, 0, 0), _hexagon(960, 352, 120))
    assert any(d.color == TargetColor.BLUE and d.shape == "HEXAGON" for d in dets), \
        "mavi altıgen tespit edilmeli"


def test_red_triangle_detected():
    dets = _detect((0, 0, 255), _triangle(960, 352, 140))
    assert any(d.color == TargetColor.RED and d.shape == "TRIANGLE" for d in dets), \
        "kırmızı üçgen tespit edilmeli"


def test_blue_triangle_not_detected():
    dets = _detect((255, 0, 0), _triangle(960, 352, 140))
    assert not dets, "BLUE+TRIANGLE ÜRETİLMEMELİ (görev dışı kombinasyon)"


def test_red_hexagon_not_detected():
    dets = _detect((0, 0, 255), _hexagon(960, 352, 120))
    assert not dets, "RED+HEXAGON ÜRETİLMEMELİ (görev dışı kombinasyon)"


def test_mixed_frame_only_valid_targets():
    """Aynı frame'de hem geçerli hem geçersiz şekiller → yalnızca geçerliler üretilir."""
    img = np.zeros((H, W, 3), dtype=np.uint8)
    cv2.fillPoly(img, [_hexagon(700, 352, 100)], (255, 0, 0))    # mavi altıgen ✓
    cv2.fillPoly(img, [_triangle(1200, 352, 120)], (255, 0, 0))  # mavi üçgen ✗
    dets = TargetDetector().detect(img)
    assert all((d.color, d.shape) in {
        (TargetColor.BLUE, "HEXAGON"), (TargetColor.RED, "TRIANGLE"),
    } for d in dets)
    assert any(d.color == TargetColor.BLUE and d.shape == "HEXAGON" for d in dets)
