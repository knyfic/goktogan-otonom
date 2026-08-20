"""
Detector sağlamlık testleri (false positive / yanlış sınıflandırma düzeltmesi).

Senaryolar:
  1. Tek başına mavi altıgen       → BLUE HEXAGON (kırmızı yok)
  2. Tek başına kırmızı üçgen      → RED TRIANGLE (mavi yok)
  3. İki hedef aynı görüntüde      → ikisi de bulunur
  4. Boş sahne                     → NO DETECTION
  5. Gürültülü görüntü             → NO DETECTION
  6. Kısmi görünen hedef           → imkânsız renk/şekil çifti üretilmez
  7. History/stability             → salınım stabilize olmaz; tip değişimi
                                     3 ardışık frame ister; tutarlı 3 frame yeter
  8. BGR doğruluğu                 → auto_swap kapalıyken renkler asla
                                     sistematik olarak takas edilmez
"""
import cv2
import numpy as np

from config import TARGET_DETECTOR_AUTO_SWAP
from models import Detection, TargetColor
from vision.target_detector import TargetDetector

W, H = 1920, 1080

_HEX = np.array(
    [[840, 375], [1080, 375], [1200, 540], [1080, 705], [840, 705], [720, 540]],
    dtype=np.int32,
)
_TRI = np.array([[960, 345], [750, 705], [1170, 705]], dtype=np.int32)


def _blue_hex_img():
    img = np.zeros((H, W, 3), dtype=np.uint8)
    cv2.fillPoly(img, [_HEX], (255, 0, 0))
    return img


def _red_tri_img():
    img = np.zeros((H, W, 3), dtype=np.uint8)
    cv2.fillPoly(img, [_TRI], (0, 0, 255))
    return img


def _mk(color: TargetColor, shape: str, cx: float = 960.0) -> Detection:
    return Detection(
        color=color, shape=shape, center_x=cx, center_y=540.0,
        area_px=5000.0, bbox_width_px=100.0, bbox_height_px=100.0,
        confidence=0.9, color_purity=0.95,
    )


# ── 1/2/3: tekli ve çiftli hedefler ───────────────────────────────────────── #
def test_single_blue_hexagon():
    dets = TargetDetector().detect(_blue_hex_img())
    assert any(d.color == TargetColor.BLUE and d.shape == "HEXAGON" for d in dets)
    assert not any(d.color == TargetColor.RED for d in dets), "mavi görüntüde kırmızı olmamalı"


def test_single_red_triangle():
    dets = TargetDetector().detect(_red_tri_img())
    assert any(d.color == TargetColor.RED and d.shape == "TRIANGLE" for d in dets)
    assert not any(d.color == TargetColor.BLUE for d in dets), "kırmızı görüntüde mavi olmamalı"


def test_two_targets_in_same_frame():
    img = np.zeros((H, W, 3), dtype=np.uint8)
    cv2.fillPoly(img, [_HEX], (255, 0, 0))
    cv2.fillPoly(img, [_TRI + np.array([-500, 0])], (0, 0, 255))
    dets = TargetDetector().detect(img)
    assert any(d.color == TargetColor.BLUE and d.shape == "HEXAGON" for d in dets)
    assert any(d.color == TargetColor.RED and d.shape == "TRIANGLE" for d in dets)


# ── 4/5: boş ve gürültülü sahne ──────────────────────────────────────────── #
def test_empty_scene_no_detection():
    img = np.zeros((H, W, 3), dtype=np.uint8)
    assert TargetDetector().detect(img) == []


def test_noise_scene_no_detection():
    rng = np.random.default_rng(42)
    img = rng.integers(0, 256, (H, W, 3), dtype=np.uint8)
    assert TargetDetector().detect(img) == []


# ── 6: kısmi hedef — imkânsız renk/şekil çifti üretilmez ─────────────────── #
def test_partial_target_no_wrong_color():
    """Kısmen görünen mavi hedef asla KIRMIZI üretmemeli (renk purity)."""
    # Kenarı kesik / 5 köşeli mavi blob
    partial = np.array([[840, 375], [1080, 375], [1200, 540], [1080, 600], [840, 600]],
                       dtype=np.int32)
    img = np.zeros((H, W, 3), dtype=np.uint8)
    cv2.fillPoly(img, [partial], (255, 0, 0))
    dets = TargetDetector().detect(img)
    assert not any(d.color == TargetColor.RED for d in dets), (
        "mavi kaynaklı kontur kırmızı olarak sınıflandırılamaz"
    )

    # Kısmen görünen kırmızı hedef asla MAVİ üretmemeli
    partial_r = np.array([[960, 400], [800, 705], [1170, 705]], dtype=np.int32)
    img = np.zeros((H, W, 3), dtype=np.uint8)
    cv2.fillPoly(img, [partial_r], (0, 0, 255))
    dets = TargetDetector().detect(img)
    assert not any(d.color == TargetColor.BLUE for d in dets), (
        "kırmızı kaynaklı kontur mavi olarak sınıflandırılamaz"
    )


def test_purity_rejects_mixed_color_blob():
    """İçi karışık renkli blob (purity düşük) kabul edilmemeli."""
    img = np.zeros((H, W, 3), dtype=np.uint8)
    # Yarı mavi yarı kırmızı altıgen — tek bir renk baskın değil
    pts = np.array(
        [[840, 375], [1080, 375], [1200, 540], [1080, 705], [840, 705], [720, 540]],
        dtype=np.int32,
    )
    cv2.fillPoly(img, [pts], (255, 0, 0))
    mask = np.zeros((H, W), dtype=np.uint8)
    cv2.fillPoly(mask, [pts[:4]], 255)  # üst yarı
    img[mask == 255] = (0, 0, 255)      # üst yarıyı kırmızı yap
    assert TargetDetector().detect(img) == []


# ── 6b: ışık değişiminde kayan tonlar doğru renge sınıflandırılır ────────── #
# Aynı fiziksel hedef ışık koşuluna göre HSV'de kayabilir: kırmızı turuncuya/
# pembeye, mavi buz/sky/koyu maviye dönebilir. Geniş bantlar bu kaymayı tolere
# etmeli; sarı/yeşil gibi ayrık tonlar ise DIŞARIDA kalmalı.
def _img_with(poly, bgr):
    img = np.zeros((H, W, 3), dtype=np.uint8)
    cv2.fillPoly(img, [poly], bgr)
    return img


def test_shifted_tones_classify_to_correct_color():
    hex_poly = _HEX - np.array([60, 0])
    tri_poly = _TRI + np.array([60, 0])
    for name, bgr, want in (
        ("turuncu üçgen", (0, 165, 255), TargetColor.RED),
        ("pembe üçgen", (180, 80, 255), TargetColor.RED),
        ("koyu kırmızı üçgen", (0, 0, 80), TargetColor.RED),
        ("buz mavisi altıgen", (200, 220, 120), TargetColor.BLUE),
        ("koyu mavi altıgen", (60, 0, 0), TargetColor.BLUE),
    ):
        shape = "TRIANGLE" if want == TargetColor.RED else "HEXAGON"
        dets = TargetDetector().detect(_img_with(tri_poly, bgr) if want == TargetColor.RED
                                       else _img_with(hex_poly, bgr))
        assert any(d.color == want and d.shape == shape for d in dets), \
            f"{name} -> {want} olarak bulunmalı"
        assert not any(d.color != want for d in dets), \
            f"{name} yanlış renge sınıflandırıldı"


def test_yellow_and_green_are_excluded():
    """Sarı/yeşil gibi ayrık tonlar mavi/kırmızı olarak algılanmamalı."""
    for name, bgr in (("sarı üçgen", (0, 255, 255)), ("yeşil üçgen", (0, 255, 0))):
        dets = TargetDetector().detect(_img_with(_TRI, bgr))
        assert not any(d.color in (TargetColor.RED, TargetColor.BLUE) for d in dets), \
            f"{name} mavi/kırmızı olarak algılanmamalı"


# ── 7: history / stability ───────────────────────────────────────────────── #
def test_oscillation_never_stabilizes():
    d = TargetDetector()
    seq = ["BLUE", "RED", "BLUE", "RED", "BLUE", "RED", "BLUE"]
    for i, color in enumerate(seq):
        dets = [_mk(TargetColor(color), "HEXAGON" if color == "BLUE" else "TRIANGLE")]
        assert d.stable_detection(dets) is None, f"salınım {i}. frame'de stabilize olmamalı"


def test_consistent_detection_stabilizes_after_3():
    d = TargetDetector()
    for i in range(4):
        dets = [_mk(TargetColor.BLUE, "HEXAGON")]
        stable = d.stable_detection(dets)
        if i < 2:
            assert stable is None, f"{i}. frame'de henüz stabilize olmamalı"
        else:
            assert stable is not None, f"{i}. frame'de stabilize olmalı"


def test_type_flip_requires_fresh_3_frames():
    d = TargetDetector()
    for _ in range(3):
        d.stable_detection([_mk(TargetColor.RED, "TRIANGLE")])
    for i in range(3):
        stable = d.stable_detection([_mk(TargetColor.BLUE, "HEXAGON")])
        if i < 2:
            assert stable is None, "tip değişimi sonrası 2. frame'de henüz stabilize olmamalı"
        else:
            assert stable is not None, "3 ardışık yeni tip frame'den sonra stabilize olmalı"


# ── 8: BGR doğruluğu — sistematik renk takası yok ────────────────────────── #
def test_no_systematic_color_swap_over_time():
    """auto_swap kapalıyken uzun süreli çalışmada renkler takas edilmemeli."""
    assert TARGET_DETECTOR_AUTO_SWAP is False, "detektör swap'ı varsayılan kapalı olmalı"

    blue_det = TargetDetector()
    for _ in range(15):
        dets = blue_det.detect(_blue_hex_img())
        assert any(d.color == TargetColor.BLUE and d.shape == "HEXAGON" for d in dets)
        assert not any(d.color == TargetColor.RED for d in dets), (
            "mavi hedef zamanla kırmızı olarak algılanmamalı"
        )

    red_det = TargetDetector()
    for _ in range(15):
        dets = red_det.detect(_red_tri_img())
        assert any(d.color == TargetColor.RED and d.shape == "TRIANGLE" for d in dets)
        assert not any(d.color == TargetColor.BLUE for d in dets), (
            "kırmızı hedef zamanla mavi olarak algılanmamalı"
        )


def test_purity_is_reported_in_detection():
    """Kabul edilen tespitlerde color_purity alanı doldurulmalı."""
    dets = TargetDetector().detect(_blue_hex_img())
    assert dets and dets[0].color_purity >= 0.5
