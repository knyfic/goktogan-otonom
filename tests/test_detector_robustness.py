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
import pytest

from config import TARGET_DETECTOR_AUTO_SWAP
from models import Detection, TargetColor
from vision.target_detector import TargetDetector

W, H = 1920, 1080

_HEX = np.array(
    [[720, 540], [840, 332], [1080, 332], [1200, 540], [1080, 748], [840, 748]],
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


# ── 9: Jagged maske (saha koşulu) sağlamlığı ─────────────────────────────── #
# Gerçek kamera maskeleri testere dişi (jagged) kontur üretir: piksel kayması
# circularity'yi çökertebilir ve approxPolyDP köşe sayısını bozabilir.
# Geometri temizlenmiş (birleştirilmiş) poligondan hesaplandığı için ±1-2px
# kontur bozukluğu hedef tespitini öldürmemeli.

def _jitter_mask(mask: np.ndarray, jit: int, seed: int) -> np.ndarray:
    """Maske piksellerini ±jit kadar kaydırarak jagged kontur üretir."""
    rng = np.random.default_rng(seed)
    out = np.zeros_like(mask)
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return out
    dx = rng.integers(-jit, jit + 1, len(xs))
    dy = rng.integers(-jit, jit + 1, len(ys))
    out[np.clip(ys + dy, 0, mask.shape[0] - 1),
        np.clip(xs + dx, 0, mask.shape[1] - 1)] = 255
    return out


def _jittered_mask_through_shape_detector(img: np.ndarray, color: str, jit: int, seed: int) -> list:
    """Maske seviyesinde jagged kontur üretip shape katmanını test eder.

    TargetDetector.frame alır; kontur bozukluğunu doğrudan maske üzerinde
    simüle etmek için shape katmanı + renk maskesi birlikte kullanılır
    (pipeline'ın shape katmanının birebir aynısı). Giriş frame'i kamera
    katmanının ürettiği gibi 1920×704 varsayılır.
    """
    from config import (
        VISION_MORPHOLOGY,
        VISION_SHAPE_SCORING,
        VISION_TARGETS,
        VISION_VERTEX_EPSILONS,
    )
    from vision.color_mask import ColorMask
    from vision.shape_detector import ShapeDetector

    tgt = VISION_TARGETS[color]
    work = cv2.resize(img, (960, 352), interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(work, cv2.COLOR_BGR2HSV)
    mask = ColorMask(tgt["hsv_ranges"], VISION_MORPHOLOGY).apply(hsv)
    mask = _jitter_mask(mask, jit, seed)
    shape = ShapeDetector(
        VISION_VERTEX_EPSILONS, VISION_SHAPE_SCORING, tgt,
        shape_name="TRIANGLE" if color == "RED_TRIANGLE" else "HEXAGON",
    )
    return shape.detect(mask)


# Gerçek boyut hedefler (11.5m'de): üçgen kenar ≈128px, altıgen kenar ≈258px @1920.
_JIT_TRI = np.array([[960, 130], [832, 352], [1088, 352]], dtype=np.int32)  # eşkenar (taban 256)
_JIT_HEX = np.array(
    [[720, 352], [840, 144], [1080, 144], [1200, 352], [1080, 560], [840, 560]],
    dtype=np.int32,
)  # düzgün altıgen (kenar 240)


def _jitter_red_tri_img() -> np.ndarray:
    img = np.zeros((704, 1920, 3), dtype=np.uint8)
    cv2.fillPoly(img, [_JIT_TRI], (0, 0, 255))
    return img


def _jitter_blue_hex_img() -> np.ndarray:
    img = np.zeros((704, 1920, 3), dtype=np.uint8)
    cv2.fillPoly(img, [_JIT_HEX], (255, 0, 0))
    return img


@pytest.mark.parametrize("seed", range(4))
def test_jittered_real_triangle_still_detected(seed):
    """Gerçek boyut eşkenar üçgen, jagged maske altında kaybolmamalı."""
    cands = _jittered_mask_through_shape_detector(
        _jitter_red_tri_img(), "RED_TRIANGLE", jit=2, seed=seed
    )
    assert cands, f"jagged üçgen tespit edilemedi (seed={seed})"
    assert cands[0].vertex_count == 3


@pytest.mark.parametrize("seed", range(4))
def test_jittered_real_hexagon_still_detected(seed):
    """Gerçek boyut düzgün altıgen, jagged maske altında kaybolmamalı."""
    cands = _jittered_mask_through_shape_detector(
        _jitter_blue_hex_img(), "BLUE_HEXAGON", jit=2, seed=seed
    )
    assert cands, f"jagged altıgen tespit edilemedi (seed={seed})"
    assert 5 <= cands[0].vertex_count <= 7


@pytest.mark.parametrize("seed", range(4))
def test_jittered_square_still_rejected(seed):
    """Jagged kare decoy hâlâ hedef sayılmamalı (vertex/açı kapıları)."""
    img = np.zeros((704, 1920, 3), dtype=np.uint8)
    cv2.rectangle(img, (850, 240), (1070, 460), (255, 0, 0), -1)  # mavi kare
    cands = _jittered_mask_through_shape_detector(
        img, "BLUE_HEXAGON", jit=2, seed=seed
    )
    assert not any(c.shape == "HEXAGON" for c in cands), f"jagged kare kaçtı (seed={seed})"
