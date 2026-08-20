"""
color_mask modülü testleri — HSV mask + morphology (config-driven).

Senaryolar:
  * kırmızı düşük-H aralığı
  * kırmızı yüksek-H aralığı
  * kırmızı wrap-around OR (iki aralık birleşimi)
  * mavi mask
  * düşük saturation zemin reddi
  * morphology (gürültü temizliği / delik doldurma)
"""
import numpy as np
import pytest

from config import VISION_MORPHOLOGY, VISION_TARGETS
from vision.color_mask import ColorMask


def _hsv_of(img_bgr: np.ndarray) -> np.ndarray:
    import cv2
    return cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)


def _img(color_bgr, h=96, w=128) -> np.ndarray:
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:, :] = color_bgr
    return img


_RED = (0, 0, 255)
_BLUE = (255, 0, 0)
_GRAY = (128, 128, 128)

_RED_CFG = VISION_TARGETS["RED_TRIANGLE"]
_BLUE_CFG = VISION_TARGETS["BLUE_HEXAGON"]


def _red_mask():
    return ColorMask(_RED_CFG["hsv_ranges"], VISION_MORPHOLOGY)


def _blue_mask():
    return ColorMask(_BLUE_CFG["hsv_ranges"], VISION_MORPHOLOGY)


def test_red_low_h_range_masked():
    """Kırmızı H:[0,10] aralığı maskelenir (düşük-H kırmızı)."""
    mask = _red_mask().apply(_hsv_of(_img(_RED)))
    assert mask.mean() > 200, "kırmızı frame maskelenmeli"


def test_red_high_h_range_masked():
    """Kırmızı H:[160,179] aralığı maskelenir (yüksek-H kırmızı)."""
    # Morumsu-kırmızı BGR(128,0,255): R baskın, B orta → H ≈ 165 (yüksek bant).
    img = _img((128, 0, 255))
    mask = _red_mask().apply(_hsv_of(img))
    assert mask.mean() > 150, "yüksek-H kırmızı maskelenmeli"


def test_red_wrap_around_or():
    """İki kırmızı bant OR'lanır — her iki kırmızı da tek maske üretir."""
    m1 = _red_mask().apply(_hsv_of(_img(_RED)))
    # İki farklı kırmızı tonu yan yana: düşük-H (saf kırmızı) + yüksek-H (morumsu).
    img = np.zeros((64, 128, 3), dtype=np.uint8)
    img[:, :64] = (0, 0, 255)      # H≈0
    img[:, 64:] = (128, 0, 255)    # H≈165
    m2 = _red_mask().apply(_hsv_of(img))
    # Her iki yarı da maskelenmeli (bantlar birleşik çalışır).
    assert m2[:, :64].mean() > 200
    assert m2[:, 64:].mean() > 150
    assert m1 is not None


def test_blue_mask():
    mask = _blue_mask().apply(_hsv_of(_img(_BLUE)))
    assert mask.mean() > 200, "mavi frame maskelenmeli"


def test_blue_does_not_mask_red():
    """Mavi maske kırmızıyı kapsamamalı (renk ayrımı)."""
    mask = _blue_mask().apply(_hsv_of(_img(_RED)))
    assert mask.mean() < 10, "mavi maske kırmızıya girmemeli"


def test_red_does_not_mask_blue():
    mask = _red_mask().apply(_hsv_of(_img(_BLUE)))
    assert mask.mean() < 10, "kırmızı maske maviye girmemeli"


def test_low_saturation_ground_rejected():
    """Düşük saturation zemin (gri/beton) maskelenmez (S_min config'ten)."""
    for bgr in (_GRAY, (100, 100, 100), (60, 60, 60)):
        m_red = _red_mask().apply(_hsv_of(_img(bgr)))
        m_blue = _blue_mask().apply(_hsv_of(_img(bgr)))
        assert m_red.mean() < 10, f"gri {bgr} kırmızıya girmemeli"
        assert m_blue.mean() < 10, f"gri {bgr} maviye girmemeli"


def test_morphology_closes_small_holes():
    """CLOSE: içinde delik olan hedef bölge doldurulur."""
    img = _img(_RED)
    # Ortasına küçük (4x4) delik aç — CLOSE(5) doldurur.
    img[44:48, 62:66] = (0, 0, 0)
    mask = _red_mask().apply(_hsv_of(img))
    # Delik bölgesi artık maskeli (CLOSE doldurdu).
    assert mask[44:48, 62:66].mean() > 100, "CLOSE deliği doldurmalı"


def test_morphology_removes_isolated_noise():
    """OPEN: yalıtık gürültü noktaları temizlenir."""
    img = _img(np.zeros(3, dtype=np.uint8))  # siyah zemin
    img[10, 10] = _RED  # tek piksel kırmızı gürültü
    img[50:60, 70:80] = _RED  # gerçek bölge
    mask = _red_mask().apply(_hsv_of(img))
    assert mask[10, 10] == 0, "tek piksel gürültü OPEN ile temizlenmeli"
    assert mask[50:60, 70:80].mean() > 200, "gerçek bölge korunmalı"


def test_morphology_config_from_config_file():
    """Morphology parametreleri config'ten okunur (sıfır magic number)."""
    assert VISION_MORPHOLOGY["kernel_size"] >= 1
    m = ColorMask(_RED_CFG["hsv_ranges"], VISION_MORPHOLOGY)
    assert m._kernel_size == VISION_MORPHOLOGY["kernel_size"]
    assert m._open_iters == VISION_MORPHOLOGY["open_iterations"]
    assert m._close_iters == VISION_MORPHOLOGY["close_iterations"]
