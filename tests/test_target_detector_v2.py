"""
TargetDetector V2 testleri — gerçek uçuş senaryoları (1920×704 vision crop).

Kapsanan senaryolar (kullanıcı gereksinimleri):
  1.  Temiz kırmızı üçgen
  2.  Bulanık kırmızı üçgen (motion blur simülasyonu)
  3.  Küçük kırmızı üçgen (alçak çözünürlükteki hedef)
  4.  Kırmızı gürültü (saçılmış noktalar → tespit yok)
  5.  Kırmızı başka şekil (dikdörtgen → RED üretilmez)
  6.  Temiz mavi altıgen
  7.  Bulanık mavi altıgen
  8.  Küçük mavi altıgen
  9.  Mavi başka şekil (kare + daire + elips → BLUE HEXAGON üretilmez)
  10. İki hedef aynı frame'de (ayrık — bileşik maske ayrı kontur üretir)
  11. Hedef yok (boş sahne)
  12. Renk var, şekil yok (düzensiz blob → tespit yok)
  13. Şekil var, renk yanlış (gri/sarı üçgen → tespit yok)
  14. Frame-to-frame küçük hareket (hedef 3 frame'de kayar, hep bulunur)
  15. Kısa süreli detection kaybı (zayıf aday → temporal recovery)
  Ek: koordinatların ORİJİNAL çözünürlüğe geri ölçeklenmesi,
      altıgen yönelim çeşitleri (pointy-top / flat-top),
      reset_time_lock sonrası temporal belleğin temizlenmesi.
"""
import cv2
import numpy as np
import pytest

from config import MIN_DETECTION_CONFIDENCE
from models import Detection, TargetColor
from vision.target_detector import TargetDetector

# Gerçek vision crop çözünürlüğü (kamera 1920x1080 → kadraj 704 satır).
H, W = 704, 1920


# ── Frame üreticiler ──────────────────────────────────────────────────────── #

def _mk() -> np.ndarray:
    return np.zeros((H, W, 3), dtype=np.uint8)


def _flat_hex(cx=960, cy=460, s=200) -> np.ndarray:
    """Düzgün (eşkenar) flat-top altıgen: tüm kenarlar s."""
    return np.array(
        [
            [cx - s, cy],
            [cx - s // 2, cy - int(s * 0.866)],
            [cx + s // 2, cy - int(s * 0.866)],
            [cx + s, cy],
            [cx + s // 2, cy + int(s * 0.866)],
            [cx - s // 2, cy + int(s * 0.866)],
        ],
        dtype=np.int32,
    )


def _pointy_hex(cx=960, cy=420, s=200) -> np.ndarray:
    """Düzgün (eşkenar) pointy-top altıgen: tüm kenarlar s (30° dönük).

    Yükseklik 2s, genişlik √3·s (çevrel çember yarıçapı = kenar uzunluğu s).
    """
    h = int(s * 0.866)  # √3/2 · s
    return np.array(
        [
            [cx, cy - s],
            [cx + h, cy - s // 2],
            [cx + h, cy + s // 2],
            [cx, cy + s],
            [cx - h, cy + s // 2],
            [cx - h, cy - s // 2],
        ],
        dtype=np.int32,
    )


def _tri(cx=960, top=260, base=460, half=150) -> np.ndarray:
    return np.array([[cx, top], [cx - half, base], [cx + half, base]], dtype=np.int32)


def _blue_hex_img(pts=None) -> np.ndarray:
    img = _mk()
    cv2.fillPoly(img, [pts if pts is not None else _flat_hex()], (255, 0, 0))
    return img


def _red_tri_img(pts=None) -> np.ndarray:
    img = _mk()
    cv2.fillPoly(img, [pts if pts is not None else _tri()], (0, 0, 255))
    return img


# ── 1/2/3: kırmızı üçgen — temiz / bulanık / küçük ───────────────────────── #

def test_clean_red_triangle():
    dets = TargetDetector().detect(_red_tri_img())
    assert any(d.color == TargetColor.RED and d.shape == "TRIANGLE" for d in dets)


@pytest.mark.parametrize("k", [7, 11, 15])
def test_blurred_red_triangle(k):
    """Ağır blur (motion blur / odak kaybı) altında üçgen kaybolmamalı."""
    img = cv2.GaussianBlur(_red_tri_img(), (k, k), 0)
    dets = TargetDetector().detect(img)
    assert any(d.color == TargetColor.RED and d.shape == "TRIANGLE" for d in dets), \
        f"blur k={k} üçgeni kaybetti"


def test_small_red_triangle():
    """~100px bbox üçgen (960 ölçeğinde ~50px) bulunmalı."""
    img = _mk()
    cv2.fillPoly(img, [_tri(cx=960, top=330, base=420, half=55)], (0, 0, 255))
    dets = TargetDetector().detect(img)
    assert any(d.color == TargetColor.RED and d.shape == "TRIANGLE" for d in dets)


# ── 4/5: kırmızı gürültü ve yanlış şekil ──────────────────────────────────── #

def test_red_noise_no_detection():
    """Saçılmış kırmızı noktalar (gürültü) tespit üretmemeli."""
    img = _mk()
    rng = np.random.default_rng(7)
    pts = rng.integers(60, W - 60, (300, 2))
    for p in pts:
        cv2.circle(img, (int(p[0]), int(p[1])), int(rng.integers(1, 4)), (0, 0, 255), -1)
    assert TargetDetector().detect(img) == []


def test_red_rectangle_no_detection():
    """Kırmızı dikdörtgen RED TRIANGLE üretmemeli (extent/aspect eler)."""
    img = _mk()
    cv2.rectangle(img, (800, 300), (1120, 500), (0, 0, 255), -1)
    assert TargetDetector().detect(img) == []


# ── 6/7/8: mavi altıgen — temiz / bulanık / küçük ────────────────────────── #

def test_clean_blue_hexagon():
    dets = TargetDetector().detect(_blue_hex_img())
    assert any(d.color == TargetColor.BLUE and d.shape == "HEXAGON" for d in dets)


@pytest.mark.parametrize("k", [7, 11, 15])
def test_blurred_blue_hexagon(k):
    img = cv2.GaussianBlur(_blue_hex_img(), (k, k), 0)
    dets = TargetDetector().detect(img)
    assert any(d.color == TargetColor.BLUE and d.shape == "HEXAGON" for d in dets), \
        f"blur k={k} altıgeni kaybetti"


def test_small_blue_hexagon():
    """~110px bbox altıgen (960 ölçeğinde ~55px) bulunmalı."""
    img = _mk()
    cv2.fillPoly(img, [_flat_hex(cx=960, cy=460, s=55)], (255, 0, 0))
    dets = TargetDetector().detect(img)
    assert any(d.color == TargetColor.BLUE and d.shape == "HEXAGON" for d in dets)


def test_hexagon_orientation_variants():
    """Pointy-top ve flat-top altıgenlerin ikisi de bulunmalı (yönelim bağımsız)."""
    for pts in (_flat_hex(s=200), _pointy_hex(s=200)):
        dets = TargetDetector().detect(_blue_hex_img(pts))
        assert any(d.color == TargetColor.BLUE and d.shape == "HEXAGON" for d in dets)


# ── 9: mavi yanlış şekiller ───────────────────────────────────────────────── #

@pytest.mark.parametrize("shape", ["square", "circle", "ellipse"])
def test_blue_wrong_shapes_no_detection(shape):
    """Kare/daire/elips BLUE HEXAGON olarak algılanmamalı (yanlış pozitif yok)."""
    img = _mk()
    if shape == "square":
        cv2.rectangle(img, (860, 360), (1060, 560), (255, 0, 0), -1)
    elif shape == "circle":
        cv2.ellipse(img, (960, 460), (120, 120), 0, 0, 360, (255, 0, 0), -1)
    else:  # ellipse
        cv2.ellipse(img, (960, 460), (150, 100), 0, 0, 360, (255, 0, 0), -1)
    assert TargetDetector().detect(img) == [], f"mavi {shape} yanlış pozitif üretti"


# ── 10/11: iki hedef / boş sahne ──────────────────────────────────────────── #

def test_two_targets_same_frame():
    """Ayrık iki hedef: bileşik maske ayrı konturlar üretir, ikisi de bulunur."""
    img = _mk()
    cv2.fillPoly(img, [_flat_hex(cx=760, cy=500, s=150)], (255, 0, 0))
    cv2.fillPoly(img, [_tri(cx=1180, top=320, base=520, half=130)], (0, 0, 255))
    dets = TargetDetector().detect(img)
    assert any(d.color == TargetColor.BLUE and d.shape == "HEXAGON" for d in dets)
    assert any(d.color == TargetColor.RED and d.shape == "TRIANGLE" for d in dets)


def test_empty_scene_no_detection():
    assert TargetDetector().detect(_mk()) == []


# ── 12/13: renk-şekil ayrımı ──────────────────────────────────────────────── #

def test_color_without_shape_no_detection():
    """Düzensiz kırmızı blob (renk var, şekil yok) tespit üretmemeli."""
    img = _mk()
    blob = np.array(
        [[800, 400], [900, 350], [1020, 370], [1080, 440], [1000, 500],
         [880, 480], [820, 460]],
        dtype=np.int32,
    )
    cv2.fillPoly(img, [blob], (0, 0, 255))
    assert TargetDetector().detect(img) == []


@pytest.mark.parametrize("color_bgr", [(128, 128, 128), (0, 255, 255)])
def test_shape_with_wrong_color_no_detection(color_bgr):
    """Gri/sarı üçgen (şekil doğru, renk yanlış) tespit üretmemeli."""
    img = _mk()
    cv2.fillPoly(img, [_tri()], color_bgr)
    assert TargetDetector().detect(img) == []


# ── 14: frame-to-frame küçük hareket ──────────────────────────────────────── #

def test_frame_to_frame_small_motion():
    """Hedef frame'ler arasında kayarken her frame'de bulunmalı (takip stabil)."""
    d = TargetDetector()
    for dx in (0, 30, 60):  # 1920 ölçeğinde 30px kayma
        img = _red_tri_img(_tri(cx=960 + dx))
        dets = d.detect(img)
        assert any(
            x.color == TargetColor.RED and x.shape == "TRIANGLE" for x in dets
        ), f"dx={dx} frame'inde üçgen kayboldu"


# ── 15: temporal doğrulama — TemporalValidator modülünde test edilir ────── #
# (pipeline içi temporal DEFAULT KAPALI: state machine stable_detection zaten
# temporal tutarlılık sağlar; VISION_TEMPORAL.enabled=True ile açılabilir.)


# ── Ek: koordinat / çözünürlük doğrulaması ────────────────────────────────── #

def test_detection_coordinates_in_original_resolution():
    """Detection 960'ta yapılır ama çıktı 1920 çözünürlüğündedir (align uyumu).

    State machine center_x'i frame.shape[1] (1920) ile kullanır ve
    meters_per_pixel 1920 genişliğe göre hesaplanır — çıktı ölçeği kritiktir.
    """
    img = _red_tri_img(_tri(cx=960, top=260, base=460, half=150))
    dets = TargetDetector().detect(img)
    tri = next(d for d in dets if d.shape == "TRIANGLE")
    assert 900 <= tri.center_x <= 1020, f"center_x 1920 ölçeğinde olmalı: {tri.center_x}"
    assert 250 <= tri.bbox_width_px <= 320, \
        f"bbox genişliği 1920 ölçeğinde olmalı: {tri.bbox_width_px}"
    assert tri.center_y <= 470  # 704 satır içinde


def test_confidence_above_threshold():
    """Kabul edilen tespitlerin confidence'ı eşiğin üzerinde olmalı."""
    dets = TargetDetector().detect(_red_tri_img())
    assert dets
    for d in dets:
        assert d.confidence >= MIN_DETECTION_CONFIDENCE


def test_detect_runs_fast_on_full_resolution():
    """Smoke: 1920×704 frame'de detect makul sürede döner (< 100ms — 10+ FPS).

    Gerçek hedef ~30 FPS (≈33ms); eşik yalnızca regresyon guard'ıdır.
    """
    import time

    img = _blue_hex_img(_flat_hex(s=232))  # 2m kenar → 11m'deki gerçek boyut
    d = TargetDetector()
    for _ in range(3):
        d.detect(img)  # warmup
    t0 = time.perf_counter()
    for _ in range(10):
        d.detect(img)
    dt_ms = (time.perf_counter() - t0) / 10 * 1000
    assert dt_ms < 100, f"detect çok yavaş: {dt_ms:.1f} ms"


def test_color_purity_reported():
    """Kabul edilen tespitlerde color_purity doldurulmalı (interface)."""
    dets = TargetDetector().detect(_blue_hex_img())
    assert dets and dets[0].color_purity >= 0.5
