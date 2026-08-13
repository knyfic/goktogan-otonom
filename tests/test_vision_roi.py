"""
Vision ROI + koordinat sistemi testleri.

Kamera 1920x1080 kalır; vision pipeline yalnızca ÜST ROI'yi (1920x~704) işler.
Vision merkezi (960, ~352)'dir — full-frame merkezi (960,540) DEĞİL.

Doğrulananlar:
  1. Kamera çözünürlüğü değişmiyor (1920x1080).
  2. ROI genişliği 1920, yüksekliği fiziksel orandan (~704) türetiliyor.
  3. ROI (0,0) konumunda; alt sınır ≈ 704.
  4. Vision merkezi Y ≈ 352 (540 değil).
  5. ROI dışı (alt bölge) nesneler detector tarafından GÖRÜLMÜYOR (sentetik test).
  6. Alignment referansı vision merkezi.
  7. ROI ofseti iki kez uygulanmıyor (draw_detections roi_y tek ofset).
  8. HUD: ROI alt sınır çizgisi + vision merkez marker'ı doğru yerde.
"""
import cv2
import numpy as np
import pytest

from config import (
    CAMERA_HEIGHT_PX,
    CAMERA_WIDTH_PX,
    VISION_ACTIVE_RATIO,
    VISION_BLOCKED_MECHANISM_LENGTH,
    VISION_CENTER_X,
    VISION_CENTER_Y,
    VISION_ROI_HEIGHT,
    VISION_ROI_WIDTH,
    VISION_ROI_X,
    VISION_ROI_Y,
    VISION_VIEW_TOTAL_LENGTH,
)
from models import Detection, TargetColor
from vision.target_detector import TargetDetector

W, H = 1920, 1080


# ── 1-4) Config / ROI sabitleri ───────────────────────────────────────────── #
def test_camera_resolution_unchanged():
    assert CAMERA_WIDTH_PX == 1920
    assert CAMERA_HEIGHT_PX == 1080


def test_roi_config_constants():
    assert VISION_ROI_WIDTH == 1920
    assert VISION_ROI_X == 0
    assert VISION_ROI_Y == 0
    # Fiziksel oran: (11.5 - 4.0) / 11.5 ≈ 0.65217
    expected_ratio = (
        (VISION_VIEW_TOTAL_LENGTH - VISION_BLOCKED_MECHANISM_LENGTH)
        / VISION_VIEW_TOTAL_LENGTH
    )
    assert VISION_ACTIVE_RATIO == pytest.approx(expected_ratio)
    assert VISION_ACTIVE_RATIO == pytest.approx(7.5 / 11.5)
    # 1080 * 0.65217 ≈ 704.35 → 704
    assert VISION_ROI_HEIGHT == int(round(CAMERA_HEIGHT_PX * VISION_ACTIVE_RATIO))
    assert VISION_ROI_HEIGHT == 704


def test_vision_center_is_not_full_frame_center():
    assert VISION_CENTER_X == pytest.approx(960.0)
    assert VISION_CENTER_Y == pytest.approx(352.0)
    assert VISION_CENTER_Y != CAMERA_HEIGHT_PX / 2.0  # 540 DEĞİL
    assert VISION_CENTER_Y == pytest.approx(VISION_ROI_Y + VISION_ROI_HEIGHT / 2.0)


# ── 5) ROI dışı nesneler detector tarafından görülmez ─────────────────────── #
# Üst bölge (y < 704) ve alt bölge (y > 704) için ayrı mavi altıgenler
_HEX_TOP = np.array(
    [[840, 150], [1080, 150], [1200, 305], [1080, 460], [840, 460], [720, 305]],
    dtype=np.int32,
)
_HEX_BOTTOM = np.array(
    [[840, 780], [1080, 780], [1200, 905], [1080, 1030], [840, 1030], [720, 905]],
    dtype=np.int32,
)


def test_detector_ignores_bottom_region():
    img = np.zeros((H, W, 3), dtype=np.uint8)
    cv2.fillPoly(img, [_HEX_TOP], (255, 0, 0))
    cv2.fillPoly(img, [_HEX_BOTTOM], (255, 0, 0))

    # Tam frame: her iki bölgedeki hedef de görülür (alt bölge de DETECT edilir)
    full = TargetDetector().detect(img)
    blues = [d for d in full if d.color == TargetColor.BLUE]
    assert any(d.center_y <= VISION_ROI_HEIGHT for d in blues), "üst bölge hedefi görülmeli"
    assert any(d.center_y > VISION_ROI_HEIGHT for d in blues), "tam frame'de alt hedef görülmeli"

    # VISION ROI (üst 704 px): yalnızca üst bölge işlenir → alt hedef YOK
    roi_dets = TargetDetector().detect(img[:VISION_ROI_HEIGHT, :])
    assert any(d.center_y <= VISION_ROI_HEIGHT for d in roi_dets), "üst bölge hedefi ROI'de görülmeli"
    assert all(d.center_y <= VISION_ROI_HEIGHT for d in roi_dets), (
        "ROI'de alt bölge tespiti OLMAMALI — mekanizma alanı yok sayılmalı"
    )


def test_vision_roi_crop_shape():
    """state_machine._vision_roi: full frame → 1920x~704 aktif bölge."""
    from state_machine import MissionStateMachine

    sm = object.__new__(MissionStateMachine)
    frame = np.zeros((H, W, 3), dtype=np.uint8)
    roi = sm._vision_roi(frame)
    assert roi.shape == (VISION_ROI_HEIGHT, VISION_ROI_WIDTH, 3)


# ── 6) Alignment referansı vision merkezi ─────────────────────────────────── #
def test_alignment_reference_is_vision_center():
    from unittest.mock import MagicMock

    from flight.position_controller import PositionController
    from vision.geometry import CameraModel

    nav = MagicMock()
    nav.position.return_value = MagicMock(
        latitude=1.0, longitude=1.0, heading_deg=0.0, relative_altitude=1.0,
    )
    pc = PositionController(nav, CameraModel(1920, 1080, 66.0, 41.0))

    # Vision merkezindeki tespit → hizalı (komut gönderilmez)
    assert pc.align_detection(
        VISION_CENTER_X, VISION_CENTER_Y, VISION_ROI_WIDTH, VISION_ROI_HEIGHT, 1.0
    ) is True

    # Eski full-frame merkezi (960, 540) artık hizalı DEĞİL (dy=188px → komut)
    assert pc.align_detection(
        960.0, 540.0, VISION_ROI_WIDTH, VISION_ROI_HEIGHT, 1.0
    ) is False
    assert nav.goto_global.called


# ── 7) ROI ofseti tek kez uygulanır (draw_detections) ─────────────────────── #
def _mk_det(cx: float = 100.0, cy: float = 200.0) -> Detection:
    return Detection(
        color=TargetColor.BLUE, shape="HEXAGON", center_x=cx, center_y=cy,
        area_px=5000.0, bbox_width_px=100.0, bbox_height_px=100.0,
        confidence=0.9, color_purity=0.95,
    )


def test_draw_detections_roi_offset_applied_once():
    frame = np.zeros((H, W, 3), dtype=np.uint8)
    det = _mk_det()

    base = TargetDetector.draw_detections(frame, [det], roi_y=0)
    shifted = TargetDetector.draw_detections(frame, [det], roi_y=100)

    # roi_y=0 → çizim (100,200)'de; roi_y=100 → (100,300)'de (tek ofset, iki kez değil)
    assert tuple(base[200, 100]) != (0, 0, 0)
    assert tuple(base[300, 100]) == (0, 0, 0)
    assert tuple(shifted[300, 100]) != (0, 0, 0)
    assert tuple(shifted[200, 100]) == (0, 0, 0)


# ── 8) HUD: ROI sınırı + vision merkez marker'ı ───────────────────────────── #
def test_draw_hud_roi_border_and_vision_marker():
    from state_machine import MissionStateMachine

    frame = np.zeros((H, W, 3), dtype=np.uint8)
    out = MissionStateMachine._draw_hud(frame, "TEST", "alt")

    # ROI alt sınır çizgisi: tam ölçekte y=704'te sarı çizgi
    assert tuple(out[VISION_ROI_HEIGHT, 960]) != (0, 0, 0)
    # VISION merkez marker'ı (960, 352) civarında
    assert tuple(out[int(VISION_CENTER_Y), int(VISION_CENTER_X)]) != (0, 0, 0)
    # Eski full-frame merkezi (960, 540) artık marker DEĞİL
    assert tuple(out[540, 960]) == (0, 0, 0)
