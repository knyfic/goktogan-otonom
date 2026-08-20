"""
vision_stream_only.py — yalnızca yayın scripti testleri.

Bu script MAVLink/payload import etmez; yalnızca kamera → tespit → HUD →
GStreamer yayın zinciri kurar. Testler HUD yardımcılarını ve tek-frame
pipeline'ını doğrular (uygulama döngüsü/donanım başlatılmaz).
"""
from unittest.mock import MagicMock

import numpy as np

from config import PREVIEW_DISPLAY_WIDTH_PX, VISION_HEIGHT_PX, VISION_WIDTH_PX
from models import TargetColor
from vision.target_detector import TargetDetector
from vision_stream_only import (
    _draw_hud,
    _scale_for_display,
    detection_summary,
    process_frame,
)


def _frame(w=VISION_WIDTH_PX, h=VISION_HEIGHT_PX):
    return np.zeros((h, w, 3), dtype=np.uint8)


def test_draw_hud_keeps_shape_and_adds_band():
    frame = _frame()
    out = _draw_hud(frame, "VISION-ONLY STREAM", "TESPIT: tespit yok")
    assert out.shape == frame.shape
    # Bant bölgesi (üst) opak gri ile karıştırılmış olmalı — piksel farkı var
    assert not np.array_equal(out[:20], frame[:20])


def test_scale_for_display_reduces_width():
    frame = _frame()
    out = _scale_for_display(frame)
    assert out.shape[1] == PREVIEW_DISPLAY_WIDTH_PX
    assert out.shape[0] == round(VISION_HEIGHT_PX * PREVIEW_DISPLAY_WIDTH_PX / VISION_WIDTH_PX)


def test_process_frame_blank_no_detections():
    """Boş frame → 'tespit yok', yayın frame'i küçültülmüş, tespit yok."""
    detector = TargetDetector()
    disp, detections, summary = process_frame(_frame(), detector)
    assert not detections
    assert summary == "tespit yok"
    assert disp.shape[1] == PREVIEW_DISPLAY_WIDTH_PX


def test_detection_summary_formats_targets():
    det = MagicMock()
    det.color = TargetColor.RED
    det.shape = "TRIANGLE"
    det.confidence = 0.92
    det2 = MagicMock()
    det2.color = TargetColor.BLUE
    det2.shape = "HEXAGON"
    det2.confidence = 0.87
    s = detection_summary([det, det2])
    assert "RED TRIANGLE 0.92" in s
    assert "BLUE HEXAGON 0.87" in s
    assert detection_summary([]) == "tespit yok"
