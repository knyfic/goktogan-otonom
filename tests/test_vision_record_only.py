"""
vision_record_only.py — yalnızca kayıt (SD kart) scripti testleri.

Bu script MAVLink/flight/payload import etmez; yalnızca kamera → tespit →
HUD → MP4 yazıcı zinciri kurar. Testler bağımsız parçaları doğrular
(uygulama döngüsü/donanım başlatılmaz; encoder gerçek cv2 ile denenir).
"""
import datetime
import sys
from pathlib import Path

import numpy as np
import pytest

from config import VISION_RECORDING_WIDTH_PX
from models import TargetColor
from vision.target_detector import TargetDetector
import vision_record_only as vr


def test_module_does_not_import_flight_payload_state_machine():
    """Güvenlik: recorder flight/payload/state_machine/pymavlink çekmemeli."""
    import vision_record_only  # noqa: F401  (modül zaten import edildi)
    for forbidden in ("flight", "payload", "state_machine", "pymavlink"):
        assert forbidden not in sys.modules, \
            f"vision_record_only {forbidden!r} import ediyor — uçuş kontrolü yasak"


def test_recording_path_creates_dir_and_timestamp(tmp_path):
    now = datetime.datetime(2026, 8, 19, 14, 50, 32)
    p = vr.recording_path(tmp_path, now=now)
    assert p.parent == tmp_path
    assert p.name == "vision_2026-08-19_14-50-32.mp4"
    assert p.parent.exists()
    # Aynı isim üzerine yazmama: her çağrı yeni timestamp üretir
    p2 = vr.recording_path(tmp_path, now=now + datetime.timedelta(seconds=5))
    assert p2.name != p.name


def test_recording_path_relative_to_project_root(tmp_path, monkeypatch):
    monkeypatch.setattr(vr, "__file__", str(tmp_path / "vision_record_only.py"))
    p = vr.recording_path("recordings", now=datetime.datetime(2026, 1, 1, 0, 0, 0))
    assert p.parent == tmp_path / "recordings"


def test_scale_for_display_uses_recording_width():
    frame = np.zeros((704, 1920, 3), dtype=np.uint8)
    out = vr._scale_for_display(frame)
    assert out.shape[1] == VISION_RECORDING_WIDTH_PX
    assert out.shape[0] == round(704 * VISION_RECORDING_WIDTH_PX / 1920)


def test_process_frame_blank_no_detections():
    detector = TargetDetector()
    disp, detections, summary = vr.process_frame(
        np.zeros((704, 1920, 3), dtype=np.uint8), detector
    )
    assert not detections
    assert summary == "tespit yok"
    assert disp.shape[1] == VISION_RECORDING_WIDTH_PX


def test_detection_summary_formats_targets():
    det = _det(TargetColor.RED, "TRIANGLE", 0.92)
    det2 = _det(TargetColor.BLUE, "HEXAGON", 0.87)
    s = vr.detection_summary([det, det2])
    assert "RED TRIANGLE 0.92" in s
    assert "BLUE HEXAGON 0.87" in s
    assert vr.detection_summary([]) == "tespit yok"


def test_open_writer_real_encoder_writes_mp4(tmp_path):
    """Gerçek cv2 encoder (host'ta avc1/H.264): açılır, frame yazar, finalize eder."""
    out = tmp_path / "real.mp4"
    writer, tag = vr.open_writer(out, 320, 240, 10)
    assert writer.isOpened()
    assert tag in ("avc1", "h264", "x264", "mp4v")
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    writer.write(frame)
    writer.release()
    assert out.exists() and out.stat().st_size > 0, "MP4 finalize edilmeli"

def test_open_writer_falls_back_to_mp4v(monkeypatch, tmp_path):
    """avc1/h264/x264 açılamazsa mp4v'e düşer — fourcc zinciri testi."""
    calls: list = []

    class FakeWriter:
        def __init__(self, path, fourcc, fps, size):
            calls.append((path, fourcc, fps, size))
            self._opened = len(calls) >= 4  # ilk 3 aday başarısız, 4. (mp4v) açılır

        def isOpened(self):
            return self._opened

        def release(self):
            pass

    monkeypatch.setattr(vr.cv2, "VideoWriter", FakeWriter)
    writer, tag = vr.open_writer(tmp_path / "x.mp4", 960, 352, 30)
    assert tag == "mp4v"
    assert len(calls) == 4
    path, fourcc, fps, size = calls[-1]
    assert size == (960, 352)
    assert fps == 30
    writer.release()


def _det(color, shape, conf):
    from unittest.mock import MagicMock
    d = MagicMock()
    d.color = color
    d.shape = shape
    d.confidence = conf
    return d


def test_open_writer_raises_when_none_open(monkeypatch, tmp_path):
    class FakeWriter:
        def __init__(self, *a, **k):
            pass

        def isOpened(self):
            return False

        def release(self):
            pass

    monkeypatch.setattr(vr.cv2, "VideoWriter", FakeWriter)
    with pytest.raises(RuntimeError):
        vr.open_writer(tmp_path / "x.mp4", 960, 352, 30)
