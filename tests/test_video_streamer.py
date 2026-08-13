"""
VideoStreamer + StreamCamera testleri (Aşama 3 — REAL mode GStreamer).

GStreamer (gi) devrede olsa da olmasa da:
  - start() asla raise ETMEZ (başarısızsa enabled=False — görev devam eder)
  - push() asla raise ETMEZ (kapalıysa no-op)
  - stop() idempotent

StreamCamera, kamera arayüzünü (latest/show/pump/close_preview) korur ve
show() → streamer.push() aktarımını sağlar.
"""
import logging
from unittest.mock import MagicMock

import numpy as np

from services.video_streamer import StreamCamera, VideoStreamer

_log = logging.getLogger("test_video_stream")


# ── 1) VideoStreamer güvenli yaşam döngüsü ────────────────────────────────── #
def test_video_streamer_start_stop_never_raises():
    """GStreamer kurulu olsa bile (x264enc eksik vb.) asla exception yok."""
    s = VideoStreamer(host="127.0.0.1", port=5000, bitrate=2_000_000, fps=30, logger=_log)
    s.start()      # başarısız olabilir — ama raise etmemeli
    s.stop()       # idempotent
    s.stop()       # ikinci çağrı da güvenli


def test_video_streamer_push_noop_when_disabled():
    """start() çağrılmadan push → no-op (hata yok)."""
    s = VideoStreamer(host="127.0.0.1", port=5000, bitrate=2_000_000, fps=30, logger=_log)
    frame = np.zeros((540, 960, 3), dtype=np.uint8)
    s.push(frame)
    s.push(frame)
    s.stop()


def test_video_streamer_push_after_failed_start():
    """start() başarısız olsa bile push() güvenli — görev bloklanmaz."""
    s = VideoStreamer(host="127.0.0.1", port=5000, bitrate=2_000_000, fps=30, logger=_log)
    s.start()
    s.push(np.zeros((540, 960, 3), dtype=np.uint8))
    s.stop()


def test_video_streamer_unknown_codec_disabled():
    """Desteklenmeyen kodc → akış devre dışı, asla raise yok."""
    s = VideoStreamer(host="127.0.0.1", port=5000, bitrate=2_000_000,
                      codec="v4l2h264", fps=30, logger=_log)
    s.start()
    s.push(np.zeros((64, 64, 3), dtype=np.uint8))
    assert s.enabled is False
    s.stop()


def test_video_streamer_preset_normalization():
    """Geçersiz preset → veryfast fallback; geçerli preset korunur."""
    s = VideoStreamer(host="127.0.0.1", port=5000, bitrate=2_000_000, fps=30,
                      preset="BOGUS", logger=_log)
    assert s._preset == "veryfast"
    s2 = VideoStreamer(host="127.0.0.1", port=5000, bitrate=2_000_000, fps=30,
                       preset="faster", logger=_log)
    assert s2._preset == "faster"


def test_video_streamer_optimized_pipeline_builds():
    """Optimize edilmiş pipeline (veryfast + vbv + leaky=upstream) kurulabiliyor.
    GStreamer yoksa enabled=False kalır — asla raise etmez."""
    s = VideoStreamer(host="127.0.0.1", port=5000, bitrate=2_000_000, fps=30,
                      preset="veryfast", logger=_log)
    s.start()
    s.push(np.zeros((540, 960, 3), dtype=np.uint8))
    # GStreamer kuruluysa pipeline kurulur ve frame push edilir
    if s.enabled:
        assert s.pushed_frames >= 1
        assert s.error_count == 0
    s.stop()


def test_video_streamer_stats_properties():
    """pushed/dropped/error sayaçları — kapalıyken push no-op ve sayaçlar 0."""
    s = VideoStreamer(host="127.0.0.1", port=5000, bitrate=2_000_000, fps=30, logger=_log)
    assert s.pushed_frames == 0
    assert s.dropped_frames == 0
    assert s.error_count == 0

    frame = np.zeros((64, 64, 3), dtype=np.uint8)
    s.push(frame)               # start() çağrılmadı → no-op
    assert s.pushed_frames == 0
    assert s.dropped_frames == 0
    s.stop()


# ── 2) StreamCamera arayüzü ────────────────────────────────────────────────── #
class _FakeCamera:
    def __init__(self):
        self.closed = False
        self.latest_calls = 0
        self.shown = 0

    def latest(self):
        self.latest_calls += 1
        return np.zeros((540, 960, 3), dtype=np.uint8)

    def show(self, frame):
        self.shown += 1

    def pump(self):
        pass

    def close_preview(self):
        self.closed = True


def test_stream_camera_delegates_and_stops_streamer():
    inner = _FakeCamera()
    streamer = MagicMock()
    cam = StreamCamera(inner, streamer, logger=_log)

    frame = cam.latest()
    assert frame is not None
    assert inner.latest_calls == 1

    cam.show(frame)
    streamer.push.assert_called_once()

    cam.pump()                       # no-op, hata yok
    cam.close_preview()
    streamer.stop.assert_called_once()
    assert inner.closed is True


def test_stream_camera_show_survives_streamer_failure():
    """streamer.push raise etse bile kamera davranışı bozulmamalı."""
    inner = _FakeCamera()
    streamer = MagicMock()
    streamer.push.side_effect = RuntimeError("gstreamer öldü")
    cam = StreamCamera(inner, streamer, logger=_log)

    frame = cam.latest()
    cam.show(frame)                  # push hatası → sadece log, kamera çalışır
    assert inner.shown == 1          # iç kameranın show()'u yine çağrıldı


def test_stream_camera_close_preview_survives_streamer_failure():
    inner = _FakeCamera()
    streamer = MagicMock()
    streamer.stop.side_effect = RuntimeError("gstreamer öldü")
    cam = StreamCamera(inner, streamer, logger=_log)

    cam.close_preview()              # stop hatası → yine de kamera kapanır
    assert inner.closed is True
