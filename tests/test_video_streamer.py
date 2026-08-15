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
    def __init__(self, frame=None):
        self.closed = False
        self.latest_calls = 0
        self.shown = 0
        self._frame = frame if frame is not None else np.zeros((540, 960, 3), dtype=np.uint8)

    def latest(self):
        self.latest_calls += 1
        return self._frame

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


# ── 3) TEK display frame → GStreamer (cv2.imshow ile aynı kaynak) ─────────── #
class _RecordingStreamer:
    """GStreamer yerine geçen kaydedici — push edilen frame'leri saklar."""

    def __init__(self):
        self.frames: list = []
        self.stopped = False

    def push(self, frame):
        self.frames.append(frame.copy())

    def stop(self):
        self.stopped = True

    @property
    def last_frame(self):
        return self.frames[-1] if self.frames else None


def _preview_sm(camera, pending=None):
    """_pump_preview için minimal state machine örneği (ağır __init__ yok)."""
    from state_machine import MissionStateMachine

    sm = object.__new__(MissionStateMachine)
    sm.camera = camera
    sm._pending_display = pending
    sm._mission_timeout_triggered = False
    sm.log = logging.getLogger("t")
    return sm


def test_pump_preview_streams_exact_display_frame():
    """
    Kritik garanti: _pump_preview'in ürettiği display frame (görüntü işleme +
    HUD sonrası) streamer'a BİREBİR gider — np.array_equal True olmalı.
    """
    import cv2 as _cv2
    from config import PREVIEW_DISPLAY_WIDTH_PX

    inner = _FakeCamera()
    recorder = _RecordingStreamer()
    cam = StreamCamera(inner, recorder, logger=_log)

    display = np.zeros((176, PREVIEW_DISPLAY_WIDTH_PX, 3), dtype=np.uint8)
    _cv2.rectangle(display, (20, 20), (120, 120), (255, 255, 255), -1)  # bilinen marker

    sm = _preview_sm(cam, pending=display)
    sm._pump_preview()

    assert recorder.last_frame is not None, "streamer'a frame gitmedi"
    assert np.array_equal(recorder.last_frame, display), \
        "streamer'a giden frame, display frame'in aynısı olmalı (np.array_equal)"


def test_streamed_frame_is_processed_not_raw():
    """Ham frame değil, İŞLENMİŞ (marker'lı) frame yayınlanır."""
    import cv2 as _cv2
    from config import PREVIEW_DISPLAY_WIDTH_PX

    raw = np.zeros((176, PREVIEW_DISPLAY_WIDTH_PX, 3), dtype=np.uint8)
    display = raw.copy()
    _cv2.rectangle(display, (10, 10), (90, 90), (0, 0, 255), -1)  # detection/bbox taklidi

    inner = _FakeCamera()
    recorder = _RecordingStreamer()
    cam = StreamCamera(inner, recorder, logger=_log)
    sm = _preview_sm(cam, pending=display)
    sm._pump_preview()

    assert recorder.last_frame is not None
    assert not np.array_equal(recorder.last_frame, raw), \
        "GStreamer ham kamerayı yayınlamamalı — işlenmiş frame yayınlanmalı"
    assert np.array_equal(recorder.last_frame, display), \
        "işlenmiş frame (marker dahil) streamer'a aynen gitmeli"


def test_pump_preview_hud_frame_streamed():
    """Pending display yokken HUD'lu frame yayınlanır (ham frame değil)."""
    from config import PREVIEW_DISPLAY_WIDTH_PX
    from unittest.mock import MagicMock

    raw = np.zeros((176, PREVIEW_DISPLAY_WIDTH_PX, 3), dtype=np.uint8)
    inner = _FakeCamera(frame=raw)
    recorder = _RecordingStreamer()
    cam = StreamCamera(inner, recorder, logger=_log)

    sm = _preview_sm(cam, pending=None)
    sm.nav = MagicMock()
    sm.nav.position.return_value = MagicMock(
        relative_altitude=8.0, heading_deg=90.0,
        latitude=41.0, longitude=28.9,
        battery_voltage_v=24.8, battery_percent=85,
    )
    sm.data = MagicMock()
    sm.data.state.value = "SEARCHING_FIRST_TARGET"
    sm._mission_start_t = None

    sm._pump_preview()

    assert recorder.last_frame is not None
    assert recorder.last_frame.shape == raw.shape
    assert not np.array_equal(recorder.last_frame, raw), \
        "HUD'lu frame yayınlanmalı (üst bant + telemetri) — ham değil"


def test_stream_camera_pushes_what_was_shown():
    """StreamCamera.show() ne gösterildiyse onu push eder (ayrı pipeline yok)."""
    from config import PREVIEW_DISPLAY_WIDTH_PX

    inner = _FakeCamera()
    recorder = _RecordingStreamer()
    cam = StreamCamera(inner, recorder, logger=_log)

    display = np.zeros((270, PREVIEW_DISPLAY_WIDTH_PX, 3), dtype=np.uint8)
    cam.show(display)

    assert recorder.last_frame is not None
    assert np.array_equal(recorder.last_frame, display)
    assert inner.shown == 1
