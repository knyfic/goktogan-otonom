#!/usr/bin/env python3
"""
Gerçek Pi + Camera Module 3: display frame (vision + HUD) → GStreamer zinciri.

Mission'ın REAL mod akışını birebir izler:

  kamera (1920x1080)
    → VisionCropCamera (1920x704 — payload mekanizması kırpılır)
    → HUD/annotation (state_machine._draw_hud — mission ile AYNI çizim)
    → display frame (PREVIEW_DISPLAY_WIDTH_PX'e küçültülür — mission ile aynı)
    → StreamCamera.show(display) → VideoStreamer (GStreamer UDP/H.264)

Doğrular:
  • streamer'a verilen frame == display frame  (np.array_equal)
  • display frame != ham kamera frame          (işlenmiş içerik yayında)
  • loopback alıcı (udpsrc → decode → appsink) uçtan uca frame çözer

  python tests/hardware/test_video_display_stream.py [--duration 5] [--loopback]

pytest: normal suite'te ASLA gerçek donanıma bağlanmaz —
  IHA_HW_VIDEO_TEST=1 python -m pytest tests/hardware/test_video_display_stream.py -v
"""
from __future__ import annotations

import argparse
import logging
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest
_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import pytest

from tests.hardware import hw_common as hw

# Normal pytest suite'i asla gerçek donanıma bağlanmamalı:
# Raspberry Pi + açık env var (IHA_HW_VIDEO_TEST=1) ikisi de şart.
pytestmark = pytest.mark.skipif(
    not (hw.is_raspberry_pi() and os.environ.get("IHA_HW_VIDEO_TEST") == "1"),
    reason="REAL donanım: Raspberry Pi + IHA_HW_VIDEO_TEST=1 gerekli",
)

_log = logging.getLogger("video_display_hw_test")


class _LoopbackReceiver:
    """Aynı makinede ground-station pipeline'ı (udpsrc → decode → appsink)."""

    def __init__(self, port: int, logger: logging.Logger | None = None) -> None:
        self._port = port
        self._count = 0
        self._stop_evt = threading.Event()
        self._thread: threading.Thread | None = None
        self._log = logger or _log

    def start(self) -> None:
        def _run() -> None:
            try:
                import gi
                gi.require_version("Gst", "1.0")
                from gi.repository import Gst  # type: ignore[import-not-found]
                Gst.init(None)

                pipe = Gst.parse_launch(
                    f'udpsrc port={self._port} '
                    'caps="application/x-rtp,media=video,encoding-name=H264,payload=96" '
                    "! rtph264depay ! h264parse ! avdec_h264 "
                    "! videoconvert ! video/x-raw,format=BGR "
                    "! appsink name=sink max-buffers=2 drop=true"
                )
                sink = pipe.get_by_name("sink")
                sink.set_property("emit-signals", True)

                def on_new(_sink):
                    sample = _sink.emit("pull-sample")
                    if sample is not None:
                        self._count += 1
                    return Gst.FlowReturn.OK

                sink.connect("new-sample", on_new)
                pipe.set_state(Gst.State.PLAYING)
                self._stop_evt.wait()
                pipe.set_state(Gst.State.NULL)
            except Exception as exc:
                self._log.warning("loopback receiver hatası: %s", exc)

        self._thread = threading.Thread(target=_run, daemon=True)
        self._thread.start()
        time.sleep(0.5)   # alıcının dinlemeye başlaması için

    def stop(self) -> None:
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=5)

    @property
    def count(self) -> int:
        return self._count


class _RecordingStreamer:
    """
    VideoStreamer sarmalayıcısı — push()'a VERİLEN frame'i kaydeder.
    Böylece GStreamer pipeline'ına gerçekte hangi frame'in girdiği kanıtlanır.
    """

    def __init__(self, streamer):
        self._streamer = streamer
        self.last_pushed = None
        self.push_calls = 0

    def push(self, frame) -> None:
        self.push_calls += 1
        self.last_pushed = frame.copy()
        self._streamer.push(frame)

    def stop(self) -> None:
        self._streamer.stop()

    @property
    def pushed_frames(self) -> int:
        return self._streamer.pushed_frames

    @property
    def error_count(self) -> int:
        return self._streamer.error_count


def _build_display_frame(frame):
    """Mission'ın _pump_preview 'pending yok' yoluyla AYNI display frame üretimi:
    küçült + HUD bandı çiz. HUD, state machine ile birebir aynı fonksiyondur."""
    from state_machine import MissionStateMachine

    display = MissionStateMachine._scale_for_display(frame)
    return MissionStateMachine._draw_hud(
        display,
        "HW DISPLAY STREAM TEST",
        "Alt --.-m  Hdg ---  Bat --.-V  (HUD + vision yayında)",
    )


def run_display_stream(
    duration_s: float,
    host: str,
    port: int,
    bitrate: int,
    fps: int,
    loopback: bool = False,
    logger: logging.Logger | None = None,
) -> tuple[bool, dict]:
    from config import (
        CAMERA_HEIGHT_PX,
        CAMERA_WIDTH_PX,
        PREVIEW_DISPLAY_WIDTH_PX,
        REAL_CAMERA_FPS,
        RPI_CAMERA_NUM,
        VISION_HEIGHT_PX,
    )
    from services.video_streamer import StreamCamera, VideoStreamer
    from vision.camera import VisionCropCamera
    from vision.rpi_camera import RPiCam3

    log = logger or _log
    cam_cfg = hw.load_payload_config().get("camera", {}) or {}
    orientation = str(cam_cfg.get("orientation", "NORMAL"))

    receiver: _LoopbackReceiver | None = None
    if loopback:
        receiver = _LoopbackReceiver(port, logger=log)
        receiver.start()
        host = "127.0.0.1"

    cam = RPiCam3(
        camera_num=RPI_CAMERA_NUM,
        width=CAMERA_WIDTH_PX,
        height=CAMERA_HEIGHT_PX,
        fps=REAL_CAMERA_FPS,
        orientation=orientation,
        logger=log,
    )
    if cam.is_stub:
        print("[FAIL] Camera Module 3 açılamadı (stub modda — picamera2 yok)")
        cam.close()
        return False, {}

    # Mission'ın REAL kurulumuyla birebir aynı sarmalama zinciri
    cropped = VisionCropCamera(cam, VISION_HEIGHT_PX)
    streamer = _RecordingStreamer(
        VideoStreamer(host=host, port=port, bitrate=bitrate, fps=fps, logger=log)
    )
    streamer._streamer.start()
    stream_cam = StreamCamera(cropped, streamer, logger=log)

    print(
        f"Display stream: {host}:{port}  display={PREVIEW_DISPLAY_WIDTH_PX}px genişlik "
        f"(kamera {CAMERA_WIDTH_PX}x{CAMERA_HEIGHT_PX} → crop {VISION_HEIGHT_PX} → display) "
        f"h264 {bitrate // 1000} kbps  fps-cap={fps}"
    )

    cpu_before = os.times()
    user_before, sys_before = cpu_before[0], cpu_before[1]

    eq_ok = 0
    processed_ok = 0
    first_disp_w = 0
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < duration_s:
            raw = cropped.latest()
            if raw is None:
                time.sleep(0.01)
                continue
            display = _build_display_frame(raw)
            if first_disp_w == 0:
                first_disp_w = display.shape[1]
            # Mission ile aynı: StreamCamera.show(display) → streamer.push(display)
            stream_cam.show(display)
            if streamer.last_pushed is not None:
                eq_ok += int(np.array_equal(streamer.last_pushed, display))
                processed_ok += int(not np.array_equal(display, raw))
            time.sleep(0.01)
        wall_s = time.monotonic() - t0
        cpu_after = os.times()
        cpu_s = (cpu_after[0] + cpu_after[1]) - (user_before + sys_before)

        stats = {
            "wall_s": wall_s,
            "pushed": streamer.pushed_frames,
            "push_calls": streamer.push_calls,
            "eq_ok": eq_ok,
            "processed_ok": processed_ok,
            "errors": streamer.error_count,
            "received": receiver.count if receiver else None,
            "display_w": first_disp_w,
            "cpu_s": cpu_s,
            "cpu_pct_single_core": (cpu_s / wall_s * 100.0) if wall_s > 0 else 0.0,
        }

        print(f"  süre:            {wall_s:.1f}s")
        print(f"  push edilen:     {stats['pushed']} frame")
        print(f"  streamer input == display frame:  {eq_ok}/{stats['push_calls']}")
        print(f"  display != raw (işlenmiş içerik): {processed_ok}/{stats['push_calls']}")
        print(f"  display genişliği: {first_disp_w}px")
        print(f"  push hata:       {stats['errors']}")
        print(f"  CPU:             {cpu_s:.2f}s ≈ {stats['cpu_pct_single_core']:.0f}% tek çekirdek")
        if receiver is not None:
            print(f"  loopback alıcı:  {receiver.count} frame çözüldü")
        print()
        print("Mission başlatılmadı. No flight commands were sent.")
        if stats["pushed"] == 0:
            print("[FAIL] hiç frame push edilemedi — GStreamer kurulu mu? (gi/Gst yok)")
        return (
            stats["errors"] == 0
            and stats["pushed"] > 0
            and eq_ok == stats["push_calls"]
            and processed_ok > 0
        ), stats
    finally:
        streamer.stop()
        cam.close()
        if receiver is not None:
            receiver.stop()


def main() -> int:
    ap = argparse.ArgumentParser(description="Display frame → GStreamer zincir testi (Pi)")
    ap.add_argument("--duration", type=float, default=5.0)
    ap.add_argument("--host", default=None, help="Varsayılan: config video_stream.host")
    ap.add_argument("--port", type=int, default=None, help="Varsayılan: config video_stream.port")
    ap.add_argument("--bitrate", type=int, default=None, help="bps (varsayılan: config)")
    ap.add_argument("--fps", type=int, default=None, help="FPS tavanı (varsayılan: config)")
    ap.add_argument("--loopback", action="store_true",
                    help="Aynı makinede alıcı ile uçtan uca ölçüm")
    args = ap.parse_args()

    cfg = hw.load_payload_config()
    vs = cfg.get("video_stream", {}) or {}
    host = args.host or vs.get("host", "127.0.0.1")
    port = args.port or int(vs.get("port", 5000))
    bitrate = args.bitrate or int(vs.get("bitrate", 2_000_000))
    fps = args.fps or int(vs.get("fps", 30))

    ok, _ = run_display_stream(
        args.duration, host, port, bitrate, fps, loopback=args.loopback,
        logger=_log,
    )
    return 0 if ok else 1


def test_video_display_stream_hardware():
    """Gerçek donanımda: display frame (HUD'lu) GStreamer'a aynen gidiyor ve
    loopback alıcı uçtan uca çözüyor."""
    cfg = hw.load_payload_config()
    vs = cfg.get("video_stream", {}) or {}
    port = int(vs.get("port", 5000))
    bitrate = int(vs.get("bitrate", 2_000_000))
    fps = int(vs.get("fps", 30))

    ok, stats = run_display_stream(
        duration_s=3.0, host="127.0.0.1", port=port,
        bitrate=bitrate, fps=fps, loopback=True, logger=_log,
    )
    assert ok, f"display stream zinciri başarısız: {stats}"
    assert stats["push_calls"] > 0, "hiç frame gönderilmedi"
    assert stats["eq_ok"] == stats["push_calls"], (
        "GStreamer'a giden frame, display frame'den farklı (np.array_equal başarısız)"
    )
    assert stats["processed_ok"] > 0, "işlenmiş (HUD'lu) frame kanıtlanamadı"
    assert stats["received"] is not None and stats["received"] > 0, (
        "loopback alıcı hiç frame çözemedi — RTP/decode zincirini kontrol et"
    )
    assert stats["errors"] == 0


if __name__ == "__main__":
    sys.exit(main())
