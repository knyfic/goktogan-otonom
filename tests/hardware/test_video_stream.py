#!/usr/bin/env python3
"""
Kamera + GStreamer video yayın testi (Raspberry Pi 5).

  python tests/hardware/test_video_stream.py [--duration 10] [--host IP] [--port 5000]
                                             [--loopback] [--bitrate 2000000] [--fps 30]

Kamera Module 3'ü açar, GStreamer pipeline'ını başlatır, birkaç saniye yayın yapar,
ardından ölçer:
  • gerçek yayın FPS'i (appsrc'ye verilen frame / süre)
  • encoder frame drop oranı (leaky queue)
  • push hata sayısı
  • CPU kullanımı (os.times — tek çekirdek ölçeği)
  • --loopback: aynı makinede gerçek alıcı (udpsrc → decode → appsink) ile
    uçtan uca ulaşan frame sayısı + kayıp oranı

MISSION BAŞLATMAZ — yalnızca kamera + stream. READ-ONLY (MAVLink'e dokunmaz).

pytest olarak da çalışır — Raspberry Pi dışında otomatik SKIP:
  python -m pytest tests/hardware/test_video_stream.py -v
"""
from __future__ import annotations

import argparse
import logging
import os
import sys
import threading
import time
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import pytest

from tests.hardware import hw_common as hw

pytestmark = pytest.mark.skipif(
    not hw.is_raspberry_pi(), reason="Raspberry Pi + Camera Module 3 gerekli"
)

_log = logging.getLogger("video_hw_test")


class _LoopbackReceiver:
    """
    Aynı makinede gerçek ground-station pipeline'ı (udpsrc → decode → appsink).
    Gönderilen frame'lerin kaçının uçtan uca çözüldüğünü sayar.
    """

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


def run_stream(
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
        CAMERA_ORIENTATION,
        CAMERA_WIDTH_PX,
        REAL_CAMERA_FPS,
        RPI_CAMERA_NUM,
    )
    from services.video_streamer import VideoStreamer
    from vision.rpi_camera import RPiCam3

    log = logger or _log
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
        orientation=CAMERA_ORIENTATION,
        logger=log,
    )
    if cam.is_stub:
        print("[FAIL] Camera Module 3 açılamadı (stub modda — picamera2 yok)")
        cam.close()
        return False, {}

    streamer = VideoStreamer(host=host, port=port, bitrate=bitrate, fps=fps, logger=log)
    streamer.start()

    # CPU ölçümü — os.times() yalnızca bu süreç (kamera thread + encoder dahil)
    cpu_before = os.times()
    user_before = cpu_before[0]
    sys_before = cpu_before[1]

    try:
        print(
            f"Yayın: {host}:{port}  {CAMERA_WIDTH_PX}x{CAMERA_HEIGHT_PX} "
            f"h264 {bitrate // 1000} kbps  fps-cap={fps}"
        )
        t0 = time.monotonic()
        loop_until = t0 + duration_s
        while time.monotonic() < loop_until:
            frame = cam.latest()
            if frame is not None:
                streamer.push(frame)
            time.sleep(0.01)
        wall_s = time.monotonic() - t0
        cpu_after = os.times()

        cpu_s = (cpu_after[0] + cpu_after[1]) - (user_before + sys_before)
        pushed = streamer.pushed_frames
        dropped = streamer.dropped_frames
        errors = streamer.error_count
        effective_fps = pushed / wall_s if wall_s > 0 else 0.0
        cpu_pct = (cpu_s / wall_s * 100.0) if wall_s > 0 else 0.0

        stats = {
            "wall_s": wall_s,
            "pushed": pushed,
            "dropped": dropped,
            "errors": errors,
            "effective_fps": effective_fps,
            "cpu_s": cpu_s,
            "cpu_pct_single_core": cpu_pct,
            "received": receiver.count if receiver else None,
        }

        print(f"  süre:           {wall_s:.1f}s")
        print(f"  push edilen:    {pushed} frame  →  {effective_fps:.1f} FPS (efektif)")
        print(f"  encoder drop:   {dropped} frame  ({dropped / max(pushed, 1) * 100:.1f}%)")
        print(f"  push hata:      {errors}")
        print(f"  CPU:            {cpu_s:.2f}s  ≈ {cpu_pct:.0f}% tek çekirdek ölçeği")
        if receiver is not None:
            loss = (1 - receiver.count / max(pushed, 1)) * 100.0
            print(
                f"  loopback alıcı: {receiver.count} frame çözüldü "
                f"({loss:.1f}% kayıp — ilk keyframe gecikmesi dahil)"
            )
        print()
        print("Mission başlatılmadı. No flight commands were sent.")
        if pushed == 0:
            print("[FAIL] hiç frame push edilemedi — GStreamer kurulu mu? (gi/Gst yok)")
        return (errors == 0 and pushed > 0), stats
    finally:
        streamer.stop()
        cam.close()
        if receiver is not None:
            receiver.stop()


def main() -> int:
    ap = argparse.ArgumentParser(description="Kamera + GStreamer yayın testi (Pi)")
    ap.add_argument("--duration", type=float, default=10.0)
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

    ok, _ = run_stream(
        args.duration, host, port, bitrate, fps, loopback=args.loopback,
        logger=_log,
    )
    return 0 if ok else 1


def test_video_stream_hardware():
    """Kamera + GStreamer (loopback): frame üretimi + uçtan uca alım + 0 hata."""
    cfg = hw.load_payload_config()
    vs = cfg.get("video_stream", {}) or {}
    port = int(vs.get("port", 5000))
    bitrate = int(vs.get("bitrate", 2_000_000))
    fps = int(vs.get("fps", 30))

    ok, stats = run_stream(
        duration_s=3.0, host="127.0.0.1", port=port,
        bitrate=bitrate, fps=fps, loopback=True, logger=_log,
    )
    assert ok, f"GStreamer push hatası: {stats}"
    assert stats["pushed"] > 0, "hiç frame push edilmedi"
    assert stats["received"] is not None and stats["received"] > 0, (
        "loopback alıcı hiç frame çözemedi — RTP/decode zincirini kontrol et"
    )
    assert stats["errors"] == 0


if __name__ == "__main__":
    sys.exit(main())
