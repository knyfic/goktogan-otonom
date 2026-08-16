"""
Görüntü işleme + GStreamer yayın scripti (REAL, headless) — YALNIZCA YAYIN.

TEK AMAÇ:
  Raspberry Pi'deki gerçek RPi Camera Module 3'ten frame al → projenin mevcut
  görüntü işleme pipeline'ı (vision/target_detector.py) ile MAVİ ALTIGEN ve
  KIRMIZI ÜÇGEN tespit et → tespit overlay + HUD bandı çiz (state machine ile
  aynı UI) → GStreamer ile payload/config.yaml → video_stream.host'daki
  ground station'a yayınla.

KASITLI SINIRLAR (güvenlik — bu script UÇUŞ YAPMAZ):
  • MAVLink bağlantısı AÇILMAZ — flight.* / payload.* ASLA import edilmez;
    dolayısıyla arm / takeoff / goto / rtl gibi HİÇBİR hareket komutu
    gönderilemez.
  • Pencere açılmaz (headless) — görüntü yalnızca GStreamer'a gider.
  • Tespit sonucunda HİÇBİR aksiyon alınmaz; yalnızca kadraja çizilir.

KULLANIM (Raspberry Pi'de, İHA 11.5m'de Mission Planner ile HOLD'da iken):
  python3 vision_stream_only.py
  Durdurma: Ctrl+C

KONFIG:
  payload/config.yaml:
    camera.orientation   → fiziksel kamera montaj yönü (REAL: 'ROTATE_180')
    video_stream.enabled → true
    video_stream.host    → ground station IP (yayın hedefi)
    video_stream.port / bitrate / fps / preset
  config.py (tek kaynak — main.py ile aynı):
    CAMERA_WIDTH_PX / CAMERA_HEIGHT_PX / REAL_CAMERA_FPS / RPI_CAMERA_NUM
    VISION_HEIGHT_PX / PREVIEW_DISPLAY_WIDTH_PX
"""
from __future__ import annotations

import os
import sys
import time

import cv2
import numpy as np
import yaml

from config import (
    CAMERA_HEIGHT_PX,
    CAMERA_ORIENTATION,
    CAMERA_WIDTH_PX,
    PREVIEW_DISPLAY_WIDTH_PX,
    REAL_CAMERA_FPS,
    RPI_CAMERA_NUM,
    VISION_HEIGHT_PX,
)
from services.video_streamer import VideoStreamer
from utils.logging import setup_logging
from vision.camera import VisionCropCamera
from vision.rpi_camera import RPiCam3
from vision.target_detector import TargetDetector

_PAYLOAD_CONFIG = os.path.join(os.path.dirname(__file__), "payload", "config.yaml")


# --------------------------------------------------------------------------- #
#  HUD yardımcıları — state_machine.MissionStateMachine._draw_hud /
#  _scale_for_display ile BİREBİR aynı (UI tutarlılığı). state machine import
#  edilmez (flight/payload zincirini çekmemek için).
# --------------------------------------------------------------------------- #

def _draw_hud(
    frame: np.ndarray,
    line1: str,
    line2: str = "",
    line1_color: tuple = (0, 230, 0),
    line2_color: tuple = (0, 210, 255),
) -> np.ndarray:
    """Şeffaf HUD bandı çiz (state_machine._draw_hud ile aynı)."""
    out = frame.copy()
    h, w = out.shape[:2]

    ui_scale = w / 640.0
    band_h = int(56 * ui_scale)

    overlay = out.copy()
    cv2.rectangle(overlay, (0, 0), (w, band_h), (10, 10, 10), -1)
    cv2.addWeighted(overlay, 0.55, out, 0.45, 0, out)
    cv2.line(out, (0, band_h), (w, band_h), (60, 60, 60), 1)

    font_scale1 = 0.65 * ui_scale
    font_scale2 = 0.58 * ui_scale
    thickness1 = max(1, int(2 * ui_scale))
    thickness2 = max(1, int(1 * ui_scale))
    y1 = int(22 * ui_scale)
    y2 = int(46 * ui_scale)

    cv2.putText(out, line1, (10, y1), cv2.FONT_HERSHEY_SIMPLEX,
                font_scale1, line1_color, thickness1, cv2.LINE_AA)
    if line2:
        cv2.putText(out, line2, (10, y2), cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale2, line2_color, thickness2, cv2.LINE_AA)

    cx, cy = w // 2, h // 2
    marker_size = int(24 * ui_scale)
    cv2.drawMarker(out, (cx, cy), (255, 255, 255),
                   cv2.MARKER_CROSS, marker_size, max(1, int(ui_scale)), cv2.LINE_AA)
    return out


def _scale_for_display(frame: np.ndarray) -> np.ndarray:
    """Gösterim/yayın için frame'i PREVIEW_DISPLAY_WIDTH_PX'e küçült.

    Tespit TAM çözünürlükte yapılır; yalnızca yayın frame'i küçültülür
    (encoder yükü düşer — main.py REAL yayın yolu ile aynı).
    """
    h, w = frame.shape[:2]
    if w <= PREVIEW_DISPLAY_WIDTH_PX:
        return frame
    nw = PREVIEW_DISPLAY_WIDTH_PX
    nh = max(1, int(round(h * nw / w)))
    return cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_AREA)


def detection_summary(detections: list) -> str:
    """Tespit özeti — HUD line2 için (ör. 'RED TRIANGLE 0.92 | BLUE HEXAGON 0.87')."""
    if not detections:
        return "tespit yok"
    return "  ".join(
        f"{d.color.value} {d.shape} {d.confidence:.2f}" for d in detections[:3]
    )


def process_frame(frame_bgr: np.ndarray, detector: TargetDetector) -> tuple[np.ndarray, list, str]:
    """Tek frame: tespit → overlay → HUD → yayın frame'i.

    Dönen: (yayın frame'i, detections, tespit özeti) — tespit ASLA aksiyon
    üretmez, yalnızca görselleştirilir.
    """
    detections = detector.detect(frame_bgr)
    disp = TargetDetector.draw_detections(frame_bgr, detections)
    summary = detection_summary(detections)
    disp = _draw_hud(disp, "VISION-ONLY STREAM", f"TESPIT: {summary}")
    return _scale_for_display(disp), detections, summary


# --------------------------------------------------------------------------- #
#  Ana döngü
# --------------------------------------------------------------------------- #

def main() -> int:
    logger = setup_logging()
    logger.info("VISION-ONLY STREAM başlatılıyor "
                "(MAVLink YOK, hareket komutu YOK, pencere YOK).")

    try:
        with open(_PAYLOAD_CONFIG, encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
    except OSError as exc:
        logger.error("payload/config.yaml okunamadı: %s", exc)
        return 1

    logger.info("payload/config.yaml → backend=%s (bu script backend'den bağımsızdır)",
                cfg.get("backend"))

    vs = cfg.get("video_stream", {}) or {}
    host = str(vs.get("host", "") or "").strip()
    if not vs.get("enabled", False) or not host:
        logger.error(
            "Yayın hedefi yok — payload/config.yaml → video_stream.enabled: true "
            "ve video_stream.host: <ground station IP> ayarlanmalı."
        )
        return 1
    port = int(vs.get("port", 5000))

    # ── Kamera: RPi Camera Module 3 (REAL donanım, headless) ────────────────
    cam_cfg = cfg.get("camera", {}) or {}
    camera = RPiCam3(
        camera_num=RPI_CAMERA_NUM,
        width=CAMERA_WIDTH_PX,
        height=CAMERA_HEIGHT_PX,
        fps=REAL_CAMERA_FPS,
        preview=False,  # headless — pencere AÇILMAZ
        window_name="İHA Kamera",
        logger=logger,
        orientation=str(cam_cfg.get("orientation", CAMERA_ORIENTATION)),
    )
    camera = VisionCropCamera(camera, VISION_HEIGHT_PX)
    detector = TargetDetector()

    # ── GStreamer yayını (tek amaç) ─────────────────────────────────────────
    streamer = VideoStreamer(
        host=host,
        port=port,
        bitrate=int(vs.get("bitrate", 2_000_000)),
        codec=str(vs.get("codec", "h264")),
        fps=int(vs.get("fps", 30)),
        preset=str(vs.get("preset", "veryfast")),
        logger=logger,
    )
    if not streamer.start():
        logger.error("GStreamer başlatılamadı — yayın yapılamaz, çıkılıyor.")
        camera.close_preview()
        return 2

    logger.info(
        "Yayın hedefi udp://%s:%d | kamera %dx%d → vision %d satır | "
        "tespit: MAVİ ALTIGEN + KIRMIZI ÜÇGEN | yayın genişliği %d px",
        host, port, CAMERA_WIDTH_PX, CAMERA_HEIGHT_PX, VISION_HEIGHT_PX,
        PREVIEW_DISPLAY_WIDTH_PX,
    )

    frame_count = 0
    fps_window_start = time.monotonic()
    fps_display = 0.0
    status_t = time.monotonic()
    last_summary = "tespit yok"

    try:
        while True:
            frame = camera.latest()
            if frame is None:
                time.sleep(0.02)
                continue

            disp, _detections, summary = process_frame(frame, detector)
            streamer.push(disp)
            last_summary = summary

            frame_count += 1
            now = time.monotonic()
            if now - fps_window_start >= 1.0:
                fps_display = frame_count / (now - fps_window_start)
                frame_count = 0
                fps_window_start = now

            if now - status_t >= 5.0:
                status_t = now
                logger.info(
                    "yayın aktif: işleme fps≈%.0f | pushed=%d dropped=%d hata=%d | %s",
                    fps_display, streamer.pushed_frames, streamer.dropped_frames,
                    streamer.error_count, last_summary,
                )
            time.sleep(0.01)
    except KeyboardInterrupt:
        logger.info("Durduruldu (Ctrl+C).")
    finally:
        streamer.stop()
        camera.close_preview()

    return 0


if __name__ == "__main__":
    sys.exit(main())
