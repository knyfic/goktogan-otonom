"""
Renk maskesi GÖRSEL kayıt scripti (REAL, headless) — TEŞHİS AMAÇLI.

AMAC:
  "Üçgen algılanıyor ama altıgen algılanamıyor" sorununu GÖRSEL olarak teşhis
  etmek. Tespit zincirinin EN BAŞINDAKİ adım olan RENK MASKESİNİN gerçekte ne
  ürettiğini gösterir:

    kamera (1920x1080)
      → VisionCropCamera (960x352 — mission ile birebir aynı)
      → LAB-CLAHE → HSV (TargetDetector._preprocess_for_hsv — aynı)
      → MAVİ maske + KIRMIZI maske (TargetDetector._color_mask + _clean_mask)

  Her kare [ ORİJİNAL | MAVİ MASKE | KIRMIZI MASKE ] yan yana, maskeler
  SİYAH-BEYAZ olarak recordings/ dizinine MP4 kaydedilir. Ayrıca her karede
  maske doluluk oranı (ham + temizlenmiş piksel sayısı) ve tespit özeti
  loglanır — videoyu izlemeden bile maskenin tepki verip vermediği görülür.

NASIL OKUNUR:
  • Altıgenin olduğu yerde MAVİ MASKE paneli BEYAZ görünüyorsa  → maske çalışıyor,
    sorun şekil aşamasında (VISION_DEBUG=1 ile REJECT loglarına bak).
  • MAVİ MASKE paneli SİYAH (boş) ise → maske kontur üretmiyor; sorun renk/ışıkta.
  • MAVİ panelde beyaz AMA şekil bozuk (yamuk/kısmi) ise → morfoloji/ışık
    maskeyi bozuyor.

KULLANIM (Raspberry Pi'de, kameraya altıgeni göstererek):
  python3 vision_mask_record.py                # maskeleri kaydet + logla
  VISION_DEBUG=1 python3 vision_mask_record.py # ayrıca her adayın REJECT logu

  Durdurma: Ctrl+C
  ÇIKTI: recordings/vision_mask_<timestamp>.mp4  (3 panel → 1440x176)
"""
from __future__ import annotations

import logging
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
    REAL_CAMERA_FPS,
    RPI_CAMERA_NUM,
    VISION_CROP_HEIGHT_PX,
    VISION_HEIGHT_PX,
    VISION_RECORDING_BITRATE,
    VISION_RECORDING_DIR,
    VISION_WIDTH_PX,
)
from models import TargetColor
from utils.logging import setup_logging
from vision.camera import VisionCropCamera
from vision.rpi_camera import RPiCam3
from vision.target_detector import TargetDetector
from vision_record_only import (
    _draw_hud,
    detection_summary,
    open_writer,
    recording_path,
)

_PAYLOAD_CONFIG = os.path.join(os.path.dirname(__file__), "payload", "config.yaml")

# Panel düzeni: [ORİJİNAL | MAVİ MASKE | KIRMIZI MASKE] — her panel bu genişlikte
PANEL_W = 480


def _mask_panel(mask: np.ndarray, width_px: int = PANEL_W) -> np.ndarray:
    """Tek kanal maskeyi siyah-beyaz BGR panele ölçekler."""
    bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    h, w = bgr.shape[:2]
    nh = max(1, int(round(h * width_px / w)))
    return cv2.resize(bgr, (width_px, nh), interpolation=cv2.INTER_AREA)


def compose_frame(
    frame_bgr: np.ndarray,
    blue_mask: np.ndarray,
    red_mask: np.ndarray,
    detections: list,
    stats: str,
) -> np.ndarray:
    """[ORİJİNAL | MAVİ MASKE | KIRMIZI MASKE] yan yana kayıt frame'i üretir."""
    h, w = frame_bgr.shape[:2]
    target_h = max(1, int(round(h * PANEL_W / w)))

    # Orijinal panel: tespit kutuları + özet (tespit varsa görünür)
    panel_orig = TargetDetector.draw_detections(frame_bgr, detections)
    panel_orig = cv2.resize(panel_orig, (PANEL_W, target_h),
                            interpolation=cv2.INTER_AREA)

    p_blue = _mask_panel(blue_mask)
    p_red = _mask_panel(red_mask)

    panel_orig = _draw_hud(panel_orig, "ORİJİNAL", stats)
    p_blue = _draw_hud(p_blue, "MAVİ MASKE",
                       f"doluluk {int(blue_mask.sum() / 255)} px")
    p_red = _draw_hud(p_red, "KIRMIZI MASKE",
                      f"doluluk {int(red_mask.sum() / 255)} px")

    return cv2.hconcat([panel_orig, p_blue, p_red])


def main() -> int:
    logger = setup_logging()
    logger.info("RENK MASKESİ KAYIT başlatılıyor (teşhis — uçuş komutu YOK).")

    try:
        with open(_PAYLOAD_CONFIG, encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
    except OSError as exc:
        logger.error("payload/config.yaml okunamadı: %s", exc)
        return 1
    logger.info("payload/config.yaml → backend=%s (bu script backend'den bağımsızdır)",
                cfg.get("backend"))

    # ── Kamera: mission ile BİREBİR aynı zincir ────────────────────────────
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
    camera = VisionCropCamera(camera, VISION_CROP_HEIGHT_PX)
    detector = TargetDetector()

    # ── Kayıt hedefi: recordings/vision_mask_<timestamp>.mp4 ───────────────
    out_path = recording_path(prefix="vision_mask")
    target_h = max(1, int(round(VISION_HEIGHT_PX * PANEL_W / VISION_WIDTH_PX)))
    writer, fourcc = open_writer(
        out_path,
        PANEL_W * 3,
        target_h,
        REAL_CAMERA_FPS,
        bitrate=VISION_RECORDING_BITRATE,
        logger=logger,
    )

    logger.info(
        "Kamera: %dx%d @ %d FPS | vision: %dx%d | maskeler: MAVİ + KIRMIZI | "
        "kayıt: %s (%dx%d) | codec: %s",
        CAMERA_WIDTH_PX, CAMERA_HEIGHT_PX, REAL_CAMERA_FPS,
        VISION_WIDTH_PX, VISION_HEIGHT_PX,
        out_path, PANEL_W * 3, target_h, fourcc,
    )

    frame_count = 0
    status_t = time.monotonic()
    fps_window_start = time.monotonic()
    last_summary = "tespit yok"
    try:
        while True:
            if not writer.isOpened():
                logger.error("Video encoder kapandı — kayıt durduruluyor.")
                break

            frame = camera.latest()
            if frame is None:
                time.sleep(0.02)
                continue

            # ── Maske zinciri: detektörün kullandığı adımların AYNISI ──────
            hsv = TargetDetector._preprocess_for_hsv(frame)
            blue_raw = TargetDetector._color_mask(hsv, TargetColor.BLUE)
            blue = TargetDetector._clean_mask(blue_raw)
            red_raw = TargetDetector._color_mask(hsv, TargetColor.RED)
            red = TargetDetector._clean_mask(red_raw)

            detections = detector.detect(frame)
            summary = detection_summary(detections)

            n_blue_raw = int(blue_raw.sum() / 255)
            n_blue = int(blue.sum() / 255)
            n_red_raw = int(red_raw.sum() / 255)
            n_red = int(red.sum() / 255)

            stats = (f"mavi {n_blue_raw}->{n_blue}px  kırmızı {n_red_raw}->{n_red}px"
                     f"  {summary}")
            disp = compose_frame(frame, blue, red, detections, stats)
            writer.write(disp)
            frame_count += 1

            now = time.monotonic()
            if now - status_t >= 2.0:
                fps_display = frame_count / (now - fps_window_start)
                frame_count = 0
                fps_window_start = now
                logger.info(
                    "[maske] mavi_raw=%d mavi_clean=%d | kırmızı_raw=%d "
                    "kırmızı_clean=%d | fps≈%.1f | %s",
                    n_blue_raw, n_blue, n_red_raw, n_red, fps_display, summary,
                )
                last_summary = summary
                status_t = now
            time.sleep(0.01)
    except KeyboardInterrupt:
        logger.info("Durduruldu (Ctrl+C).")
    finally:
        try:
            writer.release()
            dur_info = f" | son tespit: {last_summary}" if last_summary != "tespit yok" else ""
            size_mb = out_path.stat().st_size / 1e6 if out_path.exists() else 0.0
            logger.info("Kayıt tamam: %s | %.1f MB%s",
                        out_path.name, size_mb, dur_info)
        except Exception as exc:
            logger.warning("Video finalize hatası: %s", exc)
        camera.close_preview()

    return 0


if __name__ == "__main__":
    sys.exit(main())
