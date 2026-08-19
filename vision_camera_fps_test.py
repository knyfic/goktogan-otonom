"""
Kamera yakalama FPS testi (REAL, headless) — GÖRÜNTÜ İŞLEME OLMADAN.

TEK AMAÇ: RPi Camera Module 3'ün MEVCUT kamera ayarlarıyla (config.py) gerçekte
kaç FPS teslim ettiğini ölçmek. Vision (detect) devreye GİRMEDEN önceki kademeyi
test eder — sorun kamera katmanında mı, görüntü işlemede mi ayırt etmek için.

Ölçümler:
  • latest() çağrı sayısı (döngü hızı)
  • BENZERSİZ frame sayısı → GERÇEK yakalama FPS'i (kamera katmanının teslim hızı)
  • ardışık yeni frame aralığı (ort / min / max)
  • frame boyutu/format doğrulaması (beklenen 1920x1080x3)

İsteğe bağlı --fps sweep: 15,30,50,60 gibi değerleri tek komutla test edip
kameranın hangi FPS'e kadar çıkabildiğini tablo halinde verir (config'i DEĞİŞTİRMEZ;
her değer için kamera o FPS ile yeniden açılır).

GÜVENLİK: flight.* / payload.* / state_machine import edilmez; MAVLink yok;
hareket komutu yok; pencere yok.

KULLANIM (Raspberry Pi'de):
  python3 vision_camera_fps_test.py                 # config'teki REAL_CAMERA_FPS ile
  python3 vision_camera_fps_test.py --duration 10
  python3 vision_camera_fps_test.py --fps 50        # 50 FPS denemesi
  python3 vision_camera_fps_test.py --fps 15,30,50,60   # sweep — tablo
"""
from __future__ import annotations

import argparse
import logging
import os
import sys
import time

import numpy as np
import yaml

from config import (
    CAMERA_HEIGHT_PX,
    CAMERA_ORIENTATION,
    CAMERA_WIDTH_PX,
    REAL_CAMERA_FPS,
    RPI_CAMERA_NUM,
)
from utils.logging import setup_logging
from vision.rpi_camera import RPiCam3

_PAYLOAD_CONFIG = os.path.join(os.path.dirname(__file__), "payload", "config.yaml")

# Benzersiz-frame tespiti için örneklenen bölge sayısı (tam kopya karşılaştırma
# yerine ucuz bir özet — yeni frame gelip gelmediğini ayırt etmek yeterli).
_SAMPLE_REGIONS = (
    lambda f: (f[0, 0]),
    lambda f: (f[0, -1]),
    lambda f: (f[-1, 0]),
    lambda f: (f[len(f) // 2, len(f[0]) // 2]),
    lambda f: (f[len(f) // 2, 0]),
)


def _frame_hash(frame: np.ndarray) -> int:
    """Birkaç küçük bölgenin toplamından ucuz frame özeti (tam kopya yok)."""
    h = 0
    for region in _SAMPLE_REGIONS:
        h = h * 31 + int(region(frame).sum())
    return h


def measure_camera(camera, duration_s: float, sleep_s: float = 0.001) -> dict:
    """Kameradan duration_s süre frame topla; istatistikleri döndür.

    latest() aynı frame'i döndürebilir (yakalama döngüden yavaşsa) — BENZERSİZ
    frame sayısı kamera katmanının GERÇEK teslim hızını verir.
    """
    deadline = time.monotonic() + duration_s
    calls = 0
    unique = 0
    last_hash: int | None = None
    intervals: list[float] = []
    last_unique_t: float | None = None
    shape: tuple | None = None

    while time.monotonic() < deadline:
        frame = camera.latest()
        calls += 1
        if frame is not None:
            if shape is None:
                shape = frame.shape
            h = _frame_hash(frame)
            if h != last_hash:
                unique += 1
                now = time.monotonic()
                if last_unique_t is not None:
                    intervals.append(now - last_unique_t)
                last_unique_t = now
                last_hash = h
        time.sleep(sleep_s)

    fps = unique / max(duration_s, 1e-6)
    return {
        "duration": duration_s,
        "calls": calls,
        "unique": unique,
        "fps": fps,
        "shape": shape,
        "intervals": intervals,
        "stalled": unique == 0,
    }


def _build_camera(fps: int, logger: logging.Logger):
    try:
        with open(_PAYLOAD_CONFIG, encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
    except OSError:
        cfg = {}
    cam_cfg = cfg.get("camera", {}) or {}
    return RPiCam3(
        camera_num=RPI_CAMERA_NUM,
        width=CAMERA_WIDTH_PX,
        height=CAMERA_HEIGHT_PX,
        fps=max(int(fps), 1),
        preview=False,  # headless
        window_name="İHA Kamera",
        logger=logger,
        orientation=str(cam_cfg.get("orientation", CAMERA_ORIENTATION)),
    )


def _fmt_interval(intervals: list[float]) -> str:
    if not intervals:
        return "-"
    avg_ms = sum(intervals) / len(intervals) * 1000.0
    mn = min(intervals) * 1000.0
    mx = max(intervals) * 1000.0
    return f"{avg_ms:.1f} ms (min {mn:.1f} / max {mx:.1f})"


def main() -> int:
    parser = argparse.ArgumentParser(description="Kamera yakalama FPS testi (işleme YOK)")
    parser.add_argument("--duration", type=float, default=5.0, help="her ölçüm süresi (sn)")
    parser.add_argument(
        "--fps", default=None,
        help="istenen FPS (tek değer veya virgüllü liste: 15,30,50,60). "
             "Verilmezse config'teki REAL_CAMERA_FPS kullanılır.",
    )
    args = parser.parse_args()

    logger = setup_logging()
    logger.info("KAMERA FPS TESTİ — işleme YOK (yalnızca yakalama katmanı).")
    logger.info("MAVLink YOK. Flight control YOK. Payload YOK.")

    if args.fps:
        values = [int(v.strip()) for v in args.fps.split(",") if v.strip()]
    else:
        values = [REAL_CAMERA_FPS]
    if not values:
        parser.error("--fps boş")

    print(f"\nKamera: {CAMERA_WIDTH_PX}x{CAMERA_HEIGHT_PX} @ istenen {values} FPS "
          f"(CAM{RPI_CAMERA_NUM}) | süre: {args.duration:.0f} sn/ölçüm\n")
    print(f"{'İstenen FPS':>11} | {'Gerçek FPS':>10} | {'Benzersiz':>9} | "
          f"{'Ort aralık':>22} | Durum")
    print("-" * 78)

    results = []
    for fps in values:
        camera = None
        try:
            camera = _build_camera(fps, logger)
            stats = measure_camera(camera, args.duration)
            results.append((fps, stats))
            shape_ok = stats["shape"] == (CAMERA_HEIGHT_PX, CAMERA_WIDTH_PX, 3)
            if stats["stalled"]:
                status = "⚠ YAKALAMA YOK (frame gelmiyor)"
            elif shape_ok is False:
                status = f"⚠ BOYUT FARKLI: {stats['shape']}"
            elif stats["fps"] >= fps * 0.9:
                status = "OK"
            elif stats["fps"] >= fps * 0.6:
                status = "DÜŞÜK (~sınırda)"
            else:
                status = "⚠ ÇOK DÜŞÜK"
            print(f"{fps:>11} | {stats['fps']:>10.1f} | {stats['unique']:>9} | "
                  f"{_fmt_interval(stats['intervals']):>22} | {status}")
        except Exception as exc:
            logger.error("FPS=%d ölçümü başarısız: %s", fps, exc)
            print(f"{fps:>11} | {'-':>10} | {'-':>9} | {'-':>22} | HATA")
        finally:
            if camera is not None:
                try:
                    camera.close_preview()
                except Exception:
                    pass
            time.sleep(0.5)  # sensör değişimi arası kısa bekleme

    print("-" * 78)
    print("\nNot: 'Gerçek FPS' = BENZERSİZ frame teslim hızı (kamera katmanı).")
    print("Eğer görüntü işleme ile (vision_record/stream) aynı değeri göremiyorsan,")
    print("darboğaz kamerada değil, İŞLEME katmanındadır.\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
