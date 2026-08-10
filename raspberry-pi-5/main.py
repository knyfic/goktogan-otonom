"""
Raspberry Pi 5 — Payload + Kamera Servisleri Ana Giriş Noktası.

Çalıştırma:
  cd raspberry-pi-5/
  python main.py

Gereksinimler (Payload):
  pip install lgpio PyYAML

Gereksinimler (Kamera — HIL testi için GERÇEK RPi Camera Module 3):
  apt install python3-picamera2   # Raspbian'da ön kurulu gelir, yoksa bu komut
  Kamera: CAM / DISP port 0 (CSI 0) — config/payload.yaml camera_server.camera_num

Servisler:
  • TCP 5700: Payload komut servisi (Ubuntu → RPi GPIO)
  • TCP 5701: Kamera akış servisi   (Ubuntu mission → RPi Camera Module 3 JPEG akışı)
    camera_server.enable: false ise bu servis başlatılmaz (eski HIL uyumluluğu)
"""
from __future__ import annotations

import logging
import signal
import sys
import threading
from pathlib import Path

import yaml

# raspberry-pi-5/ kök dizini sys.path'e ekle
sys.path.insert(0, str(Path(__file__).parent))

from camera_service.camera import RPiCam3
from camera_service.server import CameraTCPServer
from payload_service.dispatcher import CommandDispatcher
from payload_service.local_backend import LocalRPiBackend
from payload_service.server import PayloadTCPServer

# ── Logging ──────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)-8s | %(name)s | %(message)s",
)
logger = logging.getLogger("payload_service")


def main() -> int:
    config_path = Path(__file__).parent / "config" / "payload.yaml"
    logger.info("Konfigürasyon: %s", config_path)

    cfg = yaml.safe_load(config_path.read_text())
    server_cfg = cfg.get("server", {})
    host = server_cfg.get("host", "0.0.0.0")
    port = server_cfg.get("port", 5700)

    # ── Payload Backend ────────────────────────────────────────────────────
    backend = LocalRPiBackend.from_config(
        config_path=str(config_path),
        logger=logger,
    )
    res = backend.initialize()
    if not res.get("success"):
        logger.error("Backend initialize başarısız: %s", res.get("message"))
        return 1
    logger.info("Payload backend hazır.")

    # ── Payload Dispatcher + Server ────────────────────────────────────────
    dispatcher = CommandDispatcher(backend=backend, logger=logger)
    payload_server = PayloadTCPServer(
        host=host, port=port, dispatcher=dispatcher, logger=logger,
    )

    # ── (İsteğe bağlı) Kamera Servisi (RPi Camera Module 3) ───────────────
    camera_cfg = cfg.get("camera_server", {})
    camera_enabled = bool(camera_cfg.get("enable", False))
    rpi_cam: RPiCam3 | None = None
    camera_server: CameraTCPServer | None = None
    camera_thread: threading.Thread | None = None

    if camera_enabled:
        cam_port = int(camera_cfg.get("port", 5701))
        cam_host = camera_cfg.get("host", "0.0.0.0")
        # swap_rb_channels: YAML'da yoksa None (otomatik strateji), yoksa True/False
        swap_raw = camera_cfg.get("swap_rb_channels", None)
        if swap_raw is None:
            cam_swap: bool | None = None
        else:
            cam_swap = bool(swap_raw)
        try:
            rpi_cam = RPiCam3(
                camera_num=int(camera_cfg.get("camera_num", 0)),
                width=int(camera_cfg.get("width", 1920)),
                height=int(camera_cfg.get("height", 1080)),
                fps=int(camera_cfg.get("fps", 30)),
                swap_rb_channels=cam_swap,
                logger=logger,
                orientation=str(camera_cfg.get("orientation", "NORMAL")),
            )
            camera_server = CameraTCPServer(
                host=cam_host,
                port=cam_port,
                camera=rpi_cam,
                jpeg_quality=int(camera_cfg.get("jpeg_quality", 80)),
                logger=logger,
            )
            camera_thread = threading.Thread(
                target=camera_server.start,
                name="camera-tcp-server",
                daemon=True,
            )
            camera_thread.start()
            logger.info(
                "Kamera servisi başlatıldı (RPi Camera Module 3) %s:%d — thread: %s",
                cam_host, cam_port, "stub" if rpi_cam.is_stub else "donanım",
            )
        except Exception as exc:
            logger.warning(
                "Kamera servisi AÇILAMADI (devam ediyor — payload servisi çalışır): %s",
                exc,
            )
            camera_enabled = False
            rpi_cam = None
            camera_server = None
            camera_thread = None

    # ── Güvenli kapanış ────────────────────────────────────────────────────
    def _shutdown(sig, frame):
        logger.warning("Sinyal %s alındı — güvenli kapatılıyor...", sig)
        # Önce payload server durdur (blocking main thread'de çalışıyor)
        try:
            payload_server.stop()
        except Exception:
            pass
        # Sonra kamera servisi
        if camera_server is not None:
            try:
                camera_server.stop()
            except Exception:
                pass
        if rpi_cam is not None:
            try:
                rpi_cam.close()
            except Exception:
                pass
        # Donanım temizliği
        try:
            backend.cleanup()
        except Exception:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    # ── Başlat ─────────────────────────────────────────────────────────────
    logger.info("Payload TCP servisi başlıyor: %s:%d", host, port)
    try:
        payload_server.start()   # Blocking — ana thread
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt.")
    finally:
        # Yedek temizlik
        if camera_server is not None:
            try:
                camera_server.stop()
            except Exception:
                pass
        if rpi_cam is not None:
            try:
                rpi_cam.close()
            except Exception:
                pass
        backend.cleanup()

    return 0


if __name__ == "__main__":
    sys.exit(main())
