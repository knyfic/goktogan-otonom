"""
Raspberry Pi 5 — HIL Payload + Kamera Servisleri Giriş Noktası.

Bu script, eski raspberry-pi-5/main.py'nin yerini alır ve TEK config dosyasından
(payload/config.yaml) okur. HİÇBİR donanım implementasyonu içermez — hepsi ortak
kaynaklardan gelir:

  payload/backends/local_rpi.py   (TEK gerçek payload backend — REAL ile aynı)
  payload/hardware/*              (TEK donanım sürücüleri — REAL ile aynı)
  vision/rpi_camera.py            (TEK gerçek kamera — REAL ile aynı)

Servisler:
  • TCP 5700: Payload komut servisi (Ubuntu → RPi GPIO)
  • TCP 5701: Kamera akış servisi   (Ubuntu mission → RPi Camera Module 3 JPEG)
    camera_server.enable: false ise kamera servisi başlatılmaz.

Çalıştırma (Raspberry Pi'de, repo kökünden):
  python -m services.hil_entrypoint
  # veya: bash raspberry-pi-5/run_hil_server.sh
"""
from __future__ import annotations

import logging
import signal
import sys
import threading
from pathlib import Path

import yaml

# Doğrudan `python services/hil_entrypoint.py` çalıştırılırsa da repo kökü
# sys.path'te olsun (normal kullanım: `python -m services.hil_entrypoint`).
_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from config import (  # noqa: E402 — _REPO_ROOT sys.path insert'inden sonra
    CAMERA_HEIGHT_PX,
    CAMERA_ORIENTATION,
    CAMERA_WIDTH_PX,
    REAL_CAMERA_FPS,
    RPI_CAMERA_NUM,
)
from payload.backends.local_rpi import LocalRPiBackend
from services.camera_server import CameraTCPServer
from services.payload_dispatcher import CommandDispatcher
from services.payload_server import PayloadTCPServer
from vision.rpi_camera import RPiCam3

# ── Logging ──────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)-8s | %(name)s | %(message)s",
)
logger = logging.getLogger("hil_service")


def main() -> int:
    config_path = _REPO_ROOT / "payload" / "config.yaml"
    logger.info("Konfigürasyon: %s", config_path)

    cfg = yaml.safe_load(config_path.read_text())
    server_cfg = cfg.get("server", {})
    host = server_cfg.get("host", "0.0.0.0")
    port = server_cfg.get("port", 5700)

    # ── Payload Backend (TEK KAYNAK: payload/backends/local_rpi.py) ─────────
    backend = LocalRPiBackend.from_config(
        config_path=str(config_path),
        logger=logger,
    )
    res = backend.initialize()
    if not res.success:
        logger.error("Backend initialize başarısız: %s", res.message)
        return 1
    logger.info("Payload backend hazır.")

    # ── Payload Dispatcher + Server ────────────────────────────────────────
    dispatcher = CommandDispatcher(backend=backend, logger=logger)
    payload_server = PayloadTCPServer(
        host=host, port=port, dispatcher=dispatcher, logger=logger,
    )

    # ── (İsteğe bağlı) Kamera Servisi (RPi Camera Module 3) ───────────────
    server_cfg = cfg.get("camera_server", {})
    camera_enabled = bool(server_cfg.get("enable", False))
    # Gerçek kamera donanımı (TEK kaynak): config.yaml → camera
    # Çözünürlük/FPS/CSI port config.py'den gelir; orientation/jpeg_quality buradan.
    cam_cfg = cfg.get("camera", {}) or {}
    rpi_cam: RPiCam3 | None = None
    camera_server: CameraTCPServer | None = None
    camera_thread: threading.Thread | None = None

    if camera_enabled:
        cam_port = int(server_cfg.get("port", 5701))
        cam_host = server_cfg.get("host", "0.0.0.0")
        try:
            rpi_cam = RPiCam3(
                camera_num=RPI_CAMERA_NUM,
                width=CAMERA_WIDTH_PX,
                height=CAMERA_HEIGHT_PX,
                fps=REAL_CAMERA_FPS,
                logger=logger,
                orientation=str(cam_cfg.get("orientation", CAMERA_ORIENTATION)),
            )
            camera_server = CameraTCPServer(
                host=cam_host,
                port=cam_port,
                camera=rpi_cam,
                jpeg_quality=int(cam_cfg.get("jpeg_quality", 80)),
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
