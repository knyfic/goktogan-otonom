"""
Raspberry Pi 5 — Payload Servisi Ana Giriş Noktası.

Çalıştırma:
  cd raspberry-pi-5/
  python main.py

Gereksinimler:
  pip install RPi.GPIO PyYAML
"""
from __future__ import annotations

import logging
import signal
import sys
from pathlib import Path

import yaml

# raspberry-pi-5/ kök dizini sys.path'e ekle
sys.path.insert(0, str(Path(__file__).parent))

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

    # ── Backend ──────────────────────────────────────────────────────────────
    backend = LocalRPiBackend.from_config(
        config_path=str(config_path),
        logger=logger,
    )
    res = backend.initialize()
    if not res.get("success"):
        logger.error("Backend initialize başarısız: %s", res.get("message"))
        return 1
    logger.info("Payload backend hazır.")

    # ── Dispatcher + Server ───────────────────────────────────────────────────
    dispatcher = CommandDispatcher(backend=backend, logger=logger)
    server     = PayloadTCPServer(host=host, port=port,
                                  dispatcher=dispatcher, logger=logger)

    # ── Güvenli kapanış ───────────────────────────────────────────────────────
    def _shutdown(sig, frame):
        logger.warning("Sinyal %s alındı — güvenli kapatılıyor...", sig)
        server.stop()
        backend.cleanup()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    # ── Başlat ────────────────────────────────────────────────────────────────
    logger.info("Payload TCP servisi başlıyor: %s:%d", host, port)
    try:
        server.start()   # Blocking
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt.")
    finally:
        backend.cleanup()

    return 0


if __name__ == "__main__":
    sys.exit(main())
