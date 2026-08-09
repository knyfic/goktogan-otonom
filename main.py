"""
İHA Mission Ana Giriş Noktası.

Payload backend seçimi payload/config.yaml üzerinden yapılır.
Bu dosyada backend ile ilgili magic string veya magic number yoktur.

  payload/config.yaml → backend: simulation   # Gazebo
  payload/config.yaml → backend: hil          # Ubuntu → Wi-Fi → RPi
"""
import os
import sys

import yaml

from config import (
    CAMERA_TOPIC,
    ENABLE_LIVE_PREVIEW,
    GAZEBO_MSGS_MODULE,
    GAZEBO_TRANSPORT_MODULE,
    MAVLINK_CONNECTION,
    PREVIEW_WINDOW_NAME,
)
from flight.navigation import Navigation
from payload.payload_controller import PayloadController
from state_machine import MissionStateMachine
from utils.logging import setup_logging
from vision.camera import GazeboCamera

# Payload config dosyasının yolu — bu dosyaya göre göreceli
_PAYLOAD_CONFIG = os.path.join(os.path.dirname(__file__), "payload", "config.yaml")


def _load_payload_config() -> dict:
    with open(_PAYLOAD_CONFIG, encoding="utf-8") as f:
        return yaml.safe_load(f)


def _build_backend(cfg: dict, logger):
    """
    payload/config.yaml'daki 'backend' anahtarına göre uygun backend'i oluşturur.
    Tüm bağlantı parametreleri (host, port, timeout) config'den okunur.
    """
    mode = cfg.get("backend", "simulation")
    logger.info("Payload backend modu: %s  (kaynak: payload/config.yaml)", mode)

    if mode == "simulation":
        from payload.backends.simulation import SimulationBackend
        return SimulationBackend(logger=logger)

    if mode == "hil":
        from payload.backends.remote_rpi import RemoteRPiBackend
        remote = cfg.get("remote", {})
        return RemoteRPiBackend(
            host=remote["host"],              # config.yaml'dan — kod içinde IP yok
            port=remote["port"],
            timeout=remote.get("timeout_s", 10.0),
            logger=logger,
        )

    raise ValueError(
        f"Bilinmeyen payload backend: {mode!r}. "
        f"payload/config.yaml içinde 'backend' değerini "
        f"'simulation' veya 'hil' olarak ayarlayın."
    )


def main():
    logger = setup_logging()
    logger.info("Starting İHA Gazebo autonomy mission.")

    camera: GazeboCamera | None = None
    try:
        payload_cfg = _load_payload_config()

        navigation = Navigation(MAVLINK_CONNECTION)

        camera = GazeboCamera(
            topic=CAMERA_TOPIC,
            msgs_module=GAZEBO_MSGS_MODULE,
            transport_module=GAZEBO_TRANSPORT_MODULE,
            preview=ENABLE_LIVE_PREVIEW,
            window_name=PREVIEW_WINDOW_NAME,
        )

        backend = _build_backend(payload_cfg, logger)
        payload = PayloadController(backend=backend, logger=logger)

        mission = MissionStateMachine(
            navigation=navigation,
            camera=camera,
            payload=payload,
            logger=logger,
        )
        mission.run()

    except KeyboardInterrupt:
        logger.warning("Keyboard interrupt.")
        return 130
    except Exception:
        logger.exception("Mission terminated by exception.")
        return 1
    finally:
        if camera is not None:
            camera.close_preview()

    return 0


if __name__ == "__main__":
    sys.exit(main())
