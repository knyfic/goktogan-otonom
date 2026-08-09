"""
İHA Mission Ana Giriş Noktası.

Tüm mod/donanım seçimi payload/config.yaml üzerinden yapılır.
Bu dosyada magic string veya magic number yoktur.

  payload/config.yaml → backend: simulation   Gazebo + simüle payload
  payload/config.yaml → backend: hil          Gazebo + gerçek RPi GPIO (Wi-Fi)
  payload/config.yaml → backend: real         Gerçek kamera + doğrudan GPIO (RPi üstünde)
"""
import os
import sys

import yaml

from config import (
    CAMERA_HEIGHT_PX,
    CAMERA_TOPIC,
    CAMERA_WIDTH_PX,
    ENABLE_LIVE_PREVIEW,
    GAZEBO_MSGS_MODULE,
    GAZEBO_TRANSPORT_MODULE,
    MAVLINK_CONNECTION,
    PREVIEW_WINDOW_NAME,
    REAL_CAMERA_FPS,
    REAL_CAMERA_SOURCE,
)
from flight.navigation import Navigation
from payload.payload_controller import PayloadController
from state_machine import MissionStateMachine
from utils.logging import setup_logging

_PAYLOAD_CONFIG = os.path.join(os.path.dirname(__file__), "payload", "config.yaml")


def _load_payload_config() -> dict:
    with open(_PAYLOAD_CONFIG, encoding="utf-8") as f:
        return yaml.safe_load(f)


def _build_camera(mode: str, cfg: dict, logger):
    """
    simulation / hil → GazeboCamera  (Gazebo topic'ten frame alır)
    real             → RealCamera    (USB/CSI kamera, OpenCV ile)

    Her iki sınıf da aynı arayüzü sunar: latest(), show(), pump(), close_preview()
    State machine kamera türünü bilmez.
    """
    if mode in ("simulation", "hil"):
        from vision.camera import GazeboCamera
        return GazeboCamera(
            topic=CAMERA_TOPIC,
            msgs_module=GAZEBO_MSGS_MODULE,
            transport_module=GAZEBO_TRANSPORT_MODULE,
            preview=ENABLE_LIVE_PREVIEW,
            window_name=PREVIEW_WINDOW_NAME,
        )

    if mode == "real":
        from vision.real_camera import RealCamera
        return RealCamera(
            source=REAL_CAMERA_SOURCE,   # config.py'den
            width=CAMERA_WIDTH_PX,       # config.py'den — FOV ile tutarlı
            height=CAMERA_HEIGHT_PX,     # config.py'den
            fps=REAL_CAMERA_FPS,         # config.py'den
            preview=ENABLE_LIVE_PREVIEW,
            window_name=PREVIEW_WINDOW_NAME,
            logger=logger,
        )

    raise ValueError(f"Bilinmeyen mod için kamera oluşturulamadı: {mode!r}")


def _build_backend(mode: str, cfg: dict, logger):
    """
    Payload backend'ini config'e göre oluşturur.
    Tüm parametreler config.yaml'dan okunur — kod içinde IP/pin yoktur.
    """
    logger.info("Payload backend: %s  (kaynak: payload/config.yaml)", mode)

    if mode == "simulation":
        from payload.backends.simulation import SimulationBackend
        return SimulationBackend(logger=logger)

    if mode == "hil":
        from payload.backends.remote_rpi import RemoteRPiBackend
        remote = cfg.get("remote", {})
        return RemoteRPiBackend(
            host=remote["host"],
            port=remote["port"],
            timeout=remote.get("timeout_s", 10.0),
            logger=logger,
        )

    if mode == "real":
        # payload/hardware/ altındaki lgpio sürücüleri kullanılır.
        # Bu kod Raspberry Pi 5 üzerinde çalışır.
        from payload.backends.local_rpi import LocalRPiBackend
        return LocalRPiBackend.from_config(
            config_path=_PAYLOAD_CONFIG,
            logger=logger,
        )

    raise ValueError(
        f"Bilinmeyen payload backend: {mode!r}. "
        f"payload/config.yaml → 'backend' değerini "
        f"'simulation', 'hil' veya 'real' yapın."
    )


def main():
    logger = setup_logging()
    logger.info("İHA mission başlatılıyor.")

    camera = None
    try:
        payload_cfg = _load_payload_config()
        mode = payload_cfg.get("backend", "simulation")

        navigation = Navigation(MAVLINK_CONNECTION)
        camera     = _build_camera(mode, payload_cfg, logger)
        backend    = _build_backend(mode, payload_cfg, logger)
        # Renk haritası: hangi renk payload hangi mıknatısta
        color_map  = payload_cfg.get("payload_color_map", {})
        payload    = PayloadController(backend=backend, logger=logger,
                                       color_map=color_map or None)

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
