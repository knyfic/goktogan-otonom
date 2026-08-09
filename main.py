import sys

from config import (
    CAMERA_TOPIC,
    ENABLE_LIVE_PREVIEW,
    GAZEBO_MSGS_MODULE,
    GAZEBO_TRANSPORT_MODULE,
    MAVLINK_CONNECTION,
    PREVIEW_WINDOW_NAME,
)
from flight.navigation import Navigation
from payload.backends.simulation import SimulationBackend
from payload.payload_controller import PayloadController
from state_machine import MissionStateMachine
from utils.logging import setup_logging
from vision.camera import GazeboCamera

# ── Backend seçimi: tek bir yer ──────────────────────────────────────────────
#
# SIMULATION (Gazebo):
#   backend = SimulationBackend(logger=logger)
#
# HIL (Ubuntu → Wi-Fi → Raspberry Pi):
#   from payload.backends.remote_rpi import RemoteRPiBackend
#   backend = RemoteRPiBackend(host="192.168.1.42", port=5700, logger=logger)
#
# GERÇEK UÇUŞ (Raspberry Pi üzerinde çalıştırılır):
#   from raspberry_pi_5.payload_service.local_backend import LocalRPiBackend
#   backend = LocalRPiBackend(config_path="config/payload.yaml", logger=logger)
#
# ─────────────────────────────────────────────────────────────────────────────
BACKEND_MODE = "simulation"   # "simulation" | "hil" | "real"


def _build_backend(mode: str, logger):
    if mode == "simulation":
        return SimulationBackend(logger=logger)

    if mode == "hil":
        from payload.backends.remote_rpi import RemoteRPiBackend
        import yaml, os
        cfg_path = os.path.join(os.path.dirname(__file__), "payload", "config.yaml")
        with open(cfg_path) as f:
            cfg = yaml.safe_load(f)
        remote = cfg.get("remote", {})
        return RemoteRPiBackend(
            host=remote.get("host", "raspberry-pi-5.local"),
            port=remote.get("port", 5700),
            logger=logger,
        )

    if mode == "real":
        # Gerçek uçuşta bu kod Raspberry Pi üzerinde çalışır.
        # raspberry-pi-5/payload_service/local_backend.py kullanılır.
        raise RuntimeError(
            "Gerçek uçuş için kodu Raspberry Pi üzerinde çalıştırın "
            "ve LocalRPiBackend kullanın."
        )

    raise ValueError(f"Bilinmeyen backend modu: {mode!r}")


def main():
    logger = setup_logging()
    logger.info("Starting İHA Gazebo autonomy mission.")
    logger.info("Payload backend modu: %s", BACKEND_MODE)

    camera: GazeboCamera | None = None
    try:
        navigation = Navigation(MAVLINK_CONNECTION)

        camera = GazeboCamera(
            topic=CAMERA_TOPIC,
            msgs_module=GAZEBO_MSGS_MODULE,
            transport_module=GAZEBO_TRANSPORT_MODULE,
            preview=ENABLE_LIVE_PREVIEW,
            window_name=PREVIEW_WINDOW_NAME,
        )

        backend = _build_backend(BACKEND_MODE, logger)
        payload = PayloadController(backend=backend, logger=logger)

        mission = MissionStateMachine(
            navigation=navigation,
            camera=camera,
            payload=payload,
            logger=logger,
        )
        mission.run()

    except KeyboardInterrupt:
        logger.warning(
            "Keyboard interrupt. No automatic DISARM command is sent."
        )
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
