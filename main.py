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
from payload.payload_controller import PayloadController
from state_machine import MissionStateMachine
from utils.logging import setup_logging
from vision.camera import GazeboCamera


def main():
    logger = setup_logging()
    logger.info("Starting İHA Gazebo autonomy mission.")

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

        payload = PayloadController(logger)

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
