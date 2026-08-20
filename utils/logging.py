import logging
import os


def setup_logging() -> logging.Logger:
    # VISION_DEBUG=1 → DEBUG seviyesi (vision teşhis logları için).
    # Normal çalışmada INFO kalır — görev loglarını gürültüye boğmaz.
    level = logging.INFO
    if os.environ.get("VISION_DEBUG", "").strip().lower() in ("1", "true", "yes"):
        level = logging.DEBUG
    logging.basicConfig(
        level=level,
        format="%(asctime)s | %(levelname)-8s | %(name)s | %(message)s",
    )
    return logging.getLogger("iha_mission")
