import cv2
import numpy as np


# Supported orientation values:
#  - "NORMAL": no change
#  - "ROTATE_90": rotate 90 degrees clockwise
#  - "ROTATE_180": rotate 180 degrees
#  - "ROTATE_270": rotate 270 degrees clockwise
#  - "FLIP_H": horizontal flip (left-right)
#  - "FLIP_V": vertical flip (up-down)
# This small helper centralizes image-space correction for camera mounting.

def apply_camera_orientation(frame: np.ndarray, orientation: str) -> np.ndarray:
    """Apply a spatial transform to normalize camera image to vehicle coordinates.

    The function is intentionally simple and uses OpenCV primitives. It does not
    modify color channels; only spatial transform is applied.

    Args:
        frame: HxWx3 BGR image
        orientation: one of the supported strings above

    Returns:
        Transformed image (new array) with the same dtype as input.
    """
    if orientation is None:
        orientation = "NORMAL"
    o = str(orientation).upper()

    if o == "NORMAL" or o == "NONE":
        return frame
    if o == "ROTATE_90":
        # rotate 90 clockwise
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    if o == "ROTATE_180":
        return cv2.rotate(frame, cv2.ROTATE_180)
    if o == "ROTATE_270":
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    if o == "FLIP_H":
        return cv2.flip(frame, 1)
    if o == "FLIP_V":
        return cv2.flip(frame, 0)

    # Unknown option — don't change but log via return same frame
    return frame
