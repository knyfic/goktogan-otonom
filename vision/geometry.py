import math
from dataclasses import dataclass

# Altitude sıfıra çok yakın olduğunda hesaplamalar bozulmasın diye
# kullanılan minimum irtifa değeri (metre).
_MIN_ALTITUDE_M = 0.05


@dataclass
class CameraModel:
    width_px: int
    height_px: int
    horizontal_fov_deg: float
    vertical_fov_deg: float

    def ground_footprint(self, altitude_m: float) -> tuple[float, float]:
        alt = max(altitude_m, _MIN_ALTITUDE_M)
        width  = 2.0 * alt * math.tan(math.radians(self.horizontal_fov_deg) / 2)
        height = 2.0 * alt * math.tan(math.radians(self.vertical_fov_deg)   / 2)
        return width, height

    def meters_per_pixel(self, altitude_m: float) -> tuple[float, float]:
        width_m, height_m = self.ground_footprint(altitude_m)
        mpp_x = width_m  / max(self.width_px,  1)
        mpp_y = height_m / max(self.height_px, 1)
        return mpp_x, mpp_y

    def pixel_offset_to_body_offset(
        self, dx_px: float, dy_px: float, altitude_m: float
    ) -> tuple[float, float]:
        mpp_x, mpp_y = self.meters_per_pixel(altitude_m)

        # Nadir camera assumption:
        # image +X -> vehicle right
        # image +Y -> vehicle backward
        forward_m = -dy_px * mpp_y
        right_m = dx_px * mpp_x
        return forward_m, right_m


def expected_payload_pixels(
    physical_width_m: float,
    physical_height_m: float,
    altitude_m: float,
    camera: CameraModel,
) -> tuple[float, float]:
    mpp_x, mpp_y = camera.meters_per_pixel(altitude_m)
    mpp_x = max(mpp_x, 1e-9)
    mpp_y = max(mpp_y, 1e-9)
    return physical_width_m / mpp_x, physical_height_m / mpp_y
