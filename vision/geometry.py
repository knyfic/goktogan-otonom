import math
from dataclasses import dataclass


@dataclass
class CameraModel:
    width_px: int
    height_px: int
    horizontal_fov_deg: float
    vertical_fov_deg: float

    def ground_footprint(self, altitude_m: float) -> tuple[float, float]:
        width = 2.0 * altitude_m * math.tan(math.radians(self.horizontal_fov_deg) / 2)
        height = 2.0 * altitude_m * math.tan(math.radians(self.vertical_fov_deg) / 2)
        return width, height

    def meters_per_pixel(self, altitude_m: float) -> tuple[float, float]:
        width_m, height_m = self.ground_footprint(altitude_m)
        return width_m / self.width_px, height_m / self.height_px

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
    return physical_width_m / mpp_x, physical_height_m / mpp_y
