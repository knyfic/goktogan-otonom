import math
from dataclasses import dataclass


@dataclass
class SearchPoint:
    x_m: float
    y_m: float


class LawnmowerPlanner:
    """30x100 m alan için boustrophedon/lawnmower coverage planner."""

    def __init__(self, width_m, length_m, line_spacing_m, margin_m):
        self.width = width_m
        self.length = length_m
        self.spacing = line_spacing_m
        self.margin = margin_m

    def generate(self):
        x_min = -self.width / 2 + self.margin
        x_max = self.width / 2 - self.margin
        y_min = self.margin
        y_max = self.length - self.margin

        points = []
        x = x_min
        left_to_right = True

        while x <= x_max + 1e-6:
            if left_to_right:
                points.extend([SearchPoint(x, y_min), SearchPoint(x, y_max)])
            else:
                points.extend([SearchPoint(x, y_max), SearchPoint(x, y_min)])

            left_to_right = not left_to_right
            x += self.spacing

        return points


def local_to_gps(
    origin_lat: float,
    origin_lon: float,
    north_m: float,
    east_m: float,
) -> tuple[float, float]:
    """
    Yerel NED ofsetini (kuzey/doğu metre) GPS koordinatına çevirir.
    Küçük mesafeler (< ~1 km) için yeterince doğrudur.
    """
    earth_radius = 6378137.0
    d_lat = north_m / earth_radius
    d_lon = east_m / (earth_radius * math.cos(math.radians(origin_lat)))
    return origin_lat + math.degrees(d_lat), origin_lon + math.degrees(d_lon)
