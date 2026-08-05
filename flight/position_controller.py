import math

from config import PIXEL_ALIGNMENT_TOLERANCE_PX
from flight.navigation import Navigation
from vision.geometry import CameraModel


class PositionController:
    """
    Kamera piksel offset'ini GPS komutuna çevirir.

    Koordinat dönüşüm zinciri:
      1. Piksel offset (dx, dy) → araç gövde ekseninde metre (forward, right)
         - Kamera aşağı bakıyor (nadir)
         - Kamera araçla birlikte döner
         - image +X = araç sağı, image +Y = araç gerisi
         - forward = -dy * mpp_y,  right = dx * mpp_x

      2. Body frame (forward, right) → NED frame (north, east)
         - Araç heading (yaw) kullanılarak rotate edilir
         - heading=0 → araç kuzeye bakıyor
         - north =  forward * cos(yaw) - right * sin(yaw)
         - east  =  forward * sin(yaw) + right * cos(yaw)

      3. NED offset → hedef GPS koordinatı
         - Mevcut GPS + (north_m, east_m) → hedef lat/lon

      4. goto_global(hedef_lat, hedef_lon, mevcut_irtifa)
    """

    def __init__(self, navigation: Navigation, camera: CameraModel):
        self.navigation = navigation
        self.camera = camera

    def align_detection(
        self,
        center_x: float,
        center_y: float,
        image_width: int,
        image_height: int,
        altitude_m: float,
    ) -> bool:
        """
        Tespit merkezini kamera merkezine hizalar.
        Hizalanmışsa True, komut gönderildiyse False döner.
        """
        camera_cx = image_width / 2.0
        camera_cy = image_height / 2.0

        dx = center_x - camera_cx  # + = hedef kamerada sağda
        dy = center_y - camera_cy  # + = hedef kamerada aşağıda

        if abs(dx) <= PIXEL_ALIGNMENT_TOLERANCE_PX and abs(dy) <= PIXEL_ALIGNMENT_TOLERANCE_PX:
            return True

        # Adım 1: Piksel → araç gövde ekseni (body frame)
        mpp_x, mpp_y = self.camera.meters_per_pixel(altitude_m)
        forward_m = -dy * mpp_y   # image +Y → araç gerisi → forward negatif
        right_m   =  dx * mpp_x   # image +X → araç sağı   → right pozitif

        # Adım 2: Body → NED  (araç heading'i ile rotate)
        pos = self.navigation.position()
        if pos.latitude == 0.0 and pos.longitude == 0.0:
            return False  # GPS henüz hazır değil

        yaw_rad = math.radians(pos.heading_deg)
        north_m =  forward_m * math.cos(yaw_rad) - right_m * math.sin(yaw_rad)
        east_m  =  forward_m * math.sin(yaw_rad) + right_m * math.cos(yaw_rad)

        # Adım 3: NED offset → hedef GPS
        target_lat, target_lon = _offset_gps(
            pos.latitude, pos.longitude, north_m, east_m
        )

        # Adım 4: Mevcut irtifayı koruyarak hedefe git
        self.navigation.goto_global(target_lat, target_lon, altitude_m)
        return False


def _offset_gps(
    lat: float, lon: float, north_m: float, east_m: float
) -> tuple[float, float]:
    """Verilen GPS noktasına kuzey/doğu metre offset ekler."""
    earth_radius = 6378137.0
    d_lat = north_m / earth_radius
    d_lon = east_m / (earth_radius * math.cos(math.radians(lat)))
    return lat + math.degrees(d_lat), lon + math.degrees(d_lon)
