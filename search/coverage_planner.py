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


# ══════════════════════════════════════════════════════════════════════════ #
#  Yarışma alanı (competition) frame'i — 4 GPS köşesi → SABİT local koordinatlar
# ══════════════════════════════════════════════════════════════════════════ #

def _north_east_m(lat1: float, lon1: float, lat2: float, lon2: float) -> tuple[float, float]:
    """İki GPS noktası arasındaki kuzey/doğu metre farkı (küçük mesafeler)."""
    radius = 6378137.0
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    north = d_lat * radius
    east = d_lon * radius * math.cos(math.radians((lat1 + lat2) / 2.0))
    return north, east


class CompetitionField:
    """
    4 GPS köşesi (A,B,C,D) ile tanımlı SABİT yarışma alanı local frame'i.

    Frame (metre):
      A = (0, 0)          — alt-sol (kavramsal)
      B = (0, length_m)   — üst-sol
      C = (width_m, length_m) — üst-sağ
      D = (width_m, 0)    — alt-sağ

    Eksenler alanın GERÇEK GPS yönlerinden türetilir (kuzeye hizalı olmak
    zorunda değildir) ve dünya sabitidir — İHA'nın heading/body frame'inden
    bağımsızdır. Dikdörtgen alan varsayılır: u = B−A (uzunluk yönü),
    v = D−A (genişlik yönü), u ⊥ v.
    """

    def __init__(self, corners: dict):
        a_lat, a_lon = corners["A"]
        b_lat, b_lon = corners["B"]
        d_lat, d_lon = corners["D"]
        self._a_lat = float(a_lat)
        self._a_lon = float(a_lon)

        u = _north_east_m(a_lat, a_lon, b_lat, b_lon)   # (north, east) uzunluk
        v = _north_east_m(a_lat, a_lon, d_lat, d_lon)   # (north, east) genişlik
        self._length_m = math.hypot(*u)
        self._width_m = math.hypot(*v)
        self._u_hat = (u[0] / self._length_m, u[1] / self._length_m)
        self._v_hat = (v[0] / self._width_m, v[1] / self._width_m)

    # ── Geometri ──────────────────────────────────────────────────────────── #
    @property
    def width_m(self) -> float:
        return self._width_m

    @property
    def length_m(self) -> float:
        return self._length_m

    # ── Dönüşümler ────────────────────────────────────────────────────────── #
    def gps_to_local(self, lat: float, lon: float) -> tuple[float, float]:
        """GPS → competition-local (x_m genişlik, y_m uzunluk)."""
        north, east = _north_east_m(self._a_lat, self._a_lon, lat, lon)
        x = north * self._v_hat[0] + east * self._v_hat[1]
        y = north * self._u_hat[0] + east * self._u_hat[1]
        return x, y

    def local_to_gps(self, x_m: float, y_m: float) -> tuple[float, float]:
        """Competition-local (x_m, y_m) → GPS."""
        north = x_m * self._v_hat[0] + y_m * self._u_hat[0]
        east = x_m * self._v_hat[1] + y_m * self._u_hat[1]
        radius = 6378137.0
        lat = self._a_lat + math.degrees(north / radius)
        lon = self._a_lon + math.degrees(
            east / (radius * math.cos(math.radians(self._a_lat)))
        )
        return lat, lon

    # ── Yönler ─────────────────────────────────────────────────────────────── #
    def length_heading_deg(self) -> float:
        """Uzun eksenin (y) pusula yönü (derece) — kısa kenara DİK yön.

        atan2(east, north) → pusula kerterizi. İlk arama (tek geçiş) sırasında
        İHA burnunun kısa kenara dik durması için kullanılır.
        """
        return math.degrees(math.atan2(self._u_hat[1], self._u_hat[0])) % 360.0

    # ── Alan sınırları ────────────────────────────────────────────────────── #
    def contains_local(self, x_m: float, y_m: float, eps: float = 1e-6) -> bool:
        return (
            -eps <= x_m <= self._width_m + eps
            and -eps <= y_m <= self._length_m + eps
        )

    def contains(self, lat: float, lon: float) -> bool:
        x, y = self.gps_to_local(lat, lon)
        return self.contains_local(x, y)


def reorder_search_from_drone(
    points: list[SearchPoint],
    drone_x_m: float,
    drone_y_m: float,
) -> list[SearchPoint]:
    """
    Lawnmower çıktısını İHA'nın konumuna EN YAKIN search hattından başlatır.

    Girdi: competition frame'de planner üretim sırası (her x sütununda 2 nokta,
    boustrophedon/zigzag sırası). Çıktı: İHA'nın x'ine en yakın sütunun en
    yakın ucundan başlayan; ardından sağa doğru, alan sonuna gelince başa
    dönüp sol sütunları tarayan rota. Sütun içi yönler (zigzag) üretim
    sırasından korunur — lawnmower pattern'i yeniden tasarlanmaz.
    """
    if not points:
        return []
    if len(points) <= 2:
        return points[:]

    cols = [points[i:i + 2] for i in range(0, len(points), 2)]
    k = min(range(len(cols)), key=lambda i: abs(cols[i][0].x_m - drone_x_m))

    col_k = cols[k]
    if abs(col_k[0].y_m - drone_y_m) <= abs(col_k[1].y_m - drone_y_m):
        start, rest = col_k[0], col_k[1]
    else:
        start, rest = col_k[1], col_k[0]

    order = [k] + list(range(k + 1, len(cols))) + list(range(0, k))
    route = [start, rest]
    for i in order[1:]:
        route.extend(cols[i])
    return route
