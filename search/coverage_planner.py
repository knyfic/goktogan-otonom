import math
from dataclasses import dataclass

from config import SEARCH_FOV_WIDTH_M


@dataclass
class SearchPoint:
    x_m: float
    y_m: float


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
    v = D−A (genişlik yönü), u ⊥ v. Gerçek GPS hataları nedeniyle alan bozuk
    bir dörtgen olabilir; search line geometrisi gerçek köşelerden hesaplanır.
    """

    def __init__(self, corners: dict):
        self._corners = {k: (float(lat), float(lon)) for k, (lat, lon) in corners.items()}
        a_lat, a_lon = self._corners["A"]
        b_lat, b_lon = self._corners["B"]
        d_lat, d_lon = self._corners["D"]
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

    @property
    def axis_cos(self) -> float:
        """Local frame eksenleri (û, v̂) arasındaki kosinüs.

        Gerçek alan tam dikdörtgen değilse û ⊥ v̂ olmaz; bu değer transversal
        mesafeleri GERÇEK metreye çevirmek için kullanılır (skew düzeltmesi).
        """
        return self._u_hat[0] * self._v_hat[0] + self._u_hat[1] * self._v_hat[1]

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

    def width_heading_deg(self) -> float:
        """Genişlik ekseninin (x) pusula yönü (derece) — uzunluk eksenine dik."""
        return math.degrees(math.atan2(self._v_hat[1], self._v_hat[0])) % 360.0

    def search_direction_deg(self) -> float:
        """Uzun kenarların GENEL doğrultusunun pusula kerterizi (derece).

        Search sırasında İHA bu burnu KORUR; line değişimlerinde yaw YAPILMAZ
        (ileri/geri hareketle tarama yapılır — doğrultu sabit, yön değişir).
        """
        dx, dy = self.search_direction_local()
        north = dx * self._v_hat[0] + dy * self._u_hat[0]
        east = dx * self._v_hat[1] + dy * self._u_hat[1]
        return math.degrees(math.atan2(east, north)) % 360.0

    # ── Alan sınırları ────────────────────────────────────────────────────── #
    def contains_local(self, x_m: float, y_m: float, eps: float = 1e-6) -> bool:
        return (
            -eps <= x_m <= self._width_m + eps
            and -eps <= y_m <= self._length_m + eps
        )

    def contains(self, lat: float, lon: float) -> bool:
        x, y = self.gps_to_local(lat, lon)
        return self.contains_local(x, y)

    # ── Köşeler / kenarlar ─────────────────────────────────────────────────── #
    def corner_local(self) -> dict:
        """Dört köşenin local koordinatları — GERÇEK GPS köşelerinden."""
        return {k: self.gps_to_local(v[0], v[1]) for k, v in self._corners.items()}

    def edge_lengths_m(self) -> dict:
        """Dört kenarın GERÇEK uzunlukları (metre) — doğrudan GPS köşelerinden.

        AB, BC, CD, DA. 'Kısa/uzun' varsayımı YOK — hepsi ölçülür.
        """
        out = {}
        for name in ("AB", "BC", "CD", "DA"):
            lat1, lon1 = self._corners[name[0]]
            lat2, lon2 = self._corners[name[1]]
            n, e = _north_east_m(lat1, lon1, lat2, lon2)
            out[name] = math.hypot(n, e)
        return out

    def long_short_edges(self) -> tuple[list[str], list[str]]:
        """Kenar uzunluklarına göre (UZUN, KISA) kenar adları.

        Dört kenar uzunluğa göre azalan sıralanır; en uzun ikisi 'uzun',
        kalan ikisi 'kısa' kabul edilir. Sabit indeks/A-B varsayımı YOK.
        """
        lengths = self.edge_lengths_m()
        names = sorted(lengths, key=lambda n: lengths[n], reverse=True)
        return names[:2], names[2:]

    # ── Search line geometrisi ─────────────────────────────────────────────── #
    def search_direction_local(self) -> tuple[float, float]:
        """Uzun kenarların GENEL doğrultusu — local frame'de BİRİM vektör.

        İki uzun kenarın vektörü aynı yöne hizalanıp toplanır; GPS hatasından
        kaynaklanan doğrultu farkı böylece ortalama ile törpülenir.
        """
        long_names, _ = self.long_short_edges()
        c = self.corner_local()
        vecs = []
        for name in long_names:
            p = c[name[0]]
            q = c[name[1]]
            vecs.append((q[0] - p[0], q[1] - p[1]))
        if len(vecs) == 2:
            dot = vecs[0][0] * vecs[1][0] + vecs[0][1] * vecs[1][1]
            if dot < 0.0:
                vecs[1] = (-vecs[1][0], -vecs[1][1])
        sx = sum(v[0] for v in vecs)
        sy = sum(v[1] for v in vecs)
        norm = math.hypot(sx, sy)
        if norm < 1e-9:
            return (1.0, 0.0)
        return (sx / norm, sy / norm)

    def search_lines_local(self, fov_width_m: float, line_count: int = 3):
        """fov genişliğindeki kamera ile alanı tarayan paralel search line'ları üretir.

        Kurallar (gerçek GPS geometrisinden):
          - Line'lar UZUN kenarlara paraleldir (genel uzun kenar doğrultusu).
          - İlk line uzun kenardan fov/2 İÇERİDE; son line karşı uzun kenardan
            fov/2 İÇERİDE — kameranın görüş dış sınırı alan kenarına oturur,
            alan dışı gereksiz görüş olmaz.
          - Orta line'lar iki sınır line'ı arasını eşit böler (toplam: line_count).
          - Her line'ın uçları gerçek dörtgen sınırıyla kesiştirilir — line'lar
            alan dışına taşmaz, uçlar kısa kenarlara oturur.

        Dönen: her line için ((x1, y1), (x2, y2)) — local metre.
        """
        d = self.search_direction_local()
        t = (-d[1], d[0])                                     # transversal birim (local)
        scale_t = math.sqrt(1.0 + 2.0 * self.axis_cos * t[0] * t[1])  # m / local-birim
        poly = list(self.corner_local().values())

        p_vals = [x * t[0] + y * t[1] for (x, y) in poly]
        p_min, p_max = min(p_vals), max(p_vals)
        width_real = (p_max - p_min) * scale_t                # kısa-doğrultu genişlik (m)
        half = fov_width_m / 2.0

        if line_count <= 1:
            pos_real = [width_real / 2.0]
        elif width_real <= 2.0 * half:
            # Alan FOV'dan dar: line'ları genişliğe eşit aralıkla yerleştir (3 line korunur)
            pos_real = [width_real * (i + 1) / (line_count + 1) for i in range(line_count)]
        else:
            span = width_real - 2.0 * half
            pos_real = [half + span * i / (line_count - 1) for i in range(line_count)]

        lines = []
        for pr in pos_real:
            p_local = p_min + pr / scale_t                    # gerçek metre → local
            origin = (p_local * t[0], p_local * t[1])         # X·t = p_local üzerinde
            seg = _clip_line_to_polygon(origin, d, poly)
            if seg is not None:
                lines.append(seg)
        return lines

    # ── Kısa kenarlar (geriye dönük geometri yardımcıları) ─────────────────── #
    def short_edge_midpoints_local(self) -> list[tuple[float, float]]:
        """İki KISA kenarın orta noktaları — GERÇEK kenar uzunluklarına göre.

        'Kısa' olmak varsayılmaz; kenar uzunlukları GPS köşelerinden türetilir:
          - genişlik (A-D/B-C) kısa  → (W/2, 0) ve (W/2, L)  [eski 30x100 alan]
          - uzunluk (A-B/C-D) kısa  → (0, L/2) ve (W, L/2)  [gerçek yarışma alanı]
        """
        w, h = self._width_m, self._length_m
        if w <= h:
            return [(w / 2.0, 0.0), (w / 2.0, h)]
        return [(0.0, h / 2.0), (w, h / 2.0)]

    def nearest_short_edge_midpoint_local(self, x_m: float, y_m: float) -> tuple[float, float]:
        """İki kısa kenar orta noktasından (x_m, y_m)'ye EN YAKIN olanı döndürür.

        İHA ister alan İÇİNDE ister DIŞINDA olsun aynı hesap geçerlidir —
        dışarıda başlamak özel/abort durumu değildir. Eşitlikte ilk aday
        seçilir (eski davranış: ly == L/2 → alt kenar).
        """
        m1, m2 = self.short_edge_midpoints_local()
        d1 = (x_m - m1[0]) ** 2 + (y_m - m1[1]) ** 2
        d2 = (x_m - m2[0]) ** 2 + (y_m - m2[1]) ** 2
        return m1 if d1 <= d2 else m2


# ══════════════════════════════════════════════════════════════════════════ #
#  FOV tabanlı 3 uzun search line planner
# ══════════════════════════════════════════════════════════════════════════ #

class LawnmowerPlanner:
    """Gerçek 4 GPS köşesinden FOV tabanlı paralel search line üreten planner.

    Hiçbir boyut varsaymaz (30×100 DEĞİL): kenar uzunlukları köşelerden
    hesaplanır, uzun kenarlara paralel SEARCH_FOV_WIDTH_M tabanlı line'lar
    üretilir, ilk/son line uzun kenarlardan FOV/2 içeridedir, line uçları
    gerçek dörtgen sınırıyla kesişir. Çıktı zig-zag sıradadır (→ ← →) ve
    İHA line değişimlerinde yaw yapmadan ileri/geri hareket eder.
    """

    def __init__(self, field: CompetitionField, fov_width_m: float = SEARCH_FOV_WIDTH_M,
                 line_count: int = 3):
        self.field = field
        self.fov_width_m = fov_width_m
        self.line_count = line_count

    def generate(self) -> list[SearchPoint]:
        """line_count line → 2·line_count waypoint (zig-zag: line1 →, line2 ←, ...)."""
        lines = self.field.search_lines_local(self.fov_width_m, self.line_count)
        points: list[SearchPoint] = []
        for i, (s, e) in enumerate(lines):
            if i % 2 == 0:
                points.extend([SearchPoint(*s), SearchPoint(*e)])
            else:
                points.extend([SearchPoint(*e), SearchPoint(*s)])
        return points

    def debug_summary(self) -> str:
        """Detaylı debug raporu: kenar uzunlukları, uzun/kısa, FOV, line'lar (GPS)."""
        f = self.field
        edges = f.edge_lengths_m()
        long_names, short_names = f.long_short_edges()
        half = self.fov_width_m / 2.0
        lines = f.search_lines_local(self.fov_width_m, self.line_count)

        out = []
        out.append("Detected edge lengths:")
        for name in ("AB", "BC", "CD", "DA"):
            out.append(f"  edge {name}: {edges[name]:.2f} m")
        out.append("Short edges:")
        for name in short_names:
            out.append(f"  {name}: {edges[name]:.2f} m")
        out.append("Long edges:")
        for name in long_names:
            out.append(f"  {name}: {edges[name]:.2f} m")
        out.append(f"Search FOV width: {self.fov_width_m:.3f} m")
        out.append(f"Search half FOV: {half:.3f} m")
        out.append(f"Search line count: {self.line_count}")
        for i, (s, e) in enumerate(lines, start=1):
            length = math.hypot(e[0] - s[0], e[1] - s[1])
            s_lat, s_lon = f.local_to_gps(*s)
            e_lat, e_lon = f.local_to_gps(*e)
            out.append(
                f"Line {i}: start=({s_lat:.7f}, {s_lon:.7f}) "
                f"end=({e_lat:.7f}, {e_lon:.7f}) length={length:.2f} m"
            )
        return "\n".join(out)


# ══════════════════════════════════════════════════════════════════════════ #
#  Doğru/dörtgen kesişim yardımcıları
# ══════════════════════════════════════════════════════════════════════════ #

def _segment_line_intersection(a, b, origin, direction):
    """Segment (a,b) ile doğru (origin + s·direction) kesişimi — yoksa None."""
    r = direction
    s = (b[0] - a[0], b[1] - a[1])
    denom = r[0] * s[1] - r[1] * s[0]
    if abs(denom) < 1e-12:
        return None
    qmp = (a[0] - origin[0], a[1] - origin[1])
    t_param = (qmp[0] * s[1] - qmp[1] * s[0]) / denom
    u_param = (qmp[0] * r[1] - qmp[1] * r[0]) / denom
    if -1e-9 <= u_param <= 1.0 + 1e-9:
        return (origin[0] + t_param * r[0], origin[1] + t_param * r[1])
    return None


def _clip_line_to_polygon(origin, direction, poly):
    """Doğruyu (origin + s·direction) dışbükey dörtgenle keser.

    Dönen: ((x1, y1), (x2, y2)) — doğrultu boyunca sıralı — veya None.
    Line uçları alan dışına taşmaz (gerçek kısa kenarlara oturur).
    """
    hits = []
    n = len(poly)
    for i in range(n):
        pt = _segment_line_intersection(poly[i], poly[(i + 1) % n], origin, direction)
        if pt is not None:
            hits.append(pt)
    if not hits:
        return None
    uniq = []
    for p in hits:
        if all(math.hypot(p[0] - q[0], p[1] - q[1]) > 1e-6 for q in uniq):
            uniq.append(p)
    if len(uniq) < 2:
        return (uniq[0], uniq[0])
    uniq.sort(key=lambda p: (p[0] - origin[0]) * direction[0]
                             + (p[1] - origin[1]) * direction[1])
    return (uniq[0], uniq[-1])


def point_in_polygon(px: float, py: float, poly: list) -> bool:
    """Noktanın dörtgen (poly) İÇİNDE olup olmadığı (ray casting + sınır toleransı)."""
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        seg_len_sq = (x2 - x1) ** 2 + (y2 - y1) ** 2
        if seg_len_sq > 1e-12:
            t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / seg_len_sq
            if -1e-6 <= t <= 1.0 + 1e-6:
                dist_sq = ((px - (x1 + t * (x2 - x1))) ** 2
                           + (py - (y1 + t * (y2 - y1))) ** 2)
                if dist_sq <= 1e-6 ** 2:
                    return True
        if (y1 > py) != (y2 > py):
            x_int = x1 + (py - y1) * (x2 - x1) / (y2 - y1)
            if px < x_int:
                inside = not inside
    return inside
