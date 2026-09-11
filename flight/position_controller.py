import math

from config import (
    ALIGN_ALT_GAIN_REF_M,
    ALIGN_ALT_MODE_HI_M,
    ALIGN_ALT_MODE_LO_M,
    ALIGN_GAIN,
    ALIGN_GAIN_FULL_ERROR_PX,
    ALIGN_MAX_STEP_LARGE_M,
    ALIGN_MAX_STEP_M,
    ALIGN_STEP_FULL_ERROR_PX,
    ALIGN_TOL_ALT_HI_M,
    ALIGN_TOL_ALT_LO_M,
    ALIGN_TOL_HI_PX,
    ALIGN_TOL_LO_PX,
)
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

    def aligned_tolerance_px(self, altitude_m: float) -> float:
        """İrtifaya göre 'ortalandım' KABUL toleransı (piksel).

        SADECE acceptance eşiğidir — hareket/gain/adım hesabını ETKİLEMEZ.
        Yüksek irtifada hedef küçük + kamera jitter'ı (rüzgâr/pitch/yaw) →
        daha geniş tolerans; alçak irtifada (bırakma öncesi) hassas tolerans
        korunur. Eşikler config.py → ALIGN_TOL_* (iri değil, piksel).
        """
        if altitude_m <= ALIGN_TOL_ALT_LO_M:
            return ALIGN_TOL_LO_PX
        if altitude_m >= ALIGN_TOL_ALT_HI_M:
            return ALIGN_TOL_HI_PX
        w = (altitude_m - ALIGN_TOL_ALT_LO_M) / (ALIGN_TOL_ALT_HI_M - ALIGN_TOL_ALT_LO_M)
        return ALIGN_TOL_LO_PX + (ALIGN_TOL_HI_PX - ALIGN_TOL_LO_PX) * w

    def detection_ground_gps(
        self,
        center_x: float,
        center_y: float,
        image_width: int,
        image_height: int,
        altitude_m: float,
    ) -> tuple[float, float] | None:
        """Tespitin ZEMİN GPS koordinatı (komut GÖNDERMEZ).

        align_detection ile AYNI piksel→gövde→NED zincirini kullanır; ancak
        kazanç/adım sınırı UYGULANMAZ (tam offset) — hedefin gerçek zemin
        konumunu verir. GPS hazır değilse None.
        """
        pos = self.navigation.position()
        if pos.latitude == 0.0 and pos.longitude == 0.0:
            return None
        dx = center_x - image_width / 2.0
        dy = center_y - image_height / 2.0
        mpp_x, mpp_y = self.camera.meters_per_pixel(altitude_m)
        forward_m = -dy * mpp_y
        right_m = dx * mpp_x
        yaw_rad = math.radians(pos.heading_deg)
        north_m = forward_m * math.cos(yaw_rad) - right_m * math.sin(yaw_rad)
        east_m = forward_m * math.sin(yaw_rad) + right_m * math.cos(yaw_rad)
        return _offset_gps(pos.latitude, pos.longitude, north_m, east_m)

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

        # KABUL eşiği irtifaya göre uyarlanır (aligned_tolerance_px): yüksek
        # irtifada kamera jitter'ına toleranslı, alçak irtifada hassas.
        tol = self.aligned_tolerance_px(altitude_m)
        if abs(dx) <= tol and abs(dy) <= tol:
            return True

        # Adım 1: Piksel → araç gövde ekseni (body frame)
        mpp_x, mpp_y = self.camera.meters_per_pixel(altitude_m)
        error_px = math.hypot(dx, dy)

        # ── Davranış modu ────────────────────────────────────────────────────
        # 8m ve ALTINDA: ESKİ alignment hesabı birebir korunur — sabit
        # ALIGN_GAIN, adım sınırı YOK (8m altında stabil çalışan davranış
        # değişmez; ALIGN_SPEED_MPS / ALIGN_CMD_INTERVAL_S de değişmez).
        # 8m ÜSTÜNDE (11.5m): yüksek-irtifa adaptive hesap — hata oranı kazancı
        # + irtifa kompanzasyonu + piksel hatasıyla büyüyen adım sınırı.
        # 8–9m arasında iki hesap ağırlıkla KARIŞTIRILIR (yumuşak geçiş).
        if altitude_m <= ALIGN_ALT_MODE_LO_M:
            w = 0.0   # tamamen eski hesap
        elif altitude_m >= ALIGN_ALT_MODE_HI_M:
            w = 1.0   # tamamen adaptive hesap
        else:
            w = (altitude_m - ALIGN_ALT_MODE_LO_M) / (
                ALIGN_ALT_MODE_HI_M - ALIGN_ALT_MODE_LO_M
            )

        # Yüksek-irtifa kazanç bileşenleri:
        #   a) Hata oranı: hata ALIGN_GAIN_FULL_ERROR_PX üzerindeyken kazanç tam
        #      ALIGN_GAIN'dir; hedefe yaklaştıkça (hata/eşik) oranında küçülür →
        #      adımlar otomatik küçülür (uzak hedef → büyük adım, yakın → küçük).
        #   b) İrtifa kompanzasyonu: mpp'nin irtifayla büyümesini dengeler.
        ratio_gain = min(1.0, error_px / ALIGN_GAIN_FULL_ERROR_PX)
        alt_gain = min(
            1.0, ALIGN_ALT_GAIN_REF_M / max(altitude_m, ALIGN_ALT_GAIN_REF_M)
        )
        # Eski kazanç (ALIGN_GAIN) ↔ yeni kazanç (ALIGN_GAIN·ratio·alt) karışımı
        gain = ALIGN_GAIN * (1.0 - w) + ALIGN_GAIN * ratio_gain * alt_gain * w

        forward_m = -dy * mpp_y * gain   # image +Y → araç gerisi → forward negatif
        right_m   =  dx * mpp_x * gain   # image +X → araç sağı   → right pozitif

        # Adım sınırı — yalnızca yüksek-irtifa davranışı (w > 0): piksel hatasıyla
        # büyüyen adaptive sınır (ALIGN_MAX_STEP_M → ALIGN_MAX_STEP_LARGE_M,
        # yani ~0.25 → ~0.5m). Geçiş bandında sınır ağırlıkla açılır; 8m'de
        # sınır YOKTUR (eski davranış birebir korunur).
        step_m = math.hypot(forward_m, right_m)
        if w > 0.0:
            step_frac = min(1.0, error_px / ALIGN_STEP_FULL_ERROR_PX)
            max_step_m = ALIGN_MAX_STEP_M + (
                ALIGN_MAX_STEP_LARGE_M - ALIGN_MAX_STEP_M
            ) * step_frac
            if w < 1.0:
                max_step_m = max_step_m / w   # 8m'ye yaklaştıkça gevşer
            if step_m > max_step_m:
                scale = max_step_m / step_m
                forward_m *= scale
                right_m *= scale

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

        # Adım 4: Mevcut irtifayı koruyarak hedefe git.
        # Yaw kilidi: İHA burnu komutlar arasında dönmesin — düzeltme vektörleri
        # tutarlı frame'de hesaplanır (hedef etrafında dönme/daire önlenir).
        self.navigation.goto_global(
            target_lat, target_lon, altitude_m, yaw_deg=pos.heading_deg
        )
        return False


def _offset_gps(
    lat: float, lon: float, north_m: float, east_m: float
) -> tuple[float, float]:
    """Verilen GPS noktasına kuzey/doğu metre offset ekler."""
    earth_radius = 6378137.0
    d_lat = north_m / earth_radius
    d_lon = east_m / (earth_radius * math.cos(math.radians(lat)))
    return lat + math.degrees(d_lat), lon + math.degrees(d_lon)
