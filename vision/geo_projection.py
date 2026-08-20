"""
Piksel → zemin → dünya dönüşümü (basitleştirilmiş nadir projeksiyon).

Sorumluluk: kontur merkezi (cx, cy) → GSD ile kamera-merkezli zemin
uzaklığı → araç yaw'ı ile NED → araç konumuna eklenerek (lat, lon).

- Tam homography ZORUNLU DEĞİLDİR (kamera nadir görüşe çok yakın; küçük
  pitch/roll hataları config'teki konum toleransıyla yönetilir).
- pitch/roll config sınırını aşarsa projection sonucu geçersiz işaretlenir
  (confidence ölçeği düşer) — detection durdurulmak zorunda değildir.
- Bu modül flight paketine BAĞIMLI DEĞİLDİR: vehicle pose (lat/lon/yaw/
  pitch/roll) dışarıdan verilir. Böylece vision-record/stream gibi güvenlik
  kısıtlı script'ler flight import etmez.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

_EARTH_RADIUS_M = 6378137.0


@dataclass
class VehiclePose:
    """Araç duruşu — flight controller'dan gelen (MAVLink) veriler."""

    latitude: float
    longitude: float
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    roll_deg: float = 0.0


@dataclass
class ProjectionResult:
    """Projeksiyon çıktısı."""

    latitude: float
    longitude: float
    valid: bool            # pitch/roll sınırı aşıldıysa False
    confidence_scale: float  # [0,1] — sınır aşımında düşer


class GeoProjection:
    """Config'ten okunan sınırlarla piksel → dünya dönüşümü."""

    def __init__(self, cfg: dict[str, Any], gsd_cfg: dict[str, Any]) -> None:
        """
        cfg: {"simplified_nadir_projection", "max_pitch_deg", "max_roll_deg",
              "position_uncertainty_radius_m"}.
        gsd_cfg: {"gsd_ref_m_per_px", "gsd_ref_altitude_m"}.
        """
        self._simplified = bool(cfg["simplified_nadir_projection"])
        self._max_pitch = float(cfg["max_pitch_deg"])
        self._max_roll = float(cfg["max_roll_deg"])
        self._uncertainty_m = float(cfg["position_uncertainty_radius_m"])
        self._gsd_ref = float(gsd_cfg["gsd_ref_m_per_px"])
        self._h_ref = float(gsd_cfg["gsd_ref_altitude_m"])

    def gsd_at(self, altitude_m: float) -> float:
        """GSD(h) = GSD_ref * (h / h_ref) — irtifa ile doğrusal."""
        alt = max(float(altitude_m), 0.1)
        return self._gsd_ref * (alt / self._h_ref)

    def project(
        self,
        cx: float,
        cy: float,
        image_width: int,
        image_height: int,
        altitude_m: float,
        pose: VehiclePose,
    ) -> ProjectionResult:
        """Kontur merkezini dünya koordinatına projekte eder."""
        if not self._simplified:
            # Şu an yalnızca basitleştirilmiş nadir projeksiyon desteklenir.
            return ProjectionResult(pose.latitude, pose.longitude, False, 0.0)

        # Pitch/roll sınır kontrolü → güven ölçeği.
        valid = (
            abs(float(pose.pitch_deg)) <= self._max_pitch
            and abs(float(pose.roll_deg)) <= self._max_roll
        )
        confidence_scale = 1.0 if valid else 0.3

        # Piksel farkı (görüntü merkezine göre).
        dx = float(cx) - image_width / 2.0    # + = sağ
        dy = float(cy) - image_height / 2.0   # + = aşağı

        # GSD ile kamera-merkezli zemin uzaklığı (nadir varsayımı).
        gsd = self.gsd_at(altitude_m)
        # image +X → araç sağı; image +Y → araç gerisi.
        right_m = dx * gsd
        forward_m = -dy * gsd

        # Yaw ile NED'e çevir (heading=0 → kuzey).
        yaw_rad = math.radians(float(pose.yaw_deg))
        north_m = forward_m * math.cos(yaw_rad) - right_m * math.sin(yaw_rad)
        east_m = forward_m * math.sin(yaw_rad) + right_m * math.cos(yaw_rad)

        lat, lon = _offset_lat_lon(pose.latitude, pose.longitude, north_m, east_m)
        return ProjectionResult(lat, lon, valid, confidence_scale)

    @property
    def uncertainty_radius_m(self) -> float:
        """Konum belirsizlik yarıçapı (kalibrasyonla belirlenir)."""
        return self._uncertainty_m


def _offset_lat_lon(lat: float, lon: float, north_m: float, east_m: float) -> tuple[float, float]:
    """Kuzey/doğu metre offsetini (lat, lon)'a ekler."""
    d_lat = north_m / _EARTH_RADIUS_M
    d_lon = east_m / (_EARTH_RADIUS_M * math.cos(math.radians(lat)))
    return lat + math.degrees(d_lat), lon + math.degrees(d_lon)
