"""
Temporal doğrulayıcı — tek frame detection "kesin hedef" değildir.

Sorumluluk: son N frame içindeki adayları tutarlılık açısından doğrular.

- Aynı (renk, şekil) adaylarının DÜNYA koordinatı birbirine yeterince yakınsa
  (position_match_radius_m) tutarlı sayılır.
- Dünya koordinatı yoksa (pose/altitude verilmediyse) PİKSEL-uzay yarıçap
  fallback'i kullanılır (pixel_match_radius_px).
- window_size frame içinde min_consistent_ratio oranında tutarlılık → valid.
- Timestamp: her güncelleme çağrısına frame zamanı verilir; en eski
  girişler pencere dışına düşer (deque maxlen).
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Optional


@dataclass
class TemporalEntry:
    """Bir frame'deki tek bir aday."""

    color: str
    shape: str
    cx: float
    cy: float
    frame_t: float
    world_lat: Optional[float] = None
    world_lon: Optional[float] = None


@dataclass
class _TrackedType:
    """Pencere içinde aynı (renk, şekil) tipinin girişleri."""

    entries: deque = field(default_factory=list)


class TemporalValidator:
    """Config'ten okunan pencere/tutarlılık parametreleriyle doğrular."""

    def __init__(self, cfg: dict[str, Any]) -> None:
        """
        cfg: {"enabled", "window_size", "min_consistent_ratio",
              "min_consistent_frames", "position_match_radius_m",
              "pixel_match_radius_px"}.
        """
        self._enabled = bool(cfg["enabled"])
        self._window = int(cfg["window_size"])
        self._min_ratio = float(cfg["min_consistent_ratio"])
        self._min_frames = int(cfg["min_consistent_frames"])
        self._match_radius_m = float(cfg["position_match_radius_m"])
        self._pixel_radius = float(cfg["pixel_match_radius_px"])
        # Tip → pencere içi girişleri.
        self._history: dict[tuple[str, str], deque] = {}
        self._frame_counter: int = 0

    def reset(self) -> None:
        """Yeni hedef aramasında temporal belleği temizle."""
        self._history.clear()
        self._frame_counter = 0

    @property
    def enabled(self) -> bool:
        return self._enabled

    def validate(self, entries: list[TemporalEntry]) -> list[TemporalEntry]:
        """Yeni frame adaylarını pencereyle karşılaştırıp doğrulananları döner.

        Doğrulanan aday, kendi tipi içinde yeterli oranda eşleşme gören
        adaydır (kendi görüldüğü frame dahil).
        """
        if not self._enabled or not entries:
            return []

        self._frame_counter += 1
        validated: list[TemporalEntry] = []
        for entry in entries:
            key = (entry.color, entry.shape)
            bucket = self._history.setdefault(key, deque(maxlen=self._window))
            bucket.append(entry)

            # Aynı tipteki tüm pencere girişleriyle eşleşme sayısı.
            matches = 0
            for other in bucket:
                if self._matches(entry, other):
                    matches += 1
            # Yetersiz frame sayısı: tek frame asla "kesin hedef" değildir.
            if len(bucket) < self._min_frames:
                continue
            ratio = matches / max(len(bucket), 1)
            if ratio >= self._min_ratio:
                validated.append(entry)
        return validated

    def _matches(self, a: TemporalEntry, b: TemporalEntry) -> bool:
        """İki aday aynı fiziksel hedefe ait mi?"""
        if a.world_lat is not None and a.world_lon is not None \
                and b.world_lat is not None and b.world_lon is not None:
            return (
                _world_distance_m(a.world_lat, a.world_lon, b.world_lat, b.world_lon)
                <= self._match_radius_m
            )
        # Piksel-uzay fallback (dünya koordinatı yokken).
        d = math.hypot(a.cx - b.cx, a.cy - b.cy)
        return d <= self._pixel_radius


def _world_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Equirectangular yaklaşım — küçük mesafeler için yeterli."""
    r_lat1 = math.radians(lat1)
    r_lat2 = math.radians(lat2)
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    x = d_lon * math.cos((r_lat1 + r_lat2) / 2.0)
    y = d_lat
    return math.hypot(x, y) * 6371000.0
