"""
Boyut filtresi — GSD tabanlı fiziksel alan doğrulaması.

Sorumluluk: şekil adayını güncel irtifa ve GSD ile beklenen piksel alanına
karşı filtreler. 11.5 m'ye HARDCODE YOKTUR.

GSD(h) = GSD_ref * (h / h_ref)   (pinhole + nadir görüş varsayımı)

Beklenen alan (fiziksel kenar + şekil formülü):
    üçgen : (√3 / 4)  * edge_m² / GSD²
    altıgen: (3√3 / 2) * edge_m² / GSD²

area_tolerance (örn. [0.60, 1.40]) beklenen alanın kabul aralığıdır.

İRTİFA BİLİNMİYORSA (state machine eski `detect(frame)` çağrısı):
  Fiziksel alan eşleşmesi yapılamaz → yalnızca mutlak alt sınır (min_area_px)
  ve kadraj oranı üst sınırı (max_blob_fraction) uygulanır (eski davranış).
  Bu, mevcut sistemin davranışını bozmaz; altitude verildiğinde fiziksel
  filtre otomatik devreye girer.
"""
from __future__ import annotations

import math
from typing import Any, Optional

# Matematiksel sabitler — geometrik formüllerin katsayıları.
_SQRT3 = math.sqrt(3.0)
_TRIANGLE_AREA_COEF = _SQRT3 / 4.0
_HEXAGON_AREA_COEF = 3.0 * _SQRT3 / 2.0

# Şekil adı → alan katsayısı.
_AREA_COEFS = {
    "TRIANGLE": _TRIANGLE_AREA_COEF,
    "HEXAGON": _HEXAGON_AREA_COEF,
}


class SizeFilter:
    """Adayları beklenen fiziksel boyutla karşılaştırır (GSD tabanlı)."""

    def __init__(self, geometry_cfg: dict[str, Any], size_cfg: dict[str, Any]) -> None:
        """
        geometry_cfg: {"gsd_ref_m_per_px", "gsd_ref_altitude_m"}.
        size_cfg: {"min_area_px", "max_blob_fraction"}.
        """
        self._gsd_ref = float(geometry_cfg["gsd_ref_m_per_px"])
        self._h_ref = float(geometry_cfg["gsd_ref_altitude_m"])
        self._min_area_px = float(size_cfg["min_area_px"])
        self._max_blob_fraction = float(size_cfg["max_blob_fraction"])

    # ── GSD ──────────────────────────────────────────────────────────────── #
    def gsd_at(self, altitude_m: float) -> float:
        """Güncel irtifada GSD (m/px) — irtifa ile doğrusal ölçeklenir."""
        alt = max(float(altitude_m), 0.1)
        return self._gsd_ref * (alt / self._h_ref)

    @staticmethod
    def expected_area_px(shape: str, real_edge_m: float, gsd: float) -> float:
        """Fiziksel kenar + GSD → beklenen piksel alanı."""
        coef = _AREA_COEFS.get(shape)
        if coef is None or gsd <= 0:
            return 0.0
        return coef * (float(real_edge_m) ** 2) / (gsd * gsd)

    @staticmethod
    def expected_edge_px(real_edge_m: float, gsd: float) -> float:
        """Fiziksel kenar + GSD → beklenen piksel kenarı."""
        if gsd <= 0:
            return 0.0
        return float(real_edge_m) / gsd

    # ── Filtre ───────────────────────────────────────────────────────────── #
    def accept(
        self,
        area_px: float,
        shape: Optional[str],
        real_edge_m: float,
        altitude_m: Optional[float],
        frame_area_px: float,
        area_tolerance: tuple[float, float],
    ) -> bool:
        """Aday alanını kabul/red eder.

        area_px: aday kontur alanı (çalışma çözünürlüğünde).
        shape: sınıflandırılmış şekil (None → fiziksel filtre uygulanamaz).
        real_edge_m: hedefin gerçek kenarı (m).
        altitude_m: güncel irtifa; None → mutlak eşikler (eski davranış).
        frame_area_px: kadraj alanı (çalışma çözünürlüğünde) — üst sınır için.
        area_tolerance: [lo, hi] — beklenen alanın kabul aralığı.
        """
        if area_px < self._min_area_px:
            return False
        if area_px > self._max_blob_fraction * frame_area_px:
            return False  # kadrajın çoğunu kaplayan blob hedef değildir
        if altitude_m is None or shape is None:
            return True  # fiziksel eşleşme yapılamaz — mutlak eşikler yeterli

        gsd = self.gsd_at(altitude_m)
        expected = self.expected_area_px(shape, real_edge_m, gsd)
        if expected <= 0:
            return False
        lo, hi = (float(t) for t in area_tolerance)
        return lo * expected <= area_px <= hi * expected
