"""
Frame kapısı — motion blur riskli frame'leri pipeline başında eleme.

Sorumluluk: gyro (açısal hız) bilgisi varsa yüksek açısal hızı olan frame'leri
elemek. Gyro verisi YOKSA pipeline ÇALIŞMAYA DEVAM EDER (config'e göre).

- Yüksek gyro otomatik olarak kesin blur anlamına gelmez; bu yalnızca
  configurable bir frame quality gate'dir (default kapalıdır: enabled=False
  ile detection hiç etkilenmez).
- require_gyro_data=False iken gyro yokluğu frame'i engellemez.
"""
from __future__ import annotations

from typing import Any, Optional


class FrameGate:
    """Config'ten okunan gyro eşiğiyle frame kabul/red kararı."""

    def __init__(self, cfg: dict[str, Any]) -> None:
        """
        cfg: {"enabled", "max_gyro_deg_s", "require_gyro_data"}.
        """
        self._enabled = bool(cfg["enabled"])
        self._max_gyro = float(cfg["max_gyro_deg_s"])
        self._require_gyro = bool(cfg["require_gyro_data"])

    def accept(self, gyro_deg_s: Optional[float]) -> bool:
        """Frame pipeline'a girsin mi?

        gyro_deg_s: açısal hız (deg/s); None = gyro verisi yok.
        """
        if not self._enabled:
            return True
        if gyro_deg_s is None:
            return not self._require_gyro
        return abs(float(gyro_deg_s)) <= self._max_gyro
