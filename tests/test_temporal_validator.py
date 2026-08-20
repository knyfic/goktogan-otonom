"""
temporal_validator modülü testleri — N frame tutarlılık.

Senaryolar:
  * tutarlı adaylar → validated
  * tutarsız adaylar → rejected
  * yetersiz frame sayısı → rejected (tek frame asla "kesin" değildir)
  * position tolerance (dünya koordinatı)
  * piksel-uzay fallback
"""
import pytest

from config import VISION_TEMPORAL
from vision.temporal_validator import TemporalEntry, TemporalValidator

# Birim testler modülün KENDİ davranışını test eder → temporal etkin.
# (Pipeline entegrasyonunda default kapalı: state machine stable_detection yeterli.)
_TEMPORAL_ON = {**VISION_TEMPORAL, "enabled": True}


def _entry(color="RED", shape="TRIANGLE", cx=480.0, cy=176.0, lat=None, lon=None):
    return TemporalEntry(color=color, shape=shape, cx=cx, cy=cy,
                         frame_t=0.0, world_lat=lat, world_lon=lon)


@pytest.fixture
def validator():
    return TemporalValidator(_TEMPORAL_ON)


def test_insufficient_frames_rejected(validator):
    """Yetersiz frame sayısı: tek aday doğrulanmaz (min_consistent_frames)."""
    assert validator.validate([_entry()]) == []
    assert validator.validate([_entry()]) == []


def test_consistent_frames_validated(validator):
    """Aynı konumdaki adaylar pencere dolunca doğrulanır."""
    for _ in range(VISION_TEMPORAL["min_consistent_frames"]):
        out = validator.validate([_entry()])
    assert len(out) == 1


def test_inconsistent_positions_rejected(validator):
    """Konum her frame'de çok değişirse tutarlılık sağlanmaz."""
    results = []
    for dx in (0, 300, 600, 900):
        out = validator.validate([_entry(cx=480 + dx)])
        results.extend(out)
    assert results == [], "sürekli kaybolan/atlayan adaylar doğrulanmamalı"


def test_world_position_tolerance():
    """Dünya koordinatı: radius içindeki adaylar tutarlı, dışındakiler değil."""
    v = TemporalValidator(_TEMPORAL_ON)
    lat0 = 40.0
    lon0 = 29.0
    # ~0.00001° ≈ 1.1m — position_match_radius_m (1.5m) içinde.
    for i in range(_TEMPORAL_ON["min_consistent_frames"]):
        out = v.validate([_entry(lat=lat0 + i * 1e-5, lon=lon0)])
    assert len(out) >= 1, "küçük dünya kayması tutarlı sayılmalı"

    v2 = TemporalValidator(_TEMPORAL_ON)
    for i in range(_TEMPORAL_ON["min_consistent_frames"]):
        out = v2.validate([_entry(lat=lat0 + i * 1e-3, lon=lon0)])  # ~100m kayma
    assert out == [], "büyük dünya kayması tutarsız sayılmalı"


def test_mixed_consistency_ratio():
    """6 frame'de 4 tutarlı + 2 tutarsız → %66 ≥ %60 → doğrulanır."""
    v = TemporalValidator(_TEMPORAL_ON)
    validated = []
    for i in range(6):
        cx = 480.0 if i % 3 != 2 else 800.0  # 2 frame tutarsız
        out = v.validate([_entry(cx=cx)])
        validated.extend(out)
    assert any(e.cx == 480.0 for e in validated)


def test_disabled_returns_empty():
    cfg = dict(VISION_TEMPORAL)
    cfg["enabled"] = False
    v = TemporalValidator(cfg)
    assert v.validate([_entry()]) == []


def test_reset_clears_history():
    v = TemporalValidator(_TEMPORAL_ON)
    for _ in range(_TEMPORAL_ON["min_consistent_frames"]):
        v.validate([_entry()])
    v.reset()
    assert v.validate([_entry()]) == [], "reset sonrası pencere boş"
