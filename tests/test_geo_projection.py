"""
geo_projection modülü testleri — piksel → zemin → dünya dönüşümü.

Senaryolar:
  * görüntü merkezi → sıfır displacement (kamera konumu)
  * sağ / sol piksel displacement
  * farklı altitude (GSD değişimi)
  * yaw değişimi (NED rotasyonu)
  * küçük pitch/roll → geçerli
  * threshold üstü pitch/roll → geçersiz (confidence düşer)
"""
import math

import pytest

from config import VISION_CAMERA_GEOMETRY, VISION_GEO_PROJECTION
from vision.geo_projection import GeoProjection, VehiclePose

_PROJ = GeoProjection(VISION_GEO_PROJECTION, VISION_CAMERA_GEOMETRY)

_W = 960
_H = 352
_LAT = 40.0
_LON = 29.0


def _pose(yaw=0.0, pitch=0.0, roll=0.0) -> VehiclePose:
    return VehiclePose(latitude=_LAT, longitude=_LON,
                       yaw_deg=yaw, pitch_deg=pitch, roll_deg=roll)


def test_center_projects_to_vehicle_position():
    res = _PROJ.project(_W / 2, _H / 2, _W, _H, 11.5, _pose())
    assert res.valid is True
    assert res.confidence_scale == 1.0
    assert res.latitude == pytest.approx(_LAT, abs=1e-9)
    assert res.longitude == pytest.approx(_LON, abs=1e-9)


def test_right_pixel_displacement_moves_east():
    """Sağdaki piksel (dx>0) heading=0'da doğuya (east+) gider."""
    res = _PROJ.project(_W / 2 + 100, _H / 2, _W, _H, 11.5, _pose(yaw=0))
    assert res.longitude > _LON
    assert res.latitude == pytest.approx(_LAT, abs=1e-6)


def test_left_pixel_displacement_moves_west():
    res = _PROJ.project(_W / 2 - 100, _H / 2, _W, _H, 11.5, _pose(yaw=0))
    assert res.longitude < _LON


def test_higher_altitude_gives_larger_displacement():
    """Aynı piksel kayması daha yüksek irtifada daha büyük zemin mesafesidir."""
    res_low = _PROJ.project(_W / 2 + 100, _H / 2, _W, _H, 11.5, _pose())
    res_high = _PROJ.project(_W / 2 + 100, _H / 2, _W, _H, 23.0, _pose())
    d_low = res_low.longitude - _LON
    d_high = res_high.longitude - _LON
    assert d_high > d_low


def test_yaw_rotation_changes_direction():
    """Heading -90° (batıya bakış): sağdaki piksel kuzeye gider."""
    res0 = _PROJ.project(_W / 2 + 100, _H / 2, _W, _H, 11.5, _pose(yaw=0))
    res_w = _PROJ.project(_W / 2 + 100, _H / 2, _W, _H, 11.5, _pose(yaw=-90))
    # yaw=0: dx → east (longitude artar); yaw=-90: dx → north (latitude artar).
    assert res0.longitude > _LON
    assert res_w.latitude > _LAT + 1e-5


def test_small_pitch_roll_kept_valid():
    res = _PROJ.project(_W / 2, _H / 2, _W, _H, 11.5,
                        _pose(pitch=3.0, roll=-2.0))
    assert res.valid is True
    assert res.confidence_scale == 1.0


def test_excessive_pitch_invalid():
    max_pitch = float(VISION_GEO_PROJECTION["max_pitch_deg"])
    res = _PROJ.project(_W / 2, _H / 2, _W, _H, 11.5,
                        _pose(pitch=max_pitch + 1.0))
    assert res.valid is False
    assert res.confidence_scale < 1.0


def test_excessive_roll_invalid():
    max_roll = float(VISION_GEO_PROJECTION["max_roll_deg"])
    res = _PROJ.project(_W / 2, _H / 2, _W, _H, 11.5,
                        _pose(roll=max_roll + 1.0))
    assert res.valid is False


def test_thresholds_from_config():
    assert _PROJ._max_pitch == float(VISION_GEO_PROJECTION["max_pitch_deg"])
    assert _PROJ._max_roll == float(VISION_GEO_PROJECTION["max_roll_deg"])
    assert _PROJ.uncertainty_radius_m == float(
        VISION_GEO_PROJECTION["position_uncertainty_radius_m"]
    )


def test_gsd_matches_size_filter():
    from vision.size_filter import SizeFilter
    sf = SizeFilter(VISION_CAMERA_GEOMETRY, {"min_area_px": 100.0,
                                             "max_blob_fraction": 0.75})
    assert _PROJ.gsd_at(11.5) == pytest.approx(sf.gsd_at(11.5), rel=1e-6)
