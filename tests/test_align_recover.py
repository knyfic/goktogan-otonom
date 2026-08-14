"""
Hedef üzerinde kabul (recover yerine iniş) davranışı testleri.

İHA hedefe yaklaştığında hedef görüntüyü doldurur → şekil doğrulaması bozulur
→ tespit kaybolur. Eski davranış kaba GPS'e dönüp tekrar yaklaşmak (sonsuz
döngü → daire çizme). Yeni davranış: İHA hedef kaba GPS'ine yakınken tespit
kaybını "hedef üzerinde" kabul edip inişe geçer.
"""
from unittest.mock import MagicMock

import numpy as np

from models import TargetColor
from state_machine import MissionStateMachine


def _sm():
    sm = object.__new__(MissionStateMachine)
    sm._target_rough_lat = 1.0
    sm._target_rough_lon = 1.0
    sm._locked_color = None
    sm._locked_shape = None
    sm.nav = MagicMock()
    return sm


def _pos(lat: float, lon: float):
    return MagicMock(latitude=lat, longitude=lon)


def test_is_over_rough_target_near():
    sm = _sm()
    sm.nav.position.return_value = _pos(1.00001, 1.00001)  # ~1.5m kuzeydoğu
    assert sm._is_over_rough_target() is True


def test_is_over_rough_target_far():
    sm = _sm()
    sm.nav.position.return_value = _pos(1.0001, 1.0001)  # ~15m
    assert sm._is_over_rough_target() is False


def test_is_over_rough_target_none():
    sm = _sm()
    sm._target_rough_lat = None
    sm.nav.position.return_value = _pos(1.0, 1.0)
    assert sm._is_over_rough_target() is False


def test_synthetic_center_detection():
    sm = _sm()
    det = sm._synthetic_center_detection(np.zeros((704, 1920, 3), dtype=np.uint8))
    assert det.center_x == 960.0
    assert det.center_y == 352.0
    assert det.color is not None
    assert det.confidence == 0.5


def test_synthetic_center_detection_uses_lock():
    sm = _sm()
    sm._locked_color = TargetColor.RED
    sm._locked_shape = "TRIANGLE"
    det = sm._synthetic_center_detection(np.zeros((704, 1920, 3), dtype=np.uint8))
    assert det.color == TargetColor.RED
    assert det.shape == "TRIANGLE"


# ── Yaw kilidi: hizalama komutlarında İHA burnu sabitlenir ─────────────────── #
import logging
import math
from unittest.mock import patch

import pytest


def _mock_master():
    master = MagicMock()
    master.target_system = 1
    master.target_component = 1
    master.wait_heartbeat.return_value = None
    master.mode_mapping.return_value = {"GUIDED": 4}
    master.recv_match.return_value = None
    return master


@patch("flight.navigation.mavutil")
def test_goto_global_yaw_lock_active(mavutil_mock):
    """yaw_deg verilirse yaw AKTİF olur ve o heading gönderilir (burnu sabitler)."""
    from flight.navigation import Navigation

    master = _mock_master()
    mavutil_mock.mavlink_connection.return_value = master
    nav = Navigation("udp:127.0.0.1:14550")
    nav.position = MagicMock(
        return_value=MagicMock(latitude=1.0, longitude=1.0,
                               heading_deg=90.0, relative_altitude=2.0)
    )
    nav.goto_global(1.0001, 1.0001, 2.0, yaw_deg=45.0)
    args = master.mav.set_position_target_global_int_send.call_args[0]
    type_mask = args[4]
    assert (type_mask >> 10) & 1 == 0, "yaw ignore biti KAPALI olmalı (yaw aktif)"
    assert args[14] == pytest.approx(math.radians(45.0)), "yaw alanı heading olmalı"


@patch("flight.navigation.mavutil")
def test_goto_global_default_keeps_old_behavior(mavutil_mock):
    """yaw_deg verilmezse eski davranış: yaw ignore (arama/RTL vs. değişmez)."""
    from flight.navigation import Navigation

    master = _mock_master()
    mavutil_mock.mavlink_connection.return_value = master
    nav = Navigation("udp:127.0.0.1:14550")
    nav.position = MagicMock(
        return_value=MagicMock(latitude=1.0, longitude=1.0,
                               heading_deg=90.0, relative_altitude=2.0)
    )
    nav.goto_global(1.0001, 1.0001, 2.0)
    args = master.mav.set_position_target_global_int_send.call_args[0]
    type_mask = args[4]
    assert (type_mask >> 10) & 1 == 1, "yaw ignore biti açık kalmalı (eski davranış)"


# ── Hedef çok yakın (bbox büyük) → komut yok, hizalı sayılır ──────────────── #
def test_align_target_big_bbox_skips_command():
    """Hedef görüntüyü dolduruyorsa (bbox >= %50 frame) align komutu gönderilmez;
    hedef üzerinde kabul edilir. Ancak tek frame değil — ardışık onay (ALIGNED_
    FRAMES_REQUIRED) dolunca inişe geçilir (milisaniyelik denk gelmeye karşı)."""
    import state_machine as sm_mod
    from models import MissionState

    nav = MagicMock()
    nav.position.return_value = MagicMock(
        relative_altitude=4.0, latitude=1.0, longitude=1.0, heading_deg=0.0,
    )
    camera = MagicMock()
    camera.latest.return_value = np.zeros((704, 1920, 3), dtype=np.uint8)

    sm = sm_mod.MissionStateMachine(
        navigation=nav, camera=camera, payload=MagicMock(),
        logger=logging.getLogger("t"),
    )
    sm.data.state = MissionState.ALIGNING_TARGET
    sm._align_phase = "HORIZONTAL"
    sm._last_align_cmd_t = 0.0
    sm._target_rough_lat = 1.0
    sm._target_rough_lon = 1.0
    sm.position_controller = MagicMock()   # align komutları bu mock üzerinden izlenir
    sm.position_controller.align_detection.return_value = False

    det = MagicMock()
    det.center_x = 1000.0
    det.center_y = 400.0
    det.bbox_width_px = 1500.0   # >= 0.5 * 1920
    det.confidence = 0.9
    sm.data.pending_detection = det

    # Tek çağrı: bbox büyük → komut yok, ama ardışık onay dolmadı → henüz iniş yok
    sm._do_align_target()
    sm.position_controller.align_detection.assert_not_called()
    assert sm._align_phase == "HORIZONTAL"

    # Ardışık onay (ALIGNED_FRAMES_REQUIRED) dolunca kademeli iniş fazına geçilir
    from config import ALIGNED_FRAMES_REQUIRED
    for _ in range(ALIGNED_FRAMES_REQUIRED - 1):
        sm._last_align_cmd_t = 0.0
        sm._do_align_target()
    sm.position_controller.align_detection.assert_not_called()
    assert sm._align_phase == "DESCENDING"
