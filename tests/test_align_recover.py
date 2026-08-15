"""
Hedef kaybı → REACQUIRE (kör bırakma YOK) davranışı testleri.

Eski davranış: İHA hedef kaba GPS'ine yakınken tespit kaybını "hedef üzerinde"
KABUL EDİP sentetik merkez tespitiyle inişe/kör bırakmaya geçiyordu (hedef aslında
orada olmasa bile).

Yeni davranış: tespit limit dolunca — nerede olursa olsun — güvenli irtifaya (4m)
yüksel, beklenen türü (kilit: renk/şekil) yeniden ara; bulunamazsa arama pattern'i
kaldığı yerden devam eder. Tespit olmadan bırakma yapılmaz.
"""
import logging
from unittest.mock import MagicMock

import numpy as np

from config import APPROACH_SPEED_MPS, SEARCH_SPEED_MPS, TARGET_APPROACH_ALTITUDE, VERTICAL_SPEED_MPS
from models import MissionState, TargetColor
from state_machine import MissionStateMachine


def _make_sm(nav=None, rough_lat=1.0, rough_lon=1.0):
    nav = nav or MagicMock()
    nav.position.return_value = MagicMock(
        relative_altitude=4.0, latitude=1.0, longitude=1.0, heading_deg=0.0,
        battery_voltage_v=0.0, battery_percent=-1,
    )
    camera = MagicMock()
    camera.latest.return_value = np.zeros((704, 1920, 3), dtype=np.uint8)

    sm = MissionStateMachine(
        navigation=nav, camera=camera, payload=MagicMock(),
        logger=logging.getLogger("t"),
    )
    sm.data.state = MissionState.ALIGNING_TARGET
    sm._align_phase = "HORIZONTAL"
    sm._last_align_cmd_t = 0.0
    sm._target_rough_lat = rough_lat
    sm._target_rough_lon = rough_lon
    sm._locked_color = TargetColor.RED
    sm._locked_shape = "TRIANGLE"
    sm.data.pending_detection = None
    return sm, nav


def test_lost_target_over_rough_triggers_reacquire_no_blind_drop():
    """Tespit kaybı + limit doldu → KÖR BIRAKMA YOK: REACQUIRE'a geç (4m yüksel)."""
    sm, nav = _make_sm()   # drone kaba GPS'te (hedefin üstünde)
    sm._no_detection_count = sm._NO_DETECTION_LIMIT + 1

    sm._do_align_target()

    assert sm._align_phase == "REACQUIRE", "tespit yokken inişe/bırakmaya geçilmemeli"
    assert sm.data.state == MissionState.ALIGNING_TARGET, "TARGET_ALIGNED'e geçilmemeli (kör bırakma)"
    assert sm.data.pending_detection is None, "sentetik merkez tespiti ÜRETİLMEMELİ"
    # Kilit korunur → yeniden arama yalnızca beklenen türü arar (kırmızı üçgen)
    assert sm._locked_color == TargetColor.RED
    assert sm._locked_shape == "TRIANGLE"
    # Yükselme komutu: APPROACH hızı + TARGET_APPROACH_ALTITUDE
    assert nav.set_max_speed.call_args[0][0] == APPROACH_SPEED_MPS
    nav.set_relative_altitude.assert_called_once_with(TARGET_APPROACH_ALTITUDE)


def test_lost_target_far_from_rough_also_reacquires():
    """Kaba GPS'ten uzakta kaybolsa da → yüksel + yeniden ara (kaba GPS'e dönüş YOK)."""
    sm, nav = _make_sm(rough_lat=1.01, rough_lon=1.01)   # ~1.4km uzaklıkta kaba GPS
    sm._no_detection_count = sm._NO_DETECTION_LIMIT + 1

    sm._do_align_target()

    assert sm._align_phase == "REACQUIRE"
    assert sm.data.state == MissionState.ALIGNING_TARGET
    nav.goto_global.assert_not_called()   # kaba GPS'e geri gitme — yüksel + ara


def test_reacquire_ascend_done_returns_to_search_keeping_lock():
    """Yükselme bitince → search'e dön (kilit korunur, search hızı, pattern aynı)."""
    sm, nav = _make_sm()
    sm._search_idx = 5                       # pattern kaldığı yer
    sm.data.first_target = None
    sm._align_phase = "REACQUIRE"
    sm._alt_waiting = True
    sm._alt_target = TARGET_APPROACH_ALTITUDE
    nav.position.return_value = MagicMock(
        relative_altitude=TARGET_APPROACH_ALTITUDE, latitude=1.0, longitude=1.0,
        heading_deg=0.0, battery_voltage_v=0.0, battery_percent=-1,
    )

    sm._do_align_target()

    assert sm.data.state == MissionState.SEARCHING_FIRST_TARGET
    assert nav.set_max_speed.call_args[0][0] == SEARCH_SPEED_MPS, "search hızı geri gelmeli"
    assert sm._locked_color == TargetColor.RED, "kilit korunmalı (beklenen tür)"
    assert sm._locked_shape == "TRIANGLE"
    assert sm._search_idx == 5, "arama kaldığı yerden devam etmeli"


def test_reacquire_keeps_search_points_for_second_target():
    """İkinci hedef aranırken kaybolursa → SEARCHING_SECOND_TARGET'e döner."""
    from models import Target, TargetType

    sm, nav = _make_sm()
    sm.data.first_target = Target(
        target_id=1, shape=TargetType.TRIANGLE, color=TargetColor.RED,
        latitude=1.0, longitude=1.0, confidence=0.9,
    )
    sm._search_points = [object(), object(), object()]
    sm._search_idx = 2
    sm._align_phase = "REACQUIRE"
    sm._alt_waiting = True
    sm._alt_target = TARGET_APPROACH_ALTITUDE
    nav.position.return_value = MagicMock(
        relative_altitude=TARGET_APPROACH_ALTITUDE, latitude=1.0, longitude=1.0,
        heading_deg=0.0, battery_voltage_v=0.0, battery_percent=-1,
    )

    sm._do_align_target()

    assert sm.data.state == MissionState.SEARCHING_SECOND_TARGET
    assert len(sm._search_points) == 3
    assert sm._search_idx == 2


def test_no_blind_drop_without_real_detection():
    """Çekirdek garanti: gerçek tespit olmadan TARGET_ALIGNED'e asla geçilmez."""
    sm, _nav = _make_sm()
    sm._no_detection_count = sm._NO_DETECTION_LIMIT + 1

    for _ in range(5):
        sm._do_align_target()

    # Yükselme mock'ta anında biter → yeniden aramaya dönülür; asla bırakmaya değil.
    assert sm.data.state != MissionState.TARGET_ALIGNED, "kör bırakma yok"
    assert sm.data.state in (MissionState.SEARCHING_FIRST_TARGET,
                             MissionState.SEARCHING_SECOND_TARGET), \
        "yeniden aramaya dönülür"


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

    # Ardışık onay (TARGET_ALIGNED_FRAMES_REQUIRED) dolunca kademeli iniş fazına geçilir
    from config import TARGET_ALIGNED_FRAMES_REQUIRED
    for _ in range(TARGET_ALIGNED_FRAMES_REQUIRED - 1):
        sm._last_align_cmd_t = 0.0
        sm._do_align_target()
    sm.position_controller.align_detection.assert_not_called()
    assert sm._align_phase == "DESCENDING"


def test_align_target_premature_centering_does_not_descend():
    """Şansa birkaç frame merkezde kalma (eski 3 onay) inişi BAŞLATMAMALI —
    yalnızca TARGET_ALIGNED_FRAMES_REQUIRED (8) ARDIŞIK onaydan sonra inilir."""
    import state_machine as sm_mod
    from config import ALIGNED_FRAMES_REQUIRED, TARGET_ALIGNED_FRAMES_REQUIRED
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
    sm.position_controller = MagicMock()
    sm.position_controller.align_detection.return_value = True

    det = MagicMock()
    det.center_x = 960.0
    det.center_y = 352.0
    det.bbox_width_px = 200.0   # küçük bbox → pixel-center yoluna girer
    det.confidence = 0.9
    sm.data.pending_detection = det

    # Eski eşik (3 onay) kadar merkezde kalma → YENİ eşik (8) dolmaz, iniş YOK
    for _ in range(ALIGNED_FRAMES_REQUIRED):
        sm._last_align_cmd_t = 0.0
        sm._do_align_target()
    assert sm._align_phase == "HORIZONTAL", "erken merkezleme iniş başlatmamalı"
    assert sm._aligned_frame_count == ALIGNED_FRAMES_REQUIRED

    # Kalan onaylar tamamlanınca (8) kademeli iniş başlar
    for _ in range(TARGET_ALIGNED_FRAMES_REQUIRED - ALIGNED_FRAMES_REQUIRED):
        sm._last_align_cmd_t = 0.0
        sm._do_align_target()
    assert sm._align_phase == "DESCENDING"
