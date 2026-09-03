"""
Hedef kaybı → recovery (kör bırakma YOK) davranışı testleri.

Davranış (hedef kaybında İRTİFA DÜŞÜRME YOK):
  - Search/approach irtifasında (≥4m, ör. 11.5m arama) hedef kaybolursa:
    irtifa KORUNUR ve kaydedilmiş kaba GPS'e dönülüp orada yeniden aranır
    (asla 4m'ye inilmez — eski regresyon: 11.5m → 4.3m alçalması).
  - 4m ALTINDA (drop dizisi): güvenli irtifaya (4m) yüksel, beklenen türü
    (kilit: renk/şekil) yeniden ara; bulunamazsa arama pattern'i kaldığı
    yerden devam eder.
  - Tespit olmadan bırakma yapılmaz (kör bırakma YOK).
"""
import logging
import time
from unittest.mock import MagicMock

import numpy as np

from config import (
    APPROACH_SPEED_MPS,
    COMPETITION_CORNERS,
    SEARCH_SPEED_MPS,
    TAKEOFF_ALTITUDE,
    TARGET_APPROACH_ALTITUDE,
    VERTICAL_SPEED_MPS,
)
from models import MissionState, Target, TargetColor, TargetType
from search.coverage_planner import CompetitionField, SearchPoint
from state_machine import MissionStateMachine


def _make_sm(nav=None, rough_lat=2.0, rough_lon=2.0, altitude=TAKEOFF_ALTITUDE):
    nav = nav or MagicMock()
    nav.position.return_value = MagicMock(
        relative_altitude=altitude, latitude=1.0, longitude=1.0, heading_deg=0.0,
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


# ── Search irtifasında hedef kaybı: irtifa KORUNUR, kaba GPS'e dön ────────── #
def test_lost_target_at_search_altitude_keeps_altitude_returns_to_rough():
    """FIRST hedef: 11.5m'de hedef kayboldu → 4m'ye İNMEZ; kaba GPS'e 11.5m'de döner."""
    sm, nav = _make_sm(rough_lat=2.0, rough_lon=2.0, altitude=TAKEOFF_ALTITUDE)
    sm._no_detection_count = sm._NO_DETECTION_LIMIT + 1

    sm._do_align_target()

    # İrtifa komutu YOK — asla TARGET_APPROACH_ALTITUDE (4m) ile alçalma
    nav.set_relative_altitude.assert_not_called()
    # Kaba GPS'e MEVCUT irtifada dönüş (11.5m korunur)
    nav.goto_global.assert_called_once()
    args = nav.goto_global.call_args[0]
    assert args[0] == 2.0, "kaydedilmiş hedef GPS lat'i korunmalı"
    assert args[1] == 2.0, "kaydedilmiş hedef GPS lon'u korunmalı"
    assert args[2] == TAKEOFF_ALTITUDE, "recovery irtifası search irtifası olmalı"
    # Kör bırakma yok; state align'de kalır (hedef orada yeniden aranır)
    assert sm.data.state == MissionState.ALIGNING_TARGET
    assert sm.data.pending_detection is None, "sentetik merkez tespiti ÜRETİLMEMELİ"
    assert sm._locked_color == TargetColor.RED, "kilit korunmalı (beklenen tür)"
    assert sm._locked_shape == "TRIANGLE"


def test_lost_target_second_search_keeps_altitude_returns_to_rough():
    """SECOND hedef: aynı davranış ikinci aramada da geçerli (11.5m korunur)."""
    sm, nav = _make_sm(rough_lat=3.0, rough_lon=4.0, altitude=TAKEOFF_ALTITUDE)
    sm.data.first_target = Target(
        target_id=1, shape=TargetType.TRIANGLE, color=TargetColor.RED,
        latitude=1.0, longitude=1.0, confidence=0.9,
    )
    sm._locked_color = TargetColor.BLUE
    sm._locked_shape = "HEXAGON"
    sm._no_detection_count = sm._NO_DETECTION_LIMIT + 1

    sm._do_align_target()

    nav.set_relative_altitude.assert_not_called()   # 4m'ye inme YOK
    nav.goto_global.assert_called_once()
    args = nav.goto_global.call_args[0]
    assert args[0] == 3.0 and args[1] == 4.0, "kaydedilmiş hedef GPS kullanılmalı"
    assert args[2] == TAKEOFF_ALTITUDE, "recovery irtifası 11.5m olmalı"


# ── False-positive 10s timeout: kaba GPS'e dönme döngüsü süresiz değildir ─── #
def test_lost_target_before_10s_still_recovers_to_rough():
    """Hedef son görüldüğünden 10s DOLMADIYSA mevcut davranış (kaba GPS'e
    dönüş) değişmez."""
    sm, nav = _make_sm(rough_lat=2.0, rough_lon=2.0, altitude=TAKEOFF_ALTITUDE)
    sm._no_detection_count = sm._NO_DETECTION_LIMIT + 1
    sm._last_target_seen_t = time.monotonic() - 5.0   # 5s önce görüldü (< 10s)

    sm._do_align_target()

    nav.goto_global.assert_called_once()   # kaba GPS'e dönüş devam ediyor
    assert sm.data.state == MissionState.ALIGNING_TARGET
    assert sm._locked_color == TargetColor.RED   # kilit henüz temizlenmedi


def test_lost_target_after_10s_returns_to_search_preserving_progress():
    """Hedef 10s+ boyunca tekrar görülmediyse (muhtemel false-positive):
    kaba GPS'e dönme döngüsü BİRAKILIR, search state'ine (SEARCHING_FIRST_TARGET)
    dönülür ve mevcut search_idx/search_points/search_field DEĞİŞMEZ (baştan
    başlamaz, kaldığı yerden devam eder)."""
    sm, nav = _make_sm(rough_lat=2.0, rough_lon=2.0, altitude=TAKEOFF_ALTITUDE)
    sm._no_detection_count = sm._NO_DETECTION_LIMIT + 1
    sm._last_target_seen_t = time.monotonic() - 11.0   # 11s önce (≥ 10s)

    # Search ilerlemesi zaten var olsun — değişmemeli
    sm._search_field = object()
    sm._search_points = [SearchPoint(5.0, 5.0), SearchPoint(5.0, 95.0)]
    sm._search_idx = 1

    sm._do_align_target()

    nav.goto_global.assert_not_called()   # kaba GPS'e dönme döngüsüne GİRİLMEDI
    assert sm.data.state == MissionState.SEARCHING_FIRST_TARGET
    assert sm._locked_color is None and sm._locked_shape is None
    assert sm.data.pending_detection is None
    # Search ilerlemesi korunmalı — baştan başlamadı, sıfırlanmadı
    assert sm._search_idx == 1
    assert sm._search_points == [SearchPoint(5.0, 5.0), SearchPoint(5.0, 95.0)]


def test_false_positive_timeout_resumes_from_line_leave_point():
    """Timeout sonrası arama çizgisine dönüş: drone önce AYRILMA NOKTASINA
    (search line üzerindeki konuma) uçar — kaldığı waypoint'e değil."""
    sm, nav = _make_sm(rough_lat=2.0, rough_lon=2.0, altitude=TAKEOFF_ALTITUDE)
    sm._no_detection_count = sm._NO_DETECTION_LIMIT + 1
    sm._last_target_seen_t = time.monotonic() - 11.0

    # Gerçek alan + yarıda kesilmiş search rotası (index=1 → waypoint 2'ye gidiyordu)
    field = CompetitionField(COMPETITION_CORNERS)
    sm._search_field = field
    sm._search_points = [
        SearchPoint(5.0, 5.0), SearchPoint(5.0, 60.0),
        SearchPoint(15.0, 60.0), SearchPoint(15.0, 5.0),
    ]
    sm._search_idx = 1
    sm._search_yaw_deg = field.search_direction_deg()
    # Ayrılma noktası: 1. ve 2. waypoint ARASINDA, çizgi üzerinde
    resume_lat, resume_lon = field.local_to_gps(5.0, 30.0)
    sm._search_resume_lat = resume_lat
    sm._search_resume_lon = resume_lon

    sm._do_align_target()   # 10s timeout → SEARCHING_FIRST_TARGET
    assert sm.data.state == MissionState.SEARCHING_FIRST_TARGET

    # Drone uzakta (1.0, 1.0) — ilk komut AYRILMA NOKTASINA gitmeli, waypoint'e değil
    nav.goto_global.reset_mock()
    sm._advance_search()
    call = nav.goto_global.call_args[0]
    assert call[0] == pytest.approx(resume_lat, abs=1e-6), "ayrılma noktasına dönülmeli"
    assert call[1] == pytest.approx(resume_lon, abs=1e-6)
    assert sm._search_idx == 1, "henüz waypoint ilerletilmemeli"

    # Drone ayrılma noktasına varınca → kaldığı waypoint'ten devam
    nav.position.return_value = MagicMock(
        relative_altitude=TAKEOFF_ALTITUDE, latitude=resume_lat, longitude=resume_lon,
        heading_deg=0.0,
    )
    nav.goto_global.reset_mock()
    sm._advance_search()
    assert sm._search_resume_lat is None and sm._search_resume_lon is None
    wp = sm._search_points[sm._search_idx]
    wp_lat, wp_lon = field.local_to_gps(wp.x_m, wp.y_m)
    call = nav.goto_global.call_args[0]
    assert call[0] == pytest.approx(wp_lat, abs=1e-6), "kaldığı waypoint'e devam"
    assert call[1] == pytest.approx(wp_lon, abs=1e-6)


def test_advance_search_returns_to_resume_point_even_without_timeout():
    """Ayrılma noktası set edildiyse (_return_search ile her dönüşte) önce oraya
    dönülür — bu yalnızca 10s timeout'a değil, tüm search dönüşlerine uygulanır
    (hedef kaybı/REACQUIRE/classify sonrası da çizgi korunur)."""
    sm, nav = _make_sm(altitude=TAKEOFF_ALTITUDE)
    sm.data.state = MissionState.SEARCHING_FIRST_TARGET
    field = CompetitionField(COMPETITION_CORNERS)
    sm._search_field = field
    sm._search_points = [SearchPoint(5.0, 5.0), SearchPoint(5.0, 60.0)]
    sm._search_idx = 1
    sm._search_yaw_deg = field.search_direction_deg()
    resume_lat, resume_lon = field.local_to_gps(5.0, 30.0)
    sm._search_resume_lat = resume_lat
    sm._search_resume_lon = resume_lon

    nav.position.return_value = MagicMock(
        relative_altitude=TAKEOFF_ALTITUDE, latitude=1.0, longitude=1.0, heading_deg=0.0,
    )
    nav.goto_global.reset_mock()
    sm._advance_search()

    call = nav.goto_global.call_args[0]
    assert call[0] == pytest.approx(resume_lat, abs=1e-6)
    assert call[1] == pytest.approx(resume_lon, abs=1e-6)
    assert sm._search_idx == 1


def test_lost_target_no_timeout_set_recovers_normally():
    """_last_target_seen_t hiç set edilmediyse (ör. eski/edge davranış) mevcut
    kaba GPS'e dönüş davranışı bozulmaz."""
    sm, nav = _make_sm(rough_lat=2.0, rough_lon=2.0, altitude=TAKEOFF_ALTITUDE)
    sm._no_detection_count = sm._NO_DETECTION_LIMIT + 1
    sm._last_target_seen_t = None

    sm._do_align_target()

    nav.goto_global.assert_called_once()
    assert sm.data.state == MissionState.ALIGNING_TARGET


def test_lost_target_no_detection_never_commands_4m():
    """Hedef hiç görünmese bile irtifa komutu 4m'ye (4.3m) düşemez."""
    sm, nav = _make_sm(altitude=TAKEOFF_ALTITUDE)
    sm._no_detection_count = sm._NO_DETECTION_LIMIT + 1

    for _ in range(5):
        sm._do_align_target()

    nav.set_relative_altitude.assert_not_called()
    assert sm.data.state != MissionState.TARGET_ALIGNED, "kör bırakma yok"


# ── Drop dizisi (4m altı): eski REACQUIRE davranışı korunur ───────────────── #
def test_lost_target_below_reacquire_altitude_ascends_to_4m():
    """4m ALTINDA hedef kaybolursa (drop dizisi) → eski davranış: 4m'ye yüksel."""
    sm, nav = _make_sm(rough_lat=1.01, rough_lon=1.01, altitude=2.0)
    sm._no_detection_count = sm._NO_DETECTION_LIMIT + 1

    sm._do_align_target()

    assert sm._align_phase == "REACQUIRE", "4m altında yükseliş + yeniden arama"
    assert sm.data.state == MissionState.ALIGNING_TARGET
    nav.goto_global.assert_not_called()   # kaba GPS'e geri gitme — yüksel + ara
    assert nav.set_max_speed.call_args[0][0] == APPROACH_SPEED_MPS
    nav.set_relative_altitude.assert_called_once_with(TARGET_APPROACH_ALTITUDE)


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
    sm, _nav = _make_sm(altitude=TAKEOFF_ALTITUDE)
    sm._no_detection_count = sm._NO_DETECTION_LIMIT + 1

    for _ in range(5):
        sm._do_align_target()

    # Search irtifasında kayıp → kaba GPS'e dönüş döngüsü (irtifa düşmez);
    # asla bırakmaya (TARGET_ALIGNED) geçilmez.
    assert sm.data.state != MissionState.TARGET_ALIGNED, "kör bırakma yok"
    assert sm.data.state == MissionState.ALIGNING_TARGET, \
        "search irtifasında recovery kaba GPS'e dönüşle devam eder"


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
