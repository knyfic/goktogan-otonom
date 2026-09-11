"""
Payload ARAMA denemesi sınırı testleri.

SEARCHING_PAYLOAD, PAYLOAD_SEARCH_TIMEOUT içinde payload bulamazsa yükselip
hedefi yeniden ortalar ve tekrar arar. Bu döngü sınırsız kalmamalı:
MAX_PAYLOAD_SEARCH_ATTEMPTS aşılırsa görev başarısız → RTL (TASK_2_FAILED).

  • Limit ALTINDA  → ASCENDING_FOR_PAYLOAD_RETRY (yeniden dene)
  • Limit DOLUNCA  → TASK_2_FAILED + mission_failed=True
  • Payload BULUNUNCA → sayaç sıfırlanır (yeni deneme bütçesi)
"""
import logging
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np

from config import MAX_PAYLOAD_SEARCH_ATTEMPTS, PAYLOAD_SEARCH_TIMEOUT
from models import MissionState, TargetColor, TargetType


def _real_sm():
    """Gerçek __init__ ile kurulan örnek; nav/kamera/payload mock."""
    import state_machine as sm_mod

    nav = MagicMock()
    nav.position.return_value = MagicMock(
        relative_altitude=2.0, latitude=1.0, longitude=1.0, heading_deg=0.0,
    )
    camera = MagicMock()
    camera.latest.return_value = np.zeros((704, 1920, 3), dtype=np.uint8)

    sm = sm_mod.MissionStateMachine(
        navigation=nav, camera=camera, payload=MagicMock(),
        logger=logging.getLogger("t"),
    )
    sm.rtl_controller = MagicMock()
    sm.data.state = MissionState.SEARCHING_PAYLOAD
    sm.data.first_target = SimpleNamespace(
        target_id=1, color=TargetColor.BLUE, shape=TargetType.HEXAGON,
        latitude=1.0, longitude=1.0, confidence=0.9, payload_released=True,
    )
    return sm, nav


def _force_timeout(sm) -> None:
    """SEARCHING_PAYLOAD timeout'unu tetikle."""
    sm._payload_search_start = time.monotonic() - PAYLOAD_SEARCH_TIMEOUT - 1.0
    sm._do_search_payload()


def test_below_limit_retries():
    """Limit altındaki timeout'lar yeniden denemeye (ASCENDING_FOR_PAYLOAD_RETRY)
    yönlendirmeli — mevcut davranış korunur."""
    sm, _nav = _real_sm()
    for i in range(MAX_PAYLOAD_SEARCH_ATTEMPTS - 1):
        _force_timeout(sm)
        assert sm.data.state == MissionState.ASCENDING_FOR_PAYLOAD_RETRY, \
            f"{i+1}. deneme limit altı — yeniden denemeli"
        # Döngü: irtifa/merkezleme sonrası tekrar SEARCHING_PAYLOAD'a dönülür
        sm.data.state = MissionState.SEARCHING_PAYLOAD


def test_limit_reached_fails_to_rtl():
    """Limit dolunca görev başarısız → TASK_2_FAILED (RTL)."""
    sm, _nav = _real_sm()
    for _ in range(MAX_PAYLOAD_SEARCH_ATTEMPTS):
        _force_timeout(sm)
        if sm.data.state != MissionState.TASK_2_FAILED:
            sm.data.state = MissionState.SEARCHING_PAYLOAD

    assert sm.data.state == MissionState.TASK_2_FAILED
    assert sm.data.mission_failed is True


def test_successful_find_resets_counter():
    """Payload bulununca deneme sayacı sıfırlanır ve ALIGNING_PAYLOAD'a geçilir."""
    sm, _nav = _real_sm()
    sm._payload_search_attempts = MAX_PAYLOAD_SEARCH_ATTEMPTS - 1   # neredeyse limit
    sm._payload_search_start = time.monotonic()   # timeout YOK

    best = SimpleNamespace(
        color=TargetColor.RED, center_x=480.0, center_y=350.0,
        confidence=0.9, width_px=20.0, height_px=20.0, area_px=400.0,
    )
    sm.payload_detector = MagicMock()
    sm.payload_detector.detect.return_value = [best]
    sm._filter_payload_results = MagicMock(side_effect=lambda results, alt: results)
    sm.payload_tracker = MagicMock()
    sm.payload_tracker.update.return_value = best

    sm._do_search_payload()

    assert sm.data.state == MissionState.ALIGNING_PAYLOAD
    assert sm._payload_search_attempts == 0
