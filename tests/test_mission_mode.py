"""
Görev modu seçimi testleri — ENABLE_RETURN_PICKUP.

Tek config ayarı ile pickup (yükü geri alma + redrop) aşaması açılır/kapanır:

  ENABLE_RETURN_PICKUP = True  → normal bırakma → pickup → redrop → RTL
  ENABLE_RETURN_PICKUP = False → normal bırakma → (pickup YOK) → doğrudan RTL

Kritik garanti: pickup kapalıyken hiçbir pickup/redrop state'ine geçilmez ve
normal bırakma akışı (1. hedef → 2. hedef) birebir aynı çalışır.
"""
import logging
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np

import config
import state_machine as sm_mod
from models import MissionState, TargetColor, TargetType


def _real_sm():
    """Gerçek __init__ ile kurulan örnek (tüm öznitelikler hazır)."""
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
    sm.rtl_controller = MagicMock()   # gerçek MAVLink RTL çağrısını engelle
    return sm, nav


def _target(target_id: int, color: TargetColor, shape: TargetType,
            payload_released: bool = False) -> SimpleNamespace:
    return SimpleNamespace(
        target_id=target_id, color=color, shape=shape,
        latitude=1.0 + target_id * 1e-4, longitude=1.0 + target_id * 1e-4,
        confidence=0.9, payload_released=payload_released,
    )


# ── 0) Config varsayılanı ──────────────────────────────────────────────────── #
def test_config_default_keeps_pickup_enabled():
    """Varsayılan değer mevcut davranışı korumalı: pickup AKTİF (True)."""
    assert config.ENABLE_RETURN_PICKUP is True


# ── 1) Pickup açık (varsayılan) ────────────────────────────────────────────── #
def test_release_second_target_enters_pickup_when_enabled():
    """Pickup açıkken 2. yük bırakılınca NAVIGATING_TO_FIRST_TARGET'a (pickup'a)
    geçilir — mevcut davranış değişmez."""
    sm, _nav = _real_sm()
    sm.data.state = MissionState.RELEASING_PAYLOAD
    sm.data.first_target = _target(1, TargetColor.RED, TargetType.TRIANGLE,
                                   payload_released=True)
    sm.data.second_target = _target(2, TargetColor.BLUE, TargetType.HEXAGON,
                                    payload_released=False)
    sm.payload.release_payload.return_value = SimpleNamespace(success=True)

    sm._do_release()

    assert sm.data.second_target.payload_released is True
    assert sm.data.state == MissionState.NAVIGATING_TO_FIRST_TARGET
    sm.rtl_controller.start.assert_not_called()


def test_after_task2_enters_return_payload_when_enabled():
    """Pickup açıkken TASK_2_COMPLETE sonrası RETURNING_PAYLOAD'a (redrop) geçilir."""
    sm, _nav = _real_sm()
    sm.data.state = MissionState.TASK_2_COMPLETE
    sm.data.second_target = _target(2, TargetColor.BLUE, TargetType.HEXAGON)

    sm._do_after_task2()

    assert sm.data.state == MissionState.RETURNING_PAYLOAD
    sm.rtl_controller.start.assert_not_called()


# ── 2) Pickup kapalı ───────────────────────────────────────────────────────── #
def test_release_second_target_goes_rtl_when_disabled(monkeypatch):
    """Pickup kapalıyken 2. yük bırakılınca pickup'a GİDİLMEZ — doğrudan RTL."""
    monkeypatch.setattr(sm_mod, "ENABLE_RETURN_PICKUP", False)
    sm, _nav = _real_sm()
    sm.data.state = MissionState.RELEASING_PAYLOAD
    sm.data.first_target = _target(1, TargetColor.RED, TargetType.TRIANGLE,
                                   payload_released=True)
    sm.data.second_target = _target(2, TargetColor.BLUE, TargetType.HEXAGON,
                                    payload_released=False)
    sm.payload.release_payload.return_value = SimpleNamespace(success=True)

    sm._do_release()

    assert sm.data.second_target.payload_released is True
    assert sm.data.state == MissionState.RTL
    sm.rtl_controller.start.assert_called_once()


def test_after_task2_goes_rtl_when_disabled(monkeypatch):
    """Pickup kapalıyken (savunma) redrop akışına girilmez — doğrudan RTL."""
    monkeypatch.setattr(sm_mod, "ENABLE_RETURN_PICKUP", False)
    sm, _nav = _real_sm()
    sm.data.state = MissionState.TASK_2_COMPLETE
    sm.data.second_target = _target(2, TargetColor.BLUE, TargetType.HEXAGON)

    sm._do_after_task2()

    assert sm.data.state == MissionState.RTL
    sm.rtl_controller.start.assert_called_once()


# ── 3) Pickup kapalıyken normal bırakma davranışı bozulmaz ─────────────────── #
def test_first_release_unchanged_when_disabled(monkeypatch):
    """Pickup kapalıyken BİRİNCİ yük bırakma aynı akışı izler:
    1. hedefe bırak → ASCENDING_AFTER_FIRST_RELEASE (2. hedef araması)."""
    monkeypatch.setattr(sm_mod, "ENABLE_RETURN_PICKUP", False)
    sm, _nav = _real_sm()
    sm.data.state = MissionState.RELEASING_PAYLOAD
    sm.data.first_target = _target(1, TargetColor.RED, TargetType.TRIANGLE,
                                   payload_released=False)
    sm.data.second_target = None
    sm.payload.release_payload.return_value = SimpleNamespace(success=True)

    sm._do_release()

    assert sm.data.first_target.payload_released is True
    assert sm.data.state == MissionState.ASCENDING_AFTER_FIRST_RELEASE
    sm.rtl_controller.start.assert_not_called()


def test_pickup_disabled_never_reaches_pickup_states(monkeypatch):
    """Pickup kapalıyken hiçbir pickup/redrop state'ine geçilmediği doğrulanır."""
    monkeypatch.setattr(sm_mod, "ENABLE_RETURN_PICKUP", False)
    pickup_states = {
        MissionState.NAVIGATING_TO_FIRST_TARGET,
        MissionState.SEARCHING_PAYLOAD,
        MissionState.ALIGNING_PAYLOAD,
        MissionState.OFFSET_CORRECTION,
        MissionState.PICKING_UP_PAYLOAD,
        MissionState.ASCENDING_FOR_VERIFICATION,
        MissionState.VERIFYING_PAYLOAD,
        MissionState.RETURNING_PAYLOAD,
        MissionState.RELEASING_RETURNED_PAYLOAD,
    }
    sm, _nav = _real_sm()
    sm.data.state = MissionState.RELEASING_PAYLOAD
    sm.data.first_target = _target(1, TargetColor.RED, TargetType.TRIANGLE,
                                   payload_released=True)
    sm.data.second_target = _target(2, TargetColor.BLUE, TargetType.HEXAGON,
                                    payload_released=False)
    sm.payload.release_payload.return_value = SimpleNamespace(success=True)

    sm._do_release()

    assert sm.data.state not in pickup_states
    assert sm.data.state == MissionState.RTL
