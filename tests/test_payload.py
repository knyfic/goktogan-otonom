"""
Payload sistemi testleri.

Tüm testler SimulationBackend ile çalışır — GPIO gerekmez.
Ubuntu üzerinde: .venv/bin/python -m pytest tests/test_payload.py -v
"""
import logging
import pytest

from models import TargetColor
from payload.backends.simulation import SimulationBackend
from payload.models import MagnetId, OperationResult, PayloadMechanismState
from payload.payload_controller import PayloadController


def make_controller() -> PayloadController:
    logger = logging.getLogger("test_payload")
    backend = SimulationBackend(logger=logger)
    return PayloadController(backend=backend, logger=logger)


# ── Test 1: Initialization ────────────────────────────────────────────────── #
class TestInitialization:
    def test_magnets_on_after_init(self):
        """initialize() sonrası her iki mıknatıs ON olmalı."""
        backend = SimulationBackend()
        backend.initialize()
        status = backend.status()
        assert status.magnets.magnet_1_on is True, "Magnet 1 başlangıçta ON olmalı"
        assert status.magnets.magnet_2_on is True, "Magnet 2 başlangıçta ON olmalı"

    def test_state_ready_after_init(self):
        """Başlangıç state'i READY olmalı."""
        backend = SimulationBackend()
        backend.initialize()
        status = backend.status()
        assert status.mechanism_state == PayloadMechanismState.READY

    def test_controller_initializes_backend(self):
        """PayloadController oluşturulduğunda backend otomatik initialize edilmeli."""
        ctrl = make_controller()
        status = ctrl._backend.status()
        assert status.magnets.magnet_1_on is True
        assert status.magnets.magnet_2_on is True
        assert status.mechanism_state == PayloadMechanismState.READY


# ── Test 2: Payload 1 Release ─────────────────────────────────────────────── #
class TestPayload1Release:
    def test_magnet_1_off_magnet_2_on(self):
        """Payload 1 bırakılınca: Magnet 1 OFF, Magnet 2 DOKUNULMAZ (ON kalmalı)."""
        backend = SimulationBackend()
        backend.initialize()

        result = backend.release_payload_1()

        assert result.success, f"release_payload_1 başarısız: {result.message}"
        status = backend.status()
        assert status.magnets.magnet_1_on is False, "Magnet 1 OFF olmalıydı"
        assert status.magnets.magnet_2_on is True,  "Magnet 2 ON kalmalıydı (bozulmamalı)"

    def test_controller_first_release_targets_magnet_1(self):
        """PayloadController.release_payload() ilk çağrıda Magnet 1'i kapatmalı."""
        ctrl = make_controller()
        result = ctrl.release_payload(TargetColor.RED)

        assert result.success
        status = ctrl._backend.status()
        assert status.magnets.magnet_1_on is False
        assert status.magnets.magnet_2_on is True

    def test_carried_count_decreases_after_release_1(self):
        backend = SimulationBackend()
        backend.initialize()
        initial = backend.status().carried_count
        backend.release_payload_1()
        assert backend.status().carried_count == initial - 1


# ── Test 3: Payload 2 Release ─────────────────────────────────────────────── #
class TestPayload2Release:
    def test_magnet_2_off(self):
        """Payload 2 bırakılınca: Magnet 2 OFF."""
        backend = SimulationBackend()
        backend.initialize()

        result = backend.release_payload_2()

        assert result.success, f"release_payload_2 başarısız: {result.message}"
        status = backend.status()
        assert status.magnets.magnet_2_on is False, "Magnet 2 OFF olmalıydı"

    def test_magnet_1_untouched_during_payload_2_release(self):
        """Payload 2 bırakılırken Magnet 1 DOKUNULMAMALI."""
        backend = SimulationBackend()
        backend.initialize()
        backend.release_payload_2()
        status = backend.status()
        assert status.magnets.magnet_1_on is True, "Magnet 1 bozulmamalı"

    def test_controller_second_release_targets_magnet_2(self):
        """PayloadController.release_payload() ikinci çağrıda Magnet 2'yi kapatmalı."""
        ctrl = make_controller()
        ctrl.release_payload(TargetColor.RED)   # 1. bırakma → Magnet 1 OFF
        ctrl.release_payload(TargetColor.BLUE)  # 2. bırakma → Magnet 2 OFF

        status = ctrl._backend.status()
        assert status.magnets.magnet_1_on is False
        assert status.magnets.magnet_2_on is False

    def test_magnet_1_not_double_turned_off(self):
        """İkinci release'te Magnet 1 ayrıca kapatılmamalı (zaten OFF olabilir ama kasıtlı değil)."""
        backend = SimulationBackend()
        backend.initialize()
        # Sadece Payload 2'yi bırak (Payload 1 bırakılmamış senaryosu için set_magnet kullan)
        # Bu test: release_payload_2 Magnet 1'e dokunmaz
        result = backend.release_payload_2()
        assert result.success
        status = backend.status()
        # Magnet 1 initialize'da ON, release_2 dokunmamış olmalı
        assert status.magnets.magnet_1_on is True


# ── Test 4: Pickup ────────────────────────────────────────────────────────── #
class TestPickup:
    def test_both_magnets_on_before_pickup(self):
        """Pickup başında her iki mıknatıs ON garanti edilmeli."""
        backend = SimulationBackend()
        backend.initialize()
        # Mıknatısları off yap (worst-case)
        backend.set_magnet(MagnetId.MAGNET_1, False)
        backend.set_magnet(MagnetId.MAGNET_2, False)

        result = backend.send_pickup_command()

        assert result.success
        status = backend.status()
        assert status.magnets.magnet_1_on is True, "Pickup başında Magnet 1 ON olmalı"
        assert status.magnets.magnet_2_on is True, "Pickup başında Magnet 2 ON olmalı"

    def test_state_picking_up_after_send_command(self):
        """send_pickup_command sonrası state PICKING_UP olmalı."""
        backend = SimulationBackend()
        backend.initialize()
        backend.send_pickup_command()
        assert backend.status().mechanism_state == PayloadMechanismState.PICKING_UP

    def test_finish_pickup_returns_ready(self):
        """finish_pickup sonrası state READY olmalı."""
        backend = SimulationBackend()
        backend.initialize()
        backend.send_pickup_command()
        result = backend.finish_pickup()
        assert result.success
        assert backend.status().mechanism_state == PayloadMechanismState.READY

    def test_magnets_stay_on_after_pickup(self):
        """Pickup sonrası mıknatıslar ON kalmalı."""
        backend = SimulationBackend()
        backend.initialize()
        backend.send_pickup_command()
        backend.finish_pickup()
        status = backend.status()
        assert status.magnets.magnet_1_on is True
        assert status.magnets.magnet_2_on is True

    def test_controller_send_finish_cycle(self):
        """State machine akışı: send_pickup_command → finish_pickup."""
        ctrl = make_controller()
        ctrl.send_pickup_command(TargetColor.RED)
        result = ctrl.finish_pickup(TargetColor.RED)
        assert result.success
        assert ctrl.attached_payload == TargetColor.RED

    def test_invalid_state_finish_without_send(self):
        """send_pickup_command çağrılmadan finish_pickup başarısız olmalı."""
        backend = SimulationBackend()
        backend.initialize()
        result = backend.finish_pickup()
        assert not result.success
        assert result.result == OperationResult.INVALID_STATE


# ── Test 5: Backend İzolasyonu ────────────────────────────────────────────── #
class TestBackendIsolation:
    def test_simulation_backend_has_correct_interface(self):
        """SimulationBackend tüm interface metodlarını implement etmeli."""
        from payload.interface import PayloadBackend
        backend = SimulationBackend()
        assert isinstance(backend, PayloadBackend)

    def test_controller_works_with_simulation_backend(self):
        """PayloadController SimulationBackend ile eksiksiz çalışmalı."""
        ctrl = make_controller()
        # Tam görev akışı: init → release 1 → release 2 → pickup
        r1 = ctrl.release_payload(TargetColor.RED)
        r2 = ctrl.release_payload(TargetColor.BLUE)
        ctrl.send_pickup_command(TargetColor.RED)
        r3 = ctrl.finish_pickup(TargetColor.RED)
        assert r1.success
        assert r2.success
        assert r3.success

    def test_mock_remote_backend_has_same_interface(self):
        """RemoteRPiBackend da PayloadBackend interface'ini implement etmeli."""
        from payload.interface import PayloadBackend
        from payload.backends.remote_rpi import RemoteRPiBackend
        backend = RemoteRPiBackend(host="localhost", port=9999)
        assert isinstance(backend, PayloadBackend)

    def test_clear_attached_resets_state(self):
        """clear_attached() sonrası attached_payload None olmalı."""
        ctrl = make_controller()
        ctrl.send_pickup_command(TargetColor.RED)
        ctrl.finish_pickup(TargetColor.RED)
        assert ctrl.attached_payload is not None
        ctrl.clear_attached()
        assert ctrl.attached_payload is None


# ── Test 6: Emergency Stop ────────────────────────────────────────────────── #
class TestEmergencyStop:
    def test_emergency_stop_blocks_commands(self):
        """Emergency stop sonrası pickup komutu reddedilmeli."""
        backend = SimulationBackend()
        backend.initialize()
        backend.emergency_stop()

        result = backend.send_pickup_command()
        assert not result.success
        assert result.result == OperationResult.EMERGENCY_ACTIVE

    def test_emergency_stop_state(self):
        """Emergency stop sonrası state EMERGENCY_STOP olmalı."""
        backend = SimulationBackend()
        backend.initialize()
        backend.emergency_stop()
        assert backend.status().mechanism_state == PayloadMechanismState.EMERGENCY_STOP

    def test_reinitialize_clears_emergency(self):
        """initialize() sonrası emergency temizlenmeli ve sistem READY olmalı."""
        backend = SimulationBackend()
        backend.initialize()
        backend.emergency_stop()
        backend.initialize()   # yeniden başlat
        result = backend.send_pickup_command()
        assert result.success


# ── Test 7: Individual Magnet Control ─────────────────────────────────────── #
class TestMagnetControl:
    def test_set_magnet_1_independently(self):
        """Magnet 1 bağımsız kontrol edilebilmeli."""
        backend = SimulationBackend()
        backend.initialize()
        backend.set_magnet(MagnetId.MAGNET_1, False)
        status = backend.status()
        assert status.magnets.magnet_1_on is False
        assert status.magnets.magnet_2_on is True   # Magnet 2 etkilenmemeli

    def test_set_magnet_2_independently(self):
        """Magnet 2 bağımsız kontrol edilebilmeli."""
        backend = SimulationBackend()
        backend.initialize()
        backend.set_magnet(MagnetId.MAGNET_2, False)
        status = backend.status()
        assert status.magnets.magnet_2_on is False
        assert status.magnets.magnet_1_on is True   # Magnet 1 etkilenmemeli

    def test_set_magnet_back_on(self):
        """Kapatılan mıknatıs tekrar açılabilmeli."""
        backend = SimulationBackend()
        backend.initialize()
        backend.set_magnet(MagnetId.MAGNET_1, False)
        backend.set_magnet(MagnetId.MAGNET_1, True)
        assert backend.status().magnets.magnet_1_on is True


# ── Test 8: Hata Durumları ────────────────────────────────────────────────── #
class TestErrorCases:
    def test_release_when_nothing_to_release(self):
        """Payload yokken release_payload_1 INVALID_STATE döndürmeli."""
        backend = SimulationBackend()
        backend.initialize()
        # Tüm carried'ı sıfıra indir
        backend._carried = 0
        result = backend.release_payload_1()
        assert not result.success
        assert result.result == OperationResult.INVALID_STATE

    def test_status_returns_correct_carried_count(self):
        """carried_count doğru izlenmeli."""
        backend = SimulationBackend()
        backend.initialize()
        assert backend.status().carried_count == 2
        backend.release_payload_1()
        assert backend.status().carried_count == 1
        backend.release_payload_2()
        assert backend.status().carried_count == 0
