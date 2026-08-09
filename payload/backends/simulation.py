"""
SimulationBackend — Gazebo ile çalışırken kullanılır.

Gerçek GPIO yoktur. Tüm operasyonlar log + state güncelleme olarak simüle edilir.
"""
from __future__ import annotations

import logging
import time

from payload.interface import PayloadBackend
from payload.models import (
    MagnetId,
    MagnetState,
    OperationResult,
    PayloadMechanismState,
    PayloadOperationResult,
    PayloadSystemStatus,
)


class SimulationBackend(PayloadBackend):
    """
    Saf yazılım simülasyonu.
    GPIO çağrısı yoktur; tüm değişiklikler bellekte tutulur.
    """

    def __init__(self, logger: logging.Logger | None = None) -> None:
        self._log = logger or logging.getLogger(__name__)
        self._state = PayloadMechanismState.UNINITIALIZED
        self._magnets = MagnetState(magnet_1_on=False, magnet_2_on=False)
        self._carried = 2          # Başlangıçta 2 payload
        self._emergency = False

    # ------------------------------------------------------------------ #
    def initialize(self) -> PayloadOperationResult:
        self._magnets = MagnetState(magnet_1_on=True, magnet_2_on=True)
        self._state = PayloadMechanismState.READY
        self._emergency = False
        self._log.info("[SIM] initialize: Magnet1=ON Magnet2=ON Stepper=SAFE")
        return PayloadOperationResult.ok("Simulation initialized.")

    # ------------------------------------------------------------------ #
    def send_pickup_command(self) -> PayloadOperationResult:
        if self._emergency:
            return PayloadOperationResult.fail(
                OperationResult.EMERGENCY_ACTIVE, "Emergency stop aktif.")
        if self._state not in (PayloadMechanismState.READY,):
            return PayloadOperationResult.fail(
                OperationResult.INVALID_STATE,
                f"send_pickup_command için state uygunsuz: {self._state}")
        # Her iki mıknatıs ON olmalı
        self._magnets = MagnetState(magnet_1_on=True, magnet_2_on=True)
        self._state = PayloadMechanismState.PICKING_UP
        self._log.info("[SIM] send_pickup_command: Magnet1=ON Magnet2=ON, Stepper↓ başlıyor")
        return PayloadOperationResult.ok("Pickup command sent (simulation).")

    def finish_pickup(self) -> PayloadOperationResult:
        if self._state != PayloadMechanismState.PICKING_UP:
            return PayloadOperationResult.fail(
                OperationResult.INVALID_STATE,
                f"finish_pickup beklenen state PICKING_UP, mevcut: {self._state}")
        self._carried += 1
        self._state = PayloadMechanismState.READY
        self._log.info("[SIM] finish_pickup: Stepper↑ tamamlandı, carried=%d", self._carried)
        return PayloadOperationResult.ok("Pickup finished (simulation).")

    # ------------------------------------------------------------------ #
    def release_payload_1(self) -> PayloadOperationResult:
        if self._emergency:
            return PayloadOperationResult.fail(
                OperationResult.EMERGENCY_ACTIVE, "Emergency stop aktif.")
        if self._carried < 1:
            return PayloadOperationResult.fail(
                OperationResult.INVALID_STATE, "Bırakılacak payload yok.")
        self._state = PayloadMechanismState.RELEASING
        self._log.info("[SIM] release_payload_1: Stepper↓ → Magnet1 OFF → Stepper↑")
        # Magnet 2 DOKUNULMAZ
        self._magnets.magnet_1_on = False
        self._carried -= 1
        self._state = PayloadMechanismState.READY
        self._log.info("[SIM] release_payload_1 tamamlandı. Magnet1=OFF Magnet2=%s carried=%d",
                       "ON" if self._magnets.magnet_2_on else "OFF", self._carried)
        return PayloadOperationResult.ok("Payload 1 released (simulation).")

    def release_payload_2(self) -> PayloadOperationResult:
        if self._emergency:
            return PayloadOperationResult.fail(
                OperationResult.EMERGENCY_ACTIVE, "Emergency stop aktif.")
        if self._carried < 1:
            return PayloadOperationResult.fail(
                OperationResult.INVALID_STATE, "Bırakılacak payload yok.")
        self._state = PayloadMechanismState.RELEASING
        self._log.info("[SIM] release_payload_2: Stepper↓ → Magnet2 OFF → Stepper↑")
        self._magnets.magnet_2_on = False
        self._carried -= 1
        self._state = PayloadMechanismState.READY
        self._log.info("[SIM] release_payload_2 tamamlandı. Magnet1=%s Magnet2=OFF carried=%d",
                       "ON" if self._magnets.magnet_1_on else "OFF", self._carried)
        return PayloadOperationResult.ok("Payload 2 released (simulation).")

    # ------------------------------------------------------------------ #
    def set_magnet(self, magnet_id: MagnetId, on: bool) -> PayloadOperationResult:
        if magnet_id == MagnetId.MAGNET_1:
            self._magnets.magnet_1_on = on
        else:
            self._magnets.magnet_2_on = on
        self._log.info("[SIM] set_magnet: %s = %s", magnet_id.value, "ON" if on else "OFF")
        return PayloadOperationResult.ok()

    # ------------------------------------------------------------------ #
    def emergency_stop(self) -> PayloadOperationResult:
        self._emergency = True
        self._state = PayloadMechanismState.EMERGENCY_STOP
        self._log.warning("[SIM] EMERGENCY STOP tetiklendi!")
        return PayloadOperationResult.ok("Emergency stop executed (simulation).")

    def clear_attached(self) -> None:
        if self._state == PayloadMechanismState.PICKING_UP:
            self._state = PayloadMechanismState.READY
        self._log.debug("[SIM] clear_attached")

    # ------------------------------------------------------------------ #
    def status(self) -> PayloadSystemStatus:
        return PayloadSystemStatus(
            mechanism_state=self._state,
            magnets=MagnetState(
                magnet_1_on=self._magnets.magnet_1_on,
                magnet_2_on=self._magnets.magnet_2_on,
            ),
            carried_count=self._carried,
            backend_type="simulation",
        )
