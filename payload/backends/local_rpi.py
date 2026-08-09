"""
LocalRPiBackend — Raspberry Pi 5 üzerinde doğrudan lgpio GPIO kullanımı.

Bu backend gerçek uçuşta kullanılır.
Ağ bağlantısı yoktur — GPIO pinlerine doğrudan erişir.

payload/config.yaml → backend: real  →  bu sınıf yüklenir.

Akış:
  main.py → PayloadController → LocalRPiBackend → lgpio → PCB → Motor + Mıknatıslar
"""
from __future__ import annotations

import logging
import sys
import threading
from pathlib import Path

import yaml

from payload.hardware.magnets import MagnetController
from payload.hardware.stepper import DRV8834
from payload.interface import PayloadBackend
from payload.models import (
    MagnetId,
    MagnetState,
    OperationResult,
    PayloadMechanismState,
    PayloadOperationResult,
    PayloadSystemStatus,
)


def _ok(msg: str = "OK") -> PayloadOperationResult:
    return PayloadOperationResult.ok(msg)


def _fail(reason: OperationResult, msg: str = "") -> PayloadOperationResult:
    return PayloadOperationResult.fail(reason, msg)


class LocalRPiBackend(PayloadBackend):
    """
    Raspberry Pi 5 GPIO donanım backend'i.

    Kullanım (otomatik — main.py tarafından yönetilir):
        backend = LocalRPiBackend.from_config("payload/config.yaml")
        # ardından PayloadController'a inject edilir
    """

    def __init__(
        self,
        stepper: DRV8834,
        magnets: MagnetController,
        steps:   int,
        logger:  logging.Logger | None = None,
    ) -> None:
        self._stepper   = stepper
        self._magnets   = magnets
        self._steps     = steps
        self._log       = logger or logging.getLogger(__name__)
        self._state     = PayloadMechanismState.UNINITIALIZED
        self._emergency = False
        self._carried   = 2
        self._move_thread: threading.Thread | None = None

    # ── Factory ──────────────────────────────────────────────────────────── #
    @classmethod
    def from_config(
        cls,
        config_path: str,
        logger: logging.Logger | None = None,
    ) -> "LocalRPiBackend":
        """
        payload/config.yaml'dan stepper ve magnet parametrelerini oku,
        backend nesnesini oluştur.
        """
        log = logger or logging.getLogger(__name__)
        cfg = yaml.safe_load(Path(config_path).read_text())

        sc   = cfg["stepper"]
        mc   = cfg["magnets"]
        chip = cfg.get("gpio", {}).get("chip", 0)   # /dev/gpiochip0

        stepper = DRV8834(
            step_pin=sc["step_pin"],
            enable_pin=sc["enable_pin"],
            dir_pin=sc["dir_pin"],
            step_delay_s=sc.get("step_delay_s", 0.001),
            dir_setup_delay_s=sc.get("dir_setup_delay_s", 0.005),
            enable_active_low=sc.get("enable_active_low", True),
            chip=chip,
            logger=log,
        )

        hold_high = (mc.get("hold_level", "HIGH") == "HIGH")
        magnets = MagnetController(
            pin_1=mc["magnet_1_pin"],
            pin_2=mc["magnet_2_pin"],
            hold_high=hold_high,
            chip=chip,
            logger=log,
        )

        steps = sc.get("steps_for_travel", 17143)
        return cls(stepper=stepper, magnets=magnets, steps=steps, logger=log)

    # ── PayloadBackend interface ──────────────────────────────────────────── #

    def initialize(self) -> PayloadOperationResult:
        self._log.info("LocalRPiBackend initialize: Magnet1=ON Magnet2=ON Stepper=SAFE")
        self._emergency = False
        self._stepper.reset_emergency()
        self._stepper.setup()
        self._magnets.setup(initial_on=True)
        self._state = PayloadMechanismState.READY
        return _ok("Initialized.")

    def send_pickup_command(self) -> PayloadOperationResult:
        if self._emergency:
            return _fail(OperationResult.EMERGENCY_ACTIVE, "Emergency stop aktif.")
        if self._state != PayloadMechanismState.READY:
            return _fail(OperationResult.INVALID_STATE,
                         f"Beklenen READY, mevcut: {self._state}")
        self._magnets.all_on()
        self._state = PayloadMechanismState.PICKING_UP
        self._log.info("Pickup: Magnet1=ON Magnet2=ON, Stepper↓ başlıyor")
        self._stepper.enable()
        self._move_thread = threading.Thread(
            target=self._stepper.move_down, args=(self._steps,), daemon=True)
        self._move_thread.start()
        return _ok("Pickup command sent.")

    def finish_pickup(self) -> PayloadOperationResult:
        if self._state != PayloadMechanismState.PICKING_UP:
            return _fail(OperationResult.INVALID_STATE,
                         f"Beklenen PICKING_UP, mevcut: {self._state}")
        if self._move_thread and self._move_thread.is_alive():
            self._move_thread.join(timeout=30.0)
        self._log.info("Pickup tamamlandı, Stepper↑")
        self._stepper.move_up(self._steps)
        self._stepper.disable()
        self._carried += 1
        self._state = PayloadMechanismState.READY
        return _ok("Pickup finished.")

    def release_payload_1(self) -> PayloadOperationResult:
        if self._emergency:
            return _fail(OperationResult.EMERGENCY_ACTIVE)
        if self._carried < 1:
            return _fail(OperationResult.INVALID_STATE, "Bırakılacak payload yok.")
        self._log.info("Release 1: Stepper↓ → Magnet1 OFF → Stepper↑")
        self._state = PayloadMechanismState.RELEASING
        self._stepper.enable()
        self._stepper.move_down(self._steps)
        self._magnets.magnet_1_off()          # Magnet 2 DOKUNULMAZ
        self._stepper.move_up(self._steps)
        self._stepper.disable()
        self._carried -= 1
        self._state = PayloadMechanismState.READY
        return _ok("Payload 1 released.")

    def release_payload_2(self) -> PayloadOperationResult:
        if self._emergency:
            return _fail(OperationResult.EMERGENCY_ACTIVE)
        if self._carried < 1:
            return _fail(OperationResult.INVALID_STATE, "Bırakılacak payload yok.")
        self._log.info("Release 2: Stepper↓ → Magnet2 OFF → Stepper↑")
        self._state = PayloadMechanismState.RELEASING
        self._stepper.enable()
        self._stepper.move_down(self._steps)
        self._magnets.magnet_2_off()
        self._stepper.move_up(self._steps)
        self._stepper.disable()
        self._carried -= 1
        self._state = PayloadMechanismState.READY
        return _ok("Payload 2 released.")

    def set_magnet(self, magnet_id: MagnetId, on: bool) -> PayloadOperationResult:
        self._magnets.set_by_id(magnet_id.value, on)
        return _ok()

    def emergency_stop(self) -> PayloadOperationResult:
        self._emergency = True
        self._stepper.emergency_stop()
        self._magnets.safe_hold()
        self._state = PayloadMechanismState.EMERGENCY_STOP
        self._log.warning("EMERGENCY STOP! Mıknatıslar tutma konumunda.")
        return _ok("Emergency stop executed.")

    def clear_attached(self) -> None:
        if self._state == PayloadMechanismState.PICKING_UP:
            self._state = PayloadMechanismState.READY

    def status(self) -> PayloadSystemStatus:
        return PayloadSystemStatus(
            mechanism_state=self._state,
            magnets=MagnetState(
                magnet_1_on=self._magnets.state_1,
                magnet_2_on=self._magnets.state_2,
            ),
            carried_count=self._carried,
            backend_type="local_rpi",
        )

    def cleanup(self) -> None:
        """Güvenli kapanış: mıknatıslar HOLD, handle'ları kapat."""
        self._log.info("LocalRPiBackend cleanup.")
        try:
            self._magnets.safe_hold()
            self._stepper.disable()
        finally:
            self._stepper.close()
            self._magnets.close()
