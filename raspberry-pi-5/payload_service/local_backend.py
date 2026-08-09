"""
LocalRPiBackend — Raspberry Pi 5 üzerinde doğrudan GPIO kullanımı.

Gerçek uçuşta bu backend kullanılır.
Network yoktur, doğrudan stepper + magnet sürücüleri çalışır.

Kullanım (raspberry-pi-5/main.py veya payload_service/server.py içinden):
    backend = LocalRPiBackend.from_config("config/payload.yaml")
    backend.initialize()
    backend.send_pickup_command()
"""
from __future__ import annotations

import logging
import threading
import time
from pathlib import Path

import yaml

try:
    import RPi.GPIO as GPIO
    _GPIO_AVAILABLE = True
except ImportError:
    _GPIO_AVAILABLE = False

from hardware.magnets import MagnetController
from hardware.stepper import DRV8834

# Ubuntu'daki interface.py / models.py ile aynı sözleşme —
# ancak RPi tarafında bağımsız kopyalar kullanılır.
# (Import yolu: raspberry-pi-5/ projenin root'u)

_RESULT_OK   = {"success": True,  "message": "OK"}
_RESULT_FAIL = {"success": False, "message": "FAILED"}


def _ok(msg: str = "OK") -> dict:
    return {"success": True, "message": msg}


def _fail(reason: str, msg: str = "") -> dict:
    return {"success": False, "reason": reason, "message": msg}


class LocalRPiBackend:
    """
    Raspberry Pi 5 GPIO donanım backend'i.

    Akış:
      initialize() → mıknatıslar ON, stepper kurulumu
      send_pickup_command() → stepper aşağı (thread'de, non-blocking)
      finish_pickup() → stepper yukarı
      release_payload_1() → stepper ↓ → Magnet 1 OFF → stepper ↑
      release_payload_2() → stepper ↓ → Magnet 2 OFF → stepper ↑
    """

    def __init__(
        self,
        stepper:  DRV8834,
        magnets:  MagnetController,
        steps:    int,
        logger:   logging.Logger | None = None,
    ) -> None:
        self._stepper  = stepper
        self._magnets  = magnets
        self._steps    = steps       # Tam hareket adım sayısı (60 mm ≈ 17143)
        self._log      = logger or logging.getLogger(__name__)
        self._state    = "UNINITIALIZED"
        self._emergency = False
        self._carried  = 2
        self._move_thread: threading.Thread | None = None

    # ── Factory ──────────────────────────────────────────────────────────── #
    @classmethod
    def from_config(
        cls,
        config_path: str = "config/payload.yaml",
        logger: logging.Logger | None = None,
    ) -> "LocalRPiBackend":
        log = logger or logging.getLogger(__name__)
        cfg = yaml.safe_load(Path(config_path).read_text())

        sc = cfg["stepper"]
        mc = cfg["magnets"]

        if _GPIO_AVAILABLE:
            mode = GPIO.BCM if cfg.get("gpio", {}).get("mode", "BCM") == "BCM" else GPIO.BOARD
            GPIO.setmode(mode)
            GPIO.setwarnings(False)

        stepper = DRV8834(
            step_pin=sc["step_pin"],
            enable_pin=sc["enable_pin"],
            dir_pin=sc["dir_pin"],
            step_delay_s=sc.get("step_delay_s", 0.001),
            dir_setup_delay_s=sc.get("dir_setup_delay_s", 0.005),
            enable_active_low=sc.get("enable_active_low", True),
            logger=log,
        )

        hold_high = (mc.get("hold_level", "HIGH") == "HIGH")
        magnets = MagnetController(
            pin_1=mc["magnet_1_pin"],
            pin_2=mc["magnet_2_pin"],
            hold_high=hold_high,
            logger=log,
        )

        steps = sc.get("steps_for_travel", 17143)
        return cls(stepper=stepper, magnets=magnets, steps=steps, logger=log)

    # ── Lifecycle ─────────────────────────────────────────────────────────── #
    def initialize(self) -> dict:
        """GPIO kurulum + başlangıç durumu: Mıknatıslar ON, stepper SAFE."""
        self._log.info("LocalRPiBackend initialize başlıyor...")
        self._emergency = False
        self._stepper.reset_emergency()
        self._stepper.setup()
        self._magnets.setup(initial_on=True)   # Her iki mıknatıs ON — kritik!
        self._state = "READY"
        self._log.info("LocalRPiBackend hazır. Magnet1=ON Magnet2=ON Stepper=SAFE")
        return _ok("Initialized.")

    # ── Pickup ────────────────────────────────────────────────────────────── #
    def send_pickup_command(self) -> dict:
        """
        Non-blocking pickup başlatma.
        Stepper aşağı hareketi ayrı thread'de çalışır.
        Ubuntu'daki state machine PAYLOAD_SERVO_WAIT_S bekleyip finish_pickup çağırır.
        """
        if self._emergency:
            return _fail("EMERGENCY_ACTIVE", "Emergency stop aktif.")
        if self._state != "READY":
            return _fail("INVALID_STATE", f"Beklenen READY, mevcut: {self._state}")

        # Her iki mıknatıs ON olduğundan emin ol
        self._magnets.all_on()
        self._state = "PICKING_UP"
        self._log.info("Pickup başlıyor: Magnet1=ON Magnet2=ON, Stepper↓")

        self._stepper.enable()
        self._move_thread = threading.Thread(
            target=self._stepper.move_down,
            args=(self._steps,),
            daemon=True,
        )
        self._move_thread.start()
        return _ok("Pickup command sent.")

    def finish_pickup(self) -> dict:
        """Stepper geri yukarı çıkar (PAYLOAD_SERVO_WAIT_S sonrası)."""
        if self._state != "PICKING_UP":
            return _fail("INVALID_STATE", f"Beklenen PICKING_UP, mevcut: {self._state}")

        # Önceki aşağı hareket bitene kadar bekle
        if self._move_thread and self._move_thread.is_alive():
            self._move_thread.join(timeout=30.0)

        self._log.info("Pickup tamamlandı, Stepper↑")
        self._stepper.move_up(self._steps)
        self._stepper.disable()
        self._carried += 1
        self._state = "READY"
        self._log.info("finish_pickup tamamlandı. carried=%d", self._carried)
        return _ok("Pickup finished.")

    # ── Release ───────────────────────────────────────────────────────────── #
    def release_payload_1(self) -> dict:
        """
        1. Payload bırakma:
          ENABLE → Stepper ↓ → Magnet 1 OFF → Stepper ↑ → DISABLE
          Magnet 2 DOKUNULMAZ.
        """
        if self._emergency:
            return _fail("EMERGENCY_ACTIVE")
        if self._carried < 1:
            return _fail("INVALID_STATE", "Bırakılacak payload yok.")

        self._log.info("Release Payload 1: Stepper↓ → Magnet1 OFF → Stepper↑")
        self._state = "RELEASING"
        self._stepper.enable()
        self._stepper.move_down(self._steps)
        self._magnets.magnet_1_off()   # Magnet 2 DOKUNULMAZ
        self._log.info("Magnet 1 → OFF. Magnet 2 durumu: %s",
                       "ON" if self._magnets.state_2 else "OFF")
        self._stepper.move_up(self._steps)
        self._stepper.disable()
        self._carried -= 1
        self._state = "READY"
        self._log.info("release_payload_1 tamamlandı. carried=%d", self._carried)
        return _ok("Payload 1 released.")

    def release_payload_2(self) -> dict:
        """
        2. Payload bırakma:
          ENABLE → Stepper ↓ → Magnet 2 OFF → Stepper ↑ → DISABLE
        """
        if self._emergency:
            return _fail("EMERGENCY_ACTIVE")
        if self._carried < 1:
            return _fail("INVALID_STATE", "Bırakılacak payload yok.")

        self._log.info("Release Payload 2: Stepper↓ → Magnet2 OFF → Stepper↑")
        self._state = "RELEASING"
        self._stepper.enable()
        self._stepper.move_down(self._steps)
        self._magnets.magnet_2_off()
        self._log.info("Magnet 2 → OFF. Magnet 1 durumu: %s",
                       "ON" if self._magnets.state_1 else "OFF")
        self._stepper.move_up(self._steps)
        self._stepper.disable()
        self._carried -= 1
        self._state = "READY"
        self._log.info("release_payload_2 tamamlandı. carried=%d", self._carried)
        return _ok("Payload 2 released.")

    # ── Tekil mıknatıs ────────────────────────────────────────────────────── #
    def set_magnet(self, magnet_id: str, on: bool) -> dict:
        self._magnets.set_by_id(magnet_id, on)
        return _ok(f"{magnet_id} = {'ON' if on else 'OFF'}")

    # ── Emergency ─────────────────────────────────────────────────────────── #
    def emergency_stop(self) -> dict:
        self._emergency = True
        self._stepper.emergency_stop()
        # Mıknatısları HOLD konumunda bırak — yük düşmesin
        self._magnets.safe_hold()
        self._state = "EMERGENCY_STOP"
        self._log.warning("EMERGENCY STOP! Mıknatıslar tutma konumunda.")
        return _ok("Emergency stop executed.")

    # ── Yardımcılar ───────────────────────────────────────────────────────── #
    def clear_attached(self) -> None:
        if self._state == "PICKING_UP":
            self._state = "READY"

    def status(self) -> dict:
        return {
            "mechanism_state": self._state,
            "magnets": {
                "magnet_1_on": self._magnets.state_1,
                "magnet_2_on": self._magnets.state_2,
            },
            "carried_count": self._carried,
            "backend_type":  "local_rpi",
        }

    def cleanup(self) -> None:
        """Güvenli kapanış: mıknatıslar HOLD, GPIO temizle."""
        self._log.info("LocalRPiBackend cleanup: Mıknatıslar HOLD, stepper DISABLE.")
        try:
            self._magnets.safe_hold()
            self._stepper.disable()
        finally:
            if _GPIO_AVAILABLE:
                GPIO.cleanup()
