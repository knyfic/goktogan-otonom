"""
DRV8834 Stepper Motor Sürücüsü — Raspberry Pi 5 / BCM GPIO.

Indexer modda Full-step kullanımı:
  ENABLE LOW  → driver aktif
  ENABLE HIGH → driver pasif

Güvenlik kuralı:
  enable() → move_steps() → disable()
  Yük altında bekleme süresince ENABLE LOW kalmalı.
"""
from __future__ import annotations

import logging
import time

try:
    import RPi.GPIO as GPIO
    _GPIO_AVAILABLE = True
except ImportError:
    _GPIO_AVAILABLE = False


class DRV8834:
    """
    DRV8834 stepper motor sürücüsü için düşük seviye sürücü sınıfı.

    Kullanım:
        stepper = DRV8834(step_pin=13, enable_pin=19, dir_pin=26)
        stepper.enable()
        stepper.move_steps(100, direction=True)   # True = ileri/aşağı
        stepper.move_steps(100, direction=False)  # False = geri/yukarı
        stepper.disable()
    """

    # Yön sabitleri — donanım bağlantısına göre ayarla
    DIR_DOWN = True    # payload aşağıya iner
    DIR_UP   = False   # payload yukarı çıkar

    def __init__(
        self,
        step_pin:   int,
        enable_pin: int,
        dir_pin:    int,
        step_delay_s:      float = 0.001,
        dir_setup_delay_s: float = 0.005,
        enable_active_low: bool  = True,
        logger: logging.Logger | None = None,
    ) -> None:
        self._step_pin   = step_pin
        self._enable_pin = enable_pin
        self._dir_pin    = dir_pin
        self._step_delay = step_delay_s
        self._dir_delay  = dir_setup_delay_s
        self._active_low = enable_active_low
        self._log        = logger or logging.getLogger(__name__)
        self._enabled    = False
        self._emergency  = False

        if not _GPIO_AVAILABLE:
            self._log.warning("RPi.GPIO bulunamadı — stepper stub modda çalışıyor.")

    # ── GPIO kurulum ─────────────────────────────────────────────────────── #
    def setup(self) -> None:
        """GPIO pinlerini çıkış olarak yapılandır. Başlatmada bir kez çağrılır."""
        if not _GPIO_AVAILABLE:
            return
        GPIO.setup(self._step_pin,   GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self._dir_pin,    GPIO.OUT, initial=GPIO.LOW)
        # ENABLE başlangıçta pasif (yükü kaybetme)
        initial_enable = GPIO.LOW if self._active_low else GPIO.HIGH
        GPIO.setup(self._enable_pin, GPIO.OUT, initial=initial_enable)
        self._log.debug("DRV8834 GPIO kurulumu tamamlandı.")

    # ── Driver aktif/pasif ────────────────────────────────────────────────── #
    def enable(self) -> None:
        """Stepper sürücüyü aktif et (ENABLE LOW)."""
        if not _GPIO_AVAILABLE:
            self._log.debug("[STUB] stepper enable")
            self._enabled = True
            return
        level = GPIO.LOW if self._active_low else GPIO.HIGH
        GPIO.output(self._enable_pin, level)
        self._enabled = True
        self._log.debug("DRV8834 ENABLE → aktif")

    def disable(self) -> None:
        """Stepper sürücüyü pasif et (ENABLE HIGH). Yüksüz konumda çağır."""
        if not _GPIO_AVAILABLE:
            self._log.debug("[STUB] stepper disable")
            self._enabled = False
            return
        level = GPIO.HIGH if self._active_low else GPIO.LOW
        GPIO.output(self._enable_pin, level)
        self._enabled = False
        self._log.debug("DRV8834 ENABLE → pasif")

    # ── Hareket ──────────────────────────────────────────────────────────── #
    def move_steps(self, steps: int, direction: bool) -> None:
        """
        Belirtilen adım sayısı kadar hareket et.

        Args:
            steps:     Adım sayısı (pozitif tam sayı).
            direction: True = DIR_DOWN (aşağı), False = DIR_UP (yukarı).

        Dikkat: Bu metod blocking'dir. State machine PAYLOAD_SERVO_WAIT_S
                ile beklemeyi yönetir; bu fonksiyon stepper hareketi için
                ayrı bir thread'de çalıştırılır (LocalRPiBackend yönetir).
        """
        if self._emergency:
            self._log.error("move_steps: emergency stop aktif, hareket engellendi.")
            return
        if steps <= 0:
            return

        dir_level = GPIO.HIGH if direction else GPIO.LOW

        if _GPIO_AVAILABLE:
            GPIO.output(self._dir_pin, dir_level)
            time.sleep(self._dir_delay)

            for _ in range(steps):
                if self._emergency:
                    break
                GPIO.output(self._step_pin, GPIO.HIGH)
                time.sleep(self._step_delay)
                GPIO.output(self._step_pin, GPIO.LOW)
                time.sleep(self._step_delay)
        else:
            dir_name = "DOWN" if direction else "UP"
            self._log.debug("[STUB] move_steps: %d adım, yön=%s", steps, dir_name)
            # Simüle gecikme
            time.sleep(steps * self._step_delay * 2)

        self._log.info("move_steps tamamlandı: %d adım, yön=%s",
                       steps, "DOWN" if direction else "UP")

    def move_down(self, steps: int) -> None:
        """Payload aşağıya indir (release/pickup konumuna)."""
        self._log.info("Stepper ↓ aşağı: %d adım", steps)
        self.move_steps(steps, direction=self.DIR_DOWN)

    def move_up(self, steps: int) -> None:
        """Payload yukarı çıkar (transport konumuna)."""
        self._log.info("Stepper ↑ yukarı: %d adım", steps)
        self.move_steps(steps, direction=self.DIR_UP)

    # ── Acil durdurma ─────────────────────────────────────────────────────── #
    def emergency_stop(self) -> None:
        """Tüm hareketleri acilen durdur."""
        self._emergency = True
        self._log.warning("DRV8834 EMERGENCY STOP!")
        # Sürücüyü pasife alma: yük altında mı olduğunu bilmiyoruz,
        # bu nedenle ENABLE'ı KAPATMIYORUZ — pozisyonu koru.
        # Üst katman (LocalRPiBackend) güvenli kapanışı yönetir.

    def reset_emergency(self) -> None:
        """Acil durdurma bayrağını temizle (initialize sonrası)."""
        self._emergency = False
