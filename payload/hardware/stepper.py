"""
DRV8834 Stepper Motor Sürücüsü — Raspberry Pi 5 / lgpio.

Indexer modda Full-step kullanımı:
  ENABLE LOW  (0) → driver aktif
  ENABLE HIGH (1) → driver pasif

Güvenlik kuralı:
  enable() → move_steps() → disable()
  Yük altında bekleme süresince ENABLE LOW kalmalı.

lgpio kullanım notu:
  RPi 5'te RPi.GPIO çalışmaz, lgpio gereklidir.
  sudo apt install python3-lgpio  veya  pip install lgpio
"""
from __future__ import annotations

import logging
import time

try:
    import lgpio
    _GPIO_AVAILABLE = True
except ImportError:
    _GPIO_AVAILABLE = False


class DRV8834:
    """
    DRV8834 stepper motor sürücüsü — lgpio tabanlı.

    Kullanım:
        stepper = DRV8834(step_pin=13, enable_pin=19, dir_pin=26)
        stepper.setup()          # gpiochip aç, pinleri tanımla
        stepper.enable()
        stepper.move_down(17143)
        stepper.move_up(17143)
        stepper.disable()
        stepper.close()          # gpiochip kapat
    """

    DIR_DOWN = 1   # DIR HIGH → aşağı
    DIR_UP   = 0   # DIR LOW  → yukarı

    def __init__(
        self,
        step_pin:          int,
        enable_pin:        int,
        dir_pin:           int,
        step_delay_s:      float = 0.001,
        dir_setup_delay_s: float = 0.005,
        enable_active_low: bool  = True,   # True: LOW=aktif (DRV8834 varsayılanı)
        chip:              int   = 0,       # /dev/gpiochip0
        logger: logging.Logger | None = None,
    ) -> None:
        self._step_pin   = step_pin
        self._enable_pin = enable_pin
        self._dir_pin    = dir_pin
        self._step_delay = step_delay_s
        self._dir_delay  = dir_setup_delay_s
        self._active_low = enable_active_low
        self._chip       = chip
        self._log        = logger or logging.getLogger(__name__)
        self._enabled    = False
        self._emergency  = False
        self._handle     = None   # lgpio chip handle

        if not _GPIO_AVAILABLE:
            self._log.warning("lgpio bulunamadı — stepper stub modda çalışıyor.")

    # ── GPIO kurulum ─────────────────────────────────────────────────────── #
    def setup(self) -> None:
        """gpiochip aç, pinleri çıkış olarak yapılandır. Güvenli: tekrar çağrılabilir."""
        if not _GPIO_AVAILABLE:
            return

        if self._handle is not None:
            try:
                lgpio.gpiochip_close(self._handle)
            except Exception:
                pass
            self._handle = None

        handle = None
        try:
            handle = lgpio.gpiochip_open(self._chip)
            lgpio.gpio_claim_output(handle, self._step_pin,   0)
            lgpio.gpio_claim_output(handle, self._dir_pin,    0)
            enable_initial = 1 if self._active_low else 0
            lgpio.gpio_claim_output(handle, self._enable_pin, enable_initial)
            self._handle = handle
        except Exception:
            if handle is not None:
                try:
                    lgpio.gpiochip_close(handle)
                except Exception:
                    pass
            raise

        self._log.debug("DRV8834 lgpio kurulumu tamamlandı.")

    # ── Driver aktif/pasif ────────────────────────────────────────────────── #
    def enable(self) -> None:
        """Stepper sürücüyü aktif et."""
        level = 0 if self._active_low else 1   # active_low: LOW=aktif
        if _GPIO_AVAILABLE and self._handle is not None:
            lgpio.gpio_write(self._handle, self._enable_pin, level)
        else:
            self._log.debug("[STUB] stepper enable")
        self._enabled = True
        self._log.debug("DRV8834 ENABLE → aktif (level=%d)", level)

    def disable(self) -> None:
        """Stepper sürücüyü pasif et. Yüksüz konumda çağır."""
        level = 1 if self._active_low else 0   # active_low: HIGH=pasif
        if _GPIO_AVAILABLE and self._handle is not None:
            lgpio.gpio_write(self._handle, self._enable_pin, level)
        else:
            self._log.debug("[STUB] stepper disable")
        self._enabled = False
        self._log.debug("DRV8834 ENABLE → pasif (level=%d)", level)

    # ── Hareket ──────────────────────────────────────────────────────────── #
    def move_steps(self, steps: int, direction: int) -> None:
        """
        Belirtilen adım sayısı kadar hareket et.

        Args:
            steps:     Adım sayısı (pozitif tam sayı).
            direction: DIR_DOWN (1) veya DIR_UP (0).
        """
        if self._emergency:
            self._log.error("move_steps: emergency stop aktif.")
            return
        if steps <= 0:
            return

        if _GPIO_AVAILABLE and self._handle is not None:
            lgpio.gpio_write(self._handle, self._dir_pin, direction)
            time.sleep(self._dir_delay)

            for _ in range(steps):
                if self._emergency:
                    break
                lgpio.gpio_write(self._handle, self._step_pin, 1)
                time.sleep(self._step_delay)
                lgpio.gpio_write(self._handle, self._step_pin, 0)
                time.sleep(self._step_delay)
        else:
            name = "DOWN" if direction == self.DIR_DOWN else "UP"
            self._log.debug("[STUB] move_steps: %d adım %s", steps, name)
            time.sleep(steps * self._step_delay * 2)

        self._log.info("move_steps tamamlandı: %d adım, yön=%s",
                       steps, "DOWN" if direction == self.DIR_DOWN else "UP")

    def move_down(self, steps: int) -> None:
        """Payload aşağıya indir."""
        self._log.info("Stepper ↓ aşağı: %d adım", steps)
        self.move_steps(steps, self.DIR_DOWN)

    def move_up(self, steps: int) -> None:
        """Payload yukarı çıkar."""
        self._log.info("Stepper ↑ yukarı: %d adım", steps)
        self.move_steps(steps, self.DIR_UP)

    # ── Acil durdurma ─────────────────────────────────────────────────────── #
    def emergency_stop(self) -> None:
        """Tüm hareketleri durdur. ENABLE kapatılmaz — pozisyonu koru."""
        self._emergency = True
        self._log.warning("DRV8834 EMERGENCY STOP!")

    def reset_emergency(self) -> None:
        self._emergency = False

    # ── Kapatma ───────────────────────────────────────────────────────────── #
    def close(self) -> None:
        """lgpio chip handle'ı kapat."""
        if _GPIO_AVAILABLE and self._handle is not None:
            lgpio.gpiochip_close(self._handle)
            self._handle = None
            self._log.debug("lgpio gpiochip kapatıldı.")
