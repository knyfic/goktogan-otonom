"""
Mıknatıs Kontrolcüsü — Raspberry Pi 5 / BCM GPIO.

İki mıknatıs bağımsız olarak kontrol edilir:
  Magnet 1 → GPIO 5  (Mavi kablo)
  Magnet 2 → GPIO 6  (Kırmızı kablo)

Polarity:
  HIGH = HOLD  (tutma)
  LOW  = RELEASE (bırakma)
"""
from __future__ import annotations

import logging

try:
    import RPi.GPIO as GPIO
    _GPIO_AVAILABLE = True
except ImportError:
    _GPIO_AVAILABLE = False


# GPIO level sabitleri (RPi.GPIO olmadan da kullanılabilir)
_HIGH = 1
_LOW  = 0


class MagnetController:
    """
    İki bağımsız mıknatısın GPIO kontrolü.

    Kullanım:
        magnets = MagnetController(pin_1=5, pin_2=6)
        magnets.setup()
        magnets.magnet_1_on()
        magnets.magnet_2_off()
    """

    def __init__(
        self,
        pin_1: int = 5,
        pin_2: int = 6,
        hold_high: bool = True,   # HIGH = tutma (HOLD)
        logger: logging.Logger | None = None,
    ) -> None:
        self._pin_1    = pin_1
        self._pin_2    = pin_2
        self._hold_high = hold_high
        self._log      = logger or logging.getLogger(__name__)
        self._state_1  = False   # False = OFF (bırakma)
        self._state_2  = False

        if not _GPIO_AVAILABLE:
            self._log.warning("RPi.GPIO bulunamadı — magnets stub modda çalışıyor.")

    # ── GPIO kurulum ─────────────────────────────────────────────────────── #
    def setup(self, initial_on: bool = True) -> None:
        """
        GPIO pinlerini çıkış olarak yapılandır.
        initial_on=True → Her iki mıknatıs başlangıçta ON (tutma).
        """
        hold = GPIO.HIGH if self._hold_high else GPIO.LOW
        release = GPIO.LOW if self._hold_high else GPIO.HIGH
        initial = hold if initial_on else release

        if _GPIO_AVAILABLE:
            GPIO.setup(self._pin_1, GPIO.OUT, initial=initial)
            GPIO.setup(self._pin_2, GPIO.OUT, initial=initial)
        else:
            self._log.debug("[STUB] magnets GPIO kurulumu: initial=%s",
                            "ON" if initial_on else "OFF")

        self._state_1 = initial_on
        self._state_2 = initial_on
        self._log.info("MagnetController kurulumu: M1=%s M2=%s",
                       "ON" if initial_on else "OFF",
                       "ON" if initial_on else "OFF")

    # ── Tekil mıknatıs kontrolü ───────────────────────────────────────────── #
    def _set(self, pin: int, on: bool) -> None:
        level = (GPIO.HIGH if self._hold_high else GPIO.LOW) if on else \
                (GPIO.LOW  if self._hold_high else GPIO.HIGH)
        if _GPIO_AVAILABLE:
            GPIO.output(pin, level)
        else:
            self._log.debug("[STUB] GPIO pin %d → %s", pin, "HIGH" if level else "LOW")

    def magnet_1_on(self) -> None:
        """Magnet 1 tutma (HOLD) konumuna geç."""
        self._set(self._pin_1, True)
        self._state_1 = True
        self._log.info("Magnet 1 → ON (HOLD) [pin=%d]", self._pin_1)

    def magnet_1_off(self) -> None:
        """Magnet 1 bırakma (RELEASE) konumuna geç."""
        self._set(self._pin_1, False)
        self._state_1 = False
        self._log.info("Magnet 1 → OFF (RELEASE) [pin=%d]", self._pin_1)

    def magnet_2_on(self) -> None:
        """Magnet 2 tutma (HOLD) konumuna geç."""
        self._set(self._pin_2, True)
        self._state_2 = True
        self._log.info("Magnet 2 → ON (HOLD) [pin=%d]", self._pin_2)

    def magnet_2_off(self) -> None:
        """Magnet 2 bırakma (RELEASE) konumuna geç."""
        self._set(self._pin_2, False)
        self._state_2 = False
        self._log.info("Magnet 2 → OFF (RELEASE) [pin=%d]", self._pin_2)

    # ── Toplu kontrol ─────────────────────────────────────────────────────── #
    def all_on(self) -> None:
        """Her iki mıknatısı da tutma konumuna getir."""
        self.magnet_1_on()
        self.magnet_2_on()

    def all_off(self) -> None:
        """Her iki mıknatısı da bırakma konumuna getir (pickup sonrası release)."""
        self.magnet_1_off()
        self.magnet_2_off()

    def set_by_id(self, magnet_id: str, on: bool) -> None:
        """
        'MAGNET_1' veya 'MAGNET_2' string ile kontrol et.
        dispatcher.py'dan gelecek komutlar için.
        """
        if magnet_id == "MAGNET_1":
            self.magnet_1_on() if on else self.magnet_1_off()
        elif magnet_id == "MAGNET_2":
            self.magnet_2_on() if on else self.magnet_2_off()
        else:
            self._log.error("Bilinmeyen magnet_id: %s", magnet_id)

    # ── Durum ─────────────────────────────────────────────────────────────── #
    @property
    def state_1(self) -> bool:
        return self._state_1

    @property
    def state_2(self) -> bool:
        return self._state_2

    def safe_hold(self) -> None:
        """Güvenli kapanışta her iki mıknatısı HOLD'a al (yük kaybetme)."""
        self.all_on()
        self._log.info("MagnetController güvenli tutma konumu: M1=ON M2=ON")
