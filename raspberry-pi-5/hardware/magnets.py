"""
Mıknatıs Kontrolcüsü — Raspberry Pi 5 / lgpio.

İki mıknatıs bağımsız olarak kontrol edilir:
  Magnet 1 → GPIO 5  (Mavi kablo)
  Magnet 2 → GPIO 6  (Kırmızı kablo)

Polarity:
  HIGH (1) = HOLD  (tutma)
  LOW  (0) = RELEASE (bırakma)

lgpio kullanım notu:
  RPi 5'te RPi.GPIO çalışmaz, lgpio gereklidir.
  sudo apt install python3-lgpio  veya  pip install lgpio
"""
from __future__ import annotations

import logging

try:
    import lgpio
    _GPIO_AVAILABLE = True
except ImportError:
    _GPIO_AVAILABLE = False


class MagnetController:
    """
    İki bağımsız mıknatısın lgpio üzerinden GPIO kontrolü.

    Kullanım:
        magnets = MagnetController(pin_1=5, pin_2=6)
        magnets.setup()          # gpiochip aç, pinleri çıkış yap
        magnets.magnet_1_on()
        magnets.magnet_2_off()
        magnets.close()          # gpiochip kapat
    """

    def __init__(
        self,
        pin_1: int = 5,
        pin_2: int = 6,
        hold_high: bool = True,   # True → HIGH = HOLD, LOW = RELEASE
        chip: int = 0,            # /dev/gpiochip0 (RPi 5 varsayılanı)
        logger: logging.Logger | None = None,
    ) -> None:
        self._pin_1     = pin_1
        self._pin_2     = pin_2
        self._hold_high = hold_high
        self._chip      = chip
        self._log       = logger or logging.getLogger(__name__)
        self._state_1   = False
        self._state_2   = False
        self._handle    = None   # lgpio chip handle

        if not _GPIO_AVAILABLE:
            self._log.warning("lgpio bulunamadı — magnets stub modda çalışıyor.")

    # ── GPIO kurulum ─────────────────────────────────────────────────────── #
    def setup(self, initial_on: bool = True) -> None:
        """
        gpiochip aç, pinleri çıkış olarak yapılandır. Güvenli: tekrar çağrılabilir.
        initial_on=True → Her iki mıknatıs başlangıçta ON (tutma).
        """
        initial_level = 1 if (initial_on == self._hold_high) else 0

        if _GPIO_AVAILABLE:
            if self._handle is not None:
                try:
                    lgpio.gpiochip_close(self._handle)
                except Exception:
                    pass
                self._handle = None

            handle = None
            try:
                handle = lgpio.gpiochip_open(self._chip)
                lgpio.gpio_claim_output(handle, self._pin_1, initial_level)
                lgpio.gpio_claim_output(handle, self._pin_2, initial_level)
                self._handle = handle
            except Exception:
                if handle is not None:
                    try:
                        lgpio.gpiochip_close(handle)
                    except Exception:
                        pass
                raise
        else:
            self._log.debug("[STUB] magnets setup: initial=%s",
                            "ON" if initial_on else "OFF")

        self._state_1 = initial_on
        self._state_2 = initial_on
        self._log.info("MagnetController kurulumu: M1=%s M2=%s",
                       "ON" if initial_on else "OFF",
                       "ON" if initial_on else "OFF")

    # ── İç yardımcı ──────────────────────────────────────────────────────── #
    def _write(self, pin: int, on: bool) -> None:
        """on=True → HOLD seviyesi, on=False → RELEASE seviyesi."""
        level = 1 if (on == self._hold_high) else 0
        if _GPIO_AVAILABLE and self._handle is not None:
            lgpio.gpio_write(self._handle, pin, level)
        else:
            self._log.debug("[STUB] lgpio pin %d → %d", pin, level)

    # ── Tekil mıknatıs kontrolü ───────────────────────────────────────────── #
    def magnet_1_on(self) -> None:
        """Magnet 1 tutma (HOLD) konumuna geç."""
        self._write(self._pin_1, True)
        self._state_1 = True
        self._log.info("Magnet 1 → ON (HOLD) [pin=%d]", self._pin_1)

    def magnet_1_off(self) -> None:
        """Magnet 1 bırakma (RELEASE) konumuna geç."""
        self._write(self._pin_1, False)
        self._state_1 = False
        self._log.info("Magnet 1 → OFF (RELEASE) [pin=%d]", self._pin_1)

    def magnet_2_on(self) -> None:
        """Magnet 2 tutma (HOLD) konumuna geç."""
        self._write(self._pin_2, True)
        self._state_2 = True
        self._log.info("Magnet 2 → ON (HOLD) [pin=%d]", self._pin_2)

    def magnet_2_off(self) -> None:
        """Magnet 2 bırakma (RELEASE) konumuna geç."""
        self._write(self._pin_2, False)
        self._state_2 = False
        self._log.info("Magnet 2 → OFF (RELEASE) [pin=%d]", self._pin_2)

    # ── Toplu kontrol ─────────────────────────────────────────────────────── #
    def all_on(self) -> None:
        self.magnet_1_on()
        self.magnet_2_on()

    def all_off(self) -> None:
        self.magnet_1_off()
        self.magnet_2_off()

    def set_by_id(self, magnet_id: str, on: bool) -> None:
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
        """Güvenli kapanış: her iki mıknatısı HOLD'a al."""
        self.all_on()
        self._log.info("MagnetController güvenli tutma: M1=ON M2=ON")

    def close(self) -> None:
        """lgpio chip handle'ı kapat. Uygulama kapanırken çağır."""
        if _GPIO_AVAILABLE and self._handle is not None:
            lgpio.gpiochip_close(self._handle)
            self._handle = None
            self._log.debug("lgpio gpiochip kapatıldı.")
