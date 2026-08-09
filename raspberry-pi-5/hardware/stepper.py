"""
DRV8834 Stepper Motor Sürücüsü — Raspberry Pi 5 / lgpio.

Indexer modda Full-step kullanımı:
  ENABLE LOW  (0) → driver aktif
  ENABLE HIGH (1) → driver pasif

Step pulse yöntemi (Arduino referans kodundan):
  10µs HIGH → kalan bekleme LOW
  → Sürücünün STEP pinine sadece kısa bir tetik verilir,
    uzun bekleme LOW'da yapılarak gereksiz ısınma önlenir.

Rampa profili (Arduino referans kodundan):
  başlangıç_hız = 1500µs  → yavaş başla
  hedef_hız     = 1200µs  → hedefe ivmelen
  rampa_adımı   = 250     → kaç adımda hıza ulaş
  Hem başta hem sonda rampa uygulanır.

Mekanik (mil / leadscrew):
  mm_per_revolution   = 0.7    → Mil 1 tam devir = 0.7 mm doğrusal hareket
  steps_per_revolution = 200   → NEMA 17 full-step (1.8°/adım)
  mm_per_step         = 0.0035 → 0.7 / 200 = 0.0035 mm / adım
  50 mm seyahat       = 14286 adım (tam sayı)
"""
from __future__ import annotations

import logging
import time

try:
    import lgpio
    _GPIO_AVAILABLE = True
except ImportError:
    _GPIO_AVAILABLE = False

# Arduino referans kodundan alınan sabitler — config'e taşınmış,
# burada varsayılan olarak da bulunur.
_DEFAULT_START_DELAY_US  = 1500   # µs — başlangıç pulse aralığı (yavaş)
_DEFAULT_TARGET_DELAY_US = 1200   # µs — hedef pulse aralığı (hızlı)
_DEFAULT_RAMP_STEPS      = 250     # adım sayısı — rampa uzunluğu
_STEP_HIGH_US            = 10     # µs — STEP pin'in HIGH kalma süresi

# Mekanik sabitler (mil / leadscrew) — config'ten de ayarlanabilir
_MM_PER_REVOLUTION_DEFAULT    = 0.7    # 1 tam devir = 0.7 mm doğrusal
_STEPS_PER_REVOLUTION_DEFAULT = 200    # Full-step, NEMA 17


class DRV8834:
    """
    DRV8834 stepper motor sürücüsü — rampalı lgpio tabanlı.

    Kullanım:
        stepper = DRV8834(step_pin=13, enable_pin=19, dir_pin=26)
        stepper.setup()
        stepper.enable()
        stepper.move_down(10000)
        stepper.move_up(10000)
        stepper.disable()
        stepper.close()
    """

    DIR_DOWN = 0   # DIR LOW  → aşağı (fiziksel bağlantıya göre)
    DIR_UP   = 1   # DIR HIGH → yukarı

    def __init__(
        self,
        step_pin:           int,
        enable_pin:         int,
        dir_pin:            int,
        dir_setup_delay_s:  float = 0.005,
        enable_active_low:  bool  = True,
        chip:               int   = 0,
        # Rampa parametreleri (µs cinsinden — Arduino ile aynı birim)
        start_delay_us:     int   = _DEFAULT_START_DELAY_US,
        target_delay_us:    int   = _DEFAULT_TARGET_DELAY_US,
        ramp_steps:         int   = _DEFAULT_RAMP_STEPS,
        # Mekanik parametreler (mil / leadscrew) — mm ↔ step çevirisi için
        mm_per_revolution:  float = _MM_PER_REVOLUTION_DEFAULT,
        steps_per_revolution: int = _STEPS_PER_REVOLUTION_DEFAULT,
        logger: logging.Logger | None = None,
    ) -> None:
        self._step_pin    = step_pin
        self._enable_pin  = enable_pin
        self._dir_pin     = dir_pin
        self._dir_delay   = dir_setup_delay_s
        self._active_low  = enable_active_low
        self._chip        = chip
        self._log         = logger or logging.getLogger(__name__)
        self._enabled     = False
        self._emergency   = False
        self._handle      = None

        # Rampa profili
        self._start_us  = start_delay_us
        self._target_us = target_delay_us
        self._ramp      = ramp_steps

        # Mekanik (mil) — mm ↔ adım çevirisi
        self._mm_per_rev    = float(mm_per_revolution)
        self._steps_per_rev = int(steps_per_revolution)
        self._mm_per_step   = self._mm_per_rev / max(self._steps_per_rev, 1)

        if not _GPIO_AVAILABLE:
            self._log.warning("lgpio bulunamadı — stepper stub modda çalışıyor.")
        else:
            self._log.info(
                "Stepper mekanik: %.4f mm/adım (%d adım/devir × %.2f mm/devir)",
                self._mm_per_step, self._steps_per_rev, self._mm_per_rev,
            )

    # ── Yardımcı: mm ↔ adım çevirisi ──────────────────────────────────────── #
    @property
    def mm_per_step(self) -> float:
        """Bir adımın doğrusal karşılığı (mm)."""
        return self._mm_per_step

    def mm_to_steps(self, mm: float) -> int:
        """Verilen doğrusal mesafeyi (mm) tam adım sayısına çevir."""
        if abs(mm) < 1e-9:
            return 0
        steps = int(round(mm / self._mm_per_step))
        return max(0, steps)

    def steps_to_mm(self, steps: int) -> float:
        """Verilen adım sayısını doğrusal mesafeye (mm) çevir."""
        return steps * self._mm_per_step

    # ── GPIO kurulum ─────────────────────────────────────────────────────── #
    def setup(self) -> None:
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
        level = 0 if self._active_low else 1
        if _GPIO_AVAILABLE and self._handle is not None:
            lgpio.gpio_write(self._handle, self._enable_pin, level)
        else:
            self._log.debug("[STUB] stepper enable")
        self._enabled = True

    def disable(self) -> None:
        level = 1 if self._active_low else 0
        if _GPIO_AVAILABLE and self._handle is not None:
            lgpio.gpio_write(self._handle, self._enable_pin, level)
        else:
            self._log.debug("[STUB] stepper disable")
        self._enabled = False

    # ── Hareket (rampalı) ─────────────────────────────────────────────────── #
    def move_steps(self, steps: int, direction: int) -> None:
        """
        Rampalı step hareketi — Arduino referans kodundan adapte.

        Rampa profili:
          - İlk ramp_steps adımda: start_delay_us → target_delay_us (ivmelenme)
          - Orta kısım: sabit target_delay_us (tam hız)
          - Son ramp_steps adımda: target_delay_us → start_delay_us (yavaşlama)

        Step pulse: 10µs HIGH + delay µs LOW
          Arduino'daki gibi sadece kısa tetik, bekleme LOW'da.
        """
        if self._emergency:
            self._log.error("move_steps: emergency stop aktif.")
            return
        if steps <= 0:
            return

        if _GPIO_AVAILABLE and self._handle is not None:
            lgpio.gpio_write(self._handle, self._dir_pin, direction)
            time.sleep(self._dir_delay)

            ivme = (self._start_us - self._target_us) / max(self._ramp, 1)
            anlik_us = float(self._start_us)

            for i in range(steps):
                if self._emergency:
                    break

                # 10µs HIGH tetik (Arduino: delayMicroseconds(10))
                lgpio.gpio_write(self._handle, self._step_pin, 1)
                time.sleep(_STEP_HIGH_US * 1e-6)
                lgpio.gpio_write(self._handle, self._step_pin, 0)

                # Kalan bekleme LOW'da (Arduino: delayMicroseconds(anlikHiz))
                time.sleep(anlik_us * 1e-6)

                # Rampa güncelle
                if i < self._ramp:
                    anlik_us -= ivme          # ivmelen
                elif i >= steps - self._ramp:
                    anlik_us += ivme          # yavaşla
                # Sınırla — aşırı ivme nedeniyle negatife düşmesin
                anlik_us = max(self._target_us, min(self._start_us, anlik_us))

        else:
            # Stub: gerçek süreyi simüle et
            name = "DOWN" if direction == self.DIR_DOWN else "UP"
            avg_us = (self._start_us + self._target_us) / 2
            sim_duration = steps * (avg_us + _STEP_HIGH_US) * 1e-6
            self._log.debug("[STUB] move_steps: %d adım %s (~%.1fs)",
                            steps, name, sim_duration)
            time.sleep(sim_duration)

        self._log.info("move_steps tamamlandı: %d adım, yön=%s",
                       steps, "DOWN" if direction == self.DIR_DOWN else "UP")

    def move_down(self, steps: int) -> None:
        self._log.info("Stepper ↓ aşağı: %d adım (%s mm)",
                       steps, f"{self.steps_to_mm(steps):.2f}")
        self.move_steps(steps, self.DIR_DOWN)

    def move_up(self, steps: int) -> None:
        self._log.info("Stepper ↑ yukarı: %d adım (%s mm)",
                       steps, f"{self.steps_to_mm(steps):.2f}")
        self.move_steps(steps, self.DIR_UP)

    # ── MM bazlı hareket (ergonomik) ─────────────────────────────────────── #
    def move_down_mm(self, mm: float) -> None:
        """Verilen doğrusal mesafe (mm) kadar aşağı iner."""
        steps = self.mm_to_steps(mm)
        self.move_down(steps)

    def move_up_mm(self, mm: float) -> None:
        """Verilen doğrusal mesafe (mm) kadar yukarı çıkar."""
        steps = self.mm_to_steps(mm)
        self.move_up(steps)

    # ── Hareket süresi tahmini ────────────────────────────────────────────── #
    def estimate_duration_s(self, steps: int) -> float:
        """
        Belirtilen adım sayısı için tahmini hareket süresi (saniye).
        local_backend.py tarafından stepper_duration hesabında kullanılır.
        """
        avg_us = (self._start_us + self._target_us) / 2.0
        return steps * (avg_us + _STEP_HIGH_US) * 1e-6

    def estimate_duration_mm(self, mm: float) -> float:
        """Doğrusal mesafe (mm) için tahmini hareket süresi (saniye)."""
        return self.estimate_duration_s(self.mm_to_steps(mm))

    # ── Acil durdurma ─────────────────────────────────────────────────────── #
    def emergency_stop(self) -> None:
        self._emergency = True
        self._log.warning("DRV8834 EMERGENCY STOP!")

    def reset_emergency(self) -> None:
        self._emergency = False

    # ── Kapatma ───────────────────────────────────────────────────────────── #
    def close(self) -> None:
        if _GPIO_AVAILABLE and self._handle is not None:
            lgpio.gpiochip_close(self._handle)
            self._handle = None
            self._log.debug("lgpio gpiochip kapatıldı.")
