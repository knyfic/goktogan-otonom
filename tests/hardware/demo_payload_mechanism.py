#!/usr/bin/env python3
"""
Görev mekanizması teknik kontrol / jüri gösterim betiği (Raspberry Pi 5 — GPIO).

Görev (yarışma) sırasında step motor KULLANILMAZ — yalnızca mıknatıslar
(LocalRPiBackend: send_pickup_command yalnızca magnetleri açar). Bu betik
YALNIZCA teknik kontrolde "mekanizmayı çalıştırın" denildiğinde gösterim içindir.

Daha önce kaydedilmiş hız + adım sayıları payload/config.yaml → stepper
bölümünden okunur (steps_for_travel = 11429 adım ≈ 40 mm, rampa hızları dahil).

Gösterim sırası (her adım arasında --pause saniye bekler):

  [1/7] Başlangıç        : mıknatıslar AKTİF (M1 ON, M2 ON) — yükler tutuluyor
  [2/7] Yük 1 bırakma    : Magnet 1 OFF  → payload 1 düşer
  [3/7] Yük 2 bırakma    : Magnet 2 OFF  → payload 2 düşer
  [4/7] Stepper AŞAĞI    : mil step motorla AŞAĞI indirilir (platform yere yaklaşır)
  [5/7] Yük alımı        : mıknatıslar AKTİF → payloadlar platforma tutunur
  [6/7] Yük bırakma      : mıknatıslar OFF (platform aşağıdayken kontrollü bırakma)
  [7/7] Stepper YUKARI   : mil ters yönde YUKARI çekilir → ilk konuma dönüş

GÜVENLİK / ÖN KOŞULLAR:
  • SADECE Raspberry Pi 5'te çalışır (lgpio gerekir: sudo apt install python3-lgpio).
  • Uçuş komutu GÖNDERMEZ (MAVLink'e dokunmaz).
  • İlk denemede mutlaka küçük adımla test et:  --steps 200
  • Varsayılan --steps: config'teki steps_for_travel (11429) → 40 mm TAM seyahat.
    İlk konumda platform YUKARIDA ve aşağı yönde serbest hareket OLMALIDIR;
    mil son noktadaysa motor sıkışır. İlk hareket yönü ters ise --invert kullan.
  • Mıknatıslarda gerçek payload varsa bırakma adımlarında YERE DÜŞER.

Kullanım:
  python3 tests/hardware/demo_payload_mechanism.py                # tam çevrim
  python3 tests/hardware/demo_payload_mechanism.py --steps 500    # güvenli mini
  python3 tests/hardware/demo_payload_mechanism.py --pause 2.0    # bekleme süresi
  python3 tests/hardware/demo_payload_mechanism.py --invert       # ilk hareket yukarı
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from tests.hardware import hw_common as hw
from payload.backends.local_rpi import LocalRPiBackend
from payload.models import MagnetId


# ── Yardımcılar ────────────────────────────────────────────────────────────── #
def _banner(phase: str) -> None:
    print("\n" + "=" * 64)
    print(phase)
    print("=" * 64)


def _status(backend) -> None:
    st = backend.status()
    print(
        f"  state={st.mechanism_state.value}  "
        f"M1={'ON' if st.magnets.magnet_1_on else 'OFF'}  "
        f"M2={'ON' if st.magnets.magnet_2_on else 'OFF'}"
    )


def _pause(pause_s: float) -> None:
    if pause_s <= 0:
        return
    print(f"  ... {pause_s:.1f}s bekleniyor ...")
    time.sleep(pause_s)


def _travel_steps(cfg: dict, steps_arg: int | None) -> int:
    """--steps verilmediyse config'teki steps_for_travel (tam seyahat)."""
    if steps_arg and steps_arg > 0:
        return steps_arg
    return int(cfg["stepper"].get("steps_for_travel", 11429))


# ── Ana gösteri ────────────────────────────────────────────────────────────── #
def run_demo(
    steps: int,
    pause_s: float = 3.0,
    invert: bool = False,
) -> tuple[bool, str]:
    if not hw.lgpio_available():
        return False, "lgpio yok — gerçek GPIO test edilemez (sudo apt install python3-lgpio)"

    config_path = str(hw.REPO_ROOT / "payload" / "config.yaml")
    backend = LocalRPiBackend.from_config(config_path)
    stepper = backend._stepper      # DRV8834 — gösterim için doğrudan sürülür
    magnets = backend._magnets      # MagnetController

    down_label = "YUKARI" if invert else "AŞAĞI"
    up_label = "AŞAĞI" if invert else "YUKARI"
    move_first = (stepper.move_up if invert else stepper.move_down)
    move_last = (stepper.move_down if invert else stepper.move_up)

    try:
        # [1/7] Başlangıç — pin kurulumu, mıknatıslar HOLD
        _banner("[1/7] BAŞLANGIÇ — mıknatıslar AKTİF (M1 ON, M2 ON)")
        res = backend.initialize()
        if not res.success:
            return False, f"initialize başarısız: {res.message}"
        _status(backend)
        _pause(pause_s)

        # [2/7] Yük 1 bırakma — Magnet 1 OFF
        _banner("[2/7] YÜK 1 BIRAKILIYOR — Magnet 1 OFF (payload 1 düşer)")
        res = backend.set_magnet(MagnetId.MAGNET_1, False)
        if not res.success:
            return False, f"Magnet 1 OFF başarısız: {res.message}"
        _status(backend)
        _pause(pause_s)

        # [3/7] Yük 2 bırakma — Magnet 2 OFF
        _banner("[3/7] YÜK 2 BIRAKILIYOR — Magnet 2 OFF (payload 2 düşer)")
        res = backend.set_magnet(MagnetId.MAGNET_2, False)
        if not res.success:
            return False, f"Magnet 2 OFF başarısız: {res.message}"
        _status(backend)
        _pause(pause_s)

        # [4/7] Stepper — mil AŞAĞI (platform yere doğru)
        _banner(f"[4/7] STEPPER — mil {down_label} indiriliyor ({steps} adım, ~40 mm)")
        stepper.enable()
        move_first(steps)
        print(f"  stepper {down_label} tamam ({steps} adım)")
        _status(backend)
        _pause(pause_s)

        # [5/7] Yük alımı — mıknatıslar AKTİF
        _banner("[5/7] YÜK ALIMI — mıknatıslar AKTİF (M1 ON, M2 ON)")
        res = backend.set_magnet(MagnetId.MAGNET_1, True)
        if not res.success:
            return False, f"Magnet 1 ON başarısız: {res.message}"
        res = backend.set_magnet(MagnetId.MAGNET_2, True)
        if not res.success:
            return False, f"Magnet 2 ON başarısız: {res.message}"
        _status(backend)
        _pause(pause_s)

        # [6/7] Yük bırakma — mıknatıslar OFF (platform aşağıda, kontrollü bırakma)
        _banner("[6/7] YÜK BIRAKMA — mıknatıslar OFF (platform aşağıda)")
        res = backend.set_magnet(MagnetId.MAGNET_1, False)
        if not res.success:
            return False, f"Magnet 1 OFF başarısız: {res.message}"
        res = backend.set_magnet(MagnetId.MAGNET_2, False)
        if not res.success:
            return False, f"Magnet 2 OFF başarısız: {res.message}"
        _status(backend)
        _pause(pause_s)

        # [7/7] Stepper — mil YUKARI (ilk konuma dönüş)
        _banner(f"[7/7] STEPPER — mil {up_label} çekilerek İLK KONUMA dönülüyor ({steps} adım)")
        move_last(steps)
        stepper.disable()
        print(f"  stepper {up_label} tamam — mekanizma ilk konumda")
        _status(backend)

        # Kapanış: güvenli tutma (mıknatıslar tekrar HOLD)
        magnets.safe_hold()
        _status(backend)
        return True, "gösterim tamamlandı (mıknatıslar HOLD, stepper kapalı)"
    finally:
        backend.cleanup()
        print("\n  cleanup: OK (mıknatıslar HOLD, stepper kapalı)")


def main() -> int:
    ap = argparse.ArgumentParser(description="Görev mekanizması teknik kontrol gösterimi (Pi)")
    ap.add_argument("--steps", type=int, default=None,
                    help="Stepper adım sayısı (varsayılan: config steps_for_travel = 11429). "
                         "İlk denemede küçük değer önerilir (örn. 200).")
    ap.add_argument("--pause", type=float, default=3.0,
                    help="Adımlar arası bekleme süresi, saniye (varsayılan: 3.0)")
    ap.add_argument("--invert", action="store_true",
                    help="İlk step hareketini YUKARI yap (mekanizma yönü ters bağlıysa)")
    args = ap.parse_args()

    if not hw.is_raspberry_pi():
        print("⚠  Bu makine Raspberry Pi değil — GPIO testi yalnızca Pi üzerinde çalışır.")
        return 1

    cfg = hw.load_payload_config()
    steps = _travel_steps(cfg, args.steps)
    mm = steps * float(cfg["stepper"].get("mm_per_revolution", 0.7)) / float(
        cfg["stepper"].get("steps_per_revolution", 200)
    )

    print(f"Görev mekanizması teknik kontrol gösterimi")
    print(f"  steps={steps} ({mm:.1f} mm seyahat)  pause={args.pause}s  invert={args.invert}")
    print(f"  Uçuş komutu GÖNDERİLMEZ (yalnızca GPIO: stepper + mıknatıslar)")
    print()

    ok, msg = run_demo(steps, pause_s=args.pause, invert=args.invert)
    print()
    print("[PASS]" if ok else "[FAIL]", msg)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
