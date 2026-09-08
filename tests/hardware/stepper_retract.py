#!/usr/bin/env python3
"""
Stepper milini İÇERİ çeker (retract) — Raspberry Pi 5 / GPIO.

  python3 tests/hardware/stepper_retract.py [--steps N] [--out]

TEK AMAÇ: Mil dışarıda kalmışsa (ör. elle içeri sokulamıyorsa) step motoru
sürüp mili içeri/yukarı (ilk konum) doğru çekmek.

  • Varsayılan yön: İÇERİ (move_up / DIR_UP) — config'teki hız ve rampa
    profili kullanılır.
  • Adım sayısı: config'teki steps_for_travel (11429 ≈ 40 mm) varsayılandır.
  • --out verilirse TERS yön (dışarı) sürülür — yalnızca yön testi için.

UÇUŞ KOMUTU GÖNDERMEZ (MAVLink'e dokunmaz) — yalnızca GPIO.

GÜVENLİK:
  • İlk denemede MUTLAKA küçük adımla yönü doğrula:
      python3 tests/hardware/stepper_retract.py --steps 200
    Mil içeri mi giriyor? Evet → tam çekme için tekrar çalıştır.
    Hayır (dışarı itiyor) → --out ile ters yönü dene.
  • Mil son noktaya dayanırsa motor adım kaybeder (stall) — rampa profili
    nedeniyle zarar riski düşüktür ama mil zaten içerideyse tam 11429 adım
    sürme; önce 200-500 adımla başla.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from tests.hardware import hw_common as hw
from payload.backends.local_rpi import LocalRPiBackend


def run_retract(steps: int, direction_in: bool = True) -> tuple[bool, str]:
    """Mili içeri (direction_in=True) veya dışarı (False) sürer."""
    if not hw.lgpio_available():
        return False, "lgpio yok — gerçek GPIO sürülemez (sudo apt install python3-lgpio)"

    config_path = str(hw.REPO_ROOT / "payload" / "config.yaml")
    backend = LocalRPiBackend.from_config(config_path)
    stepper = backend._stepper   # DRV8834 — config'ten kurulmuş (pin+hız)

    label = "İÇERİ (yukarı/retract)" if direction_in else "DIŞARI (aşağı)"
    try:
        stepper.setup()
        stepper.enable()
        print(f"Stepper mil {label} çekiliyor: {steps} adım...")
        if direction_in:
            stepper.move_up(steps)
        else:
            stepper.move_down(steps)
        stepper.disable()
        print(f"Tamam: mil {label} yönünde {steps} adım sürüldü.")
        return True, f"stepper {steps} adım {label}"
    finally:
        stepper.disable()
        stepper.close()
        print("  cleanup: OK (stepper kapalı)")


def main() -> int:
    ap = argparse.ArgumentParser(description="Stepper milini içeri çeker (retract)")
    ap.add_argument("--steps", type=int, default=None,
                    help="Adım sayısı (varsayılan: config steps_for_travel = 11429)")
    ap.add_argument("--out", action="store_true",
                    help="Ters yön: mili DIŞARI sür (yalnızca yön testi için)")
    args = ap.parse_args()

    if not hw.is_raspberry_pi():
        print("Bu script Raspberry Pi 5 gerektirir (GPIO). Bu makine Pi değil.")
        return 2

    cfg = hw.load_payload_config()
    steps = args.steps or int(cfg["stepper"].get("steps_for_travel", 11429))
    mm = steps * float(cfg["stepper"].get("mm_per_revolution", 0.7)) / float(
        cfg["stepper"].get("steps_per_revolution", 200)
    )

    print(f"Stepper retract: steps={steps} ({mm:.1f} mm) yön={'içeri' if not args.out else 'dışarı'}")
    print("Uyarı: ilk denemede --steps 200 ile yönü doğrulayın!")
    print()
    ok, msg = run_retract(steps, direction_in=not args.out)
    print()
    print("[PASS]" if ok else "[FAIL]", msg)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
