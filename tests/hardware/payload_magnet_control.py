#!/usr/bin/env python3
"""
Payload mıknatıs kontrolü — Raspberry Pi 5 / GPIO (komut satırı).

  python3 tests/hardware/payload_magnet_control.py [--red] [--blue] [--off]

TEK AMAÇ: Step motor HİÇ aktifleştirilmeden / çağrılmadan, komut satırı
argümanlarıyla mıknatısları açıp kapatmak (teknik kontrol/jüri gösterimi).

  --red          → KIRMIZI payload mıknatısını (Magnet 1) AKTİF et
  --blue         → MAVİ payload mıknatısını (Magnet 2) AKTİF et
  --red --blue   → İkisini de AKTİF et
  --red --off    → KIRMIZI mıknatısı DEAKTİF et (bırakma)
  --blue --off   → MAVİ mıknatısı DEAKTİF et
  --red --blue --off → İkisini de DEAKTİF et
  (argümansız)   → Yalnızca mevcut durumu göster (durum değiştirmez)

Renk→mıknatıs eşlemesi TEK kaynaktan gelir: payload/config.yaml →
payload_color_map (RED: magnet_1, BLUE: magnet_2). Kod içinde hard-code YOK.

GÜVENLİK:
  • Step motor import edilmez / kurulmaz / sürülmez (yalnızca MagnetController).
  • Uçuş komutu GÖNDERMEZ (MAVLink'e dokunmaz).
  • Mıknatıslar DEAKTİF edildiğinde üzerlerinde payload varsa YERE DÜŞER.
  • Script sonunda mıknatıslar güvenli HOLD'a (ikisi de ON) alınır — kaza ile
    payload düşmesini önler. Gösterim sırasındaki aç/kapa durumu ekranda
    adım adım görünür.
"""
from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from tests.hardware import hw_common as hw

_log = logging.getLogger("payload_magnet_control")


def _print_status(magnets, pins: dict) -> None:
    print()
    print("Durum:")
    for label, (state, pin) in pins.items():
        print(f"  {label:<14}: {'ON (tutuyor)' if state else 'OFF (bıraktı)'}  "
              f"[pin {pin}]")


def _resolve_magnet_id(color: str, color_map: dict) -> str:
    """config'teki payload_color_map'ten 'RED'/'BLUE' → 'MAGNET_1'/'MAGNET_2'."""
    magnet = str(color_map.get(color, "")).strip()
    if not magnet:
        raise ValueError(f"payload_color_map içinde {color} eşleşmesi yok")
    return magnet.upper()


def run_magnet_control(red: bool, blue: bool, off: bool) -> tuple[bool, str]:
    """Mıknatısları istenen duruma getirir. (Stepper DOKUNULMAZ.)"""
    if not hw.lgpio_available():
        return False, "lgpio yok — gerçek GPIO sürülemez (sudo apt install python3-lgpio)"

    from payload.hardware.magnets import MagnetController

    cfg = hw.load_payload_config()
    mc = cfg.get("magnets", {}) or {}
    chip = int(cfg.get("gpio", {}).get("chip", 0))
    color_map = cfg.get("payload_color_map", {}) or {}

    pin_1 = int(mc.get("magnet_1_pin", 5))
    pin_2 = int(mc.get("magnet_2_pin", 6))
    hold_high = (mc.get("hold_level", "HIGH") == "HIGH")

    magnets = MagnetController(pin_1=pin_1, pin_2=pin_2,
                               hold_high=hold_high, chip=chip, logger=_log)
    try:
        # Güvenli başlangıç: her iki mıknatıs HOLD (ON)
        magnets.setup(initial_on=True)
        _print_status(
            magnets,
            {"Magnet 1": (magnets.state_1, pin_1), "Magnet 2": (magnets.state_2, pin_2)},
        )

        if red:
            mid = _resolve_magnet_id("RED", color_map)
            magnets.set_by_id(mid, not off)
        if blue:
            mid = _resolve_magnet_id("BLUE", color_map)
            magnets.set_by_id(mid, not off)

        _print_status(
            magnets,
            {"Magnet 1": (magnets.state_1, pin_1), "Magnet 2": (magnets.state_2, pin_2)},
        )

        if not (red or blue):
            print("\nArgüman verilmedi — yalnızca durum gösterildi, değişiklik yapılmadı.")

        ok = True
        detail = f"M1={'ON' if magnets.state_1 else 'OFF'} " \
                 f"M2={'ON' if magnets.state_2 else 'OFF'}"
        return ok, detail
    finally:
        # Güvenli kapanış: kaza ile payload düşmesin diye her iki mıknatıs HOLD
        magnets.safe_hold()
        magnets.close()
        print("\n  cleanup: mıknatıslar HOLD (M1=ON M2=ON), stepper hiç kullanılmadı")


def main() -> int:
    ap = argparse.ArgumentParser(description="Payload mıknatıs kontrolü (Pi)")
    ap.add_argument("--red", action="store_true", help="KIRMIZI payload mıknatısını AKTİF et")
    ap.add_argument("--blue", action="store_true", help="MAVİ payload mıknatısını AKTİF et")
    ap.add_argument("--off", action="store_true",
                    help="--red/--blue ile birlikte: DEAKTİF et (bırakma)")
    args = ap.parse_args()

    if not hw.is_raspberry_pi():
        print("Bu script Raspberry Pi 5 gerektirir (GPIO). Bu makine Pi değil.")
        return 2

    if args.off and not (args.red or args.blue):
        print("Hata: --off tek başına kullanılamaz; --red ve/veya --blue ile kullanın.")
        return 2

    print("Payload mıknatıs kontrolü (stepper KULLANILMAZ, uçuş komutu YOK)")
    ok, msg = run_magnet_control(red=args.red, blue=args.blue, off=args.off)
    print()
    print("[PASS]" if ok else "[FAIL]", msg)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
