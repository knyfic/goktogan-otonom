#!/usr/bin/env python3
"""
Payload mıknatıs kontrolü — Raspberry Pi 5 / GPIO (komut satırı).

  python3 tests/hardware/payload_magnet_control.py --red --on
  python3 tests/hardware/payload_magnet_control.py --red --off
  python3 tests/hardware/payload_magnet_control.py --blue --on
  python3 tests/hardware/payload_magnet_control.py --blue --off
  python3 tests/hardware/payload_magnet_control.py --red --blue --on
  python3 tests/hardware/payload_magnet_control.py --red --blue --off

TEK AMAÇ: Step motor HİÇ aktifleştirilmeden / çağrılmadan, komut satırı
argümanlarıyla mıknatısları açıp kapatmak (teknik kontrol/jüri gösterimi).

  --red          → KIRMIZI payload mıknatısı (Magnet 1) seç
  --blue         → MAVİ payload mıknatısı (Magnet 2) seç
  --on           → Seçilen mıknatıs(lar)ı AKTİF et (tutma)
  --off          → Seçilen mıknatıs(lar)ı DEAKTİF et (bırakma)

  --on / --off verilmezse seçilen mıknatıs AKTİF edilir (--on varsayılan).
  Yalnızca --red veya --blue (durum bayrağı olmadan) → AKTİF et.

KALICILIK (ÖNEMLİ):
  Script mıknatısı istediğiniz durumda BIRAKIR — sonunda zorla ON'a ÇEKMEZ.
    • --off verdiyseniz mıknatıs OFF kalır, üzerindeki yük düşer.
    • --on verdiyseniz mıknatıs ON kalır, yük tutulur.
  Durum, bir sonraki komut / script / Pi yeniden başlatılana kadar korunur.
  (Not: Görev yazılımı (LocalRPiBackend.initialize) başlarken mıknatısları
  kendi güvenli HOLD durumuna alır.)

GÜVENLİK:
  • Step motor import edilmez / kurulmaz / sürülmez (yalnızca MagnetController).
  • Uçuş komutu GÖNDERMEZ (MAVLink'e dokunmaz).
  • --off verildiğinde mıknatısta payload varsa YERE DÜŞER — altına yumuşak
    bir yüzey koyun veya elle tutun.
"""
from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from tests.hardware import hw_common as hw

_log = logging.getLogger("payload_magnet_control")

# --off sonrası bekleme (sn): yükün fiziksel olarak düşmesi için tanınan süre.
_OFF_SETTLE_S = 1.0


def _print_status(magnets, pins: dict) -> None:
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
    """Mıknatısları istenen duruma getirir ve O DURUMDA bırakır.

    Stepper'a DOKUNULMAZ. Script sonunda mıknatıs durumu zorla değiştirilmez —
    --off verildiyse OFF kalır (yük düşer), --on verildiyse ON kalır.
    """
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
        # Pinleri kur (her iki mıknatıs ON başlar — güvenli varsayılan).
        # Ardından istenen komut uygulanır.
        magnets.setup(initial_on=True)

        target = not off   # --off yoksa → AKTİF (--on varsayılan)

        if red:
            magnets.set_by_id(_resolve_magnet_id("RED", color_map), target)
        if blue:
            magnets.set_by_id(_resolve_magnet_id("BLUE", color_map), target)

        _print_status(
            magnets,
            {"Magnet 1": (magnets.state_1, pin_1), "Magnet 2": (magnets.state_2, pin_2)},
        )

        if off:
            # Yükün fiziksel olarak bırakılması için kısa süre tanı
            print(f"\n  --off uygulandı; yükün düşmesi için {_OFF_SETTLE_S:.0f}s "
                  "bekleniyor...")
            time.sleep(_OFF_SETTLE_S)

        if not (red or blue):
            print("\nMıknatıs seçilmedi (--red/--blue) — yalnızca durum gösterildi, "
                  "değişiklik yapılmadı.")

        detail = f"M1={'ON' if magnets.state_1 else 'OFF'} " \
                 f"M2={'ON' if magnets.state_2 else 'OFF'}"
        return True, detail
    finally:
        # ÖNEMLİ: Durum zorla ON'a çekilmez. Mıknatıslar komutun bıraktığı
        # durumda kalır. Yalnızca GPIO handle kapatılır.
        magnets.close()
        print("\n  cleanup: GPIO kapatıldı — mıknatıs durumu KORUNDU "
              "(isteğe bağlı ON/OFF), stepper hiç kullanılmadı")


def main() -> int:
    ap = argparse.ArgumentParser(description="Payload mıknatıs kontrolü (Pi)")
    ap.add_argument("--red", action="store_true", help="KIRMIZI payload mıknatısını seç")
    ap.add_argument("--blue", action="store_true", help="MAVİ payload mıknatısını seç")
    grp = ap.add_mutually_exclusive_group()
    grp.add_argument("--on", dest="on", action="store_true",
                     help="Seçilen mıknatısları AKTİF et (tutma)")
    grp.add_argument("--off", dest="on", action="store_false",
                     help="Seçilen mıknatısları DEAKTİF et (bırakma)")
    ap.set_defaults(on=True)   # --on / --off verilmezse → AKTİF et
    args = ap.parse_args()

    if not hw.is_raspberry_pi():
        print("Bu script Raspberry Pi 5 gerektirir (GPIO). Bu makine Pi değil.")
        return 2

    if not (args.red or args.blue):
        print("Hata: en az bir mıknatıs seçmelisiniz (--red ve/veya --blue).")
        print("Durum görüntülemek için bile mıknatıs seçin; örn: --red --on")
        return 2

    print(f"Payload mıknatıs kontrolü: red={args.red} blue={args.blue} "
          f"durum={'OFF (bırak)' if not args.on else 'ON (tut)'} "
          f"(stepper KULLANILMAZ, uçuş komutu YOK)")
    ok, msg = run_magnet_control(red=args.red, blue=args.blue, off=not args.on)
    print()
    print("[PASS]" if ok else "[FAIL]", msg)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
