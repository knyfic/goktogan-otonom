#!/usr/bin/env python3
"""
Payload mıknatıs kontrolü — Raspberry Pi 5 / GPIO (komut satırı).

  python3 tests/hardware/payload_magnet_control.py --red            # M1 AKTİF
  python3 tests/hardware/payload_magnet_control.py --red --off      # M1 DEAKTİF
  python3 tests/hardware/payload_magnet_control.py --blue           # M2 AKTİF
  python3 tests/hardware/payload_magnet_control.py --blue --off     # M2 DEAKTİF
  python3 tests/hardware/payload_magnet_control.py --red --blue     # İkisi AKTİF
  python3 tests/hardware/payload_magnet_control.py --red --blue --off  # İkisi DEAKTİF

  --on / --off verilmezse seçilen mıknatıs AKTİF edilir (--on varsayılan).

BU SCRIPT, demo_payload_mechanism.py'nin BİREBİR mıknatıs kontrol desenini
kullanır:
    backend = LocalRPiBackend.from_config("payload/config.yaml")
    backend.set_magnet(MagnetId.MAGNET_1, True/False)   # demo [2/7],[3/7] ile aynı
    backend.status() → st.magnets.magnet_1_on / magnet_2_on

STEP MOTOR:
  • LocalRPiBackend.from_config stepper nesnesini yalnızca KURAR (GPIO'ya
    dokunmaz). Bu script backend.initialize() ÇAĞIRMAZ (o, stepper pinlerini
    de kurar); yalnızca mıknatıs pinleri kurulur.
  • Stepper enable/move asla çağrılmaz — motor HİÇ hareket etmez.

UÇUŞ KOMUTU GÖNDERMEZ (MAVLink'e dokunmaz). Yalnızca GPIO.

GÜVENLİK:
  • --off verildiğinde mıknatısta payload varsa YERE DÜŞER — altına yumuşak
    bir yüzey koyun veya elle tutun.
  • Mıknatıs sürücü kartı failsafe çalışıyorsa (GPIO hattı boşta kalınca ON'a
    dönüyorsa), OFF'u korumak için script'in AÇIK kalması gerekir. Bu script
    --off sonrası --hold-s süresi boyunca (varsayılan 3 sn) açık kalır; süre
    dolunca kapanır.
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

# --off sonrası açık kalma süresi (sn): yükün düşmesi + OFF'un görülmesi için.
_OFF_HOLD_S = 3.0


def _status(magnets) -> None:
    """M1/M2 durumunu mıknatıs kontrolcüsünden okur (demo ile aynı kaynak)."""
    print(
        f"  M1={'ON' if magnets.state_1 else 'OFF'}  "
        f"M2={'ON' if magnets.state_2 else 'OFF'}"
    )


def _magnet_id(color: str, color_map: dict):
    """payload_color_map'ten 'RED'/'BLUE' → MagnetId enum üyesi (demo ile aynı)."""
    from payload.models import MagnetId

    magnet = str(color_map.get(color, "")).strip().upper()   # "MAGNET_1"
    return MagnetId(magnet)


def run_magnet_control(red: bool, blue: bool, off: bool,
                       hold_s: float = _OFF_HOLD_S) -> tuple[bool, str]:
    """Mıknatısları demo_payload_mechanism.py deseniyle kontrol eder.

    backend.set_magnet() → mıknatıs ON/OFF. Stepper'a HİÇ dokunulmaz.
    """
    if not hw.lgpio_available():
        return False, "lgpio yok — gerçek GPIO test edilemez (sudo apt install python3-lgpio)"

    from payload.backends.local_rpi import LocalRPiBackend

    cfg = hw.load_payload_config()
    color_map = cfg.get("payload_color_map", {}) or {}
    config_path = str(hw.REPO_ROOT / "payload" / "config.yaml")

    backend = LocalRPiBackend.from_config(config_path)
    try:
        # Yalnızca mıknatıs pinlerini kur (backend.initialize DEĞİL — o stepper
        # pinlerini de kurar; stepper'a hiç dokunulmaz). Demo initialize eder
        # ama burada stepper istenmiyor → magnets doğrudan kurulur.
        magnets = backend._magnets          # demo ile aynı erişim
        magnets.setup(initial_on=True)
        print("  mıknatıs pinleri kuruldu (M1=ON M2=ON), stepper'a dokunulmadı")
        _status(magnets)

        target = not off   # --off yoksa → AKTİF (--on varsayılan)

        if red:
            mid = _magnet_id("RED", color_map)
            res = backend.set_magnet(mid, target)   # demo [2/7] ile AYNI API
            if not res.success:
                return False, f"{mid.value} başarısız: {res.message}"
            print(f"  {mid.value} → {'ON (tutuyor)' if target else 'OFF (bıraktı)'}")
            _status(magnets)

        if blue:
            mid = _magnet_id("BLUE", color_map)
            res = backend.set_magnet(mid, target)
            if not res.success:
                return False, f"{mid.value} başarısız: {res.message}"
            print(f"  {mid.value} → {'ON (tutuyor)' if target else 'OFF (bıraktı)'}")
            _status(magnets)

        detail = (f"M1={'ON' if magnets.state_1 else 'OFF'} "
                  f"M2={'ON' if magnets.state_2 else 'OFF'}")

        if off:
            # OFF'un gözle görülmesi / yükün düşmesi için script açık kalır
            print(f"\n  --off uygulandı; {hold_s:.0f}s açık tutuluyor (yük düşer, "
                  "OFF görünür)...")
            time.sleep(hold_s)
        return True, detail
    finally:
        # Mıknatısları son durumda bırak; GPIO handle'ı kapat (kart failsafe'i
        # devralır). Stepper disable/close edilmez (hiç kurulmadı).
        backend._magnets.close()


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
    ap.add_argument("--hold-s", type=float, default=_OFF_HOLD_S,
                    help=f"--off sonrası açık kalma süresi, sn (varsayılan: {_OFF_HOLD_S})")
    args = ap.parse_args()

    if not hw.is_raspberry_pi():
        print("Bu script Raspberry Pi 5 gerektirir (GPIO). Bu makine Pi değil.")
        return 2

    if not (args.red or args.blue):
        print("Hata: en az bir mıknatıs seçmelisiniz (--red ve/veya --blue).")
        return 2

    print(f"Payload mıknatıs kontrolü (demo_payload_mechanism deseni): "
          f"red={args.red} blue={args.blue} "
          f"durum={'OFF (bırak)' if not args.on else 'ON (tut)'}")
    print("  Stepper KULLANILMAZ — uçuş komutu YOK — yalnızca GPIO")
    print()
    ok, msg = run_magnet_control(red=args.red, blue=args.blue, off=not args.on,
                                 hold_s=args.hold_s)
    print()
    print("[PASS]" if ok else "[FAIL]", msg)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
