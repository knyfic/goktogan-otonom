#!/usr/bin/env python3
"""
Payload mekanizması donanım testi (Raspberry Pi 5 — GPIO).

  python tests/hardware/test_payload_hardware.py [--steps 50] [--full-cycle]
                                                 [--release-recover]

Mevcut abstraction'ları KULLANIR — doğrudan GPIO/socket kodu YOK:
  LocalRPiBackend (payload/backends/local_rpi.py)
  → DRV8834 stepper + MagnetController (payload/hardware/*)

Test adımları:
  1. initialize()      — pin kurulumu, mıknatıslar HOLD (güvenli)
  2. status()          — READY + M1/M2 ON doğrula
  3. mini pickup       — stepper ↓ N adım → stepper ↑ N adım (gerçek motor dönüşü)
  4. --release-recover — Magnet 1'i bırakıp geri tutar (DİKKAT: payload düşebilir)

GÜVENLİK:
  • Varsayılan --steps 50 → ~0.2 mm hareket (0.0035 mm/adım), mıknatıslar HOLD'da.
  • --full-cycle → config'teki steps_for_travel (11429 adım, ~28 sn) — gerçek
    pickup sekansı boyu. Önce küçük adımla doğrula.
  • --release-recover: mıknatıs üzerinde payload varsa YERE DÜŞER.

Uçuş komutu GÖNDERMEZ (MAVLink'e dokunmaz).

pytest olarak da çalışır — Raspberry Pi dışında otomatik SKIP:
  python -m pytest tests/hardware/test_payload_hardware.py -v
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

import pytest

from tests.hardware import hw_common as hw

pytestmark = pytest.mark.skipif(
    not hw.is_raspberry_pi(), reason="Raspberry Pi + payload GPIO gerekli"
)

_log = logging.getLogger("payload_hw_test")


def _print_status(backend) -> None:
    st = backend.status()
    print(
        f"  state={st.mechanism_state.value}  "
        f"M1={'ON' if st.magnets.magnet_1_on else 'OFF'}  "
        f"M2={'ON' if st.magnets.magnet_2_on else 'OFF'}  "
        f"carried={st.carried_count}"
    )


def run_payload_test(
    steps: int,
    release_recover: bool = False,
    logger: logging.Logger | None = None,
) -> tuple[bool, str]:
    from payload.backends.local_rpi import LocalRPiBackend

    log = logger or _log
    if not hw.lgpio_available():
        return False, "lgpio yok — gerçek GPIO test edilemez (sudo apt install python3-lgpio)"
    config_path = str(hw.REPO_ROOT / "payload" / "config.yaml")

    backend = LocalRPiBackend.from_config(config_path, steps=steps, logger=log)
    try:
        # 1) initialize — pin kurulumu, mıknatıslar HOLD
        res = backend.initialize()
        if not res.success:
            return False, f"initialize başarısız: {res.message}"
        print("  initialize: OK")
        _print_status(backend)

        st = backend.status()
        if not (st.mechanism_state.value == "READY"
                and st.magnets.magnet_1_on and st.magnets.magnet_2_on):
            return False, f"başlangıç durumu beklenen gibi değil: state={st.mechanism_state.value}"

        # 2) mini pickup — stepper ↓ steps adım (thread) → ↑ steps adım
        print(f"  pickup başlıyor: stepper ↓ {steps} adım...")
        res = backend.send_pickup_command()
        if not res.success:
            return False, f"send_pickup_command başarısız: {res.message}"
        # Küçük adımda thread hızlı biter; yine de güvenli pay bırak
        time.sleep(steps * 0.002 + 0.5)
        res = backend.finish_pickup()
        if not res.success:
            return False, f"finish_pickup başarısız: {res.message}"
        print(f"  pickup tamam: stepper ↑ {steps} adım")
        _print_status(backend)

        # 3) (opsiyonel) magnet release + recover
        if release_recover:
            print("  [DİKKAT] Magnet 1 bırakılıyor (payload varsa düşer)...")
            from payload.models import MagnetId

            res = backend.release_payload_1()
            if not res.success:
                return False, f"release_payload_1 başarısız: {res.message}"
            time.sleep(0.5)
            _print_status(backend)
            res = backend.set_magnet(MagnetId.MAGNET_1, True)
            if not res.success:
                return False, f"set_magnet(MAGNET_1, ON) başarısız: {res.message}"
            time.sleep(0.3)
            print("  Magnet 1 geri tutuldu (recover OK)")
            _print_status(backend)

        st = backend.status()
        ok = st.mechanism_state.value == "READY"
        return ok, f"stepper ↓+↑ {steps} adım + magnet durumu OK"
    finally:
        backend.cleanup()
        print("  cleanup: OK (mıknatıslar HOLD, stepper kapalı)")


def main() -> int:
    ap = argparse.ArgumentParser(description="Payload mekanizması donanım testi (Pi)")
    ap.add_argument("--steps", type=int, default=50, help="Stepper adım sayısı (varsayılan: 50 ~ 0.2mm)")
    ap.add_argument("--full-cycle", action="store_true",
                    help="Config'teki tam steps_for_travel (11429) — ~28 sn pickup sekansı")
    ap.add_argument("--release-recover", action="store_true",
                    help="Magnet 1'i bırakıp geri tutar — payload DÜŞEBİLİR")
    args = ap.parse_args()

    steps = int(args.steps)
    if args.full_cycle:
        cfg = hw.load_payload_config()
        steps = int(cfg["stepper"].get("steps_for_travel", 11429))
        print(f"--full-cycle: {steps} adım (gerçek pickup sekansı)")

    print(f"Payload donanım testi: steps={steps} release_recover={args.release_recover}")
    ok, msg = run_payload_test(steps, release_recover=args.release_recover, logger=_log)
    print()
    print("[PASS]" if ok else "[FAIL]", msg)
    print("No flight commands were sent.")
    return 0 if ok else 1


def test_payload_hardware_small_cycle():
    """Küçük adımla (20) güvenli pickup döngüsü + magnet durumu doğrulaması."""
    ok, msg = run_payload_test(steps=20, release_recover=False, logger=_log)
    assert ok, msg


if __name__ == "__main__":
    sys.exit(main())
