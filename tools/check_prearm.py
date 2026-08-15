#!/usr/bin/env python3
"""
ArduPilot pre-arm durumu teşhisi (READ-ONLY — ARM/TAKEOFF GÖNDERMEZ).

  python3 tools/check_prearm.py [--duration 120] [--interval 1.0]

Mevcut Navigation üzerinden (payload/config.yaml → flight_controller.connection):
  • SYS_STATUS sağlık bitleri (AHRS + GPS + PREARM_CHECK) → prearm_good()
  • STATUSTEXT 'PreArm: ...' → prearm_reason()

Örnek çıktı:
  MAVLink: CONNECTED
  ARMABLE: NO    PREARM: NOT GOOD  |  PreArm: GPS 1: Bad fix
  ARMABLE: NO    PREARM: NOT GOOD  |  PreArm: Compass not calibrated
  ARMABLE: YES   PREARM: GOOD

Durum her değiştiğinde yazdırılır. GOOD olunca exit 0, süre dolunca exit 1.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from tests.hardware import hw_common as hw


def main() -> int:
    ap = argparse.ArgumentParser(description="Pre-arm durumu teşhisi (READ-ONLY)")
    ap.add_argument("--duration", type=float, default=120.0,
                    help="Maksimum bekleme (sn); GOOD gelirse erken çıkar")
    ap.add_argument("--interval", type=float, default=1.0,
                    help="Kontrol aralığı (sn)")
    args = ap.parse_args()

    cfg = hw.load_payload_config()
    try:
        nav = hw.build_real_navigation(cfg)
    except Exception as exc:
        print(f"MAVLink: CONNECT FAIL  ({exc})")
        return 1
    print("MAVLink: CONNECTED")

    deadline = time.monotonic() + args.duration
    last_line = ""
    while time.monotonic() < deadline:
        nav.position()   # SYS_STATUS + STATUSTEXT dahil tüm telemetriyi boşalt

        good = nav.prearm_good()
        reason = nav.prearm_reason()

        if good is True:
            print("ARMABLE: YES   PREARM: GOOD")
            return 0
        status = "NOT GOOD" if good is False else "UNKNOWN (SYS_STATUS yok/bayat)"
        line = f"ARMABLE: NO   PREARM: {status}"
        if reason:
            line += f"   |   {reason}"
        if line != last_line:
            print(line)
            last_line = line
        time.sleep(max(0.1, args.interval))

    print(f"\n{args.duration:.0f}s içinde pre-arm GOOD olmadı.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
