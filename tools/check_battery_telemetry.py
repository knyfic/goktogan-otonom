#!/usr/bin/env python3
"""
REAL mod batarya telemetri teşhisi (Raspberry Pi + Cube Orange).

  python tools/check_battery_telemetry.py [--duration 15]

Mevcut Navigation sınıfını + payload/config.yaml → flight_controller.connection
kullanır (main.py REAL mod kurulumuyla birebir aynı). UÇUŞ KOMUTU GÖNDERMEZ.

Örnek çıktı:
  Connected to FC: OK
  GPS: 41.007751, 28.978222
  Battery voltage: 24.8 V
  Battery percentage: 78 %
  Battery telemetry: OK
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from tests.hardware import hw_common as hw


def main() -> int:
    ap = argparse.ArgumentParser(description="Batarya telemetri teşhisi (READ-ONLY)")
    ap.add_argument("--duration", type=float, default=15.0,
                    help="Telemetri toplama süresi (sn)")
    args = ap.parse_args()

    cfg = hw.load_payload_config()
    try:
        nav = hw.build_real_navigation(cfg)
    except Exception as exc:
        print(f"Connected to FC: FAIL  ({exc})")
        return 1
    print("Connected to FC: OK")

    v, pct = hw.collect_battery_telemetry(nav, duration_s=args.duration)
    pos = nav.position()
    print(f"GPS: {pos.latitude:.6f}, {pos.longitude:.6f}")
    print(f"Battery voltage: {v:.1f} V")
    print(f"Battery percentage: {pct} %")
    print(
        "VehiclePosition: "
        f"battery_voltage_v={pos.battery_voltage_v:.3f} "
        f"battery_percent={pos.battery_percent}"
    )

    ok = v > 0
    print(f"Battery telemetry: {'OK' if ok else 'FAIL — BATTERY_STATUS gelmedi'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
