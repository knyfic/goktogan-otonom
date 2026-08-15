#!/usr/bin/env python3
"""
READ-ONLY pre-arm durum testi (Raspberry Pi 5 + Cube Orange).

Gerçek MAVLink üzerinden ArduPilot pre-arm durumunu mevcut Navigation
sınıfıyla okur — ARM/TAKEOFF GÖNDERMEZ.

  python tests/hardware/test_prearm_status.py [--duration 15]

Doğrular:
  1. MAVLink bağlantısı kurulabiliyor mu? (Navigation — serial)
  2. SYS_STATUS geliyor mu?  (onboard_control_sensors_health)
  3. prearm_good() True/False dönebiliyor mu? (ahrs+gps+prearm bitleri)
  4. 'PreArm: ...' STATUSTEXT nedenleri okunuyor mu? (prearm_reason)

pytest olarak da çalışır — normal suite'te ASLA donanıma bağlanmaz:
  IHA_HW_PREARM_TEST=1 python -m pytest tests/hardware/test_prearm_status.py -v
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import pytest

from tests.hardware import hw_common as hw

# Normal pytest suite'i asla gerçek Cube'a bağlanmamalı:
# Raspberry Pi + açık env var (IHA_HW_PREARM_TEST=1) ikisi de şart.
pytestmark = pytest.mark.skipif(
    not (hw.is_raspberry_pi() and os.environ.get("IHA_HW_PREARM_TEST") == "1"),
    reason="REAL donanım: Raspberry Pi + IHA_HW_PREARM_TEST=1 gerekli",
)


def run_check(duration_s: float = 15.0) -> bool:
    """Navigation üzerinden pre-arm durumunu toplar ve raporlar (READ-ONLY)."""
    cfg = hw.load_payload_config()
    conn = cfg["flight_controller"]["connection"]
    print(f"MAVLink REAL (Navigation): {conn['device']} @ {conn['baudrate']}")

    try:
        nav = hw.build_real_navigation(cfg)
    except Exception as exc:
        print(f"[FAIL] MAVLink bağlantısı  ({exc})")
        return False
    print("[PASS] MAVLink: connected")

    saw_sys_status = False
    saw_reason = False
    last_line = ""
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        nav.position()  # telemetriyi boşalt: SYS_STATUS + STATUSTEXT dahil

        good = nav.prearm_good()
        reason = nav.prearm_reason()
        if good is not None:
            saw_sys_status = True
        if reason:
            saw_reason = True

        status = "GOOD" if good is True else ("UNKNOWN" if good is None else "NOT GOOD")
        line = f"ARMABLE: {'YES' if good is True else 'NO'}  PREARM: {status}"
        if reason:
            line += f"  |  {reason}"
        if line != last_line:
            print(line)
            last_line = line
        time.sleep(0.5)

    ok_sys = hw.report(saw_sys_status, "SYS_STATUS alındı",
                       "prearm_good() True/False döndü (sağlık bitleri)")
    ok_reason = hw.report(saw_reason, "STATUSTEXT PreArm nedeni alındı",
                          "prearm_reason() dolu" if saw_reason else "hiç PreArm mesajı yok")
    print()
    print("No flight commands were sent (ARM/TAKEOFF yok).")
    return ok_sys


def test_real_prearm_status_readable():
    """Gerçek Cube'dan pre-arm durumu (SYS_STATUS) okunabiliyor mu?"""
    ok = run_check(duration_s=12.0)
    assert ok, "SYS_STATUS/sağlık bitleri okunamadı (yalnızca telemetri okur)"


def main() -> int:
    ap = argparse.ArgumentParser(description="Pre-arm durum READ-ONLY testi")
    ap.add_argument("--duration", type=float, default=15.0,
                    help="Toplama süresi (sn)")
    args = ap.parse_args()
    return 0 if run_check(args.duration) else 1


if __name__ == "__main__":
    sys.exit(main())
