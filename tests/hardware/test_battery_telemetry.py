#!/usr/bin/env python3
"""
READ-ONLY batarya telemetri testi (Raspberry Pi 5 + Cube Orange).

Gerçek MAVLink BATTERY_STATUS / SYS_STATUS mesajlarının mevcut Navigation
sınıfı üzerinden okunabildiğini doğrular — mock yok, SITL verisi yok.

  python tests/hardware/test_battery_telemetry.py [--duration 15]

BU TEST HİÇBİR UÇUŞ KOMUTU GÖNDERMEZ:
  ✗ arm / takeoff / goto / land / rtl / set_mode / command_long
  ✓ yalnızca MAVLink bağlantısı + telemetri okuma (GCS heartbeat el sıkışması)

pytest olarak da çalışır — normal suite'i ASLA gerçek Cube'a bağlamaz:
  - Raspberry Pi dışında otomatik SKIP.
  - Raspberry Pi'de dahi yalnızca IHA_HW_BATTERY_TEST=1 env var ile çalışır:
      IHA_HW_BATTERY_TEST=1 python -m pytest tests/hardware/test_battery_telemetry.py -v
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import pytest

from tests.hardware import hw_common as hw

# Normal pytest suite'i asla gerçek Cube'a bağlanmamalı:
# Raspberry Pi + açık env var (IHA_HW_BATTERY_TEST=1) ikisi de şart.
pytestmark = pytest.mark.skipif(
    not (hw.is_raspberry_pi() and os.environ.get("IHA_HW_BATTERY_TEST") == "1"),
    reason="REAL donanım: Raspberry Pi + IHA_HW_BATTERY_TEST=1 gerekli",
)

# Voltaj için genel sanity sınırı — belirli hücre sayısı VARSAYILMAZ.
# LiPo 14S maks ~60.9V (4.35V/hücre); 60V üstü fiziksel olarak mantıksız.
VOLTAGE_MAX_V = 60.0


def run_check(duration_s: float = 15.0) -> bool:
    """REAL bağlantı kurar, batarya telemetrisini toplar ve doğrular."""
    cfg = hw.load_payload_config()
    conn = cfg["flight_controller"]["connection"]
    device = conn["device"]
    baud = conn["baudrate"]
    flow_control = bool(conn.get("flow_control", False))
    print(f"MAVLink REAL (Navigation): {device} @ {baud}")

    try:
        nav = hw.build_real_navigation(cfg)
    except Exception as exc:
        print(f"[FAIL] MAVLink bağlantısı  ({exc})")
        return False
    print("[PASS] MAVLink: connected")

    v, pct = hw.collect_battery_telemetry(nav, duration_s=duration_s)
    pos = nav.position()

    print(f"Battery voltage: {v:.1f} V")
    print(f"Battery percentage: {pct} %")

    ok_v = v > 0
    print(f"[{'PASS' if ok_v else 'FAIL'}] BATTERY_STATUS  "
          f"({'received' if ok_v else 'gelmedi — BATT_MONITOR/stream kontrol'})")

    ok_pct = pct != -1
    print(f"[{'PASS' if ok_pct else 'SKIP'}] SYS_STATUS  "
          f"({'received' if ok_pct else 'gelmedi (FC yüzde bildirmiyor olabilir)'})")

    ok_range_v = ok_v and v < VOLTAGE_MAX_V
    print(f"[{'PASS' if ok_range_v else 'FAIL'}] Voltage sanity  "
          f"(0 < {v:.1f}V < {VOLTAGE_MAX_V:.0f}V)")

    ok_range_pct = (not ok_pct) or (0 <= pct <= 100)
    print(f"[{'PASS' if ok_range_pct else 'FAIL'}] Percentage sanity  ({pct}%)")

    print(
        "VehiclePosition: "
        f"battery_voltage_v={pos.battery_voltage_v:.3f} "
        f"battery_percent={pos.battery_percent} "
        f"gps=({pos.latitude:.5f}, {pos.longitude:.5f})"
    )

    ok = ok_v and ok_range_v and ok_range_pct
    print()
    print("No flight commands were sent.")
    return ok


def test_real_battery_telemetry():
    """Gerçek Cube Orange'dan BATTERY_STATUS / SYS_STATUS okunabiliyor mu?"""
    ok = run_check(duration_s=15.0)
    assert ok, "Gerçek batarya telemetrisi alınamadı (yalnızca telemetri okur, komut göndermez)"


def main() -> int:
    ap = argparse.ArgumentParser(description="Batarya telemetri READ-ONLY testi")
    ap.add_argument("--duration", type=float, default=15.0,
                    help="Telemetri toplama süresi (sn)")
    args = ap.parse_args()
    return 0 if run_check(args.duration) else 1


if __name__ == "__main__":
    sys.exit(main())
