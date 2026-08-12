#!/usr/bin/env python3
"""
READ-ONLY MAVLink telemetri testi (Raspberry Pi 5 + Cube Orange).

  python tests/hardware/test_mavlink_connection.py [--duration 10] [--timeout 10]

BU TEST HİÇBİR UÇUŞ KOMUTU GÖNDERMEZ:
  ✗ arm / takeoff / goto / land / rtl / set_mode / command_long
  ✓ yalnızca TELEMETRİ OKUR: heartbeat, GPS, altitude, flight mode, battery

pytest olarak da çalışır — Raspberry Pi dışında otomatik SKIP:
  python -m pytest tests/hardware/test_mavlink_connection.py -v
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import pytest

from tests.hardware import hw_common as hw

pytestmark = pytest.mark.skipif(
    not hw.is_raspberry_pi(), reason="Raspberry Pi + Cube Orange gerekli"
)

# Testin hiçbir yerinde komut gönderilmediğini doğrulamak için yasak API listesi
# (yalnızca dokümantasyon amaçlı — bu modülde bu çağrılar bulunmamalı).
_FORBIDDEN = ("command_long_send", "set_mode", "arducopter_arm", "motors_armed_wait")


def _collect_telemetry(device: str, baud: int, duration_s: float, timeout_s: float):
    """Heartbeat + GPS + altitude + mode + battery toplar. READ-ONLY."""
    master, _hb = hw.open_mavlink(device, baud, timeout_s=timeout_s)

    types = ("GLOBAL_POSITION_INT", "HEARTBEAT", "SYS_STATUS", "GPS_RAW_INT")
    samples = []
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        msg = hw.recv_telemetry(master, types, timeout_s=2.0)
        if msg is not None:
            samples.append(msg)

    stats = {
        "gps": 0,
        "alt_min": None,
        "alt_max": None,
        "mode": None,
        "battery_pct": None,
    }
    for m in samples:
        t = m.get_type()
        if t == "GLOBAL_POSITION_INT":
            stats["gps"] += 1
            alt = m.relative_alt / 1000.0
            stats["alt_min"] = alt if stats["alt_min"] is None else min(stats["alt_min"], alt)
            stats["alt_max"] = alt if stats["alt_max"] is None else max(stats["alt_max"], alt)
        elif t == "HEARTBEAT":
            stats["mode"] = hw.flight_mode_name(master, m) or stats["mode"]
        elif t == "SYS_STATUS" and m.battery_remaining != -1:
            stats["battery_pct"] = m.battery_remaining

    master.close()
    return stats


def run_check(device: str, baud: int, duration_s: float = 10.0, timeout_s: float = 10.0) -> bool:
    print(f"MAVLink READ-ONLY: {device} @ {baud}  (komut gönderilmez)")
    try:
        master, hb = hw.open_mavlink(device, baud, timeout_s=timeout_s)
    except TimeoutError as exc:
        print(f"[FAIL] heartbeat  ({exc})")
        return False
    except Exception as exc:
        print(f"[FAIL] bağlantı  ({exc})")
        return False

    mode = hw.flight_mode_name(master, hb)
    print(f"[PASS] heartbeat  sysid={hb.get_srcSystem()} compid={hb.get_srcComponent()} mode={mode}")
    master.close()

    stats = _collect_telemetry(device, baud, duration_s, timeout_s)
    print(f"[{'PASS' if stats['gps'] > 0 else 'FAIL'}] GPS telemetry  ({stats['gps']} örnek)")
    if stats["alt_min"] is not None:
        print(
            f"[PASS] Altitude telemetry  (min={stats['alt_min']:.1f}m max={stats['alt_max']:.1f}m relative)"
        )
    else:
        print("[FAIL] Altitude telemetry  (örnek yok)")
    print(
        f"[{'PASS' if stats['mode'] else 'FAIL'}] Flight mode  ({stats['mode'] or 'yok'})"
    )
    if stats["battery_pct"] is not None:
        print(f"[PASS] Battery  ({stats['battery_pct']}%)")
    else:
        print("[SKIP] Battery  (SYS_STATUS gelmedi)")

    ok = stats["gps"] > 0 and stats["mode"] is not None
    print()
    print("No flight commands were sent.")
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description="MAVLink READ-ONLY telemetri testi")
    ap.add_argument("--device", default=None, help="Seri cihaz (varsayılan: config)")
    ap.add_argument("--baud", type=int, default=None, help="Baudrate (varsayılan: config)")
    ap.add_argument("--duration", type=float, default=10.0, help="Telemetri toplama süresi (sn)")
    ap.add_argument("--timeout", type=float, default=10.0, help="Heartbeat bekleme süresi (sn)")
    args = ap.parse_args()

    cfg = hw.load_payload_config()
    conn = cfg.get("flight_controller", {}).get("connection", {}) or {}
    device = args.device or conn.get("device", "/dev/serial0")
    baud = args.baud or conn.get("baudrate", 921600)

    return 0 if run_check(device, baud, args.duration, args.timeout) else 1


def test_mavlink_readonly_telemetry():
    """Cube Orange'dan READ-ONLY telemetri: heartbeat + GPS + altitude + mode."""
    cfg = hw.load_payload_config()
    conn = cfg["flight_controller"]["connection"]
    ok = run_check(conn["device"], conn["baudrate"], duration_s=5.0, timeout_s=10.0)
    assert ok, "GPS + flight mode telemetrisi alınamadı (test yalnızca telemetri okur)"


if __name__ == "__main__":
    sys.exit(main())
