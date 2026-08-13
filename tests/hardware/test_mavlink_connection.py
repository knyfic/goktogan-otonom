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


def _collect_telemetry(master, duration_s: float, timeout_s: float, initial_mode):
    """Heartbeat + GPS + altitude + mode + battery toplar. READ-ONLY.

    Aynı bağlantı üzerinden toplanır (yeniden açma YOK — FC stream'i
    yeni bağlantıda tekrar ramp-up beklemek zorunda kalmaz).
    Mesaj gelmezse bayt/parse istatistiği de döndürür: link ölü mü
    (bayt yok) yoksa taşma/bozulma mı (bayt var, parse hata)?
    """
    types = ("GLOBAL_POSITION_INT", "HEARTBEAT", "SYS_STATUS", "GPS_RAW_INT")
    samples = []
    mav = getattr(master, "mav", None)
    bytes_before = int(getattr(mav, "total_bytes_received", 0))
    errors_before = int(getattr(mav, "total_errors", 0))
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        msg = hw.recv_telemetry(master, types, timeout_s=1.0)
        if msg is not None:
            samples.append(msg)

    stats = {
        "gps": 0,
        "alt_min": None,
        "alt_max": None,
        "mode": initial_mode,   # ilk heartbeat'ten — mod zaten okunabildi
        "battery_pct": None,
        "bytes_received": int(getattr(mav, "total_bytes_received", 0)) - bytes_before,
        "parse_errors": int(getattr(mav, "total_errors", 0)) - errors_before,
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
    return stats


def run_check(
    device: str,
    baud: int,
    duration_s: float = 10.0,
    timeout_s: float = 10.0,
    flow_control: bool = False,
) -> bool:
    print(f"MAVLink READ-ONLY: {device} @ {baud}  (komut gönderilmez)")
    try:
        master, hb = hw.open_mavlink(
            device, baud, timeout_s=timeout_s, flow_control=flow_control
        )
    except TimeoutError as exc:
        print(f"[FAIL] heartbeat  ({exc})")
        return False
    except Exception as exc:
        print(f"[FAIL] bağlantı  ({exc})")
        return False

    mode = hw.flight_mode_name(master, hb)
    print(f"[PASS] heartbeat  sysid={hb.get_srcSystem()} compid={hb.get_srcComponent()} mode={mode}")

    # Aynı bağlantı üzerinden telemetri topla (yeniden açma YOK)
    stats = _collect_telemetry(master, duration_s, timeout_s, mode)
    master.close()

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

    # Teşhis: 0 mesaj geldiyse neden? — bayt mı gelmiyor, yoksa parse mı edilemiyor?
    if stats["gps"] == 0:
        if stats["bytes_received"] == 0:
            print(
                "[DIAG] Hiç bayt gelmedi — FC göndermiyor/kilitli: SERIAL2_OPTIONS "
                "flow control (bit 4) ve SR2_* stream hızlarını kontrol et; Mission "
                "Planner ile TELEM2'ye 921600'de bağlanıp telemetri akıp akmadığını dene."
            )
        else:
            print(
                f"[DIAG] {stats['bytes_received']} bayt geldi ama parse edilemedi "
                f"({stats['parse_errors']} hata) — RX taşması/hatlı UART: daha düşük "
                f"baud dene (--baud 115200) veya flow control eşleştir."
            )

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
    flow_control = bool(conn.get("flow_control", False))

    return 0 if run_check(device, baud, args.duration, args.timeout, flow_control) else 1


def test_mavlink_readonly_telemetry():
    """Cube Orange'dan READ-ONLY telemetri: heartbeat + GPS + altitude + mode."""
    cfg = hw.load_payload_config()
    conn = cfg["flight_controller"]["connection"]
    ok = run_check(
        conn["device"], conn["baudrate"], duration_s=5.0, timeout_s=10.0,
        flow_control=bool(conn.get("flow_control", False)),
    )
    assert ok, "GPS + flight mode telemetrisi alınamadı (test yalnızca telemetri okur)"


if __name__ == "__main__":
    sys.exit(main())
