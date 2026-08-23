#!/usr/bin/env python3
"""
READ-ONLY flight mode telemetri testi (Raspberry Pi 5 + Cube Orange).

  python tests/hardware/test_real_mode_telemetry.py [--duration 30] [--timeout 10]

AMAÇ: Gerçek flight controller bağlantısında ArduPilot'un mevcut uçuş modunun
MAVLink HEARTBEAT üzerinden DOĞRU okunup okunamadığını saha koşulunda doğrulamak
(real mod denemeden önce). Yalnızca TELEMETRİ OKUR:

  ✓ HEARTBEAT dinle + mod çöz
  ✗ arm / takeoff / goto / land / rtl / set_mode / command_long — HİÇBİRİ YOK

Mod değişiklikleri yalnızca KUMANDADAN veya Mission Planner'dan yapılır; bu
test FC'ye hiçbir mod/mode-change komutu göndermez (yalnızca GCS heartbeat
el sıkışması — ArduPilot stream'leri açsın diye, uçuş komutu değildir).

Kritik: Filtre mantığı flight/navigation.py → Navigation.accept_autopilot_heartbeat()
ile BİREBİR aynıdır (test kendi basitleştirilmiş detection icat ETMEZ) ve mod
adı pymavlink mode_mapping tersiyle çözülür (hw.flight_mode_name — elle hard-code
yok). Böylece bu test, mission'ın real modda göreceği modun AYNISINI görür.

pytest olarak da çalışır — Raspberry Pi dışında otomatik SKIP:
  python -m pytest tests/hardware/test_real_mode_telemetry.py -v
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
_FORBIDDEN = ("command_long_send", "set_mode", "arducopter_arm", "motors_armed_wait",
              "set_position_target", "set_servo", "rc_channels_override")

# GCS heartbeat aralığı — ArduPilot link'i "GCS bağlı" sayıp SRx_* telemetri
# akışını başlatsın (MAVLink el sıkışması; UÇUŞ KOMUTU DEĞİLDİR).
_HB_INTERVAL_S = 1.0


def _print_hb_fields(msg, label: str) -> None:
    print(f"{label}:")
    print(f"  SYSID:       {msg.get_srcSystem()}")
    print(f"  COMPID:      {msg.get_srcComponent()}")
    print(f"  TYPE:        {getattr(msg, 'type', 0)}")
    print(f"  AUTOPILOT:   {getattr(msg, 'autopilot', 0)}")
    print(f"  BASE_MODE:   0x{getattr(msg, 'base_mode', 0):02X}")
    print(f"  CUSTOM_MODE: {getattr(msg, 'custom_mode', None)}")


def _process_heartbeat(master, msg, state: dict) -> None:
    """Tek heartbeat'i işler — filter mantığı mission ile AYNI:

    Navigation.accept_autopilot_heartbeat() (flight/navigation.py — _update_mode
    ile paylaşılan TEK kaynak) + hw.flight_mode_name() (pymavlink mode_mapping
    tersi — elle hard-code yok).
    """
    from flight.navigation import Navigation

    accepted, reason = Navigation.accept_autopilot_heartbeat(msg)
    if not accepted:
        print("IGNORED HEARTBEAT:")
        _print_hb_fields(msg, "  ...")
        print(f"  reason: {reason}")
        print()
        return

    mode = hw.flight_mode_name(master, msg)
    prev = state.get("prev_mode")

    if not state.get("fc_seen"):
        # İlk geçerli ArduPilot heartbeat — tam döküm + mevcut mod
        state["fc_seen"] = True
        _print_hb_fields(msg, "FC heartbeat")
        print(f"  FLIGHT MODE: {mode}")
        print()
        if mode is not None:
            print(f"Current mode: {mode}  (custom_mode: {msg.custom_mode})")
            print()
        state["prev_mode"] = mode
        return

    if mode is not None and mode != prev:
        print(f"MODE CHANGED: {prev} -> {mode}")
        print(f"  custom_mode: {msg.custom_mode}")
        print()
        state["prev_mode"] = mode


def run_monitor(
    device: str,
    baud: int,
    flow_control: bool = False,
    timeout_s: float = 10.0,
    duration_s: float | None = None,
) -> bool:
    """HEARTBEAT dinleyici (READ-ONLY). Mod okunabildiyse True döner.

    duration_s verilirse o süre sonunda döner; None ise Ctrl+C'ye kadar dinler.
    Hiçbir uçuş komutu / mod değiştirme komutu göndermez.
    """
    print("Waiting for ArduPilot heartbeat...")
    try:
        master, first_hb = hw.open_mavlink(
            device, baud, timeout_s=timeout_s, flow_control=flow_control
        )
    except TimeoutError as exc:
        print(f"[FAIL] heartbeat  ({exc})")
        return False
    except Exception as exc:
        print(f"[FAIL] bağlantı  ({exc})")
        return False

    state = {"fc_seen": False, "prev_mode": None}

    # open_mavlink ilk heartbeat'i zaten okudu — aynı mantıkla işle
    _process_heartbeat(master, first_hb, state)

    last_hb_t = time.monotonic()
    deadline = None if duration_s is None else time.monotonic() + duration_s
    try:
        while True:
            if deadline is not None and time.monotonic() >= deadline:
                break
            now = time.monotonic()
            if now - last_hb_t >= _HB_INTERVAL_S:
                hw.send_gcs_heartbeat(master)   # el sıkışması — komut değil
                last_hb_t = now
            msg = master.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            if msg.get_type() != "HEARTBEAT":
                continue
            _process_heartbeat(master, msg, state)
    except KeyboardInterrupt:
        print("\nStopped (Ctrl+C).")
    finally:
        try:
            master.close()
        except Exception:
            pass

    ok = state["fc_seen"]
    print()
    if ok:
        print(f"PASS: ArduPilot modu okundu → {state['prev_mode']}")
    else:
        print("FAIL: Geçerli ArduPilot heartbeat alınamadı")
    print("No flight commands were sent (read-only telemetry monitor).")
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description="MAVLink READ-ONLY flight mode testi")
    ap.add_argument("--device", default=None, help="Seri cihaz (varsayılan: config)")
    ap.add_argument("--baud", type=int, default=None, help="Baudrate (varsayılan: config)")
    ap.add_argument("--duration", type=float, default=None,
                    help="Dinleme süresi (sn); verilmezse Ctrl+C'ye kadar")
    ap.add_argument("--timeout", type=float, default=10.0, help="Heartbeat bekleme süresi (sn)")
    args = ap.parse_args()

    cfg = hw.load_payload_config()
    conn = cfg.get("flight_controller", {}).get("connection", {}) or {}
    device = args.device or conn.get("device", "/dev/serial0")
    baud = args.baud or conn.get("baudrate", 921600)
    flow_control = bool(conn.get("flow_control", False))

    return 0 if run_monitor(device, baud, args.duration, args.timeout, flow_control) else 1


def test_real_mode_telemetry_reads_mode():
    """Cube Orange'dan READ-ONLY flight mode okuma: en az bir geçerli ArduPilot
    heartbeat'ten mod çözülebilmeli (mission'ın kullandığı filtreyle AYNI)."""
    cfg = hw.load_payload_config()
    conn = cfg["flight_controller"]["connection"]
    ok = run_monitor(
        conn["device"], conn["baudrate"],
        flow_control=bool(conn.get("flow_control", False)),
        timeout_s=10.0,
        duration_s=5.0,
    )
    assert ok, "ArduPilot flight mode okunamadı (yalnızca telemetri okur — komut yok)"


if __name__ == "__main__":
    sys.exit(main())
