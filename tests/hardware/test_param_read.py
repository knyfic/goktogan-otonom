#!/usr/bin/env python3
"""
READ-ONLY parametre okuma testi (Raspberry Pi 5 + Cube Orange).

  python tests/hardware/test_param_read.py [--timeout 10]

BU TEST HİÇBİR ŞEY YAZMAZ ve HİÇBİR UÇUŞ KOMUTU GÖNDERMEZ:
  ✗ arm / takeoff / goto / land / rtl / set_mode / command_long
  ✗ PARAM_SET (parametre yazmaz — yalnızca OKUR)
  ✓ PARAM_REQUEST_READ ile LAND_DISARMDELAY okur (mevcut Navigation'ın
    start_param_read/poll_param asenkron API'si — mission ile AYNI mekanizma)

Amaç (real TELEM2 hattı teşhisi):
  Mission'daki "LAND_DISARMDELAY okunamadı" uyarılarının nedenini izole eder:
    • Parametre cevabı geliyorsa  → link/FC sağlam; sorun başka yerde.
    • Cevap hiç gelmiyorsa       → FC bu linkte parametre isteklerine yanıt
      vermiyor (başka bir process TELEM2'yi paylaşıyorsa mesajları çalıyor
      olabilir — test SIRASINDA başka hiçbir MAVLink istemcisi çalışmamalı).
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


def run_check(
    device: str,
    baud: int,
    param_name: str = "LAND_DISARMDELAY",
    timeout_s: float = 4.0,
    flow_control: bool = False,
) -> bool:
    print(f"PARAM READ-ONLY: {device} @ {baud}  (yazma yok, uçuş komutu yok)")
    try:
        master, hb = hw.open_mavlink(
            device, baud, timeout_s=10.0, flow_control=flow_control
        )
    except TimeoutError as exc:
        print(f"[FAIL] heartbeat  ({exc})")
        return False
    except Exception as exc:
        print(f"[FAIL] bağlantı  ({exc})")
        return False

    mode = hw.flight_mode_name(master, hb)
    print(f"[PASS] heartbeat  sysid={hb.get_srcSystem()} compid={hb.get_srcComponent()} mode={mode}")

    # Mission ile AYNI mekanizma: Navigation.start_param_read + poll_param
    from flight.navigation import Navigation

    nav = object.__new__(Navigation)
    nav.master = master
    nav.system_id = master.target_system
    nav.component_id = master.target_component
    nav._position = None  # position() bu testte kullanılmaz
    nav._land_disarmdelay_original = None

    ok = nav.start_param_read(param_name, timeout_s=timeout_s)
    print(f"[{'PASS' if ok else 'FAIL'}] PARAM_REQUEST_READ gönderildi ({param_name})")
    if not ok:
        master.close()
        return False

    deadline = time.monotonic() + timeout_s + 2.0
    value = None
    while time.monotonic() < deadline:
        # Telemetri akışını boşalt — yanıt position() akışında işlenir.
        msg = master.recv_match(blocking=False)
        if msg is not None:
            nav._dispatch_telemetry(msg)
        got, value = nav.poll_param()
        if got:
            break
        if not nav.param_read_active():
            break
        time.sleep(0.02)

    if got:
        print(f"[PASS] {param_name} = {value}  (FC parametre isteklerine yanıt veriyor)")
        master.close()
        return True

    print(
        f"[FAIL] {param_name} yanıtı {timeout_s:.0f}s içinde gelmedi — FC bu linkte "
        "parametre isteklerine yanıt vermiyor.\n"
        "  Olası nedenler:\n"
        "    • Bu test SIRASINDA başka bir MAVLink istemcisi TELEM2'yi paylaşıyorsa\n"
        "      (Mission Planner, monitor script) mesajları çalıyor olabilir — testi\n"
        "      başka hiçbir şey bağlı değilken tekrar dene.\n"
        "    • SERIAL2_OPTIONS / flow control eşleşmesi (Pi rtscts vs FC SERIAL2_FLOW_CTL).\n"
        "    • Mission Planner'dan AYNI bağlantıda parametre okuyup okumadığını dene."
    )
    master.close()
    return False


def main() -> int:
    ap = argparse.ArgumentParser(description="PARAM read-only teşhis (yazma YOK)")
    ap.add_argument("--timeout", type=float, default=4.0, help="Yanıt bekleme süresi (sn)")
    args = ap.parse_args()

    cfg = hw.load_payload_config()
    conn = cfg["flight_controller"]["connection"]
    ok = run_check(
        conn["device"], conn["baudrate"],
        timeout_s=args.timeout,
        flow_control=bool(conn.get("flow_control", False)),
    )
    return 0 if ok else 1


def test_param_read_land_disarmdelay():
    """LAND_DISARMDELAY FC'den READ-ONLY okunabiliyor mu? (yazma YOK)."""
    cfg = hw.load_payload_config()
    conn = cfg["flight_controller"]["connection"]
    ok = run_check(
        conn["device"], conn["baudrate"],
        timeout_s=4.0,
        flow_control=bool(conn.get("flow_control", False)),
    )
    assert ok, "LAND_DISARMDELAY okunamadı — link/param teşhisi için loglara bakın"


if __name__ == "__main__":
    sys.exit(main())
