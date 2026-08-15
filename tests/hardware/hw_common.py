"""
Raspberry Pi 5 donanım testleri — ortak yardımcılar.

Bu modül YALNIZCA Raspberry Pi 5 üzerinde çalıştırılır.
Ubuntu'da testler `pytestmark = skipif(not is_raspberry_pi())` ile SKIP edilir.

Tüm yardımcılar READ-ONLY'dir: HİÇBİRİ uçuş komutu (arm/takeoff/goto/land/rtl)
veya MAVLink komutu GÖNDERMEZ.
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent

# Script olarak çalıştırılırken (python tests/hardware/xxx.py) repo kökü
# sys.path'te olsun — hil_entrypoint ile aynı desen.
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def is_raspberry_pi() -> bool:
    """Bu makine gerçek bir Raspberry Pi mi? (platform makinesi değil, donanım)."""
    try:
        model = Path("/proc/device-tree/model").read_text(errors="ignore").strip("\x00")
        if "Raspberry Pi" in model:
            return True
    except OSError:
        pass
    return False


def load_payload_config() -> dict:
    """payload/config.yaml — TEK config (Ubuntu + RPi ortak)."""
    import yaml

    return yaml.safe_load((REPO_ROOT / "payload" / "config.yaml").read_text())


def report(ok: bool, label: str, detail: str = "") -> bool:
    """[PASS]/[FAIL] satırı basar ve sonucu döndürür."""
    status = "[PASS]" if ok else "[FAIL]"
    line = f"{status} {label}"
    if detail:
        line += f"  ({detail})"
    print(line)
    return ok


def lgpio_available() -> bool:
    """lgpio yüklü mü? (payload donanım sürücülerinin gerçek GPIO sinyali)."""
    try:
        import lgpio  # noqa: F401
        return True
    except ImportError:
        return False


def check_serial_device(device: str) -> tuple[bool, str]:
    """Seri cihaz mevcut ve okuma/yazma izni var mı? (açmaz, yalnızca kontrol)."""
    p = Path(device)
    if not p.exists():
        return False, (
            f"{device} yok — raspi-config → Interface Options → Serial Port → "
            "serial hardware YES yapılmadı"
        )
    if not (os.access(p, os.R_OK) and os.access(p, os.W_OK)):
        return False, (
            f"{device} izin yok — kullanıcı dialout grubunda değil "
            "(sudo usermod -aG dialout $USER + reboot)"
        )
    return True, f"{device} mevcut ve erişilebilir"


def send_gcs_heartbeat(master) -> None:
    """
    GCS heartbeat gönderir — ArduPilot link'i "GCS bağlı" sayar ve SRx_*
    telemetri akışını başlatır. UÇUŞ KOMUTU DEĞİLDİR; yalnızca MAVLink
    el sıkışmasıdır (READ-ONLY testlerde güvenli).
    """
    from pymavlink import mavutil

    try:
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0,
        )
    except Exception:
        pass


def open_mavlink(
    device: str, baud: int, timeout_s: float = 10.0, flow_control: bool = False
):
    """
    MAVLink bağlantısı açar ve HEARTBEAT bekler (READ-ONLY).

    HİÇBİR UÇUŞ KOMUTU GÖNDERMEZ. Heartbeat gelmezse TimeoutError fırlatır.
    flow_control=True → TELEM2 RTS/CTS bağlıysa Pi tarafında rtscts açılır
    (921600'de FC TX kilitlenmesini önler).

    ÖNEMLİ: Bağlantı açılır açılmaz GCS heartbeat gönderilir — ArduPilot
    telemetri stream'lerini (SRx_*) yalnızca GCS algıladığında başlatır.
    Heartbeat gönderilmezse yalnızca 1 Hz HEARTBEAT gelir (telemetri yok).
    """
    from pymavlink import mavutil

    master = mavutil.mavlink_connection(
        device, baud=baud, force_mavlink2=True, write=True
    )
    if flow_control:
        try:
            master.port.rtscts = True
        except Exception:
            pass

    # GCS algılansın — FC stream başlatsın (2 kez, kısa aralıkla)
    send_gcs_heartbeat(master)
    time.sleep(0.4)
    send_gcs_heartbeat(master)
    try:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout_s)
        if hb is None:
            raise TimeoutError(
                f"{timeout_s:.0f}s içinde HEARTBEAT yok: {device} @ {baud} — "
                "Cube Orange bağlı mı? TELEM2 açık mı? baud doğru mu?"
            )
        return master, hb
    except Exception:
        try:
            master.close()
        except Exception:
            pass
        raise


def flight_mode_name(master, hb_msg) -> str | None:
    """HEARTBEAT custom_mode → ArduPilot mod adı (ör. 'GUIDED', 'RTL', 'LOITER')."""
    try:
        mapping = master.mode_mapping()
        rev = {v: k for k, v in mapping.items()}
        return rev.get(hb_msg.custom_mode)
    except Exception:
        return None


def recv_telemetry(master, types, timeout_s: float = 3.0):
    """Belirtilen türde bir MAVLink mesajı bekler (READ-ONLY). None = gelmedi."""
    return master.recv_match(type=types, blocking=True, timeout=timeout_s)


def build_real_navigation(cfg: dict):
    """payload/config.yaml → mevcut Navigation (REAL: Cube Orange TELEM2 serial).

    main.py'nin REAL moddaki birebir kurulumu: device/baudrate/flow_control
    yalnızca config'ten gelir (kod içinde hard-code yok). Bağlantıyı açar ve
    HEARTBEAT bekler. UÇUŞ KOMUTU GÖNDERMEZ.
    """
    from flight.navigation import Navigation

    conn = cfg.get("flight_controller", {}).get("connection", {}) or {}
    device = conn.get("device", "/dev/ttyAMA0")
    baud = conn.get("baudrate", 921600)
    flow_control = bool(conn.get("flow_control", False))
    return Navigation(device, baud=baud, flow_control=flow_control)


def collect_battery_telemetry(nav, duration_s: float) -> tuple[float, int]:
    """Mevcut Navigation üzerinden batarya telemetrisi toplar (READ-ONLY).

    nav.position() BATTERY_STATUS / SYS_STATUS mesajlarını canlı işler ve
    VehiclePosition.battery_voltage_v / battery_percent alanlarını doldurur.

    (voltaj, yüzde) döner — veri gelmediyse (0.0, -1). Voltaj geldikten sonra
    SYS_STATUS yüzdesine ekstra 3 sn şans tanınır (FC yüzde bildirmiyorsa
    gereksiz beklenmez).
    """
    deadline = time.monotonic() + duration_s
    best_v = 0.0
    best_pct = -1
    while True:
        pos = nav.position()
        if pos.battery_voltage_v > best_v:
            best_v = pos.battery_voltage_v
        if pos.battery_percent > best_pct:
            best_pct = pos.battery_percent
        if best_v > 0 and (best_pct != -1 or time.monotonic() >= deadline - 3.0):
            break
        if time.monotonic() >= deadline:
            break
        time.sleep(0.1)
    return best_v, best_pct
