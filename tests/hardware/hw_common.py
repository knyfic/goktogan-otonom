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


def open_mavlink(device: str, baud: int, timeout_s: float = 10.0):
    """
    MAVLink bağlantısı açar ve HEARTBEAT bekler (READ-ONLY).

    HİÇBİR KOMUT GÖNDERMEZ. Heartbeat gelmezse TimeoutError fırlatır.
    """
    from pymavlink import mavutil

    master = mavutil.mavlink_connection(
        device, baud=baud, force_mavlink2=True
    )
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
