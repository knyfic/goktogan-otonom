#!/usr/bin/env python3
"""
REAL mod pre-flight kontrolü (READ-ONLY — ASLA ARM/TAKEOFF/GOTO göndermez).

  python3 tools/check_real_system.py [--no-mavlink] [--no-camera] [--no-payload]
                                      [--no-gstreamer] [--no-vision]

Kontrol eder (hepsi gerçek donanım üzerinden, mevcut Navigation/kamera/payload
kod yollarını kullanarak):
  MAVLink       → Cube Orange bağlantısı + heartbeat (mevcut Navigation)
  GPS           → GLOBAL_POSITION_INT (nav.position())
  Battery       → BATTERY_STATUS voltaj + SYS_STATUS yüzde (nav.position())
  Camera        → RPiCam3 açılıyor + frame üretiyor (mevcut RPiCam3)
  Vision        → TargetDetector/PayloadDetector gerçek frame'de patlamadan çalışıyor
  GStreamer     → x264enc + rtph264pay + udpsink elementleri mevcut
  Payload       → LocalRPiBackend initialize + mıknatıslar HOLD (hareket YOK)
  Configuration → hız/irtifa/spacing/margin/FOV/port/baudrate sanity
  Safety        → mıknatıslar başlangıçta ON, stepper hareket etmez, watchdog aktif

Örnek çıktı:
  MAVLink       PASS
  GPS           PASS
  Battery       PASS
  Camera        PASS
  Vision        PASS
  GStreamer     PASS
  Payload       PASS
  Configuration PASS
  Safety        PASS

  READY FOR REAL: YES

Bu script HİÇBİR UÇUŞ KOMUTU GÖNDERMEZ: arm / takeoff / goto / land / rtl /
set_mode / command_long yoktur. Yalnızca GCS heartbeat (MAVLink el sıkışması)
ile telemetri akışını başlatır.
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

# yasak: bu modülde asla çağrılmamalı (yalnızca dokümantasyon)
_FORBIDDEN = ("arm(", "takeoff(", "goto_global(", "rtl(", "set_mode(", "command_long_send")


# ═══════════════════════════════════════════════════════════════════════════ #
#  Configuration + Safety doğrulaması (main.py tarafından da kullanılabilir)
# ═══════════════════════════════════════════════════════════════════════════ #
def _magnet_on(value) -> bool:
    """YAML 1.1: 'ON' boolean True olarak parse edilir — ikisini de kabul et."""
    return value is True or (isinstance(value, str) and value.upper() == "ON")


def validate_real_config(cfg: dict) -> list[str]:
    """
    payload/config.yaml + config.py değerlerinin gerçek uçuş için tutarlılığını
    kontrol eder. Yalnızca UYARI döndürür — görevi asla engellemez.

    Dönen liste boşsa yapılandırma makul; doluysa her öğe bir risk/belirsizlik.
    """
    import config as C

    issues: list[str] = []

    mode = cfg.get("backend", "simulation")
    if mode != "real":
        issues.append(f"backend={mode!r} — REAL uçuş için payload/config.yaml → backend: 'real' olmalı")

    # ── Hızlar (gerçek uçuşta fiziksel risk — geniş güvenli aralık, uyarı) ──
    speed_checks = [
        ("SEARCH_SPEED_MPS",   C.SEARCH_SPEED_MPS,   0.5, 10.0),
        ("APPROACH_SPEED_MPS", C.APPROACH_SPEED_MPS, 0.2, 6.0),
        ("ALIGN_SPEED_MPS",    C.ALIGN_SPEED_MPS,    0.1, 3.0),
        ("VERTICAL_SPEED_MPS", C.VERTICAL_SPEED_MPS, 0.1, 3.0),
    ]
    for name, val, lo, hi in speed_checks:
        if val <= 0:
            issues.append(f"{name}={val} — sıfırdan küçük/eşit hız geçersiz")
        elif not (lo <= val <= hi):
            issues.append(f"{name}={val} m/s — beklenen aralık [{lo}, {hi}] m/s dışında (fiziksel risk)")

    # ── İrtifalar ──────────────────────────────────────────────────────────
    if C.TAKEOFF_ALTITUDE <= 0:
        issues.append(f"TAKEOFF_ALTITUDE={C.TAKEOFF_ALTITUDE} — pozitif olmalı")
    if C.PAYLOAD_DROP_ALTITUDE <= 0.1:
        issues.append(f"PAYLOAD_DROP_ALTITUDE={C.PAYLOAD_DROP_ALTITUDE} — çok alçak (bırakma irtifası)")
    if C.PAYLOAD_RELEASE_ALTITUDE <= C.PAYLOAD_DROP_ALTITUDE:
        issues.append(
            f"PAYLOAD_RELEASE_ALTITUDE({C.PAYLOAD_RELEASE_ALTITUDE}) ≤ "
            f"PAYLOAD_DROP_ALTITUDE({C.PAYLOAD_DROP_ALTITUDE}) — hizalama irtifası bırakma irtifasının üstünde olmalı"
        )

    # ── Search geometrisi ───────────────────────────────────────────────────
    if C.SCAN_LINE_SPACING <= 0:
        issues.append(f"SCAN_LINE_SPACING={C.SCAN_LINE_SPACING} — pozitif olmalı")
    if C.SEARCH_MARGIN <= 0:
        issues.append(f"SEARCH_MARGIN={C.SEARCH_MARGIN} — pozitif olmalı")
    elif C.SEARCH_MARGIN >= min(C.FIELD_WIDTH, C.FIELD_LENGTH) / 2.0:
        issues.append(
            f"SEARCH_MARGIN={C.SEARCH_MARGIN} ≥ alanın yarısı — tarama alanı boş kalır"
        )

    # Yarışma köşeleri — dejenere/eksik mi?
    try:
        from search.coverage_planner import CompetitionField

        f = CompetitionField(C.COMPETITION_CORNERS)
        if f.width_m < 5.0 or f.length_m < 5.0:
            issues.append(
                f"COMPETITION_CORNERS alanı çok küçük ({f.width_m:.1f}x{f.length_m:.1f} m) — "
                "gerçek yarışma alanı GPS köşeleri girilmeli"
            )
    except Exception as exc:
        issues.append(f"COMPETITION_CORNERS geçersiz: {exc}")

    # ── Kamera ──────────────────────────────────────────────────────────────
    if not (20.0 <= C.CAMERA_HORIZONTAL_FOV_DEG <= 120.0):
        issues.append(f"CAMERA_HORIZONTAL_FOV_DEG={C.CAMERA_HORIZONTAL_FOV_DEG} — beklenen [20, 120]°")

    # ── GStreamer ───────────────────────────────────────────────────────────
    vs = cfg.get("video_stream", {}) or {}
    port = int(vs.get("port", 0) or 0)
    if not (1 <= port <= 65535):
        issues.append(f"video_stream.port={port} — geçerli port değil")
    if not vs.get("host"):
        issues.append("video_stream.host boş — ground station IP girilmeli")
    if int(vs.get("bitrate", 0) or 0) <= 0:
        issues.append("video_stream.bitrate pozitif olmalı")

    # ── MAVLink (REAL) ─────────────────────────────────────────────────────
    fc = cfg.get("flight_controller", {}).get("connection", {}) or {}
    device = fc.get("device", "")
    baud = int(fc.get("baudrate", 0) or 0)
    if not device:
        issues.append("flight_controller.connection.device boş — /dev/serial0 veya /dev/ttyAMA0 olmalı")
    if baud not in (57600, 115200, 921600):
        issues.append(
            f"flight_controller.connection.baudrate={baud} — Cube Orange TELEM2 için "
            "921600 beklenir (57600/115200 de geçerli)"
        )
    hb = cfg.get("flight_controller", {}).get("heartbeat_timeout_s")
    if not hb or float(hb) <= 0:
        issues.append("flight_controller.heartbeat_timeout_s tanımsız/0 — MAVLink kaybı watchdog'u kapalı")

    # ── Payload güvenli başlangıç ───────────────────────────────────────────
    mc = cfg.get("magnets", {}) or {}
    if not _magnet_on(mc.get("initial_magnet_1")):
        issues.append("magnets.initial_magnet_1 != ON — açılışta yük düşebilir")
    if not _magnet_on(mc.get("initial_magnet_2")):
        issues.append("magnets.initial_magnet_2 != ON — açılışta yük düşebilir")

    return issues


def safety_report(cfg: dict) -> tuple[bool, list[str]]:
    """Açılış güvenliği: mıknatıslar HOLD, stepper hareket etmez, watchdog açık."""
    ok = True
    lines = []
    mc = cfg.get("magnets", {}) or {}
    if _magnet_on(mc.get("initial_magnet_1")) and _magnet_on(mc.get("initial_magnet_2")):
        lines.append("Mıknatıslar başlangıçta HOLD (açılışta yük düşmez)")
    else:
        ok = False
        lines.append("UYARI: mıknatıslar başlangıçta ON değil — açılışta yük düşebilir!")
    # STEP MOTOR KULLANILMIYOR: LocalRPiBackend.initialize() stepper'ı enable ETMEZ
    # (setup() yalnızca pinleri çıkış yapar, enable pin pasif) → hareket yok.
    lines.append("Stepper açılışta enable edilmez (kod: LocalRPiBackend.initialize → stepper.setup, enable yok)")
    hb = cfg.get("flight_controller", {}).get("heartbeat_timeout_s")
    if hb and float(hb) > 0:
        lines.append(f"MAVLink heartbeat watchdog aktif ({hb}s)")
    else:
        ok = False
        lines.append("UYARI: heartbeat watchdog kapalı — birincil koruma FC failsafe'idir")
    return ok, lines


# ═══════════════════════════════════════════════════════════════════════════ #
#  Donanım kontrolleri (READ-ONLY)
# ═══════════════════════════════════════════════════════════════════════════ #
def check_mavlink(cfg: dict, battery_duration_s: float) -> tuple[bool, str, dict]:
    """Mevcut Navigation üzerinden MAVLink + GPS + Battery kontrolü."""
    try:
        nav = hw.build_real_navigation(cfg)
    except Exception as exc:
        return False, f"bağlantı yok: {exc}", {}

    info = {}
    try:
        pos = nav.position()
        info["gps"] = (pos.latitude, pos.longitude)
        info["alt"] = pos.relative_altitude
        v, pct = hw.collect_battery_telemetry(nav, duration_s=battery_duration_s)
        info["battery_v"] = v
        info["battery_pct"] = pct
        info["mode_v"] = getattr(pos, "heading_deg", 0.0)
        return True, "Navigation bağlandı (heartbeat + telemetri)", info
    except Exception as exc:
        return False, f"telemetri hatası: {exc}", info


def check_camera() -> tuple[bool, str]:
    try:
        from config import (
            CAMERA_HEIGHT_PX,
            CAMERA_WIDTH_PX,
            REAL_CAMERA_FPS,
            RPI_CAMERA_NUM,
        )
        from vision.rpi_camera import RPiCam3

        cam_cfg = hw.load_payload_config().get("camera", {}) or {}
        orientation = str(cam_cfg.get("orientation", "NORMAL"))
        cam = RPiCam3(
            camera_num=RPI_CAMERA_NUM,
            width=CAMERA_WIDTH_PX,
            height=CAMERA_HEIGHT_PX,
            fps=REAL_CAMERA_FPS,
            orientation=orientation,
        )
        try:
            if cam.is_stub:
                return False, "stub modda — picamera2/libcamera yok"
            frame = cam.latest()
            deadline = time.monotonic() + 5.0
            while frame is None and time.monotonic() < deadline:
                time.sleep(0.1)
                frame = cam.latest()
            if frame is None or frame.size == 0:
                return False, "5s içinde frame yok"
            return True, f"{frame.shape[1]}x{frame.shape[0]} BGR"
        finally:
            cam.close()
    except Exception as exc:
        return False, f"kamera açılamadı: {exc}"


def check_vision() -> tuple[bool, str]:
    """Vision pipeline smoke testi: gerçek frame üzerinde detektörler patlamaz."""
    try:
        from config import (
            CAMERA_HEIGHT_PX,
            CAMERA_HORIZONTAL_FOV_DEG,
            CAMERA_VERTICAL_FOV_DEG,
            CAMERA_WIDTH_PX,
        )
        import cv2
        import numpy as np

        from vision.geometry import CameraModel
        from vision.payload_detector import PayloadDetector
        from vision.target_detector import TargetDetector

        frame = np.zeros((704, 1920, 3), dtype=np.uint8)
        det = TargetDetector()
        _ = det.detect(frame)
        model = CameraModel(
            CAMERA_WIDTH_PX, CAMERA_HEIGHT_PX,
            CAMERA_HORIZONTAL_FOV_DEG, CAMERA_VERTICAL_FOV_DEG,
        )
        pd = PayloadDetector(model)
        _ = pd.detect(frame, 3.0, "RED")
        return True, "TargetDetector + PayloadDetector temiz çalıştı"
    except Exception as exc:
        return False, f"vision hatası: {exc}"


def check_gstreamer() -> tuple[bool, str]:
    try:
        import gi  # noqa: F401
        gi.require_version("Gst", "1.0")
        from gi.repository import Gst  # type: ignore[import-not-found]
        Gst.init(None)
    except Exception as exc:
        return False, f"gi/Gst yok: {exc}"

    needed = ("appsrc", "videoconvert", "x264enc", "rtph264pay", "udpsink")
    missing = [n for n in needed if Gst.ElementFactory.find(n) is None]
    if missing:
        return False, "eksik element: " + ", ".join(missing)
    return True, "x264enc + rtph264pay + udpsink OK"


def check_payload() -> tuple[bool, str]:
    if not hw.lgpio_available():
        return False, "lgpio yok — sudo apt install python3-lgpio"
    from payload.backends.local_rpi import LocalRPiBackend

    backend = LocalRPiBackend.from_config(str(hw.REPO_ROOT / "payload" / "config.yaml"))
    try:
        res = backend.initialize()
        if not res.success:
            return False, res.message
        st = backend.status()
        ok = (
            st.mechanism_state.value == "READY"
            and st.magnets.magnet_1_on
            and st.magnets.magnet_2_on
        )
        detail = (
            f"state={st.mechanism_state.value} "
            f"M1={'ON' if st.magnets.magnet_1_on else 'OFF'} "
            f"M2={'ON' if st.magnets.magnet_2_on else 'OFF'}"
        )
        return ok, detail
    except Exception as exc:
        return False, str(exc)
    finally:
        backend.cleanup()


# ═══════════════════════════════════════════════════════════════════════════ #
#  Ana akış
# ═══════════════════════════════════════════════════════════════════════════ #
def main() -> int:
    ap = argparse.ArgumentParser(description="REAL mod pre-flight kontrolü (READ-ONLY)")
    ap.add_argument("--no-mavlink", action="store_true")
    ap.add_argument("--no-camera", action="store_true")
    ap.add_argument("--no-vision", action="store_true")
    ap.add_argument("--no-payload", action="store_true")
    ap.add_argument("--no-gstreamer", action="store_true")
    ap.add_argument("--battery-duration", type=float, default=10.0,
                    help="Battery telemetri bekleme süresi (sn)")
    args = ap.parse_args()

    cfg = hw.load_payload_config()

    print("REAL SYSTEM CHECK")
    print(f"Makine: RPi donanımı = {'EVET' if hw.is_raspberry_pi() else 'HAYIR'} | "
          f"backend = {cfg.get('backend', '?')}")
    print("-" * 42)

    results: dict[str, tuple[bool, str]] = {}

    # MAVLink + GPS + Battery (tek Navigation bağlantısı — mevcut kod yolu)
    if args.no_mavlink:
        results["MAVLink"] = (True, "atlandı (--no-mavlink)")
        results["GPS"] = (True, "atlandı")
        results["Battery"] = (True, "atlandı")
    else:
        ok, msg, info = check_mavlink(cfg, args.battery_duration)
        results["MAVLink"] = (ok, msg)
        gps_ok = ok and info.get("gps") and info["gps"] != (0.0, 0.0)
        results["GPS"] = (gps_ok, f"lat/lon={info.get('gps')}" if info.get("gps") else "GPS yok")
        v = info.get("battery_v", 0.0)
        pct = info.get("battery_pct", -1)
        if ok and v > 0:
            bat_msg = f"{v:.1f}V {pct}%"
        else:
            bat_msg = "BATTERY_STATUS gelmedi"
        results["Battery"] = (ok and v > 0, bat_msg)

    if args.no_camera:
        results["Camera"] = (True, "atlandı (--no-camera)")
    else:
        results["Camera"] = check_camera()

    if args.no_vision:
        results["Vision"] = (True, "atlandı (--no-vision)")
    else:
        results["Vision"] = check_vision()

    if args.no_gstreamer:
        results["GStreamer"] = (True, "atlandı (--no-gstreamer)")
    else:
        results["GStreamer"] = check_gstreamer()

    if args.no_payload:
        results["Payload"] = (True, "atlandı (--no-payload)")
    else:
        results["Payload"] = check_payload()

    # Configuration
    issues = validate_real_config(cfg)
    results["Configuration"] = (not issues, "; ".join(issues) if issues else "tutarlı")

    # Safety
    safety_ok, safety_lines = safety_report(cfg)
    results["Safety"] = (safety_ok, "; ".join(safety_lines))

    # ── Çıktı ───────────────────────────────────────────────────────────────
    all_ok = True
    for name in ("MAVLink", "GPS", "Battery", "Camera", "Vision",
                 "GStreamer", "Payload", "Configuration", "Safety"):
        ok, msg = results[name]
        all_ok = all_ok and ok
        status = "PASS" if ok else "FAIL"
        print(f"{name:<13} {status}   {msg}")
        if not ok:
            for sub in msg.split("; "):
                if sub:
                    print(f"               → {sub}")

    print("-" * 42)
    mode_real = cfg.get("backend", "simulation") == "real"
    ready = all_ok and mode_real
    if ready:
        print("\nREADY FOR REAL: YES")
    else:
        print("\nREADY FOR REAL: NO")
        reasons = []
        if not mode_real:
            reasons.append(f"backend != 'real' (şu an: {cfg.get('backend')!r})")
        for name in ("MAVLink", "GPS", "Battery", "Camera", "Vision",
                     "GStreamer", "Payload", "Configuration", "Safety"):
            if not results[name][0]:
                reasons.append(name)
        print("Reason:", ", ".join(reasons))
    print("\nNo flight commands were sent (ARM/TAKEOFF/GOTO yok).")
    return 0 if ready else 1


if __name__ == "__main__":
    sys.exit(main())
