#!/usr/bin/env python3
"""
Raspberry Pi 5 — REAL donanım kontrolü (tek komut).

  python tests/hardware/check_real_hardware.py [--no-mavlink] [--no-camera]
                                                [--no-payload] [--no-gstreamer]

Kontrol edilenler (READ-ONLY — HİÇBİR UÇUŞ KOMUTU GÖNDERMEZ):
  • /dev/serial0 mevcut + erişilebilir
  • UART açılabiliyor
  • Cube Orange HEARTBEAT geliyor
  • GPS telemetry geliyor
  • Altitude telemetry geliyor
  • Flight mode okunabiliyor
  • Camera Module 3 açılıyor (frame üretiyor)
  • GStreamer + x264enc mevcut
  • Payload GPIO pinleri kuruluyor (mıknatıslar HOLD, stepper hareket ETMEZ)

Örnek çıktı:
  === REAL HARDWARE CHECK ===
  [PASS] /dev/serial0 exists + accessible
  [PASS] UART opened
  [PASS] Cube heartbeat received
  [PASS] GPS telemetry received
  [PASS] Altitude telemetry received
  [PASS] Flight mode readable
  [PASS] Camera Module 3 opened
  [PASS] GStreamer available
  [PASS] Payload GPIO initialized
  No flight commands were sent.
"""
from __future__ import annotations

import argparse
import platform
import sys
import time
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from tests.hardware import hw_common as hw


# ── MAVLink / Cube Orange ──────────────────────────────────────────────────── #
def _check_serial(device: str) -> bool:
    ok, detail = hw.check_serial_device(device)
    return hw.report(ok, f"{device} exists + accessible", detail)


def _check_mavlink(device: str, baud: int, flow_control: bool = False) -> bool:
    try:
        master, hb = hw.open_mavlink(
            device, baud, timeout_s=10.0, flow_control=flow_control
        )
    except TimeoutError as exc:
        return hw.report(False, "UART opened", str(exc))
    except Exception as exc:
        return hw.report(False, "UART opened", str(exc))

    try:
        ok_uart = hw.report(True, "UART opened", f"{device} @ {baud}")
        # MAVLink2 anlaşması: pymavlink MAVLink2 frame alınca mavlink20=True yapar
        mav2 = bool(getattr(getattr(master, "mav", None), "mavlink20", False))
        ok_hb = hw.report(
            True, "Cube heartbeat received",
            f"sysid={hb.get_srcSystem()} compid={hb.get_srcComponent()} "
            f"mavlink2={'EVET' if mav2 else 'HAYIR'}",
        )

        # GPS bekleme döngüsü: her sn GCS heartbeat ile link aktif tutulur
        # (uçuş komutu değil — MAVLink el sıkışması). FC stream başlatır.
        pos = None
        gps_deadline = time.monotonic() + 5.0
        while time.monotonic() < gps_deadline and pos is None:
            hw.send_gcs_heartbeat(master)
            pos = hw.recv_telemetry(master, "GLOBAL_POSITION_INT", timeout_s=1.0)
        if pos is None:
            # GPS neden yok? — ham GPS verisini de dene (fix_type ayrımı)
            gps = hw.recv_telemetry(master, "GPS_RAW_INT", timeout_s=2.0)
            if gps is not None:
                fix = gps.fix_type
                sats = gps.satellites_visible
                detail = (
                    f"GPS modülü bağlı (fix_type={fix}, uydu={sats}) ama "
                    "GLOBAL_POSITION_INT yok — GPS FIX bekleniyor (içerideysen "
                    "uydu görünmez)."
                )
            else:
                detail = (
                    "GPS_RAW_INT de yok — GPS modülü bağlı değil veya kilit yok. "
                    "GPS kablosu/bağlantısını ve MP GPS ekranını kontrol et."
                )
            ok_gps = hw.report(False, "GPS telemetry received", detail)
        else:
            lat = pos.lat / 1e7
            lon = pos.lon / 1e7
            alt = pos.relative_alt / 1000.0
            ok_gps = hw.report(
                lat != 0.0 or lon != 0.0, "GPS telemetry received",
                f"lat={lat:.6f} lon={lon:.6f}",
            )
            ok_gps &= hw.report(True, "Altitude telemetry received", f"{alt:.1f} m (relative)")

        mode = hw.flight_mode_name(master, hb)
        ok_mode = hw.report(mode is not None, "Flight mode readable", mode or "mode çözülemedi")
        return ok_uart and ok_hb and ok_gps and ok_mode
    except Exception as exc:
        return hw.report(False, "MAVLink telemetry", str(exc))
    finally:
        try:
            master.close()
        except Exception:
            pass


# ── Kamera ─────────────────────────────────────────────────────────────────── #
def _check_camera() -> bool:
    try:
        from picamera2 import Picamera2  # type: ignore[import-not-found]
    except ImportError:
        return hw.report(
            False, "Camera Module 3 opened",
            "picamera2 yok — sudo apt install python3-picamera2",
        )

    from config import (
        CAMERA_HEIGHT_PX,
        CAMERA_WIDTH_PX,
        REAL_CAMERA_FPS,
        RPI_CAMERA_NUM,
    )
    from vision.rpi_camera import RPiCam3

    # Yön döndürmenin TEK kaynağı: payload/config.yaml → camera.orientation
    # (HIL sunucu + REAL ortak — fiziksel montaj). Bölüm yoksa NORMAL fallback.
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
            return hw.report(
                False, "Camera Module 3 opened",
                "stub modda — picamera2/libcamera algılanmadı",
            )
        frame = cam.latest()
        deadline = time.monotonic() + 5.0
        while frame is None and time.monotonic() < deadline:
            time.sleep(0.1)
            frame = cam.latest()
        if frame is None or frame.size == 0:
            return hw.report(False, "Camera Module 3 opened", "5s içinde frame yok")
        return hw.report(True, "Camera Module 3 opened", f"{frame.shape[1]}x{frame.shape[0]} BGR")
    except Exception as exc:
        return hw.report(False, "Camera Module 3 opened", str(exc))
    finally:
        cam.close()


# ── GStreamer ──────────────────────────────────────────────────────────────── #
def _check_gstreamer() -> bool:
    try:
        import gi  # noqa: F401
        gi.require_version("Gst", "1.0")
        from gi.repository import Gst  # type: ignore[import-not-found]
        Gst.init(None)
    except Exception as exc:
        return hw.report(
            False, "GStreamer available",
            f"gi/Gst yüklenemedi — {exc} (sudo apt install gstreamer1.0-* python3-gi)",
        )

    needed = ("appsrc", "videoconvert", "x264enc", "rtph264pay", "udpsink")
    missing = [n for n in needed if Gst.ElementFactory.find(n) is None]
    detail = "element eksik: " + ", ".join(missing) if missing else "x264enc + rtph264pay + udpsink OK"
    return hw.report(not missing, "GStreamer available", detail)


# ── Payload GPIO (pin kurulumu — hareket YOK) ──────────────────────────────── #
def _check_payload() -> bool:
    if not hw.lgpio_available():
        return hw.report(
            False, "Payload GPIO initialized",
            "lgpio yok — sudo apt install python3-lgpio (gerçek GPIO test edilemez)",
        )

    from payload.backends.local_rpi import LocalRPiBackend

    backend = LocalRPiBackend.from_config(
        str(hw.REPO_ROOT / "payload" / "config.yaml"),
    )
    try:
        res = backend.initialize()
        if not res.success:
            return hw.report(False, "Payload GPIO initialized", res.message)
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
        return hw.report(ok, "Payload GPIO initialized", detail)
    except Exception as exc:
        return hw.report(False, "Payload GPIO initialized", str(exc))
    finally:
        backend.cleanup()


# ── Ana akış ──────────────────────────────────────────────────────────────── #
def main() -> int:
    ap = argparse.ArgumentParser(description="REAL donanım kontrolü (read-only)")
    ap.add_argument("--no-mavlink", action="store_true", help="MAVLink/Cube Orange kontrolünü atla")
    ap.add_argument("--no-camera", action="store_true", help="Kamera kontrolünü atla")
    ap.add_argument("--no-payload", action="store_true", help="Payload GPIO kontrolünü atla")
    ap.add_argument("--no-gstreamer", action="store_true", help="GStreamer kontrolünü atla")
    args = ap.parse_args()

    print("=== REAL HARDWARE CHECK ===")
    print(f"Makine: {platform.machine()} | RPi donanımı: {'EVET' if hw.is_raspberry_pi() else 'HAYIR'}")
    if not hw.is_raspberry_pi():
        print("⚠  Bu makine Raspberry Pi değil — sonuçlar yalnızca ortam kontrolüdür.")
    print()

    results: list[bool] = []
    cfg = hw.load_payload_config()

    if args.no_mavlink:
        print("[SKIP] MAVLink kontrolleri (--no-mavlink)")
    else:
        fc = cfg.get("flight_controller", {}).get("connection", {}) or {}
        device = fc.get("device", "/dev/serial0")
        baud = fc.get("baudrate", 921600)
        flow_control = bool(fc.get("flow_control", False))
        results.append(_check_serial(device))
        results.append(_check_mavlink(device, baud, flow_control))

    if args.no_camera:
        print("[SKIP] Kamera kontrolü (--no-camera)")
    else:
        results.append(_check_camera())

    if args.no_gstreamer:
        print("[SKIP] GStreamer kontrolü (--no-gstreamer)")
    else:
        results.append(_check_gstreamer())

    if args.no_payload:
        print("[SKIP] Payload GPIO kontrolü (--no-payload)")
    else:
        results.append(_check_payload())

    print()
    all_ok = all(results)
    print("[PASS]" if all_ok else "[FAIL]", "GENEL DURUM")
    print()
    print("No flight commands were sent.")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
