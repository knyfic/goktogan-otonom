"""
RPiCam3 kamera entegrasyon testleri (Adım 2).

1. Import testi        — vision.rpi_camera Ubuntu'da stub modda import olur;
                         main._build_camera("real") RPiCam3 döndürür.
2. Interface testi     — RPiCam3, state machine arayüzünü sunar:
                         latest / show / pump / close_preview / close.
3. HIL loopback testi  — CameraTCPServer (stub RPiCam3) ↔ RemoteRPiCamera
                         (üretim HIL client) protokolü bozulmadı:
                         4-byte uzunluk + JPEG → BGR frame.

Bu testler picamera2 olmadan (stub) çalışır — RPi'de aynı kod gerçek
donanımla çalışır.
"""
import logging
import socket
import threading
import time
from unittest.mock import MagicMock

import pytest

from vision.rpi_camera import RPiCam3


@pytest.fixture(autouse=True)
def _no_preview_windows(monkeypatch):
    """Test ortamında pencere açmayı engelle (headless güvenli)."""
    monkeypatch.setattr("main.ENABLE_LIVE_PREVIEW", False)


# ── 1) Import / factory testi ─────────────────────────────────────────────── #
def test_rpi_camera_module_imports_on_ubuntu():
    """vision.rpi_camera, picamera2 olmadan stub modda import edilebilmeli."""
    cam = RPiCam3(camera_num=0, width=320, height=240, fps=10)
    assert cam.is_stub is True  # bu makinede picamera2 yok (veya olsa False olur)
    cam.close_preview()


def test_real_mode_camera_is_rpicam3():
    """main._build_camera('real') → TEK gerçek kamera: RPiCam3."""
    from main import _build_camera
    from vision.rpi_camera import RPiCam3

    cam = _build_camera("real", {}, logging.getLogger("test_real_cam"))
    assert isinstance(cam, RPiCam3), (
        f"real mod RPiCam3 kullanmalı, gelen: {type(cam).__name__}"
    )
    assert cam.is_stub is True
    cam.close_preview()


# ── 2) State machine arayüzü ──────────────────────────────────────────────── #
def test_rpicam3_state_machine_interface():
    """latest/show/pump/close_preview — GazeboCamera ile aynı arayüz."""
    cam = RPiCam3(camera_num=0, width=640, height=480, fps=10)

    for method in ("latest", "show", "pump", "close_preview", "close"):
        assert callable(getattr(cam, method, None)), f"{method} eksik"

    frame = cam.latest()          # stub → zero frame
    assert frame is not None
    assert frame.shape == (480, 640, 3)

    cam.show(frame)               # preview=False → no-op (hata vermemeli)
    cam.pump()                    # no-op
    cam.close_preview()           # idempotent kapanış
    cam.close_preview()           # ikinci çağrı da güvenli


def test_rpicam3_show_with_preview_calls_imshow(monkeypatch):
    """preview=True iken show() imshow + waitKey çağırmalı (q → kapanış)."""
    import cv2

    imshow = MagicMock()
    monkeypatch.setattr(cv2, "imshow", imshow)
    monkeypatch.setattr(cv2, "waitKey", lambda ms: ord("q"))

    cam = RPiCam3(camera_num=0, width=320, height=240, fps=10, preview=True)
    frame = cam.latest()
    assert frame is not None
    cam.show(frame)
    assert imshow.called
    assert cam._preview is False  # 'q' ile pencere kapandı


# ── 3) HIL camera server ↔ client loopback ───────────────────────────────── #
def _free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def test_hil_camera_server_client_loopback():
    """
    HIL protokolü: CameraTCPServer (stub RPiCam3) ↔ RemoteRPiCamera (üretim client).

    Üretim client'ı JPEG akışını çözüp BGR frame üretmeli — protokol bozulmadı.
    """
    from services.camera_server import CameraTCPServer
    from vision.remote_rpi_camera import RemoteRPiCamera

    log = logging.getLogger("test_hil_camera")

    cam = RPiCam3(camera_num=0, width=320, height=240, fps=10)  # stub
    port = _free_port()
    server = CameraTCPServer(
        host="127.0.0.1", port=port, camera=cam, jpeg_quality=80, logger=log,
    )
    thread = threading.Thread(target=server.start, daemon=True)
    thread.start()

    # Sunucunun dinlemeye başlamasını bekle
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                break
        except OSError:
            time.sleep(0.05)
    else:
        server.stop()
        pytest.fail("Kamera TCP sunucusu dinlemeye başlamadı.")

    # Üretim HIL client'ı — arka thread'de bağlanır ve frame alır
    client = RemoteRPiCamera(
        host="127.0.0.1", port=port,
        connect_timeout_s=2.0, read_timeout_s=2.0, reconnect_wait_s=0.2,
        preview=False, logger=log,
    )
    try:
        frame = None
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            frame = client.latest()
            if frame is not None:
                break
            time.sleep(0.05)
        assert frame is not None, "RPi kameradan frame alınamadı (timeout)"
        assert frame.shape == (240, 320, 3), (
            f"BGR frame bekleniyordu, gelen: {frame.shape}"
        )
        assert client.is_connected, "client bağlı olmalı"
    finally:
        client.close_preview()
        server.stop()
