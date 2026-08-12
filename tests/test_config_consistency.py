"""
Configuration source-of-truth testleri (REAL deployment hazırlığı).

Doğrulanan kurallar:
  1. HIL sunucu/client portları senkron (server.port == remote.port,
     camera_server.port == remote_camera.port) — ikiz tanımlar bilinçlidir.
  2. Gerçek RPi kamera orientation TEK kaynak: camera.orientation == 'ROTATE_180'
     (HIL sunucu + REAL aynı değeri kullanır).
  3. REAL kamera config'ten orientation alır; bölüm yoksa config.py fallback.
  4. HIL remote_rpi kamera: yön döndürmenin sahibi RPi sunucusudur → client NORMAL,
     timeout'lar config'ten (remote_camera.*) gelir.
  5. config.py'de tanımlı state machine timeout sabitleri (STATE_TIMEOUT,
     TAKEOFF_TIMEOUT, RTL_ARRIVAL_TIMEOUT_S, RTL_DISARM_TIMEOUT_S) mevcuttur.
"""
import logging
from pathlib import Path

import yaml

_REPO_ROOT = Path(__file__).resolve().parent.parent
_PAYLOAD_CONFIG = _REPO_ROOT / "payload" / "config.yaml"


def _load_config() -> dict:
    return yaml.safe_load(_PAYLOAD_CONFIG.read_text())


# ── 1) HIL port senkronizasyonu ───────────────────────────────────────────── #
def test_hil_payload_ports_in_sync():
    """server.port (RPi sunucu) == remote.port (Ubuntu client) — aynı port."""
    cfg = _load_config()
    assert cfg["server"]["port"] == cfg["remote"]["port"] == 5700


def test_hil_camera_ports_in_sync():
    """camera_server.port == remote_camera.port — aynı port (5701)."""
    cfg = _load_config()
    assert cfg["camera_server"]["port"] == cfg["remote_camera"]["port"] == 5701


# ── 2) Kamera orientation TEK kaynak ──────────────────────────────────────── #
def test_real_camera_orientation_single_source():
    """Fiziksel kamera yönü config.yaml → camera.orientation (HIL + REAL ortak)."""
    cfg = _load_config()
    assert cfg["camera"]["orientation"] == "ROTATE_180"


def test_real_camera_build_uses_config_orientation():
    """main._build_camera('real') → RPiCam3 orientation'ı config'ten gelmeli."""
    from main import _build_camera

    cam = _build_camera("real", _load_config(), logging.getLogger("t"))
    assert cam._orientation == "ROTATE_180"
    cam.close_preview()


def test_real_camera_orientation_fallback_without_section():
    """camera bölümü yoksa geriye dönük uyumluluk: config.py CAMERA_ORIENTATION."""
    from config import CAMERA_ORIENTATION
    from main import _build_camera

    cam = _build_camera("real", {}, logging.getLogger("t"))
    assert cam._orientation == CAMERA_ORIENTATION  # NORMAL
    cam.close_preview()


def test_hil_remote_camera_client_not_rotating():
    """
    HIL remote_rpi: yön döndürmenin sahibi RPi kamera sunucusudur
    (camera.orientation=ROTATE_180). Ubuntu client TEKRAR döndürmez (NORMAL)
    ve TCP timeout'ları config'ten (remote_camera.*) gelir.
    """
    from main import _build_camera

    cfg = _load_config()
    cam = _build_camera(
        "hil",
        {"hil_camera": "remote_rpi", "remote": {"host": "127.0.0.1"},
         "remote_camera": {"port": 5999}},
        logging.getLogger("t"),
    )
    assert cam._orientation == "NORMAL"
    cam.close_preview()


def test_hil_remote_camera_timeouts_from_config():
    from vision.remote_rpi_camera import RemoteRPiCamera

    cfg = _load_config()
    rc = cfg["remote_camera"]
    cam = RemoteRPiCamera(
        host="127.0.0.1", port=5999,
        connect_timeout_s=float(rc.get("connect_timeout_s", 3.0)),
        read_timeout_s=float(rc.get("read_timeout_s", 2.0)),
        reconnect_wait_s=float(rc.get("reconnect_wait_s", 1.0)),
        preview=False, logger=logging.getLogger("t"),
    )
    assert cam._conn_to == float(rc["connect_timeout_s"])
    assert cam._read_to == float(rc["read_timeout_s"])
    assert cam._recon_wait == float(rc["reconnect_wait_s"])
    cam.close_preview()


# ── 3) State machine timeout sabitleri config'te ──────────────────────────── #
def test_state_machine_timeout_constants_exist():
    """Hard-code yerine config'ten okunan timeout sabitleri tanımlı olmalı."""
    import config

    assert config.TAKEOFF_TIMEOUT == 45.0
    assert config.STATE_TIMEOUT == 120.0
    assert config.RTL_ARRIVAL_TIMEOUT_S == 300.0
    assert config.RTL_DISARM_TIMEOUT_S == 300.0


def test_removed_dead_constants_gone():
    """Kullanılmayan ölü sabitler kaldırıldı — artık yok olmalı."""
    import config

    assert not hasattr(config, "RTL_TIMEOUT")
    assert not hasattr(config, "CAMERA_TIMEOUT")
    assert not hasattr(config, "CONTROL_RATE_HZ")
