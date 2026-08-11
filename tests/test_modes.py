"""
Mod/entrypoint testleri (Adım 3 — config birleştirme).

1. Backend fabrikası: simulation / hil / real üç mod da doğru backend'i kurar.
2. Birleşik config: payload/config.yaml artık server + camera_server bölümlerini
   içerir (eski raspberry-pi-5/config/payload.yaml birleştirildi).
3. services/hil_entrypoint import edilebilir (RPi HIL sunucusu).

Bu testler donanım gerektirmez — lgpio yoksa LocalRPiBackend stub modda çalışır.
"""
import logging
from pathlib import Path

import yaml

_REPO_ROOT = Path(__file__).resolve().parent.parent
_PAYLOAD_CONFIG = _REPO_ROOT / "payload" / "config.yaml"


def _load_config() -> dict:
    return yaml.safe_load(_PAYLOAD_CONFIG.read_text())


# ── 1) Backend fabrikası ──────────────────────────────────────────────────── #
def test_backend_factory_simulation():
    from main import _build_backend

    backend = _build_backend("simulation", {}, logging.getLogger("test_sim"))
    assert type(backend).__name__ == "SimulationBackend"
    assert backend.status().backend_type == "simulation"


def test_backend_factory_hil():
    from main import _build_backend

    cfg = _load_config()
    backend = _build_backend("hil", cfg, logging.getLogger("test_hil"))
    assert type(backend).__name__ == "RemoteRPiBackend"


def test_backend_factory_real():
    from main import _build_backend
    from payload.backends.local_rpi import LocalRPiBackend

    cfg = _load_config()
    backend = _build_backend("real", cfg, logging.getLogger("test_real"))
    assert isinstance(backend, LocalRPiBackend)
    # stub modda (lgpio yok) initialize + temiz kapanış güvenli olmalı
    res = backend.initialize()
    assert res.success
    backend.cleanup()


# ── 2) Birleşik config ────────────────────────────────────────────────────── #
def test_merged_config_has_hil_server_sections():
    """Tek config dosyası hem Ubuntu (backend/remote) hem RPi (server/camera) içermeli."""
    cfg = _load_config()

    assert "backend" in cfg
    assert "remote" in cfg and cfg["remote"]["port"] == 5700
    assert "stepper" in cfg and "magnets" in cfg and "gpio" in cfg

    # RPi tarafı bölümleri (eski raspberry-pi-5/config/payload.yaml'den birleşti)
    assert cfg["server"]["port"] == 5700
    assert cfg["camera_server"]["enable"] is True
    assert cfg["camera_server"]["port"] == 5701
    assert "swap_rb_channels" in cfg["camera_server"]
    assert "orientation" in cfg["camera_server"]

    # HIL kamera seçimi (Ubuntu tarafı) hâlâ duruyor
    assert cfg.get("hil_camera") in ("gazebo", "remote_rpi")


# ── 3) HIL entrypoint ─────────────────────────────────────────────────────── #
def test_hil_entrypoint_module_imports():
    """RPi HIL sunucusu modülü stub ortamda import edilebilmeli."""
    import services.hil_entrypoint
    assert services.hil_entrypoint._REPO_ROOT == _REPO_ROOT


def test_hil_entrypoint_has_main():
    """Entrypoint main() fonksiyonu ve config yolu tanımlı olmalı."""
    import services.hil_entrypoint as entry

    assert callable(entry.main)
    assert (entry._REPO_ROOT / "payload" / "config.yaml").exists()
