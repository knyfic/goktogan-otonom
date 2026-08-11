"""
HIL loopback testi — Ubuntu HIL client ↔ RPi payload servisi protokolü.

Bu test, gerçek donanım olmadan (lgpio stub modunda) HIL protokolünün
bozulmadığını kanıtlar:

  RemoteRPiBackend (payload/backends/remote_rpi.py — üretim HIL client)
        │  TCP
        ▼
  PayloadTCPServer (services/payload_server.py)
        ▼
  CommandDispatcher (services/payload_dispatcher.py)
        ▼
  LocalRPiBackend (payload/backends/local_rpi.py — TEK gerçek backend,
                   REAL moduyla birebir aynı kod)

Adım 1 doğrulaması: raspberry-pi-5/payload_service + hardware kopyaları
silindikten sonra HIL akışının aynen çalıştığını gösterir.
"""
import logging
import socket
import threading
import time
from pathlib import Path

import pytest

from payload.backends.local_rpi import LocalRPiBackend
from payload.backends.remote_rpi import RemoteRPiBackend
from payload.models import MagnetId, OperationResult
from services.payload_dispatcher import CommandDispatcher
from services.payload_server import PayloadTCPServer

_REPO_ROOT = Path(__file__).resolve().parent.parent
_PAYLOAD_CONFIG = _REPO_ROOT / "payload" / "config.yaml"


def _free_port() -> int:
    """İşletim sisteminin atadığı boş bir port seç (küçük yarış riski kabul)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture
def hil_server():
    """
    Stub LocalRPiBackend + PayloadTCPServer'ı arka thread'de başlatır.

    steps küçük tutulur (40) — stepper stub'ının simülasyon uykusu testi
    yavaşlatmasın (gerçekte 11429 adım ≈ 13.8s).
    """
    log = logging.getLogger("test_hil_loopback")
    backend = LocalRPiBackend.from_config(
        config_path=str(_PAYLOAD_CONFIG), logger=log,
    )
    backend._steps = 40  # test hızı — donanım adım sayısı değil

    port = _free_port()
    server = PayloadTCPServer(
        host="127.0.0.1",
        port=port,
        dispatcher=CommandDispatcher(backend=backend, logger=log),
        logger=log,
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
        pytest.fail("PayloadTCP sunucusu başlatılamadı (port dinlenmedi).")

    yield backend, port

    server.stop()
    try:
        backend.cleanup()
    except Exception:
        pass


def test_hil_payload_protocol_loopback(hil_server):
    """HIL protokolü: initialize → status → release → set_magnet → pickup → emergency."""
    backend, port = hil_server

    # Üretim HIL client'ı (Ubuntu tarafı) — gerçek ağ yoluyla bağlanır
    client = RemoteRPiBackend(host="127.0.0.1", port=port, timeout=5.0,
                              logger=logging.getLogger("test_hil_client"))

    # 1) initialize — stepper_duration da STATUS ile alınmalı
    res = client.initialize()
    assert res.success, f"initialize başarısız: {res.message}"
    assert client.stepper_duration is not None, "stepper_duration STATUS'tan alınmalı"

    # 2) status — mıknatıslar başlangıçta ON; server backend'i 'local_rpi' olmalı
    status = client.status()
    assert status.magnets.magnet_1_on is True
    assert status.magnets.magnet_2_on is True
    assert backend.status().backend_type == "local_rpi"

    # 3) release_payload_1 — SADECE Magnet1 OFF
    res = client.release_payload_1()
    assert res.success, f"release_payload_1 başarısız: {res.message}"
    status = client.status()
    assert status.magnets.magnet_1_on is False
    assert status.magnets.magnet_2_on is True

    # 4) set_magnet — Magnet1 geri ON
    res = client.set_magnet(MagnetId.MAGNET_1, True)
    assert res.success
    assert backend.status().magnets.magnet_1_on is True

    # 5) send_pickup_command → finish_pickup (stepper thread + join)
    res = client.send_pickup_command()
    assert res.success, f"send_pickup_command başarısız: {res.message}"
    res = client.finish_pickup()
    assert res.success, f"finish_pickup başarısız: {res.message}"
    status = client.status()
    # carried: 2 (başlangıç) → release_payload_1 ile 1 → pickup ile 2
    assert status.carried_count == 2

    # 6) release_all_payloads — İKİ mıknatıs aynı anda OFF
    res = client.release_all_payloads()
    assert res.success
    status = client.status()
    assert status.magnets.magnet_1_on is False
    assert status.magnets.magnet_2_on is False

    # 7) emergency_stop → sonraki komutlar EMERGENCY_ACTIVE ile reddedilmeli
    res = client.emergency_stop()
    assert res.success
    res = client.release_payload_1()
    assert not res.success
    assert res.result == OperationResult.EMERGENCY_ACTIVE, (
        f"beklenen EMERGENCY_ACTIVE, gelen: {res.result}"
    )

    # 8) clear_attached — hata üretmemeli
    client.clear_attached()


def test_hil_unknown_command_returns_json_error(hil_server):
    """Bilinmeyen komut JSON hata yanıtı döndürmeli (client çökmemeli)."""
    _, port = hil_server
    raw = b'{"command": "BOGUS", "params": {}}\n'
    with socket.create_connection(("127.0.0.1", port), timeout=3.0) as sock:
        sock.sendall(raw)
        data = sock.recv(4096)
    assert b'"success": false' in data
    assert b'"reason": "UNKNOWN_COMMAND"' in data
