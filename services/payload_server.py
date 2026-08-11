"""
TCP Payload Servisi — Ubuntu'dan gelen komutları dinler.

Bu dosya, eski raspberry-pi-5/payload_service/server.py'nin taşınmış halidir.
Donanım backend'i DEĞİŞMEDİ: services/payload_dispatcher.py üzerinden
payload/backends/local_rpi.py (interface tabanlı TEK gerçek backend) kullanılır.

Çalışma şekli:
  - Raspberry Pi boot'ta services/hil_entrypoint.py tarafından başlatılır.
  - Her bağlantı için ayrı thread açılır (maksimum 1 eşzamanlı istemci).
  - Her istek için bağlantı kurulur ve kapatılır (stateless, simple).
  - Her satır bir JSON komuttur (newline terminated).
"""
from __future__ import annotations

import logging
import socket
import threading


class PayloadTCPServer:
    """
    TCP sunucusu: gelen her bağlantıda dispatcher'ı çağırır.

    Args:
        host:       Dinlenecek arayüz (genellikle "0.0.0.0")
        port:       TCP port (varsayılan 5700)
        dispatcher: CommandDispatcher instance
        logger:     Opsiyonel logger
    """

    def __init__(
        self,
        host:       str,
        port:       int,
        dispatcher,
        logger: logging.Logger | None = None,
    ) -> None:
        self._host       = host
        self._port       = port
        self._dispatcher = dispatcher
        self._log        = logger or logging.getLogger(__name__)
        self._server_sock: socket.socket | None = None
        self._running   = False

    def start(self) -> None:
        """Sunucuyu başlat (blocking — main thread'den çağrılır)."""
        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind((self._host, self._port))
        self._server_sock.listen(1)
        self._running = True
        self._log.info("Payload TCP servisi dinleniyor: %s:%d", self._host, self._port)

        try:
            while self._running:
                try:
                    conn, addr = self._server_sock.accept()
                except OSError:
                    break
                self._log.info("Bağlantı kabul edildi: %s:%d", addr[0], addr[1])
                t = threading.Thread(
                    target=self._handle,
                    args=(conn, addr),
                    daemon=True,
                )
                t.start()
        finally:
            self._server_sock.close()
            self._log.info("Payload TCP servisi kapatıldı.")

    def stop(self) -> None:
        self._running = False
        if self._server_sock:
            try:
                self._server_sock.close()
            except OSError:
                pass

    def _handle(self, conn: socket.socket, addr: tuple) -> None:
        """Tek bir istemci bağlantısını işle."""
        try:
            data = b""
            while True:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                data += chunk
                if b"\n" in data:
                    break

            if not data.strip():
                return

            response = self._dispatcher.dispatch(data.decode())
            conn.sendall(response.encode())
            self._log.info("Yanıt gönderildi → %s:%d", addr[0], addr[1])
        except Exception as exc:
            self._log.error("Bağlantı hatası %s:%d — %s", addr[0], addr[1], exc)
        finally:
            conn.close()
