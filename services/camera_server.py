"""
Raspberry Pi 5 Kamera TCP Akış Sunucusu.

Bu dosya, eski raspberry-pi-5/camera_service/server.py'nin taşınmış halidir.
Kamera implementasyonu DEĞİŞMEDİ: vision/rpi_camera.py (RPiCam3 — TEK gerçek
kamera kaynağı, REAL ile birebir aynı kod) kullanılır.

Ubuntu üzerindeki HIL test kodu buraya bağlanır (vision/remote_rpi_camera.py)
ve Raspberry Pi Camera Module 3'ün gerçek JPEG frame'lerini Wi-Fi üzerinden
çeker. Görüntü işleme kodunda (state machine, target_detector, payload_detector)
HİÇBİR DEĞİŞİKLİK YOKTUR — çerçeve formatı (BGR, 1920x1080) GazeboCamera ile
tamamen aynıdır.

Protokol (tek yönlü — server → client):
  Her frame için:
    [4 byte]  frame_byte_count   (little-endian uint32)
    [N byte]  JPEG sıkıştırılmış BGR görüntü   (kalite: config.camera.jpeg_quality)

  Bağlantı client tarafından kapatılana kadar frame akışı devam eder.
"""
from __future__ import annotations

import logging
import socket
import struct
import threading
import time

import cv2
import numpy as np

from vision.rpi_camera import RPiCam3

# 4B little-endian uint32 paketi
_LEN_STRUCT = struct.Struct("<I")


class CameraTCPServer:
    """
    TCP streaming sunucu. Her bağlantıda:
      1. Sonsuz döngü: camera.latest() → JPEG → (length + bytes) gönder.
      2. İstemci bağlantıyı kopar → dinlemeye dön.
    """

    def __init__(
        self,
        host:       str,
        port:       int,
        camera:     RPiCam3,
        jpeg_quality: int = 80,
        logger: logging.Logger | None = None,
    ) -> None:
        self._host       = host
        self._port       = port
        self._camera     = camera
        self._jpeg_q     = [int(cv2.IMWRITE_JPEG_QUALITY), int(max(1, min(100, jpeg_quality)))]
        self._log        = logger or logging.getLogger(__name__)
        self._server_sock: socket.socket | None = None
        self._running   = False
        self._frame_counter = 0

    # ------------------------------------------------------------------ #
    def start(self) -> None:
        """Blockle dinlemeye başla. Ctrl+C veya .stop() ile çık."""
        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.settimeout(0.5)  # accept() döngüsü için hızlı tepki
        self._server_sock.bind((self._host, self._port))
        self._server_sock.listen(1)
        self._running = True
        self._log.info("Kamera TCP servisi dinleniyor: %s:%d", self._host, self._port)

        try:
            while self._running:
                try:
                    conn, addr = self._server_sock.accept()
                except socket.timeout:
                    continue
                except OSError:
                    # sunucu kapatıldı
                    break

                # TCP_NODELAY = küçük paketler (JPEG header vb.) gönderiminde Nagle kapatsın → gecikme az
                try:
                    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                except OSError:
                    pass
                # Keepalive: Wi-Fi idle'da bağlantı düşmesin
                try:
                    conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                except OSError:
                    pass

                self._log.info("Kamera istemcisi bağlandı: %s:%d", addr[0], addr[1])
                self._handle_stream(conn, addr)
                try:
                    conn.close()
                except OSError:
                    pass
                self._log.info("Kamera istemcisi ayrıldı: %s:%d", addr[0], addr[1])
        finally:
            try:
                if self._server_sock is not None:
                    self._server_sock.close()
            except OSError:
                pass
            self._server_sock = None
            self._log.info("Kamera TCP servisi kapatıldı.")

    def stop(self) -> None:
        self._running = False
        if self._server_sock is not None:
            try:
                self._server_sock.close()
            except OSError:
                pass

    # ------------------------------------------------------------------ #
    def _handle_stream(self, conn: socket.socket, addr: tuple) -> None:
        """Tek bağlantı için frame akışı."""
        conn.settimeout(5.0)   # 5sn boyunca veri gönderilemezse bağlantı ölü sayılır

        # FPS loglaması (her ~30 frame'de bir)
        last_log_t = time.monotonic()
        sent_in_window = 0

        try:
            while self._running:
                frame = self._camera.latest()
                if frame is None or frame.size == 0:
                    # Henüz frame yok — CPU yakmayacak kadar uyu
                    time.sleep(0.005)
                    continue

                # JPEG sıkıştır
                ok, jpeg = cv2.imencode(".jpg", frame, self._jpeg_q)
                if not ok or jpeg is None or jpeg.size == 0:
                    continue
                jpeg_bytes = jpeg.tobytes()

                # Uzunluk + data gönder
                try:
                    conn.sendall(_LEN_STRUCT.pack(len(jpeg_bytes)) + jpeg_bytes)
                except OSError:
                    break

                self._frame_counter += 1
                sent_in_window += 1
                now = time.monotonic()
                if now - last_log_t >= 5.0:
                    dt = now - last_log_t
                    fps = sent_in_window / max(dt, 1e-6)
                    avg_kb = (sent_in_window and (
                        (len(jpeg_bytes) / 1024.0)  # son frame yaklaşık — çok yanlış değil
                    )) or 0.0
                    self._log.info(
                        "Kamera akışı → %s:%d  %5.1f FPS  ~%5.1f KB/frame  "
                        "(toplam %d frame)",
                        addr[0], addr[1], fps, avg_kb, self._frame_counter,
                    )
                    last_log_t = now
                    sent_in_window = 0
        except Exception as exc:
            self._log.warning(
                "Kamera akışı kesildi %s:%d — %s", addr[0], addr[1], exc,
            )
