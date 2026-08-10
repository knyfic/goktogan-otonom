"""
RemoteRPiCamera — HIL modu için: Ubuntu → Wi-Fi/TCP → Raspberry Pi 5 Kamera Servisi.

Raspberry Pi tarafında gerçek Raspberry Pi Camera Module 3 (CSI port 0 / CAM0)
çalışır; JPEG sıkıştırılmış frame'leri TCP üzerinden gönderir.

Bu sınıf Raspberry Pi'deki CameraTCPServer'a bağlanır, akışı çözer ve:
  - GazeboCamera / RealCamera ile AYNI public API'yi sunar (latest / show / pump / close_preview)
  - Frame formatı BGR 1920x1080 → State machine / target_detector / payload_detector
    İÇİN HİÇBİR DEĞİŞİKLİK GEREKMEZ.

Kullanım (main.py tarafından, HIL + remote_rpi seçili iken):
    cam = RemoteRPiCamera(host="192.168.1.21", port=5701, preview=True)
    while True:
        frame = cam.latest()
        if frame is not None:
            ...  # görüntü işleme
        cam.show(annotated)
"""
from __future__ import annotations

import logging
import socket
import struct
import threading
import time
from typing import Optional

import cv2
import numpy as np

# 4 byte little-endian uint32 (RPi camera server protokolü ile aynı)
_LEN_STRUCT = struct.Struct("<I")
_LEN_SIZE   = _LEN_STRUCT.size  # 4 byte


class RemoteRPiCamera:
    """
    Raspberry Pi üzerinde çalışan CameraTCPServer'a bağlan, JPEG akışını BGR frame'e çevir.

    Arayüz:
      latest()        → en son BGR frame (kopya) veya None
      show(frame)     → canlı önizlemede göster (GazeboCamera ile aynı)
      pump()          → GUI event loop besle (sadece waitKey(1))
      close_preview() → thread + bağlantı + pencereyi temiz kapat

    Özellikler:
      • Sürekli yeniden bağlanma (Wi-Fi kesintileri dayanıklı)
      • Thread-safe frame paylaşımı (ana thread yalnız latest() / show() çağırır)
      • Küçük / parçalanmış okumalar için deframer loop — TCP fragment sorunları yok
    """

    def __init__(
        self,
        host:              str,
        port:              int,
        connect_timeout_s: float = 3.0,
        read_timeout_s:    float = 2.0,
        reconnect_wait_s:  float = 1.0,
        preview:           bool  = False,
        swap_rb_channels:  bool  = False,
        window_name:       str   = "İHA Kamera",
        logger:            logging.Logger | None = None,
    ) -> None:
        self._host       = host
        self._port       = int(port)
        self._conn_to    = float(connect_timeout_s)
        self._read_to    = float(read_timeout_s)
        self._recon_wait = float(reconnect_wait_s)
        self._preview    = bool(preview)
        self._swap_rb    = bool(swap_rb_channels)
        self._win        = window_name
        self._log        = logger or logging.getLogger(__name__)

        self._lock       = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        self._running    = False
        self._connected  = False

        # Önizleme penceresi — diğer kameralarla aynı davranış
        if self._preview:
            cv2.namedWindow(self._win, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self._win, 1280, 720)

        # Rx thread başlat
        self._running = True
        self._thread  = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()
        self._log.info(
            "RemoteRPiCamera başlatıldı (HIL) — hedef %s:%d, arka thread RX, "
            "swap_rb_channels=%s",
            self._host, self._port,
            "True (B↔R TAKASLI)" if self._swap_rb else "False (doğrudan)",
        )

    # ── Alıcı döngü ─────────────────────────────────────────────────────── #
    def _rx_loop(self) -> None:
        sock: Optional[socket.socket] = None
        try:
            while self._running:
                # ── 1) Bağlan / yeniden bağlan ──────────────────────────
                if sock is None:
                    sock = self._try_connect()
                    if sock is None:
                        time.sleep(self._recon_wait)
                        continue
                    self._connected = True
                    self._log.info("[RX] RPi kameraya bağlandı: %s:%d", self._host, self._port)

                # ── 2) Tam bir frame al ─────────────────────────────────
                # 4B boyut (little-endian uint32)
                header = self._recv_exact(sock, _LEN_SIZE)
                if header is None:
                    self._disconnect(sock, "header okunamadı")
                    sock = None
                    time.sleep(self._recon_wait)
                    continue
                (nbytes,) = _LEN_STRUCT.unpack(header)

                # Mantıksal sınır — saçma büyük sayı (bozuk protokol / hata)
                if nbytes <= 0 or nbytes > 40 * 1024 * 1024:  # 40MB'tan büyük frame kabul etme
                    self._log.warning("[RX] Geçersiz frame boyutu: %d byte — yeniden bağlan.", nbytes)
                    self._disconnect(sock, "gecersiz boyut")
                    sock = None
                    time.sleep(self._recon_wait)
                    continue

                jpeg_bytes = self._recv_exact(sock, nbytes)
                if jpeg_bytes is None:
                    self._disconnect(sock, "payload okunamadı")
                    sock = None
                    time.sleep(self._recon_wait)
                    continue

                # ── 3) JPEG → BGR decode et ─────────────────────────────
                try:
                    arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if frame is None or frame.size == 0:
                        continue
                except Exception as exc:
                    self._log.warning("[RX] JPEG decode hatası (frame atlandı): %s", exc)
                    continue

                # İkinci emniyet katmanı: Client tarafı swap_rb_channels
                if self._swap_rb:
                    frame = frame[:, :, ::-1].copy()

                # ── 4) Yayınla (thread-safe) ─────────────────────────────
                with self._lock:
                    self._frame = frame
        finally:
            self._connected = False
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass

    # ── Socket yardımcıları ──────────────────────────────────────────────── #
    def _try_connect(self) -> Optional[socket.socket]:
        try:
            s = socket.create_connection((self._host, self._port), timeout=self._conn_to)
        except OSError as exc:
            if self._connected:  # daha önce bağlıysa — bağlantı koptu logu
                self._log.warning("[RX] RPi kameraya BAĞLANAMADI: %s", exc)
            else:
                self._log.debug("[RX] RPi kameraya BAĞLANAMADI: %s", exc)
            return None
        try:
            s.settimeout(self._read_to)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            try:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            except OSError:
                pass
        except OSError:
            pass
        return s

    @staticmethod
    def _recv_exact(sock: socket.socket, n: int) -> Optional[bytes]:
        """Tam olarak `n` bayt okur, hata/EOF'da None döner."""
        buf = bytearray(n)
        view = memoryview(buf)
        got = 0
        while got < n:
            try:
                chunk = sock.recv_into(view[got:], n - got)
            except socket.timeout:
                return None
            except OSError:
                return None
            if chunk == 0:
                return None
            got += chunk
        return bytes(buf)

    def _disconnect(self, sock: socket.socket, reason: str) -> None:
        self._connected = False
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass
        self._log.info("[RX] RPi kamera bağlantısı kesildi (%s). Yeniden deneniyor.", reason)

    # ── Public API (GazeboCamera / RealCamera ile birebir aynı) ──────────── #
    def latest(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def show(self, frame: np.ndarray) -> None:
        if not self._preview:
            return
        cv2.imshow(self._win, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.destroyWindow(self._win)
            self._preview = False

    def pump(self) -> None:
        if self._preview:
            cv2.waitKey(1)

    def close_preview(self) -> None:
        self._running = False
        if self._preview:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
            self._preview = False
        self._log.info("RemoteRPiCamera kapatıldı.")

    @property
    def is_connected(self) -> bool:
        return self._connected
