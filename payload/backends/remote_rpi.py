"""
RemoteRPiBackend — Ubuntu → Wi-Fi/TCP → Raspberry Pi 5 (HIL modu).

Ubuntu'daki payload.pickup() çağrısını Raspberry Pi'deki payload_service'e iletir.
Raspberry Pi lokal GPIO'sunu çalıştırır.
"""
from __future__ import annotations

import json
import logging
import socket
import time

from payload.interface import PayloadBackend
from payload.models import (
    MagnetId,
    MagnetState,
    OperationResult,
    PayloadMechanismState,
    PayloadOperationResult,
    PayloadSystemStatus,
)

# Komut adları — raspberry-pi-5/payload_service/dispatcher.py ile eşleşmeli
CMD_INITIALIZE       = "INITIALIZE"
CMD_SEND_PICKUP      = "SEND_PICKUP_COMMAND"
CMD_FINISH_PICKUP    = "FINISH_PICKUP"
CMD_RELEASE_1        = "RELEASE_PAYLOAD_1"
CMD_RELEASE_2        = "RELEASE_PAYLOAD_2"
CMD_RELEASE_ALL      = "RELEASE_ALL_PAYLOADS"
CMD_SET_MAGNET       = "SET_MAGNET"
CMD_EMERGENCY_STOP   = "EMERGENCY_STOP"
CMD_CLEAR_ATTACHED   = "CLEAR_ATTACHED"
CMD_STATUS           = "STATUS"


class RemoteRPiBackend(PayloadBackend):
    """
    HIL modunda Ubuntu'dan Raspberry Pi'ye TCP komut gönderir.
    Raspberry Pi'deki payload_service bu komutları alır ve GPIO'yu çalıştırır.

    Kullanım:
        backend = RemoteRPiBackend(host="192.168.1.42", port=5700, logger=logger)
        backend.initialize()
        backend.send_pickup_command()
    """

    def __init__(
        self,
        host: str,
        port: int,
        timeout: float = 180.0,
        logger: logging.Logger | None = None,
    ) -> None:
        self._host    = host
        self._port    = port
        self._timeout = timeout
        self._log     = logger or logging.getLogger(__name__)
        self._cached_status: PayloadSystemStatus | None = None
        # initialize() sonrası STATUS komutu ile RPi'den alınan stepper süresi
        self.stepper_duration: float | None = None

    # ------------------------------------------------------------------ #
    @staticmethod
    def _set_socket_keepalive(sock: socket.socket) -> None:
        """Uzun süreli bağlantılarda (release ~70sn) NAT/WiFi idle timeout'u önle."""
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            if hasattr(socket, "TCP_KEEPIDLE"):
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 5)
            if hasattr(socket, "TCP_KEEPINTVL"):
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 3)
            if hasattr(socket, "TCP_KEEPCNT"):
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 5)
        except OSError:
            pass

    def _send(self, command: str, params: dict | None = None) -> dict:
        """
        Raspberry Pi'ye JSON komut gönder, yanıtı parse et.
        Bağlantı her çağrıda yeniden kurulur (simple, stateless).
        """
        payload_bytes = json.dumps({
            "command": command,
            "params":  params or {},
        }).encode() + b"\n"

        try:
            with socket.create_connection((self._host, self._port),
                                          timeout=self._timeout) as sock:
                self._set_socket_keepalive(sock)
                sock.sendall(payload_bytes)
                data = b""
                while True:
                    chunk = sock.recv(4096)
                    if not chunk:
                        break
                    data += chunk
                    if b"\n" in data:
                        break
        except OSError as exc:
            raise ConnectionError(
                f"Raspberry Pi'ye bağlanılamadı ({self._host}:{self._port}): {exc}"
            ) from exc

        try:
            return json.loads(data.strip())
        except json.JSONDecodeError as exc:
            raise ValueError(f"Geçersiz JSON yanıtı: {data!r}") from exc

    def _to_result(self, resp: dict) -> PayloadOperationResult:
        """Raspberry Pi JSON yanıtını PayloadOperationResult'a dönüştür."""
        ok = resp.get("success", False)
        msg = resp.get("message", "")
        if ok:
            return PayloadOperationResult.ok(msg)
        reason_str = resp.get("reason", "FAILED")
        try:
            reason = OperationResult(reason_str)
        except ValueError:
            reason = OperationResult.FAILED
        return PayloadOperationResult.fail(reason, msg)

    # ------------------------------------------------------------------ #
    def initialize(self) -> PayloadOperationResult:
        self._log.info("[REMOTE] initialize → %s:%d", self._host, self._port)
        resp = self._send(CMD_INITIALIZE)
        result = self._to_result(resp)
        # RPi'nin stepper süresini al — state machine bekleme süresini buradan hesaplar
        if result.success:
            try:
                status_resp = self._send(CMD_STATUS)
                duration = status_resp.get("status", {}).get("stepper_duration")
                if duration is not None:
                    self.stepper_duration = float(duration)
                    self._log.info("[REMOTE] Stepper süresi RPi'den alındı: %.1fs",
                                   self.stepper_duration)
            except Exception:
                pass   # Eski RPi firmware — stepper_duration yok, config.py değeri kullanılır
        return result

    def send_pickup_command(self) -> PayloadOperationResult:
        self._log.info("[REMOTE] send_pickup_command")
        resp = self._send(CMD_SEND_PICKUP)
        return self._to_result(resp)

    def finish_pickup(self) -> PayloadOperationResult:
        self._log.info("[REMOTE] finish_pickup")
        resp = self._send(CMD_FINISH_PICKUP)
        return self._to_result(resp)

    def release_payload_1(self) -> PayloadOperationResult:
        self._log.info("[REMOTE] release_payload_1")
        resp = self._send(CMD_RELEASE_1)
        return self._to_result(resp)

    def release_payload_2(self) -> PayloadOperationResult:
        self._log.info("[REMOTE] release_payload_2")
        resp = self._send(CMD_RELEASE_2)
        return self._to_result(resp)

    def release_all_payloads(self) -> PayloadOperationResult:
        self._log.info("[REMOTE] release_all_payloads (İKİ MIKNATIS AYNI ANDA OFF)")
        resp = self._send(CMD_RELEASE_ALL)
        return self._to_result(resp)

    def set_magnet(self, magnet_id: MagnetId, on: bool) -> PayloadOperationResult:
        self._log.info("[REMOTE] set_magnet %s=%s", magnet_id.value, on)
        resp = self._send(CMD_SET_MAGNET, {
            "magnet_id": magnet_id.value,
            "on":        on,
        })
        return self._to_result(resp)

    def emergency_stop(self) -> PayloadOperationResult:
        self._log.warning("[REMOTE] EMERGENCY STOP gönderiliyor")
        resp = self._send(CMD_EMERGENCY_STOP)
        return self._to_result(resp)

    def clear_attached(self) -> None:
        self._log.debug("[REMOTE] clear_attached")
        try:
            self._send(CMD_CLEAR_ATTACHED)
        except Exception as exc:
            self._log.warning("[REMOTE] clear_attached başarısız: %s", exc)

    def status(self) -> PayloadSystemStatus:
        try:
            resp = self._send(CMD_STATUS)
            s = resp.get("status", {})
            magnets = s.get("magnets", {})
            return PayloadSystemStatus(
                mechanism_state=PayloadMechanismState(
                    s.get("mechanism_state", "UNINITIALIZED")),
                magnets=MagnetState(
                    magnet_1_on=magnets.get("magnet_1_on", True),
                    magnet_2_on=magnets.get("magnet_2_on", True),
                ),
                carried_count=s.get("carried_count", 2),
                backend_type="remote_rpi",
            )
        except Exception as exc:
            self._log.warning("[REMOTE] status sorgusu başarısız: %s", exc)
            return PayloadSystemStatus(
                mechanism_state=PayloadMechanismState.ERROR,
                backend_type="remote_rpi",
            )
