"""
Komut Dispatcher — Gelen JSON komutları LocalRPiBackend metodlarına yönlendirir.

Desteklenen komutlar:
  INITIALIZE
  SEND_PICKUP_COMMAND
  FINISH_PICKUP
  RELEASE_PAYLOAD_1
  RELEASE_PAYLOAD_2
  SET_MAGNET            params: { "magnet_id": "MAGNET_1"|"MAGNET_2", "on": true|false }
  EMERGENCY_STOP
  CLEAR_ATTACHED
  STATUS
"""
from __future__ import annotations

import json
import logging


class CommandDispatcher:
    """
    JSON komut → LocalRPiBackend metod eşleştirici.

    server.py her bağlantıda bu sınıfı kullanır.
    """

    def __init__(self, backend, logger: logging.Logger | None = None) -> None:
        self._backend = backend
        self._log     = logger or logging.getLogger(__name__)

    def dispatch(self, raw: str) -> str:
        """
        Ham JSON string'i parse et, komutu çalıştır, JSON yanıt döndür.
        Her durumda JSON döndürür — exception'ları yut, loglara yaz.
        """
        try:
            msg = json.loads(raw.strip())
        except json.JSONDecodeError:
            return self._err("JSON parse hatası")

        command = msg.get("command", "")
        params  = msg.get("params", {})
        self._log.info("Komut alındı: %s  params=%s", command, params)

        try:
            result = self._execute(command, params)
        except Exception as exc:
            self._log.exception("Komut işleme hatası: %s", exc)
            result = {"success": False, "reason": "HARDWARE_ERROR", "message": str(exc)}

        return json.dumps(result) + "\n"

    def _execute(self, command: str, params: dict) -> dict:
        b = self._backend

        if command == "INITIALIZE":
            return b.initialize()

        if command == "SEND_PICKUP_COMMAND":
            return b.send_pickup_command()

        if command == "FINISH_PICKUP":
            return b.finish_pickup()

        if command == "RELEASE_PAYLOAD_1":
            return b.release_payload_1()

        if command == "RELEASE_PAYLOAD_2":
            return b.release_payload_2()

        if command == "SET_MAGNET":
            magnet_id = params.get("magnet_id", "")
            on        = bool(params.get("on", True))
            return b.set_magnet(magnet_id, on)

        if command == "EMERGENCY_STOP":
            return b.emergency_stop()

        if command == "CLEAR_ATTACHED":
            b.clear_attached()
            return {"success": True, "message": "cleared"}

        if command == "STATUS":
            s = b.status()
            return {"success": True, "status": s}

        return self._err(f"Bilinmeyen komut: {command!r}")

    @staticmethod
    def _err(msg: str) -> dict:
        return {"success": False, "reason": "UNKNOWN_COMMAND", "message": msg}
