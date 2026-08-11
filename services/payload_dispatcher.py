"""
Komut Dispatcher — Gelen JSON komutları LocalRPiBackend metodlarına yönlendirir.

Bu sınıf, üretimdeki interface-tabanlı backend'i kullanır:
  payload/backends/local_rpi.py  (PayloadBackend / PayloadOperationResult)

Küçük bir ADAPTÖR görevi görür: PayloadOperationResult → JSON dict.
TCP protokolü DEĞİŞMEZ — payload/backends/remote_rpi.py (Ubuntu HIL client)
ile birebir uyumlu kalır.

Desteklenen komutlar:
  INITIALIZE
  SEND_PICKUP_COMMAND
  FINISH_PICKUP
  RELEASE_PAYLOAD_1
  RELEASE_PAYLOAD_2
  RELEASE_ALL_PAYLOADS   ← Görev 2: geri getirilen payload için 2 mıknatıs aynı anda OFF
  SET_MAGNET            params: { "magnet_id": "MAGNET_1"|"MAGNET_2", "on": true|false }
  EMERGENCY_STOP
  CLEAR_ATTACHED
  STATUS
"""
from __future__ import annotations

import json
import logging

from payload.models import MagnetId, PayloadOperationResult


class CommandDispatcher:
    """
    JSON komut → LocalRPiBackend (interface) metod eşleştirici.

    payload_server.py her bağlantıda bu sınıfı kullanır.
    """

    def __init__(self, backend, logger: logging.Logger | None = None) -> None:
        self._backend = backend
        self._log     = logger or logging.getLogger(__name__)

    # ── Adaptör: PayloadOperationResult → JSON dict ───────────────────────── #
    @staticmethod
    def _to_dict(res: PayloadOperationResult) -> dict:
        """Interface sonucunu HIL protokol dict'ine çevirir (remote_rpi.py uyumlu)."""
        if res.success:
            return {"success": True, "message": res.message}
        return {"success": False, "reason": res.result.value, "message": res.message}

    # ------------------------------------------------------------------ #
    def dispatch(self, raw: str) -> str:
        """
        Ham JSON string'i parse et, komutu çalıştır, JSON yanıt döndür.
        Her durumda JSON döndürür — exception'ları yut, loglara yaz.
        """
        try:
            msg = json.loads(raw.strip())
        except json.JSONDecodeError:
            return json.dumps(self._err("JSON parse hatası")) + "\n"

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
            return self._to_dict(b.initialize())

        if command == "SEND_PICKUP_COMMAND":
            return self._to_dict(b.send_pickup_command())

        if command == "FINISH_PICKUP":
            return self._to_dict(b.finish_pickup())

        if command == "RELEASE_PAYLOAD_1":
            return self._to_dict(b.release_payload_1())

        if command == "RELEASE_PAYLOAD_2":
            return self._to_dict(b.release_payload_2())

        if command == "RELEASE_ALL_PAYLOADS":
            return self._to_dict(b.release_all_payloads())

        if command == "SET_MAGNET":
            magnet_id = params.get("magnet_id", "")
            on        = bool(params.get("on", True))
            try:
                mid = MagnetId(magnet_id)
            except ValueError:
                return self._err(f"Bilinmeyen magnet_id: {magnet_id!r}")
            return self._to_dict(b.set_magnet(mid, on))

        if command == "EMERGENCY_STOP":
            return self._to_dict(b.emergency_stop())

        if command == "CLEAR_ATTACHED":
            b.clear_attached()
            return {"success": True, "message": "cleared"}

        if command == "STATUS":
            s = b.status()
            # stepper_duration'ı da ekle — Ubuntu tarafı bekleme süresini buradan alır
            return {
                "success": True,
                "status": {
                    "mechanism_state": s.mechanism_state.value,
                    "magnets": {
                        "magnet_1_on": s.magnets.magnet_1_on,
                        "magnet_2_on": s.magnets.magnet_2_on,
                    },
                    "carried_count": s.carried_count,
                    "backend_type":  s.backend_type,
                    "stepper_duration": getattr(b, "stepper_duration", None),
                },
            }

        return self._err(f"Bilinmeyen komut: {command!r}")

    @staticmethod
    def _err(msg: str) -> dict:
        return {"success": False, "reason": "UNKNOWN_COMMAND", "message": msg}
