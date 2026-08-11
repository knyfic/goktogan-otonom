"""
PayloadController — State machine'in kullandığı tek payload arayüzü.

State machine şunları bilmez:
  - GPIO pinleri
  - Stepper adım sayısı
  - TCP/IP bağlantısı
  - Mıknatıs polarity

State machine YALNIZCA şu metodları çağırır:
  release_payload(color)      → Görev 1: yük bırakma
  send_pickup_command(color)  → Görev 2: pickup başlat (non-blocking)
  finish_pickup(color)        → Görev 2: pickup tamamla (servo bekleme sonrası)
  clear_attached()            → Retry'da state sıfırla
  
Bu sınıf bu çağrıları backend'e doğru şekilde iletir ve mevcut
PayloadResult döndürür (state_machine.py ile uyumluluk için).

Backend seçimi: payload/config.yaml → main.py → bu sınıfa inject edilir.

Geçiş özeti:
  Simulation → SimulationBackend
  HIL        → RemoteRPiBackend
  Gerçek     → LocalRPiBackend (payload/backends/local_rpi.py — TEK gerçek backend)
"""
from __future__ import annotations

import logging
from dataclasses import dataclass

from models import TargetColor
from payload.interface import PayloadBackend
from payload.models import MagnetId, OperationResult, PayloadOperationResult


@dataclass
class PayloadResult:
    """
    State machine ile uyumluluk için korunan veri yapısı.
    Dahili mantık PayloadOperationResult kullanır; bu sadece dönüştürme katmanı.
    """
    success:      bool
    payload_type: TargetColor
    message:      str


def _to_legacy(
    res: PayloadOperationResult,
    color: TargetColor,
) -> PayloadResult:
    return PayloadResult(
        success=res.success,
        payload_type=color,
        message=res.message,
    )


class PayloadController:
    """
    State machine'in görev 1 ve 2 sırasında çağırdığı payload API katmanı.

    Hangi renk payload'ın hangi mıknatısta olduğunu payload/config.yaml'dan okur:
      payload_color_map:
        RED:  magnet_1
        BLUE: magnet_2

    release_payload(color) çağrılınca:
      RED  → release_payload_1()
      BLUE → release_payload_2()
    """

    def __init__(
        self,
        backend: PayloadBackend,
        logger: logging.Logger,
        color_map: dict | None = None,
    ) -> None:
        self._backend       = backend
        self.logger         = logger
        self.released_payloads: list[TargetColor] = []
        self.attached_payload: TargetColor | None = None
        self._release_count = 0

        # Renk → mıknatıs eşleşmesi.
        # config.yaml'dan inject edilir; yoksa varsayılan: RED=magnet_1, BLUE=magnet_2
        self._color_map: dict[str, str] = color_map or {
            "RED":  "magnet_1",
            "BLUE": "magnet_2",
        }
        self.logger.info(
            "Payload renk haritası: RED→%s  BLUE→%s",
            self._color_map.get("RED"), self._color_map.get("BLUE"),
        )

        self._stepper_duration: float | None = getattr(backend, "stepper_duration", None)

        res = self._backend.initialize()
        if not res.success:
            self.logger.error("Payload backend initialize başarısız: %s", res.message)
        else:
            self.logger.info("Payload backend hazır. Backend: %s",
                             self._backend.status().backend_type)

    # ================================================================== #
    # State machine tarafından çağrılan metodlar
    # ================================================================== #

    def release_payload(self, payload_type: TargetColor) -> PayloadResult:
        """
        Görev 1: Payload bırak.
        Hangi mıknatısın kapatılacağı payload_color_map'ten belirlenir:
          RED  → config'deki RED mıknatısı kapat
          BLUE → config'deki BLUE mıknatısı kapat
        Diğer mıknatıs DOKUNULMAZ.
        """
        color_key = payload_type.value.upper()   # "RED" veya "BLUE"
        magnet = self._color_map.get(color_key, "magnet_1")

        self.logger.info(
            "PAYLOAD RELEASE: %s → %s kapatılıyor",
            payload_type.value, magnet,
        )

        if magnet == "magnet_1":
            res = self._backend.release_payload_1()
        else:
            res = self._backend.release_payload_2()

        if res.success:
            self._release_count += 1
            self.released_payloads.append(payload_type)
            self.logger.info("Payload bırakıldı (#%d): %s → %s",
                             self._release_count, payload_type.value, magnet)
        else:
            self.logger.error("Payload bırakma başarısız: %s → %s — %s",
                              payload_type.value, magnet, res.message)

        return _to_legacy(res, payload_type)

    def release_returned_payload(self, payload_type: TargetColor) -> PayloadResult:
        """
        Görev 2, son adım: Yerden alınan ve geri taşınan payload'ı bırak.
        İKİ MIKNATIS AYNI ANDA OFF → her ikisi de tutuyordur, biri kaybederse diğeri korur.
        Stepper motor DOKUNULMAZ.
        """
        self.logger.info("PAYLOAD RELEASE (DÖNÜŞ) → İKİ MIKNATIS AYNI ANDA OFF. (%s)",
                         payload_type.value)
        res = self._backend.release_all_payloads()
        if res.success:
            self._release_count += 1
            self.released_payloads.append(payload_type)
            self.attached_payload = None
            self.logger.info("Dönüş payload bırakıldı: %s", payload_type.value)
        else:
            self.logger.error("Dönüş payload bırakma başarısız: %s — %s",
                              payload_type.value, res.message)
        return _to_legacy(res, payload_type)

    # ------------------------------------------------------------------ #
    def send_pickup_command(self, payload_type: TargetColor) -> None:
        """
        Görev 2: Non-blocking pickup komutu.
        State machine PAYLOAD_SERVO_WAIT_S süre bekleyip finish_pickup çağırır.
        """
        self.logger.info("PICKUP KOMUTU GÖNDERİLDİ: %s", payload_type.value)
        res = self._backend.send_pickup_command()
        if not res.success:
            self.logger.error("send_pickup_command başarısız: %s", res.message)

    def finish_pickup(self, payload_type: TargetColor) -> PayloadResult:
        """
        Görev 2: Pickup tamamla (stepper geri + state güncelle).
        State machine servo bekleme süresi dolduktan sonra bunu çağırır.
        """
        res = self._backend.finish_pickup()
        if res.success:
            self.attached_payload = payload_type
            self.logger.info("PAYLOAD PICKUP TAMAMLANDI: %s", payload_type.value)
        else:
            self.logger.error("finish_pickup başarısız: %s", res.message)
        return _to_legacy(res, payload_type)

    # ------------------------------------------------------------------ #
    def clear_attached(self) -> None:
        """Retry sırasında iç state'i sıfırla."""
        self.attached_payload = None
        self._backend.clear_attached()

    # ================================================================== #
    # Opsiyonel / yardımcı metodlar
    # ================================================================== #

    def pickup_payload(self, payload_type: TargetColor) -> PayloadResult:
        """Geriye dönük uyumluluk (test kodu için)."""
        self.send_pickup_command(payload_type)
        return self.finish_pickup(payload_type)

    def emergency_stop(self) -> PayloadResult:
        """Acil durdurma — tüm hareketleri durdur."""
        self.logger.warning("PAYLOAD EMERGENCY STOP!")
        res = self._backend.emergency_stop()
        return _to_legacy(res, TargetColor.RED)

    def set_magnet(self, magnet_id: MagnetId, on: bool) -> PayloadResult:
        """Tek mıknatısı bağımsız kontrol et (debug/override için)."""
        res = self._backend.set_magnet(magnet_id, on)
        return _to_legacy(res, TargetColor.RED)

    @property
    def servo_wait_s(self) -> float | None:
        """
        Stepper'ın aşağı hareketi için gereken süre (saniye).
        None dönerse state machine config.py'deki PAYLOAD_SERVO_WAIT_S kullanır.
        Gerçek backend'lerde steps_for_travel × step_delay × 2 olarak hesaplanır.
        """
        return self._stepper_duration
