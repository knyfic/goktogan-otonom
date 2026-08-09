"""
PayloadBackend arayüzü.

Tüm backend'lerin implement etmesi gereken sözleşme.
State machine bu interface'i kullanır — backend detaylarını bilmez.
"""
from __future__ import annotations

from abc import ABC, abstractmethod

from payload.models import MagnetId, PayloadOperationResult, PayloadSystemStatus


class PayloadBackend(ABC):
    """
    Payload donanım backend'inin abstract arayüzü.

    Üç somut implementasyon:
      SimulationBackend    → Gazebo / tam simülasyon
      RemoteRPiBackend     → Ubuntu → Wi-Fi → Raspberry Pi (HIL modu)
      LocalRPiBackend      → Raspberry Pi üzerinde doğrudan GPIO (gerçek uçuş)
    """

    # ------------------------------------------------------------------ #
    @abstractmethod
    def initialize(self) -> PayloadOperationResult:
        """
        Donanımı başlat: mıknatısları ON, stepper güvenli konumda.
        İki mıknatısın da ON başlaması zorunlu.
        """

    # ------------------------------------------------------------------ #
    @abstractmethod
    def send_pickup_command(self) -> PayloadOperationResult:
        """
        Non-blocking pickup komutu gönder.
        Stepper aşağı harekete başlar; tamamlanması beklenmez.
        State machine, PAYLOAD_SERVO_WAIT_S sonra finish_pickup çağırır.
        """

    @abstractmethod
    def finish_pickup(self) -> PayloadOperationResult:
        """
        Pickup'ı tamamla: stepper yukarı + durum güncelle.
        send_pickup_command başarılı olduktan PAYLOAD_SERVO_WAIT_S sonra çağrılır.
        """

    # ------------------------------------------------------------------ #
    @abstractmethod
    def release_payload_1(self) -> PayloadOperationResult:
        """
        1. payload'ı bırak:
          stepper release konuma in → Magnet 1 OFF → stepper yukarı çık.
          Magnet 2 DOKUNULMAZ.
        """

    @abstractmethod
    def release_payload_2(self) -> PayloadOperationResult:
        """
        2. payload'ı bırak:
          stepper release konuma in → Magnet 2 OFF → stepper yukarı çık.
          Magnet 1 DOKUNULMAZ (pickup sonrası her ikisi de ON).
        """

    # ------------------------------------------------------------------ #
    @abstractmethod
    def set_magnet(self, magnet_id: MagnetId, on: bool) -> PayloadOperationResult:
        """Tek bir mıknatısı bağımsız kontrol et."""

    # ------------------------------------------------------------------ #
    @abstractmethod
    def emergency_stop(self) -> PayloadOperationResult:
        """
        Acil durdurma: stepper durdur, sistem ERROR state'e gir.
        Yeniden çalışmak için initialize() gerekir.
        """

    @abstractmethod
    def clear_attached(self) -> None:
        """Pickup denemesi sonrası iç state'i sıfırla (retry için)."""

    # ------------------------------------------------------------------ #
    @abstractmethod
    def status(self) -> PayloadSystemStatus:
        """Anlık sistem durumunu döndür."""
