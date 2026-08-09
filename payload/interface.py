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

    ZAMANLAMA MANTIĞI (KRİTİK — DONANIM İLE UYUMLU):
      initialize()                 → Stepper EN KISA konum, 2 MIKNATIS ON
      release_payload_1()          → SADECE Magnet 1 OFF (stepper DOKUNULMAZ)
      release_payload_2()          → SADECE Magnet 2 OFF (stepper DOKUNULMAZ)
      release_all_payloads()       → İKİ MIKNATIS AYNI ANDA OFF (Görev 2 sonu)
      send_pickup_command()        → Stepper AŞAĞI + 2 Magnet ON (yerden alma)
      finish_pickup()              → Stepper YUKARI (alınan yükü taşımak)
    """

    # ------------------------------------------------------------------ #
    @abstractmethod
    def initialize(self) -> PayloadOperationResult:
        """
        Donanımı başlat: mıknatısları ON, stepper güvenli (en kısa) konumda.
        İki mıknatısın da ON başlaması zorunlu.
        """

    # ------------------------------------------------------------------ #
    @abstractmethod
    def send_pickup_command(self) -> PayloadOperationResult:
        """
        Non-blocking pickup komutu gönder.
        Stepper aşağı harekete başlar; tamamlanması beklenmez.
        İKİ MIKNATIS da ON → biri tutmazsa diğeri tutsun diye.
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
        1. payload'ı bırak (Görev 1 adım 1):
          SADECE Magnet 1 OFF. Stepper DOKUNULMAZ.
          Magnet 2 DOKUNULMAZ (kırmızı yükü taşımaya devam etmeli).
        """

    @abstractmethod
    def release_payload_2(self) -> PayloadOperationResult:
        """
        2. payload'ı bırak (Görev 1 adım 2):
          SADECE Magnet 2 OFF. Stepper DOKUNULMAZ.
        """

    @abstractmethod
    def release_all_payloads(self) -> PayloadOperationResult:
        """
        Geri getirilen payload'ı bırak (Görev 2 sonu):
          İKİ MIKNATIS AYNI ANDA OFF. Stepper DOKUNULMAZ.
          Amaç: yerden 2 mıknatısla alınan yükü 2. hedefe bırakmak.
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
