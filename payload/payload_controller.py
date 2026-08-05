import logging
from dataclasses import dataclass

from models import TargetColor


@dataclass
class PayloadResult:
    success: bool
    payload_type: TargetColor
    message: str


class PayloadController:
    """
    Gazebo'da fiziksel servo yok; release/pickup dummy abstraction'dır.

    Non-blocking tasarım:
      send_pickup_command() → servo komutunu anında gönder, DÖNMEZ (blocking yok)
      finish_pickup()       → servo tamamlandıktan sonra state'i güncelle
      pickup_payload()      → geriye dönük uyumluluk; iç çağrı yapar (test için)
    """

    def __init__(self, logger: logging.Logger):
        self.logger = logger
        self.released_payloads: list = []
        self.attached_payload: TargetColor | None = None

    # ------------------------------------------------------------------ #
    def release_payload(self, payload_type: TargetColor) -> PayloadResult:
        """Yükü bırak (servo komutu gönder)."""
        self.logger.info("PAYLOAD RELEASE: %s", payload_type.value)
        self.released_payloads.append(payload_type)
        return PayloadResult(True, payload_type, "Dummy release successful.")

    # ------------------------------------------------------------------ #
    def send_pickup_command(self, payload_type: TargetColor) -> None:
        """
        Hata 7: Non-blocking servo komutu.
        Gerçek sistemde bu fonksiyon servo/gripper sinyal pinini tetikler ve
        hemen döner. Bekleme state machine tarafından PAYLOAD_SERVO_WAIT_S
        ile non-blocking olarak yapılır.
        """
        self.logger.info(
            "PICKUP SERVO KOMUTU GÖNDERİLDİ: %s (bekleme state machine'de)",
            payload_type.value,
        )
        # Gerçek donanım: GPIO / MAVLink servo komutu burada gönderilir

    def finish_pickup(self, payload_type: TargetColor) -> PayloadResult:
        """
        Servo bekleme süresi dolduktan sonra pickup'ı tamamla.
        State machine PAYLOAD_SERVO_WAIT_S geçtikten sonra bu metodu çağırır.
        """
        self.attached_payload = payload_type
        self.logger.info("PAYLOAD PICKUP TAMAMLANDI: %s", payload_type.value)
        return PayloadResult(True, payload_type, "Dummy pickup successful.")

    # ------------------------------------------------------------------ #
    def pickup_payload(self, payload_type: TargetColor) -> PayloadResult:
        """
        Geriye dönük uyumluluk (test kodu bu metodu çağırabilir).
        State machine artık send_pickup_command + finish_pickup kullanır.
        """
        self.send_pickup_command(payload_type)
        self.finish_pickup(payload_type)
        return PayloadResult(True, payload_type, "Dummy pickup successful.")

    # ------------------------------------------------------------------ #
    def clear_attached(self) -> None:
        self.attached_payload = None
