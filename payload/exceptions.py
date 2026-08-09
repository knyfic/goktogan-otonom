"""Payload sistemi özel istisnaları."""


class PayloadError(Exception):
    """Payload sisteminin temel hatası."""


class InvalidStateError(PayloadError):
    """Payload bu operasyon için doğru state'te değil."""


class HardwareError(PayloadError):
    """GPIO veya donanım katmanından gelen hata."""


class BackendConnectionError(PayloadError):
    """Remote backend'e bağlantı kurulamadı."""


class OperationTimeoutError(PayloadError):
    """Operasyon beklenen sürede tamamlanamadı."""


class EmergencyStopError(PayloadError):
    """Acil durdurma aktif, sistem yeniden initialize edilmeli."""
