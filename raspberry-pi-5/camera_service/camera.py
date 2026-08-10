"""
Raspberry Pi Camera Module 3 (CSI / CAM0) sarıcı.

Picamera2 (libcamera tabanlı) ile 1920x1080 BGR frame üretir.
Ubuntu üzerinde test ederken Picamera2 yoksa uyarı verip stub çalışır.

ÖNEMLİ (RENK SIRASI DÜZELTME):
  Picamera2 + IMX708 (Cam Module 3) + libcamera kombinasyonunda "RGB888"
  format adı altında DONANIM ASLINDA BGR888 çıktısı verir. Bu yüzden
  önce BGR888 formatını doğrudan istemeye çalışırız (en hızlı: dönüşüm yok).
  Desteklenmezse RGB888 kullanırız ama DONÜŞÜM YAPMAYIZ — çünkü veri zaten BGR.
  Eski kod COLOR_RGB2BGR uyguluyordu ve tam tersi renkler çıkıyordu (mavi=kırmızı).
"""
from __future__ import annotations

import logging
import threading
from typing import Optional

import cv2
import numpy as np

try:
    from picamera2 import Picamera2  # type: ignore
    _PICAMERA2_AVAILABLE = True
except ImportError:
    _PICAMERA2_AVAILABLE = False


class RPiCam3:
    """
    Picamera2 tabanlı Raspberry Pi Camera Module 3.

    Varsayılan: CAM0 (CSI port 0) — camera_num=0.
    Frame formatı: BGR (OpenCV / state machine ile uyumlu).
    Arka thread'de sürekli capture eder, latest() en son frame'i döndürür.
    """

    def __init__(
        self,
        camera_num: int = 0,
        width:  int = 1920,
        height: int = 1080,
        fps:    int = 30,
        logger: logging.Logger | None = None,
    ) -> None:
        self._camera_num = camera_num
        self._width      = width
        self._height     = height
        self._fps        = fps
        self._log        = logger or logging.getLogger(__name__)
        self._lock       = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        self._running    = False
        self._picam: Optional[Picamera2] = None

        # Format stratejisi: hangi Picamera2 formatı seçildi ve cvtColor uygulanacak mı?
        #   ("BGR888", None)          en iyi — doğrudan BGR, CPU maliyeti sıfır
        #   ("RGB888", None)          fallback — donanım RGB888 izin veriyor ama BGR veriyor
        #   ("RGB888", COLOR_BGR2RGB) son fallback — gerçekten RGB gelirse kanal takası
        self._fmt_name:   str                        = "STUB"
        self._cvt_code:   Optional[int]              = None

        if not _PICAMERA2_AVAILABLE:
            self._log.warning(
                "picamera2 bulunamadı — RPiCam3 STUB modda. "
                "Raspberry Pi üzerinde libcamera ve python3-picamera2 kurulmalıdır."
            )
            return

        # ── Strateji 1: DOĞRUDAN BGR888 (en iyisi — dönüşüm yok) ────────────
        picam = Picamera2(camera_num=camera_num)
        try:
            config = picam.create_video_configuration(
                main={"size": (width, height), "format": "BGR888"},
            )
            picam.configure(config)      # BGR888 desteklenmezse burada RuntimeError verir
            self._fmt_name = "BGR888"
            self._cvt_code = None        # doğrudan BGR — dokunma
            self._log.info(
                "Picamera2 format: BGR888 (doğrudan OpenCV uyumlu — CPU kazanımı)"
            )
        except Exception as exc1:
            # ── Strateji 2: RGB888 ama DONÜŞÜMSÜZ (IMX708 aslında BGR verir) ─
            self._log.info(
                "BGR888 desteklenmiyor (%s) — RGB888 fallback, cvtColor YOK "
                "(IMX708 RGB888 format adı altında BGR üretiyor).", exc1,
            )
            picam = Picamera2(camera_num=camera_num)
            config = picam.create_video_configuration(
                main={"size": (width, height), "format": "RGB888"},
            )
            picam.configure(config)
            self._fmt_name = "RGB888"
            self._cvt_code = None  # ⚠ ESKİ KOD BURAYA COLOR_RGB2BGR KOYDUĞU İÇİN RENKLER TERSİYDİ

        # FPS kontrolü — RPi Cam 3 30 FPS 1080p rahat yapar
        try:
            config["controls"]["FrameDurationLimits"] = (
                int(1e6 / max(fps, 1)), int(1e6 / max(fps, 1))
            )
        except Exception:
            self._log.debug("FrameDurationLimits kontrolleri yok — varsayılan FPS kullanılıyor.")

        picam.start()
        self._picam = picam
        self._log.info(
            "RPi Camera Module 3 açıldı (CAM%d) %dx%d@%d — Picamera2/libcamera "
            "format=%s  cvt=%s",
            camera_num, width, height, fps,
            self._fmt_name,
            "yok (CPU kazanımı)" if self._cvt_code is None
                else f"cv2.{['BGR2RGB','RGB2BGR'][self._cvt_code//2]}",
        )

        # Capture thread başlat
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    # ── Capture loop ─────────────────────────────────────────────────────── #
    def _capture_loop(self) -> None:
        """Arka planda sürekli frame yakala ve en sonu sakla."""
        assert self._picam is not None
        while self._running:
            try:
                raw = self._picam.capture_array()
                if raw is None or raw.size == 0 or raw.ndim < 3:
                    continue

                # (H,W,4) RGBA/BGRA şeklinde gelebilir → ilk 3 kanalı al
                if raw.shape[2] == 4:
                    raw = raw[:, :, :3]

                # Renk dönüşümü (varsa)
                if self._cvt_code is None:
                    bgr = raw
                else:
                    bgr = cv2.cvtColor(raw, self._cvt_code)

                with self._lock:
                    self._frame = bgr
            except Exception as exc:
                self._log.warning("RPiCam3 capture hatası (devam ediyor): %s", exc)

    # ── Public API ──────────────────────────────────────────────────────── #
    def latest(self) -> Optional[np.ndarray]:
        """En son frame'in BGR kopyasını döndür. None = henüz frame yok."""
        with self._lock:
            if self._frame is None:
                # Stub: test ortamı için siyah frame döndür (None yerine) —
                # state machine None frame görürse döngüyü atlar, canlı testte sorun yok.
                if not _PICAMERA2_AVAILABLE:
                    return np.zeros((self._height, self._width, 3), dtype=np.uint8)
                return None
            return self._frame.copy()

    def close(self) -> None:
        """Kamerayı kapat, thread'i durdur."""
        self._running = False
        if self._picam is not None:
            try:
                self._picam.stop()
                self._picam.close()
            except Exception:
                pass
            self._picam = None
        self._log.info("RPi Camera Module 3 kapatıldı.")

    @property
    def is_stub(self) -> bool:
        """Picamera2 kurulu değil mi? Test/debug amaçlı."""
        return not _PICAMERA2_AVAILABLE
