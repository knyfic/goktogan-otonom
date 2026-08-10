"""
Raspberry Pi Camera Module 3 (CSI / CAM0) sarıcı.

Picamera2 (libcamera tabanlı) ile 1920x1080 BGR frame üretir.
Ubuntu üzerinde test ederken Picamera2 yoksa uyarı verip stub çalışır.
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

        if not _PICAMERA2_AVAILABLE:
            self._log.warning(
                "picamera2 bulunamadı — RPiCam3 STUB modda. "
                "Raspberry Pi üzerinde libcamera ve python3-picamera2 kurulmalıdır."
            )
            return

        # Gerçek donanım: Picamera2 kur
        picam = Picamera2(camera_num=camera_num)
        config = picam.create_video_configuration(
            main={
                "size":   (width, height),
                "format": "RGB888",
            },
        )
        # FPS kontrolü — RPi Cam 3 30 FPS 1080p rahat yapar
        try:
            config["controls"]["FrameDurationLimits"] = (
                int(1e6 / max(fps, 1)), int(1e6 / max(fps, 1))
            )
        except Exception:
            self._log.debug("FrameDurationLimits kontrolleri yok — varsayılan FPS kullanılıyor.")
        picam.configure(config)
        picam.start()
        self._picam = picam
        self._log.info(
            "RPi Camera Module 3 açıldı (CAM%d) %dx%d@%d — Picamera2, libcamera",
            camera_num, width, height, fps,
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
                # Picamera2 capture_array → HWC RGB (uint8)
                rgb = self._picam.capture_array()
                if rgb is None or rgb.size == 0:
                    continue
                # RGB → BGR (OpenCV / state machine uyumlu)
                # capture_array memory bazen şekli (H,W,3) değil (H,W,4) verir; düzelt
                if rgb.ndim == 3 and rgb.shape[2] == 4:
                    rgb = rgb[:, :, :3]
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
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
