"""
RealCamera — Gerçek İHA kamerası için OpenCV tabanlı kamera sınıfı.

GazeboCamera ile aynı arayüzü sunar (latest, show, pump, close_preview).
State machine bu sınıfla da değiştirilmeden çalışır.

Desteklenen kaynaklar (config.yaml → camera.source):
  - integer : USB kamera index (0, 1, ...)
  - string  : /dev/video0, RTSP URL, gstreamer pipeline vb.

Kullanım (config.yaml):
  camera:
    source: 0          # USB kamera
    width: 1920
    height: 1080
    fps: 30
"""
from __future__ import annotations

import logging
import threading
from typing import Optional

import cv2
import numpy as np


class RealCamera:
    """
    OpenCV VideoCapture tabanlı kamera.
    GazeboCamera ile aynı public arayüz:
      latest()        → son frame (np.ndarray) veya None
      show(frame)     → pencerede göster
      pump()          → GUI event loop besle
      close_preview() → pencereyi kapat
    """

    def __init__(
        self,
        source,                    # int (USB index) veya str (device path / URL)
        width:  int  = 1920,
        height: int  = 1080,
        fps:    int  = 30,
        preview: bool = False,
        window_name: str = "İHA Kamera",
        logger: logging.Logger | None = None,
    ) -> None:
        self._log         = logger or logging.getLogger(__name__)
        self._preview     = preview
        self._window_name = window_name
        self._frame: Optional[np.ndarray] = None
        self._lock        = threading.Lock()
        self._running     = False

        self._cap = cv2.VideoCapture(source)
        if not self._cap.isOpened():
            raise RuntimeError(f"Kamera açılamadı: {source!r}")

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS,          fps)

        actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self._log.info("Kamera açıldı: %s  çözünürlük=%dx%d", source, actual_w, actual_h)

        if self._preview:
            cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self._window_name, 1280, 720)

        # Frame okuma ayrı thread'de — latest() her zaman güncel frame döndürür
        self._running = True
        self._thread  = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    # ── Capture loop ─────────────────────────────────────────────────────── #
    def _capture_loop(self) -> None:
        while self._running:
            ret, frame = self._cap.read()
            if not ret:
                self._log.warning("Kamera frame okunamadı.")
                continue
            with self._lock:
                self._frame = frame

    # ── Public API (GazeboCamera ile aynı) ───────────────────────────────── #
    def latest(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def show(self, frame: np.ndarray) -> None:
        if not self._preview:
            return
        cv2.imshow(self._window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.destroyWindow(self._window_name)
            self._preview = False

    def pump(self) -> None:
        if self._preview:
            cv2.waitKey(1)

    def close_preview(self) -> None:
        self._running = False
        if self._cap.isOpened():
            self._cap.release()
        if self._preview:
            cv2.destroyAllWindows()
            self._preview = False
