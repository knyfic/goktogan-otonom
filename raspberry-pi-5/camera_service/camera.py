"""
Raspberry Pi Camera Module 3 (CSI / CAM0) sarıcı.

Picamera2 (libcamera tabanlı) ile 1920x1080 BGR frame üretir.
Ubuntu üzerinde test ederken Picamera2 yoksa uyarı verip stub çalışır.

ÖNEMLİ (RENK SIRASI DÜZELTME — IMX708 BİLİNEN SORUNU):
  Picamera2 + IMX708 (Cam Module 3) + libcamera kombinasyonu
  FORMAT ADI İLE GERÇEK BYTE SIRASINI UYUŞTURMAYABİLİR.
  Bazı firmware/sürümlerinde:
    • format="BGR888" istendiği halde ham veri RGB888 gelir
    • format="RGB888" istendiği halde ham veri BGR888 gelir
    • İkisi de tamamen karışık gelir

  Bu yüzden 4 AŞAMALI bir strateji uygularız; hiçbiri çalışmazsa
  constructor parametresinden verilen `swap_rb_channels` ile
  ZORLAMALI kanal takası yapılır (IMX708 için default=True).
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

    Parametreler:
      swap_rb_channels — None = otomatik strateji sonucu neyse onu kullan
                         True = her halükarda R ↔ B kanallarını takas et (IMX708 default)
                         False = kesinlikle takas yapma (dönüşüm doğru çalışıyorsa)
    """

    def __init__(
        self,
        camera_num: int = 0,
        width:  int = 1920,
        height: int = 1080,
        fps:    int = 30,
        swap_rb_channels: Optional[bool] = True,
        logger: logging.Logger | None = None,
        orientation: str = "NORMAL",
    ) -> None:
        self._camera_num = camera_num
        self._width      = width
        self._height     = height
        self._fps        = fps
        self._forced_swap = swap_rb_channels   # None / True / False
        self._log        = logger or logging.getLogger(__name__)
        self._lock       = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        self._running    = False
        self._picam: Optional[Picamera2] = None
        # Orientation normalization for mounting (e.g. FLIP_H, ROTATE_180, etc.)
        self._orientation = str(orientation).upper() if orientation is not None else 'NORMAL'

        # Stratejinin sonucu: capture_array() sonrası uygulanan cvtCode
        #   None              → raw == BGR (dokunma)
        #   cv2.COLOR_RGB2BGR → raw == RGB → BGR takası
        #   cv2.COLOR_BGR2RGB → raw zaten BGR idi ama tekrar takas edildi (ters döndü)
        # En sonda _force_swap (True ise) ekstra `[:, :, ::-1]` uygular
        self._fmt_name:   str                        = "STUB"
        self._cvt_code:   Optional[int]              = None

        if not _PICAMERA2_AVAILABLE:
            self._log.warning(
                "picamera2 bulunamadı — RPiCam3 STUB modda. "
                "Raspberry Pi üzerinde libcamera ve python3-picamera2 kurulmalıdır."
            )
            return

        # ══════════════════════════════════════════════════════════════
        # AŞAMALI FORMAT SEÇİM STRATEJİSİ
        # ══════════════════════════════════════════════════════════════

        # AŞAMA 1 — Doğrudan BGR888 (ideal, dönüşüm maliyeti sıfır)
        picam = Picamera2(camera_num=camera_num)
        try:
            config = picam.create_video_configuration(
                main={"size": (width, height), "format": "BGR888"},
            )
            picam.configure(config)
            self._fmt_name = "BGR888"
            self._cvt_code = None
            self._log.info(
                "[Aşama 1 OK] Picamera2 format: BGR888 (doğrudan OpenCV uyumlu)."
            )
            self._picam = picam
        except Exception as exc1:
            self._log.info("[Aşama 1 FAIL] BGR888 desteklenmiyor: %s", exc1)
            self._picam = None

        # AŞAMA 2 — RGB888 + DONANIM GERÇEKTEN BGR VERİYORSA (cvt yok)
        if self._picam is None:
            picam = Picamera2(camera_num=camera_num)
            try:
                config = picam.create_video_configuration(
                    main={"size": (width, height), "format": "RGB888"},
                )
                picam.configure(config)
                # IMX708 çoğu sürümde RGB888 adı altında BGR baytları gönderir.
                # Eğer _forced_swap=False ise buna inanıyoruz demektir.
                if self._forced_swap is False:
                    self._fmt_name = "RGB888(cvt=RGB2BGR)"
                    self._cvt_code = cv2.COLOR_RGB2BGR
                    self._log.info(
                        "[Aşama 2 OK] RGB888 seçildi, swap_rb_channels=False → "
                        "gerçekten RGB geliyor varsayılıyor → COLOR_RGB2BGR uygulanacak."
                    )
                else:
                    # forced_swap=None veya True — IMX708 davranışını varsay:
                    # format adı RGB888 ama ham baytlar BGR → cvt yok, sonraki
                    # adımda global swap uygulanır
                    self._fmt_name = "RGB888(IMX708-stub-cvt=YOK)"
                    self._cvt_code = None
                    self._log.info(
                        "[Aşama 2 OK] RGB888 seçildi, IMX708 varsayımıyla "
                        "baytlar BGR kabul edildi (cvt yok). Zorlamalı swap sonrası BGR çıkacak."
                    )
                self._picam = picam
            except Exception as exc2:
                self._log.warning("[Aşama 2 FAIL] RGB888 de desteklenmiyor: %s", exc2)
                self._picam = None

        # AŞAMA 3 — Bütün stratejiler başarısız olduysa XRGB8888 gibi geniş format
        if self._picam is None:
            picam = Picamera2(camera_num=camera_num)
            try:
                config = picam.create_video_configuration(
                    main={"size": (width, height)},  # varsayılan format neyse onu kullan
                )
                picam.configure(config)
                self._fmt_name = f"DEFAULT({config['main']['format']})"
                self._cvt_code = cv2.COLOR_RGB2BGR   # güvenli varsayım
                self._log.warning(
                    "[Aşama 3 OK] Özel formatların hepsi reddedildi, varsayılan format=%s "
                    "kullanılıyor → COLOR_RGB2BGR uygulanacak.", self._fmt_name
                )
                self._picam = picam
            except Exception as exc3:
                self._log.error("Tüm format stratejileri başarısız: %s", exc3)
                raise RuntimeError(
                    f"Picamera2 format seçimi başarısız (BGR888/RGB888/varsayılan). "
                    f"Son hata: {exc3}"
                ) from exc3

        # FPS kontrolü — RPi Cam 3 30 FPS 1080p rahat yapar
        assert self._picam is not None
        try:
            picam_cf = self._picam.camera_configuration
            if picam_cf is not None and "controls" in picam_cf:
                picam_cf["controls"]["FrameDurationLimits"] = (
                    int(1e6 / max(fps, 1)), int(1e6 / max(fps, 1))
                )
        except Exception:
            self._log.debug(
                "FrameDurationLimits kontrolleri yazılamadı — varsayılan FPS kullanılıyor."
            )

        self._picam.start()
        self._log.info(
            "RPi Camera Module 3 açıldı (CAM%d) %dx%d@%d — Picamera2/libcamera "
            "fmt=%s  cvt=%s  swap_rb=%s",
            camera_num, width, height, fps,
            self._fmt_name,
            "yok" if self._cvt_code is None
                else ("COLOR_RGB2BGR" if self._cvt_code == cv2.COLOR_RGB2BGR
                      else f"cv2.code={self._cvt_code}"),
            "ZORLU(True)" if self._forced_swap is True
                else ("YOK(False)" if self._forced_swap is False else "OTOMATIK"),
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

                # Aşama-1: Format stratejisine göre renk dönüşümü
                if self._cvt_code is None:
                    frame = raw
                else:
                    frame = cv2.cvtColor(raw, self._cvt_code)

                # Aşama-2: Zorlamalı R↔B takası (forced_swap)
                #   IMX708 + libcamera sürüm kombinasyonuna göre
                #   yukarıdaki adımda renkler yine de ters kalmış olabilir.
                if self._forced_swap is True:
                    frame = frame[:, :, ::-1].copy()   # B ↔ R kanallarını takas et
                # Not: _forced_swap=None ise Aşama-1'in sonucu neyse odur (OTOMATIK)
                # _forced_swap=False ise dokunma

                # Orientation normalization (apply mounting correction here)
                try:
                    o = self._orientation
                    if o == 'ROTATE_90':
                        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
                    elif o == 'ROTATE_180':
                        frame = cv2.rotate(frame, cv2.ROTATE_180)
                    elif o == 'ROTATE_270':
                        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                    elif o == 'FLIP_H':
                        frame = cv2.flip(frame, 1)
                    elif o == 'FLIP_V':
                        frame = cv2.flip(frame, 0)
                except Exception:
                    pass

                with self._lock:
                    self._frame = frame
            except Exception as exc:
                self._log.warning("RPiCam3 capture hatası (devam ediyor): %s", exc)

    # ── Public API ──────────────────────────────────────────────────────── #
    def latest(self) -> Optional[np.ndarray]:
        """En son frame'in BGR kopyasını döndür. None = henüz frame yok."""
        with self._lock:
            if self._frame is None:
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
