"""
Raspberry Pi Camera Module 3 (CSI / CAM0) sarıcı — TEK gerçek kamera kaynağı.

Bu sınıf, eski raspberry-pi-5/camera_service/camera.py'nin taşınmış halidir.
HIL Raspberry Pi camera server'ı (services/camera_server.py) ve REAL uygulaması
(main.py → backend: real) BİREBİR AYNI bu sınıfı kullanır.

Picamera2 (libcamera tabanlı) ile 1920x1080 BGR frame üretir.
Ubuntu üzerinde test ederken Picamera2 yoksa uyarı verip stub çalışır.

ÖNEMLİ (RENK SIRASI DÜZELTME — IMX708 BİLİNEN SORUNU):
  Picamera2 + IMX708 (Cam Module 3) + libcamera kombinasyonu
  FORMAT ADI İLE GERÇEK BYTE SIRASINI UYUŞTURMAYABİLİR.
  Bazı firmware/sürümlerinde:
    • format="BGR888" istendiği halde ham veri RGB888 gelir
    • format="RGB888" istendiği halde ham veri BGR888 gelir
    • İkisi de tamamen karışık gelir

  Bu yüzden 4 AŞAMALI bir format stratejisi uygularız ve en sonda
  B ↔ R kanal takasını SABİT olarak uygularız (config değil — saha
  testleriyle doğrulanan tek doğru davranış; çıktı her zaman BGR).

STATE MACHINE ARRAYÜZÜ: latest / show / pump / close_preview
  GazeboCamera / RemoteRPiCamera ile aynı — görüntü işleme kodu değişmez.

OTOMATİK ODAK (AF): IMX708 VCM lens destekler. libcamera varsayılanı
  AfMode=Manual (lens sabit) olduğundan, açılışta AfMode=Continuous
  (Normal menzil, Fast) açıkça set_controls() ile etkinleştirilir;
  aksi halde kamera hiç netlemez.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import TYPE_CHECKING, Optional

import cv2
import numpy as np

if TYPE_CHECKING:
    from picamera2 import Picamera2  # type: ignore
    _PICAMERA2_AVAILABLE = True
else:
    try:
        from picamera2 import Picamera2  # type: ignore
        _PICAMERA2_AVAILABLE = True
    except ImportError:
        _PICAMERA2_AVAILABLE = False
        Picamera2 = None  # type: ignore[assignment]


class RPiCam3:
    """
    Picamera2 tabanlı Raspberry Pi Camera Module 3.

    Varsayılan: CAM0 (CSI port 0) — camera_num=0.
    Frame formatı: BGR (OpenCV / state machine ile uyumlu).
    Arka thread'de sürekli capture eder, latest() en son frame'i döndürür.

    RENK KANALI (sabit davranış — config değil):
      Picamera2 + IMX708 (Cam Module 3) + libcamera kombinasyonu FORMAT ADI
      İLE GERÇEK BYTE SIRASINI UYUŞTURMAYABİLİR (örn. BGR888 istenince RGB
      gelir). Saha testleriyle doğrulanan tek doğru davranış uygulanır:
      format stratejisinden sonra B ↔ R kanal takası HER ZAMAN yapılır ve
      sonuç BGR olur (state machine / OpenCV uyumlu).

    Parametreler:
      preview          — canlı önizleme penceresi (GazeboCamera ile aynı davranış)
      window_name      — önizleme penceresi adı
    """

    def __init__(
        self,
        camera_num: int = 0,
        width:  int = 1920,
        height: int = 1080,
        fps:    int = 30,
        logger: logging.Logger | None = None,
        orientation: str = "NORMAL",
        preview: bool = False,
        window_name: str = "İHA Kamera",
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
        self._preview    = bool(preview)
        self._window_name = window_name
        # Orientation normalization for mounting (e.g. FLIP_H, ROTATE_180, etc.)
        self._orientation = str(orientation).upper() if orientation is not None else 'NORMAL'

        # Stratejinin sonucu: capture_array() sonrası uygulanan cvtCode
        #   None              → raw == BGR (dokunma)
        #   cv2.COLOR_RGB2BGR → raw == RGB → BGR takası
        self._fmt_name:   str                        = "STUB"
        self._cvt_code:   Optional[int]              = None

        if not _PICAMERA2_AVAILABLE:
            self._log.warning(
                "picamera2 bulunamadı — RPiCam3 STUB modda. "
                "Raspberry Pi üzerinde libcamera ve python3-picamera2 kurulmalıdır."
            )
        else:
            self._init_picamera()

        # Önizleme penceresi — GazeboCamera/RealCamera ile aynı davranış
        if self._preview:
            cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self._window_name, 1280, 720)

        # Capture thread başlat (yalnızca gerçek donanım modunda)
        if self._picam is not None:
            self._running = True
            self._thread = threading.Thread(target=self._capture_loop, daemon=True)
            self._thread.start()

    # ── Picamera2 kurulumu (AŞAMALI FORMAT STRATEJİSİ) ──────────────────── #
    def _init_picamera(self) -> None:
        """BGR888 → RGB888 → varsayılan format sırasıyla Picamera2'yi kurar."""
        assert _PICAMERA2_AVAILABLE

        # AŞAMA 1 — Doğrudan BGR888 (ideal, dönüşüm maliyeti sıfır)
        picam = Picamera2(camera_num=self._camera_num)
        try:
            config = picam.create_video_configuration(
                main={"size": (self._width, self._height), "format": "BGR888"},
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

        # AŞAMA 2 — RGB888: IMX708 çoğu sürümde RGB888 adı altında BGR baytları
        # gönderir → cvt yok; B↔R takası capture döngüsünde sabit uygulanır.
        if self._picam is None:
            picam = Picamera2(camera_num=self._camera_num)
            try:
                config = picam.create_video_configuration(
                    main={"size": (self._width, self._height), "format": "RGB888"},
                )
                picam.configure(config)
                self._fmt_name = "RGB888(IMX708-cvt=YOK)"
                self._cvt_code = None
                self._log.info(
                    "[Aşama 2 OK] RGB888 seçildi, IMX708 varsayımıyla "
                    "baytlar BGR kabul edildi (cvt yok). Sabit B↔R takası sonrası BGR çıkacak."
                )
                self._picam = picam
            except Exception as exc2:
                self._log.warning("[Aşama 2 FAIL] RGB888 de desteklenmiyor: %s", exc2)
                self._picam = None

        # AŞAMA 3 — Bütün stratejiler başarısız olduysa XRGB8888 gibi geniş format
        if self._picam is None:
            picam = Picamera2(camera_num=self._camera_num)
            try:
                config = picam.create_video_configuration(
                    main={"size": (self._width, self._height)},  # varsayılan format neyse onu kullan
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

        # ── FPS + OTOMATİK ODAK (AF) ────────────────────────────────────────
        # NOT: Picamera2 config dict'ini (camera_configuration) elle değiştirmek
        # kameraya UYGULANMAZ; doğru yol start() sonrası set_controls()'tür.
        assert self._picam is not None
        self._picam.start()

        # FPS sınırı — RPi Cam 3 30 FPS 1080p rahat yapar
        try:
            self._picam.set_controls({
                "FrameDurationLimits": (
                    int(1e6 / max(self._fps, 1)), int(1e6 / max(self._fps, 1))
                ),
            })
        except Exception as exc:
            self._log.debug(
                "FrameDurationLimits ayarlanamadı — varsayılan FPS kullanılıyor: %s", exc
            )

        # OTOMATİK ODAK — IMX708 (Cam Module 3) VCM lens destekler, ANCAK
        # libcamera'da varsayılan AfMode "Manual"dır: lens SABİT konumda kalır ve
        # kamera HİÇ netlemez (görüntü bulanık). Görev profili aşağı bakan kamera
        # + 0.5-11.5m çalışma mesafesi olduğu için sürekli netleme (Continuous)
        # uygundur; açılışta bir kez AfTrigger=Start ile ANINDA netleme başlatılır
        # (beklemeden net görüntü).
        try:
            from picamera2.controls import Controls  # type: ignore
            self._picam.set_controls({
                "AfMode":     Controls.AfModeEnum.Continuous,
                "AfRange":    Controls.AfRangeEnum.Normal,
                "AfSpeed":    Controls.AfSpeedEnum.Fast,
                "AfTrigger":  Controls.AfTriggerEnum.Start,
            })
            # Doğrulama 1: kamera AF kontrollerini destekliyor mu? Desteklenmiyorsa
            # VCM lens sürücüsü algılanmıyor demektir (genelde FFC kablo/konnektör).
            try:
                supported = self._picam.camera_controls
                af_supported = "AfMode" in supported and "LensPosition" in supported
            except Exception:
                af_supported = True   # okunamadı — ayarlandı kabul et
            if af_supported:
                self._log.info(
                    "[AF] Otomatik odak AÇIK: Continuous / Normal menzil / Fast "
                    "(AfTrigger=Start → anında netleme)."
                )
            else:
                self._log.warning(
                    "[AF] AfMode/LensPosition kontrolü kamerada BULUNAMADI — VCM lens "
                    "algılanmıyor. FFC kablo/konnektörü kontrol edin ve `libcamera-hello "
                    "--list-cameras` çıktısına bakın."
                )

            # Doğrulama 2 (KANIT): metadata'dan AfState (0=Idle, 1=Scanning,
            # 2=Focused, 3=Failed) ve LensPosition (dioptre) birkaç örnek okunur.
            # LensPosition DEĞİŞİYORSA lens gerçekten hareket ediyordur;
            # AfState=2 görülürse netleme tamamlanmıştır. Karanlık/kontrastsız
            # sahnede AfState hep 0 kalabilir → bu durumda sorun kod değil ışıktır.
            try:
                _af_states: list[int] = []
                _lens_pos: list[float] = []
                for _ in range(6):   # ~1.5s içinde 6 örnek
                    md = self._picam.capture_metadata()
                    _af_states.append(int(md.get("AfState", -1)))
                    lp = md.get("LensPosition", float("nan"))
                    _lens_pos.append(round(float(lp), 2))
                    time.sleep(0.25)
                _states_str = ",".join(map(str, _af_states))
                _lens_str = ",".join(map(str, _lens_pos))
                if 2 in _af_states:
                    self._log.info(
                        "[AF] DOĞRULANDI: AfState=2(Focused) görüldü | lens=%s | "
                        "durumlar=[%s]", _lens_str, _states_str,
                    )
                elif 1 in _af_states:
                    self._log.info(
                        "[AF] TARAMA SÜRÜYOR: AfState=1(Scanning) | lens=%s | "
                        "durumlar=[%s]", _lens_str, _states_str,
                    )
                else:
                    self._log.warning(
                        "[AF] NETLEME GÖZLEMLENMEDİ: durumlar=[%s] lens=%s — "
                        "sahne karanlık/kontrastsız olabilir; aydınlıkta tekrar deneyin",
                        _states_str, _lens_str,
                    )
            except Exception as exc:
                self._log.debug("AF durum okuması başarısız: %s", exc)
        except Exception as exc:
            self._log.warning(
                "[AF] Ayarlanamadı (%s) — lens varsayılan (Manual) konumda kalabilir; "
                "görüntü bulanık olur. Donanım kontrolü: `libcamera-hello "
                "--autofocus-mode continuous` ile CLI'da AF çalışıyor mu bakın, "
                "kablo (FFC) / konnektör sağlam mı doğrulayın.", exc
            )

        self._log.info(
            "RPi Camera Module 3 açıldı (CAM%d) %dx%d@%d — Picamera2/libcamera "
            "fmt=%s  cvt=%s",
            self._camera_num, self._width, self._height, self._fps,
            self._fmt_name,
            "yok" if self._cvt_code is None
                else ("COLOR_RGB2BGR" if self._cvt_code == cv2.COLOR_RGB2BGR
                      else f"cv2.code={self._cvt_code}"),
        )

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

                # Aşama-2: IMX708 + libcamera sürüm kombinasyonunda format adı
                # ile gerçek byte sırası uyuşmayabilir → B ↔ R kanal takası SABİT
                # uygulanır (saha testleriyle doğrulanan tek doğru davranış).
                frame = frame[:, :, ::-1].copy()   # B ↔ R kanallarını takas et

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

    def show(self, frame: np.ndarray) -> None:
        """
        Verilen frame'i canlı pencerede gösterir.
        GazeboCamera ile aynı davranış — yalnızca ana thread'den çağrılır.
        """
        if not self._preview:
            return
        cv2.imshow(self._window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.destroyWindow(self._window_name)
            self._preview = False

    def pump(self) -> None:
        """GUI event loop'u besle (waitKey(1)) — GazeboCamera ile aynı."""
        if self._preview:
            cv2.waitKey(1)

    def close_preview(self) -> None:
        """Kamerayı ve önizleme penceresini kapat (state machine kapanışı)."""
        self.close()
        if self._preview:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
            self._preview = False

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
