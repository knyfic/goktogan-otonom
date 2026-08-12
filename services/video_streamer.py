"""
GStreamer video akışı (yalnızca backend: real) — ground station'a düşük gecikmeli H.264.

Mimari kurallar:
  • GStreamer YALNIZCA REAL modda oluşturulur ve çalışır. gi/Gst importu
    start() içinde LAZY yapılır — simülasyon/HIL/test ortamında asla yüklenmez.
  • Video akışı uçuş kontrolü/görev için KRİTİK dependency DEĞİLDİR:
    start() ve push() ASLA exception fırlatmaz; her hata loglanır ve akış
    devre dışı kalır — görev devam eder.
  • Düşük gecikme hedefi (fiziksel sıfır değil, stabil minimum):
      appsrc (max-buffers=1, non-blocking)
        → videoconvert (BGR → I420)
        → queue (max-size-buffers=1, leaky=downstream → encoder yavaşsa frame düşer)
        → x264enc (tune=zerolatency, ultrafast, bframes=0, key-int-max=30)
        → rtph264pay (config-interval=1)
        → udpsink (sync=false)
  • FPS tavanı: push() 1/fps saniyede en fazla bir frame geçirir — state machine
    ~50 Hz çağırsa da encoder yükü sınırlanır ve duplike frame'ler birleşir.

Ground station alıcısı (README'de de var):
  gst-launch-1.0 udpsrc port=5000 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" \
    ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false
"""
from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from vision.camera import CameraLike


class VideoStreamer:
    """
    GStreamer üzerinden UDP/RTP H.264 yayıncı.

    - start()  : GStreamer kütüphanesini yükler (lazy). Başarısızlık → enabled=False.
    - push()   : BGR frame'i appsrc'ye iter. FPS tavanı + leaky queue ile asla
                 bloklamaz ve asla exception fırlatmaz.
    - stop()   : EOS gönderir, pipeline'ı kapatır (idempotent).
    """

    def __init__(
        self,
        host: str,
        port: int,
        bitrate: int = 2_000_000,
        codec: str = "h264",
        fps: int = 30,
        logger: logging.Logger | None = None,
    ) -> None:
        self._host = host
        self._port = int(port)
        self._bitrate = max(int(bitrate), 100_000)
        self._codec = str(codec).lower()
        self._fps = max(int(fps), 1)
        self._log = logger or logging.getLogger(__name__)

        self._gst = None          # gi.repository.Gst — start() içinde lazy yüklenir
        self._gst_ready = False
        self._pipeline = None
        self._appsrc = None
        self._enabled = False     # yayın aktif mi?
        self._last_push_t: float | None = None
        self._pushed_count = 0
        self._drop_count = 0
        self._error_count = 0

    # ------------------------------------------------------------------ #
    #  Yaşam döngüsü
    # ------------------------------------------------------------------ #

    @property
    def enabled(self) -> bool:
        """Yayın aktif mi? (start başarılı + pipeline kuruldu)"""
        return self._enabled

    @property
    def pushed_frames(self) -> int:
        """appsrc'ye verilen toplam frame sayısı (FPS tavanı sonrası)."""
        return self._pushed_count

    @property
    def dropped_frames(self) -> int:
        """Encoder yetişemediği için düşürülen frame sayısı (leaky queue)."""
        return self._drop_count

    @property
    def error_count(self) -> int:
        """push sırasında oluşan hata sayısı (GStreamer sorunları)."""
        return self._error_count

    def start(self) -> bool:
        """
        GStreamer kütüphanesini yükler ve başlatmaya hazırlanır.

        Pipeline ilk push()'ta frame boyutuna göre kurulur (caps her zaman
        gerçek frame ile eşleşir). Başarısızlık asla raise ETMEZ — False döner.
        """
        if self._gst_ready:
            return True
        try:
            import gi  # Ubuntu sim/HIL ortamında hiç yüklenmez
            gi.require_version("Gst", "1.0")
            from gi.repository import Gst  # type: ignore[import-not-found]
            Gst.init(None)
        except Exception as exc:
            self._log.warning(
                "GStreamer kullanılamıyor — video akışı devre dışı "
                "(görev bundan etkilenmez). %s", exc,
            )
            return False
        self._gst = Gst
        self._gst_ready = True
        self._log.info(
            "GStreamer hazır — akış ilk frame ile başlayacak (udp://%s:%d)",
            self._host, self._port,
        )
        return True

    def push(self, frame: np.ndarray) -> None:
        """
        BGR frame'i yayına iter. Asla bloklamaz, asla raise etmez.

        - FPS tavanı: aynı frame'in ~50 Hz'de tekrar tekrar push edilmesini engeller.
        - appsrc max-buffers=1 + non-blocking: encoder yetişemezse frame düşer.
        - Kodc h264 dışında bir değerse (ör. v4l2h264 — doğrulanmamış) pipeline
          kurulamaz → enabled=False, görev devam eder.
        """
        if not self._gst_ready or not self._gst:
            return

        # FPS tavanı — encoder yükünü sınırla (duplike frame'leri birleştirir)
        now = time.monotonic()
        min_interval = 1.0 / self._fps
        if self._last_push_t is not None and (now - self._last_push_t) < min_interval:
            return
        self._last_push_t = now

        # Pipeline ilk frame ile kurulur (caps = gerçek frame boyutu)
        if not self._enabled:
            self._build_pipeline(frame.shape[1], frame.shape[0])
            if not self._enabled:
                return
        if self._appsrc is None:
            return

        try:
            data = np.ascontiguousarray(frame).tobytes()
            buf = self._gst.Buffer.new_allocate(None, len(data), None)
            filled = buf.fill(0, data)
            if filled != len(data):
                self._log.warning(
                    "video stream buffer fill eksik (%d/%d)", filled, len(data),
                )
                return
            ret = self._appsrc.emit("push-buffer", buf)
            self._pushed_count += 1
            if ret != self._gst.FlowReturn.OK:
                # Dolu/boşaltılıyor → frame düşür (mission asla engellenmez)
                self._drop_count += 1
                if self._drop_count % 100 == 1:
                    self._log.debug(
                        "video stream frame düşürüldü (toplam %d)", self._drop_count,
                    )
        except Exception as exc:
            # GStreamer hatası görevi asla durdurmamalı
            self._error_count += 1
            if self._error_count <= 5:
                self._log.warning("video stream push hatası: %s", exc)

    def stop(self) -> None:
        """EOS gönderir ve pipeline'ı kapatır (idempotent, asla raise etmez)."""
        try:
            if self._appsrc is not None:
                self._appsrc.emit("end-of-stream")
        except Exception:
            pass
        self._cleanup_pipeline()
        if self._gst_ready:
            self._log.info("GStreamer video akışı durduruldu.")

    # ------------------------------------------------------------------ #
    #  Pipeline
    # ------------------------------------------------------------------ #

    def _build_pipeline(self, width: int, height: int) -> None:
        """appsrc → videoconvert → leaky queue → x264enc → rtph264pay → udpsink."""
        Gst = self._gst
        if Gst is None:
            return
        try:
            # x264enc bitrate birimi kbps'tir (config bps'tir)
            bitrate_kbps = max(self._bitrate // 1000, 100)
            launch = (
                f"appsrc name=src is-live=true format=time do-timestamp=true "
                f"max-buffers=1 block=false "
                f'caps="video/x-raw,format=BGR,width={width},height={height},'
                f"framerate={self._fps}/1\" "
                f"! videoconvert ! video/x-raw,format=I420 "
                f"! queue max-size-buffers=1 leaky=downstream "
                f"! x264enc tune=zerolatency speed-preset=ultrafast "
                f"bitrate={bitrate_kbps} bframes=0 key-int-max=30 "
                f"! rtph264pay config-interval=1 pt=96 "
                f"! udpsink host={self._host} port={self._port} sync=false"
            )
            if self._codec != "h264":
                # Yalnızca doğrulanmış kodc kullanılır — bilinmeyen kodc devre dışı
                self._log.warning(
                    "video_stream.codec=%r desteklenmiyor (yalnızca 'h264') — "
                    "video akışı devre dışı.", self._codec,
                )
                return
            self._pipeline = Gst.parse_launch(launch)
            self._appsrc = self._pipeline.get_by_name("src")
            if self._appsrc is None:
                raise RuntimeError("appsrc bulunamadı")
            self._pipeline.set_state(Gst.State.PLAYING)
            self._enabled = True
            self._log.info(
                "GStreamer yayın başladı: udp://%s:%d — %dx%d@%d h264 %.0f kbps "
                "(zerolatency, leaky queue)",
                self._host, self._port, width, height, self._fps, bitrate_kbps,
            )
        except Exception as exc:
            self._log.warning(
                "GStreamer pipeline kurulamadı — video akışı devre dışı "
                "(görev bundan etkilenmez). %s", exc,
            )
            self._cleanup_pipeline()

    def _cleanup_pipeline(self) -> None:
        try:
            if self._pipeline is not None and self._gst is not None:
                self._pipeline.set_state(self._gst.State.NULL)
        except Exception:
            pass
        self._pipeline = None
        self._appsrc = None
        self._enabled = False


class StreamCamera:
    """
    Kamera arayüzünü (latest/show/pump/close_preview) koruyan ince sarmalayıcı.

    REAL modda state machine'in gösterdiği frame'i (960x540 HUD) GStreamer'a
    iletir. Görev kodu kameranın sarmalandığını bilmez — arayüz aynıdır.
    GStreamer başarısız olsa bile kamera davranışı bozulmaz.
    """

    def __init__(
        self,
        camera: CameraLike,
        streamer: VideoStreamer,
        logger: logging.Logger | None = None,
    ) -> None:
        self._camera = camera
        self._streamer = streamer
        self._log = logger or logging.getLogger(__name__)

    def latest(self) -> np.ndarray | None:
        """Son frame'i döndürür — gerçek kameraya delege eder."""
        return self._camera.latest()

    def show(self, frame: np.ndarray) -> None:
        """Gösterim frame'i → GStreamer; ayrıca kameranın show()'una delege eder."""
        try:
            self._streamer.push(frame)
        except Exception as exc:
            # push() zaten asla raise etmez — çift koruma
            self._log.debug("video stream push hatası: %s", exc)
        self._camera.show(frame)   # preview=False → no-op

    def pump(self) -> None:
        """GUI event loop (REAL'de no-op)."""
        self._camera.pump()

    def close_preview(self) -> None:
        """Yayını durdurur ve kamerayı kapatır."""
        try:
            self._streamer.stop()
        except Exception:
            pass
        self._camera.close_preview()
