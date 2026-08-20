"""
Görüntü işleme + SD karta video kayıt scripti (REAL, headless) — YALNIZCA KAYIT.

TEK AMAÇ:
  Raspberry Pi'deki gerçek RPi Camera Module 3'ten frame al → projenin mevcut
  görüntü işleme pipeline'ı (vision/target_detector.py) ile MAVİ ALTIGEN ve
  KIRMIZI ÜÇGEN tespit et → tespit overlay + HUD bandı çiz (state machine ile
  aynı UI) → H.264/MP4 olarak recordings/ dizinine (SD kart) kaydet.

Bu, vision_stream_only.py'nin "yayın" yerine "kayıt" yapan bağımsız kardeşidir:
  • VideoStreamer / UDP KULLANILMAZ — çıktı yerel MP4 dosyasıdır.
  • Görüntü, canlı yayındaki overlay/HUD ile AYNI içeriğe sahiptir; yalnızca
    çözünürlük debug için daha yüksektir (VISION_RECORDING_WIDTH_PX=960,
    yayın 480 px) — tespit yine TAM çözünürlükte yapılır.

KASITLI SINIRLAR (güvenlik — bu script UÇUŞ YAPMAZ):
  • MAVLink bağlantısı AÇILMAZ — flight.* / payload.* / state_machine ASLA
    import edilmez; dolayısıyla arm / takeoff / goto / rtl gibi HİÇBİR hareket
    komutu gönderilemez.
  • Pencere açılmaz (headless), ağ yayını yok.
  • Tespit sonucunda HİÇBİR aksiyon alınmaz; yalnızca kadraja çizilir.

KULLANIM (Raspberry Pi'de, İHA 11.5m'de Mission Planner ile HOLD'da iken):
  python3 vision_record_only.py
  Durdurma: Ctrl+C  (MP4 finalize edilir — bozuk dosya bırakılmaz)

KONFIG:
  config.py:
    CAMERA_WIDTH_PX / CAMERA_HEIGHT_PX / REAL_CAMERA_FPS / RPI_CAMERA_NUM
    VISION_CROP_HEIGHT_PX / VISION_WIDTH_PX / VISION_HEIGHT_PX
    / VISION_RECORDING_DIR / VISION_RECORDING_BITRATE / VISION_RECORDING_WIDTH_PX
  payload/config.yaml:
    camera.orientation → fiziksel kamera montaj yönü (REAL: 'ROTATE_180')
"""
from __future__ import annotations

import datetime
import logging
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

from config import (
    CAMERA_HEIGHT_PX,
    CAMERA_ORIENTATION,
    CAMERA_WIDTH_PX,
    REAL_CAMERA_FPS,
    RPI_CAMERA_NUM,
    VISION_CROP_HEIGHT_PX,
    VISION_HEIGHT_PX,
    VISION_WIDTH_PX,
    VISION_RECORDING_BITRATE,
    VISION_RECORDING_DIR,
    VISION_RECORDING_WIDTH_PX,
)
from utils.logging import setup_logging
from vision.camera import VisionCropCamera
from vision.rpi_camera import RPiCam3
from vision.target_detector import TargetDetector

_PAYLOAD_CONFIG = os.path.join(os.path.dirname(__file__), "payload", "config.yaml")

# Denenecek video fourcc'leri (H.264 → MPEG-4 fallback).
# Raspberry Pi OpenCV (ffmpeg backend) 'avc1'/'h264' → H.264 MP4 yazar;
# desteklenmiyorsa 'mp4v' (MPEG-4 Part 2) ile devam edilir — ikisi de
# tarayıcı/VLC'de oynar.
_VIDEO_FOURCC_ORDER = ("avc1", "h264", "x264", "mp4v")


# --------------------------------------------------------------------------- #
#  HUD yardımcıları — vision_stream_only.py / state_machine.MissionStateMachine
#  ile BİREBİR aynı davranış (UI tutarlılığı). state machine import edilmez
#  (flight/payload zincirini çekmemek için).
# --------------------------------------------------------------------------- #

def _draw_hud(
    frame: np.ndarray,
    line1: str,
    line2: str = "",
    line1_color: tuple = (0, 230, 0),
    line2_color: tuple = (0, 210, 255),
) -> np.ndarray:
    """Şeffaf HUD bandı çiz (state_machine._draw_hud ile aynı)."""
    out = frame.copy()
    h, w = out.shape[:2]

    ui_scale = w / 640.0
    band_h = int(56 * ui_scale)

    overlay = out.copy()
    cv2.rectangle(overlay, (0, 0), (w, band_h), (10, 10, 10), -1)
    cv2.addWeighted(overlay, 0.55, out, 0.45, 0, out)
    cv2.line(out, (0, band_h), (w, band_h), (60, 60, 60), 1)

    font_scale1 = 0.65 * ui_scale
    font_scale2 = 0.58 * ui_scale
    thickness1 = max(1, int(2 * ui_scale))
    thickness2 = max(1, int(1 * ui_scale))
    y1 = int(22 * ui_scale)
    y2 = int(46 * ui_scale)

    cv2.putText(out, line1, (10, y1), cv2.FONT_HERSHEY_SIMPLEX,
                font_scale1, line1_color, thickness1, cv2.LINE_AA)
    if line2:
        cv2.putText(out, line2, (10, y2), cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale2, line2_color, thickness2, cv2.LINE_AA)

    cx, cy = w // 2, h // 2
    marker_size = int(24 * ui_scale)
    cv2.drawMarker(out, (cx, cy), (255, 255, 255),
                   cv2.MARKER_CROSS, marker_size, max(1, int(ui_scale)), cv2.LINE_AA)
    return out


def _scale_for_display(frame: np.ndarray, width_px: int = VISION_RECORDING_WIDTH_PX) -> np.ndarray:
    """Yazılacak frame'i hedef genişliğe küçült (detection TAM çözünürlükte yapıldı)."""
    h, w = frame.shape[:2]
    if w <= width_px:
        return frame
    nw = width_px
    nh = max(1, int(round(h * nw / w)))
    return cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_AREA)


def detection_summary(detections: list) -> str:
    """Tespit özeti — HUD line2 için (ör. 'RED TRIANGLE 0.92 | BLUE HEXAGON 0.87')."""
    if not detections:
        return "tespit yok"
    return "  ".join(
        f"{d.color.value} {d.shape} {d.confidence:.2f}" for d in detections[:3]
    )


def process_frame(
    frame_bgr: np.ndarray,
    detector: TargetDetector,
    width_px: int = VISION_RECORDING_WIDTH_PX,
    fps: float = 0.0,
    proc_ms: float = 0.0,
) -> tuple[np.ndarray, list, str]:
    """Tek frame: tespit → overlay → HUD → kayıt frame'i.

    Dönen: (kayıt frame'i, detections, tespit özeti) — tespit ASLA aksiyon
    üretmez, yalnızca görselleştirilir. HUD bandında canlı işleme FPS'i ve
    ortalama frame süresi gösterilir (performans takibi).
    """
    detections = detector.detect(frame_bgr)
    disp = TargetDetector.draw_detections(frame_bgr, detections)
    summary = detection_summary(detections)
    hud1 = f"VISION-ONLY RECORD | FPS {fps:.1f} | {proc_ms:.0f} ms"
    disp = _draw_hud(disp, hud1, f"TESPIT: {summary}")
    return _scale_for_display(disp, width_px), detections, summary


# --------------------------------------------------------------------------- #
#  Kayıt dosyası / video writer
# --------------------------------------------------------------------------- #

def recording_path(
    recordings_dir: str | os.PathLike = VISION_RECORDING_DIR,
    now: datetime.datetime | None = None,
    prefix: str = "vision",
) -> Path:
    """recordings/ dizinini oluşturur ve timestamp'li MP4 yolu döndürür.

    Yol, proje root'una göre çözülür (çalıştırma konumuna bağımlı değildir):
    recordings_dir göreli ise bu dosyanın bulunduğu klasörün altına alınır.
    prefix ile dosya adı farklılaştırılabilir (örn. 'vision_mask').
    """
    d = Path(recordings_dir)
    if not d.is_absolute():
        d = Path(__file__).resolve().parent / d
    d.mkdir(parents=True, exist_ok=True)
    ts = (now or datetime.datetime.now()).strftime("%Y-%m-%d_%H-%M-%S")
    return d / f"{prefix}_{ts}.mp4"


def _ffmpeg_command(
    path: str | os.PathLike,
    width: int,
    height: int,
    fps: int,
    bitrate: int,
) -> list[str]:
    """ffmpeg CLI komutu — garanti H.264 (libx264) + yuv420p + faststart MP4.

    Tarayıcı uyumluluğu için H.264/yuv420p zorunludur; cv2'nin 'mp4v'
    fallback'i (MPEG-4 Part 2) tarayıcılarda oynatılamaz.
    """
    br = max(int(bitrate), 500_000)
    return [
        "ffmpeg", "-y", "-loglevel", "error",
        "-f", "rawvideo", "-pix_fmt", "bgr24",
        "-s", f"{int(width)}x{int(height)}", "-r", str(max(int(fps), 1)),
        "-i", "-",
        "-c:v", "libx264", "-preset", "veryfast",
        "-b:v", str(br), "-maxrate", str(br), "-bufsize", str(br * 2),
        "-pix_fmt", "yuv420p",
        "-movflags", "+faststart",
        "-f", "mp4", str(path),
    ]


class _FfmpegWriter:
    """ffmpeg CLI tabanlı H.264/MP4 yazıcı (bgr24 rawvideo pipe).

    Arayüz cv2.VideoWriter ile aynıdır (write / isOpened / release) — ana
    döngü hangi yazıcının kullanıldığını bilmez. Encoder ölürse write()
    hatasız döner (log basar) ve isOpened() False olur → ana döngü durur.
    """

    def __init__(self, path, width, height, fps, bitrate, logger=None):
        self._log = logger or logging.getLogger(__name__)
        self._dead = False
        self._proc = subprocess.Popen(
            _ffmpeg_command(path, width, height, fps, bitrate),
            stdin=subprocess.PIPE,
        )
        self._log.info(
            "Video encoder: ffmpeg/libx264 (h264, yuv420p, faststart) → %s",
            Path(path).name,
        )

    def isOpened(self) -> bool:
        return not self._dead and self._proc.poll() is None

    def write(self, frame) -> None:
        if not self.isOpened():
            return
        try:
            self._proc.stdin.write(np.ascontiguousarray(frame).tobytes())
        except (BrokenPipeError, OSError) as exc:
            if not self._dead:
                self._dead = True
                self._log.warning("ffmpeg encoder kapandı (%s) — kayıt duruyor.", exc)

    def release(self) -> None:
        try:
            if self._proc.stdin is not None and not self._proc.stdin.closed:
                self._proc.stdin.close()
        except Exception:
            pass
        try:
            self._proc.wait(timeout=10)
        except Exception:
            try:
                self._proc.kill()
            except Exception:
                pass


def open_writer(
    path: str | os.PathLike,
    width: int,
    height: int,
    fps: int,
    bitrate: int = VISION_RECORDING_BITRATE,
    logger: logging.Logger | None = None,
) -> tuple[object, str]:
    """Video yazıcıyı açar — ÖNCELİK: ffmpeg CLI (garanti H.264).

    • ffmpeg varsa: _FfmpegWriter (libx264, yuv420p, faststart, gerçek bitrate)
      → tarayıcıda kesin oynar.
    • ffmpeg yoksa: cv2.VideoWriter fourcc zinciri (avc1 → h264 → x264 → mp4v).
      Sonuç 'mp4v' olursa tarayıcı oynatamaz — bu yalnızca son çare fallback'idir
      (Pi'de 'sudo apt install ffmpeg' ile çözülür).

    Dönen: (writer, etiket). Hiçbiri açılamazsa RuntimeError.
    """
    log = logger or logging.getLogger(__name__)
    if shutil.which("ffmpeg"):
        return _FfmpegWriter(path, width, height, fps, bitrate, log), "ffmpeg-h264"

    log.warning(
        "ffmpeg bulunamadı — cv2.VideoWriter fallback'i kullanılıyor; "
        "kodc 'mp4v' olursa tarayıcı oynatamaz. Öneri: sudo apt install ffmpeg"
    )
    last_err: Exception | None = None
    for tag in _VIDEO_FOURCC_ORDER:
        try:
            writer = cv2.VideoWriter(
                str(path),
                cv2.VideoWriter_fourcc(*tag),
                max(int(fps), 1),
                (int(width), int(height)),
            )
        except Exception as exc:  # cv2 hata fırlatabilir — sonraki adaya geç
            last_err = exc
            continue
        if writer.isOpened():
            log.info("Video encoder: %s (%dx%d@%d) → %s",
                     tag, width, height, fps, Path(path).name)
            return writer, tag
        writer.release()
    raise RuntimeError(
        f"video writer açılamadı (fourcc: {_VIDEO_FOURCC_ORDER}; son hata: {last_err})"
    )


# --------------------------------------------------------------------------- #
#  Ana döngü
# --------------------------------------------------------------------------- #

def main() -> int:
    logger = setup_logging()
    logger.info("VISION-ONLY RECORDER başlatılıyor.")
    logger.info("MAVLink YOK. Flight control YOK. Payload YOK.")

    try:
        with open(_PAYLOAD_CONFIG, encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
    except OSError as exc:
        logger.error("payload/config.yaml okunamadı: %s", exc)
        return 1
    logger.info("payload/config.yaml → backend=%s (bu script backend'den bağımsızdır)",
                cfg.get("backend"))

    # ── Kamera: RPi Camera Module 3 (REAL donanım, headless) ────────────────
    cam_cfg = cfg.get("camera", {}) or {}
    camera = RPiCam3(
        camera_num=RPI_CAMERA_NUM,
        width=CAMERA_WIDTH_PX,
        height=CAMERA_HEIGHT_PX,
        fps=REAL_CAMERA_FPS,
        preview=False,  # headless — pencere AÇILMAZ
        window_name="İHA Kamera",
        logger=logger,
        orientation=str(cam_cfg.get("orientation", CAMERA_ORIENTATION)),
    )
    camera = VisionCropCamera(camera, VISION_CROP_HEIGHT_PX)
    detector = TargetDetector()

    # ── Kayıt hedefi: recordings/vision_<timestamp>.mp4 ─────────────────────
    out_path = recording_path()
    writer, fourcc = open_writer(
        out_path,
        VISION_RECORDING_WIDTH_PX,
        int(round(VISION_HEIGHT_PX * VISION_RECORDING_WIDTH_PX / VISION_WIDTH_PX)),
        REAL_CAMERA_FPS,
        logger=logger,
    )

    logger.info(
        "Camera: %dx%d @ %d FPS | vision: %dx%d | "
        "output: %s | bitrate hedef: %.1f Mbps | codec: %s",
        CAMERA_WIDTH_PX, CAMERA_HEIGHT_PX, REAL_CAMERA_FPS,
        VISION_WIDTH_PX, VISION_HEIGHT_PX,
        out_path, VISION_RECORDING_BITRATE / 1e6, fourcc,
    )
    logger.info("Recording active — tespit: MAVİ ALTIGEN + KIRMIZI ÜÇGEN.")

    frame_count = 0
    status_t = time.monotonic()
    proc_accum = 0.0
    last_summary = "tespit yok"
    rec_start = time.monotonic()
    # HUD için canlı FPS (EMA) + son frame işleme süresi
    fps_ema = 0.0
    last_t = time.monotonic()
    last_proc_ms = 0.0

    try:
        while True:
            if not writer.isOpened():
                logger.error("Video encoder kapandı — kayıt durduruluyor.")
                break
            now = time.monotonic()
            dt = now - last_t
            last_t = now
            if dt > 0.0:
                fps_ema = fps_ema * 0.9 + (1.0 / dt) * 0.1   # yumuşatılmış FPS

            t_frame = time.monotonic()
            frame = camera.latest()
            if frame is None:
                time.sleep(0.02)
                continue

            disp, _detections, summary = process_frame(
                frame, detector, fps=fps_ema, proc_ms=last_proc_ms,
            )
            last_proc_ms = (time.monotonic() - t_frame) * 1000.0
            writer.write(disp)
            last_summary = summary
            proc_accum += (time.monotonic() - t_frame)
            frame_count += 1

            now = time.monotonic()
            if now - status_t >= 5.0:
                fps_display = frame_count / (now - status_t)
                avg_ms = proc_accum * 1000.0 / max(1, frame_count)
                logger.info(
                    "recording active: fps≈%.0f | frames=%d | ort işleme %.0f ms | %s",
                    fps_display, frame_count, avg_ms, last_summary,
                )
                frame_count = 0
                proc_accum = 0.0
                status_t = now
            time.sleep(0.005)
    except KeyboardInterrupt:
        logger.info("Durduruldu (Ctrl+C).")
    finally:
        try:
            writer.release()   # MP4 finalize (moov yazılır)
            dur = time.monotonic() - rec_start
            size_mb = out_path.stat().st_size / 1e6 if out_path.exists() else 0.0
            logger.info(
                "Kayıt tamam: %s | süre %.1f sn | %.1f MB | ~%.1f Mbps",
                out_path.name, dur, size_mb,
                size_mb * 8.0 / max(dur, 0.1),
            )
        except Exception as exc:
            logger.warning("Video finalize hatası: %s", exc)
        camera.close_preview()

    return 0


if __name__ == "__main__":
    sys.exit(main())
