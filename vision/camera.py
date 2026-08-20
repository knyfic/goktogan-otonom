import importlib
import threading
from typing import Optional, Protocol, runtime_checkable

import cv2
import numpy as np


@runtime_checkable
class CameraLike(Protocol):
    """
    State machine'in kameralardan beklediği minimum arayüz.

    GazeboCamera, RemoteRPiCamera ve RPiCam3 bu arayüzü sağlar —
    görüntü işleme / görev kodu hangi kameranın bağlı olduğunu bilmez.
    """

    def latest(self) -> Optional[np.ndarray]:
        """Son BGR frame'in kopyasını döndürür (None = henüz frame yok)."""
        ...

    def show(self, frame: np.ndarray) -> None:
        """Frame'i canlı pencerede gösterir."""
        ...

    def pump(self) -> None:
        """GUI event loop'unu besler (waitKey)."""
        ...

    def close_preview(self) -> None:
        """Kamerayı / pencereyi temiz kapatır."""
        ...


class VisionCropCamera:
    """
    Kameranın latest() çıktısını üst crop_height_px satırına kırpar ve pipeline
    için YARI çözünürlüğe küçültür (1920x704 → 960x352 — performans).

    Payload mekanizması kadrajın altını kapladığı için, görüntü işleme
    pipeline'ına mekanizmayı hiç görmeyen 960x352 bir frame gider.
    Kamera 1920x1080 üretmeye devam eder; FOV/geometri değişmez (içerik
    birebir aynı — yalnızca piksel sayısı 1/4, işleme ~4x ucuzlar).
    """

    def __init__(self, camera: CameraLike, crop_height_px: int, downscale: float = 0.5):
        self._camera = camera
        self._crop_height_px = crop_height_px
        self._downscale = downscale

    def latest(self) -> Optional[np.ndarray]:
        frame = self._camera.latest()
        if frame is None:
            return None
        frame = frame[: self._crop_height_px, :]
        if self._downscale < 1.0:
            nw = max(1, int(round(frame.shape[1] * self._downscale)))
            nh = max(1, int(round(frame.shape[0] * self._downscale)))
            frame = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_AREA)
        return frame

    def show(self, frame: np.ndarray) -> None:
        self._camera.show(frame)

    def pump(self) -> None:
        self._camera.pump()

    def close_preview(self) -> None:
        self._camera.close_preview()


class GazeboCamera:
    """
    Gazebo gz.msgs.Image subscriber.

    Kamera callback'i Gazebo'nun iç thread'inden gelir.
    OpenCV imshow/waitKey yalnızca ana thread'den çağrılabilir.
    Bu sınıf iki şeyi kesinlikle ayırır:
      - _callback  : sadece frame'i thread-safe şekilde depolar
      - show()     : ana thread'den çağrılır, pencereyi günceller
    """

    def __init__(
        self,
        topic: str,
        msgs_module: str = "gz.msgs",
        transport_module: str = "gz.transport",
        preview: bool = False,
        window_name: str = "İHA Kamera",
        orientation: str = "NORMAL",
    ):
        try:
            msgs = importlib.import_module(msgs_module)
            transport = importlib.import_module(transport_module)
        except ImportError as exc:
            raise RuntimeError(
                "Gazebo Python bindings bulunamadı. Jetty için "
                "python3-gz-transport15 ve python3-gz-msgs12 kurun."
            ) from exc

        self.topic = topic
        self._preview = preview
        self._window_name = window_name
        self._frame: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        # Orientation string from main; normalize image to vehicle coordinates here
        self._orientation = orientation

        # Pencereyi ana thread'de aç — 1920×1080'i tam göstermek büyük ekran ister,
        # yarım boyutta (960×540) gösterim yeterli ve taşınabilir.
        if self._preview:
            cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self._window_name, 1280, 720)

        self._image_type = importlib.import_module(f"{msgs_module}.image_pb2").Image
        self._node = transport.Node()

        if not self._node.subscribe(self._image_type, topic, self._callback):
            raise RuntimeError(f"Camera topic subscribe başarısız: {topic}")

    # ------------------------------------------------------------------ #

    def _callback(self, msg) -> None:
        """Gazebo thread'inden çağrılır — sadece frame'i depolar."""
        width = int(msg.width)
        height = int(msg.height)
        raw = bytes(msg.data)

        if width <= 0 or height <= 0:
            return

        channels = len(raw) // (width * height)
        if channels not in (1, 3, 4):
            return

        frame = np.frombuffer(raw, dtype=np.uint8).reshape(height, width, channels)

        if channels == 1:
            frame = np.repeat(frame, 3, axis=2)
        elif channels == 4:
            frame = frame[:, :, :3]

        # Gazebo RGB → OpenCV BGR
        frame = frame[:, :, ::-1].copy()

        # Kamera montajından kaynaklanan yön düzeltmesini uygula (pipeline başında)
        try:
            from vision.camera_utils import apply_camera_orientation
            frame = apply_camera_orientation(frame, getattr(self, '_orientation', 'NORMAL'))
        except Exception:
            # Eğer helper yoksa veya hata olursa ham frame'i kullan
            pass

        with self._lock:
            self._frame = frame

    def latest(self) -> Optional[np.ndarray]:
        """Son frame'in kopyasını döndürür. Ana thread'den çağrılabilir."""
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def show(self, frame: np.ndarray) -> None:
        """
        Verilen frame'i canlı pencerede gösterir.
        Mutlaka ana thread'den çağrılmalıdır.
        waitKey(1) ile GUI event loop'u beslenir; asla bloklamaz.
        """
        if not self._preview:
            return
        cv2.imshow(self._window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            # Kullanıcı 'q' ile pencereyi kapatmak istedi
            cv2.destroyWindow(self._window_name)
            self._preview = False

    def pump(self) -> None:
        """
        Kamera frame'i yokken bile GUI'nin donmaması için
        her döngüde çağrılabilir. Sadece waitKey(1) yapar.
        """
        if self._preview:
            cv2.waitKey(1)

    def close_preview(self) -> None:
        """Önizleme penceresini kapat."""
        if self._preview:
            cv2.destroyAllWindows()
            self._preview = False
