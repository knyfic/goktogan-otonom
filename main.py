"""
İHA Mission Ana Giriş Noktası.

Tüm mod/donanım seçimi payload/config.yaml üzerinden yapılır.
Bu dosyada magic string veya magic number yoktur.

  payload/config.yaml → backend: simulation   Gazebo + simüle payload
  payload/config.yaml → backend: hil          Gazebo + gerçek RPi GPIO (Wi-Fi)
  payload/config.yaml → backend: real         RPi üzerinde: Cube Orange MAVLink
                                              + gerçek kamera/payload + GStreamer
                                              (headless — yerel pencere açılmaz)
"""
import os
import sys

import yaml

from config import (
    CAMERA_HEIGHT_PX,
    CAMERA_TOPIC,
    CAMERA_WIDTH_PX,
    CAMERA_ORIENTATION,
    ENABLE_LIVE_PREVIEW,
    GAZEBO_MSGS_MODULE,
    GAZEBO_TRANSPORT_MODULE,
    MAVLINK_CONNECTION,
    PREVIEW_WINDOW_NAME,
    REAL_CAMERA_FPS,
    RPI_CAMERA_NUM,
)
from flight.navigation import Navigation
from payload.payload_controller import PayloadController
from state_machine import MissionStateMachine
from utils.logging import setup_logging

_PAYLOAD_CONFIG = os.path.join(os.path.dirname(__file__), "payload", "config.yaml")


def _load_payload_config() -> dict:
    with open(_PAYLOAD_CONFIG, encoding="utf-8") as f:
        return yaml.safe_load(f)


def _build_camera(mode: str, cfg: dict, logger):
    """
    Kamera nesnesini konfigürasyona göre oluştur.

    Kamera arayüzü (latest() / show() / pump() / close_preview()) HER
    senaryoda aynıdır — state machine / görüntü işleme kodu HİÇBİR
    DEĞİŞİKLİK GÖRMEZ.

    Senaryo seçimi:
      simulation             → GazeboCamera  (simülasyon topic'ten)
      hil + hil_camera=gazebo     → GazeboCamera (geleneksel HIL)
      hil + hil_camera=remote_rpi → RemoteRPiCamera (GERÇEK RPi Camera Module 3,
                                   Wi-Fi/TCP üzerinden — mekanik hala Gazebo)
      real                   → RPiCam3   (vision/rpi_camera.py — CSI Camera Module 3)
    """
    # ── Simulation — her zaman Gazebo ──────────────────────────────────────
    if mode == "simulation":
        from vision.camera import GazeboCamera
        return GazeboCamera(
            topic=CAMERA_TOPIC,
            msgs_module=GAZEBO_MSGS_MODULE,
            transport_module=GAZEBO_TRANSPORT_MODULE,
            preview=ENABLE_LIVE_PREVIEW,
            window_name=PREVIEW_WINDOW_NAME,
            orientation=CAMERA_ORIENTATION,
        )

    # ── HIL — konfigürasyona göre Gazebo ya da GERÇEK RPi Kamera ───────────
    if mode == "hil":
        hil_cam = str(cfg.get("hil_camera", "gazebo")).lower()
        logger.info("HIL kamera kaynağı seçimi: hil_camera=%s", hil_cam)

        if hil_cam == "gazebo":
            from vision.camera import GazeboCamera
            return GazeboCamera(
                topic=CAMERA_TOPIC,
                msgs_module=GAZEBO_MSGS_MODULE,
                transport_module=GAZEBO_TRANSPORT_MODULE,
                preview=ENABLE_LIVE_PREVIEW,
                window_name=PREVIEW_WINDOW_NAME,
                orientation=CAMERA_ORIENTATION,
            )

        if hil_cam == "remote_rpi":
            from vision.remote_rpi_camera import RemoteRPiCamera
            # Kamera IP'si: payload ile aynı RPi (remote.host — TEK kaynak).
            # Port kamera servisine özeldir (remote_camera.port → 5701).
            remote = cfg.get("remote", {})
            rc = cfg.get("remote_camera", {})
            host = remote.get("host")
            port = int(rc.get("port", 5701))
            logger.info(
                "HIL → GERÇEK Raspberry Pi Camera Module 3 akışı: "
                "TCP üzerinden %s:%d bekleniyor...",
                host, port,
            )
            return RemoteRPiCamera(
                host=host,
                port=port,
                connect_timeout_s=float(rc.get("connect_timeout_s", 3.0)),
                read_timeout_s=float(rc.get("read_timeout_s", 2.0)),
                reconnect_wait_s=float(rc.get("reconnect_wait_s", 1.0)),
                preview=ENABLE_LIVE_PREVIEW,
                window_name=PREVIEW_WINDOW_NAME,
                logger=logger,
                # Yön döndürmenin sahibi RPi kamera sunucusudur (config.yaml →
                # camera.orientation — ROTATE_180). Ubuntu burada TEKRAR döndürmez.
                orientation="NORMAL",
            )

        raise ValueError(
            f"Bilinmeyen HIL kamera türü hil_camera={hil_cam!r}. "
            f"Kabul edilen: 'gazebo', 'remote_rpi'."
        )

    # ── Real — Raspberry Pi üzerinde doğrudan donanım ──────────────────────
    # TEK gerçek kamera implementasyonu: vision/rpi_camera.py (RPiCam3).
    # HIL kamera sunucusu (services/camera_server.py) ile BİREBİR aynı sınıf.
    #
    # REAL modda GUI/pencere AÇILMAZ (headless uçuş — monitör yok):
    #   preview=False → cv2.imshow/namedWindow asla çağrılmaz.
    # Görüntü, state machine'in show() akışı üzerinden GStreamer ile ground
    # station'a yayınlanır (services/video_streamer.py → StreamCamera).
    #
    # ORIENTATION TEK KAYNAK: payload/config.yaml → camera.orientation
    # (fiziksel RPi kamera montajı — HIL sunucu ile aynı değer; doğrulanmış
    # değer 'ROTATE_180'). Bölüm yoksa geriye dönük uyumluluk için
    # config.py → CAMERA_ORIENTATION kullanılır.
    if mode == "real":
        from vision.rpi_camera import RPiCam3
        cam_cfg = cfg.get("camera", {}) or {}
        return RPiCam3(
            camera_num=RPI_CAMERA_NUM,      # CSI port (0 = CAM0) — config.py TEK kaynak
            width=CAMERA_WIDTH_PX,          # config.py'den — FOV ile tutarlı
            height=CAMERA_HEIGHT_PX,        # config.py'den
            fps=REAL_CAMERA_FPS,            # config.py'den
            preview=False,                  # REAL: headless — yerel pencere YOK
            window_name=PREVIEW_WINDOW_NAME,
            logger=logger,
            orientation=str(cam_cfg.get("orientation", CAMERA_ORIENTATION)),
        )

    raise ValueError(f"Bilinmeyen mod için kamera oluşturulamadı: {mode!r}")


def _build_navigation(mode: str, cfg: dict, logger):
    """
    Navigation nesnesini config'e göre kurar.

      simulation / hil  → MAVLINK_CONNECTION (config.py — Gazebo/SITL UDP, Ubuntu)
      real              → payload/config.yaml → flight_controller.connection
                          (Cube Orange TELEM2 → Raspberry Pi GPIO UART: serial + baud)

    connection.device tanımlı değilse (eski config) MAVLINK_CONNECTION'a düşer.
    """
    if mode == "real":
        fc = cfg.get("flight_controller", {}) or {}
        conn = fc.get("connection", {}) or {}
        device = conn.get("device")
        baud = conn.get("baudrate")
        if device:
            logger.info(
                "REAL → Cube Orange MAVLink: device=%s baud=%s", device, baud,
            )
            return Navigation(device, baud=baud)
        logger.warning(
            "flight_controller.connection.device tanımlı değil — "
            "MAVLINK_CONNECTION fallback: %s", MAVLINK_CONNECTION,
        )
    return Navigation(MAVLINK_CONNECTION)


def _build_backend(mode: str, cfg: dict, logger):
    """
    Payload backend'ini config'e göre oluşturur.
    Tüm parametreler config.yaml'dan okunur — kod içinde IP/pin yoktur.
    """
    logger.info("Payload backend: %s  (kaynak: payload/config.yaml)", mode)

    if mode == "simulation":
        from payload.backends.simulation import SimulationBackend
        return SimulationBackend(logger=logger)

    if mode == "hil":
        from payload.backends.remote_rpi import RemoteRPiBackend
        remote = cfg.get("remote", {})
        return RemoteRPiBackend(
            host=remote["host"],
            port=remote["port"],
            timeout=remote.get("timeout_s", 10.0),
            logger=logger,
        )

    if mode == "real":
        # payload/hardware/ altındaki lgpio sürücüleri kullanılır.
        # Bu kod Raspberry Pi 5 üzerinde çalışır.
        from payload.backends.local_rpi import LocalRPiBackend
        return LocalRPiBackend.from_config(
            config_path=_PAYLOAD_CONFIG,
            logger=logger,
        )

    raise ValueError(
        f"Bilinmeyen payload backend: {mode!r}. "
        f"payload/config.yaml → 'backend' değerini "
        f"'simulation', 'hil' veya 'real' yapın."
    )


def main():
    logger = setup_logging()
    logger.info("İHA mission başlatılıyor.")

    camera = None
    streamer = None
    try:
        payload_cfg = _load_payload_config()
        mode = payload_cfg.get("backend", "simulation")

        navigation = _build_navigation(mode, payload_cfg, logger)
        camera     = _build_camera(mode, payload_cfg, logger)
        backend    = _build_backend(mode, payload_cfg, logger)
        # Renk haritası: hangi renk payload hangi mıknatısta
        color_map  = payload_cfg.get("payload_color_map", {})
        payload    = PayloadController(backend=backend, logger=logger,
                                       color_map=color_map or None)

        # ── REAL: GStreamer video akışı (ground station'a) ─────────────────
        # StreamCamera, state machine'in kamera arayüzünü sarmalar:
        #   show(frame) → streamer.push(frame)   (960x540 HUD frame)
        #   pump()      → no-op
        #   close_preview() → streamer.stop() + kamera kapanışı
        # GStreamer başlatılamazsa / çökerse görev ASLA durmaz (bağımsız servis).
        if mode == "real":
            vs = payload_cfg.get("video_stream", {}) or {}
            if vs.get("enabled", False):
                from services.video_streamer import StreamCamera, VideoStreamer
                streamer = VideoStreamer(
                    host=str(vs.get("host", "127.0.0.1")),
                    port=int(vs.get("port", 5000)),
                    bitrate=int(vs.get("bitrate", 2_000_000)),
                    codec=str(vs.get("codec", "h264")),
                    fps=int(vs.get("fps", 30)),
                    preset=str(vs.get("preset", "veryfast")),
                    logger=logger,
                )
                streamer.start()      # başarısızsa enabled=False — uçuş devam
                camera = StreamCamera(camera, streamer, logger=logger)
                logger.info(
                    "GStreamer video akışı yapılandırıldı: %s:%d",
                    vs.get("host"), vs.get("port"),
                )

        # ── Heartbeat watchdog — yalnızca REAL mod (sim/hil'de kapalı) ─────
        # İkincil izleyici: birincil güvenlik Cube Orange'ın kendi ArduPilot
        # failsafe ayarlarıdır (dokümantasyona bakın).
        heartbeat_timeout = None
        if mode == "real":
            fc = payload_cfg.get("flight_controller", {}) or {}
            hb = fc.get("heartbeat_timeout_s")
            heartbeat_timeout = float(hb) if hb else None
            if heartbeat_timeout is not None:
                logger.info(
                    "MAVLink heartbeat watchdog aktif: %.1f sn", heartbeat_timeout,
                )

        mission = MissionStateMachine(
            navigation=navigation,
            camera=camera,
            payload=payload,
            logger=logger,
            heartbeat_timeout_s=heartbeat_timeout,
        )
        mission.run()

    except KeyboardInterrupt:
        logger.warning("Keyboard interrupt.")
        return 130
    except Exception:
        logger.exception("Mission terminated by exception.")
        return 1
    finally:
        if camera is not None:
            camera.close_preview()
        if streamer is not None:
            try:
                streamer.stop()
            except Exception:
                pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
