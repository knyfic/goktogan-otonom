"""
Görev State Machine

Görev 1: İki hedefi (mavi altıgen + kırmızı üçgen) bul, GPS kaydet, yük bırak.
Görev 2: İlk hedefin GPS'ine dön, bırakılan yükü bul, hizala, al, doğrula.
Görev 3: RTL.

Tasarım kuralları:
- run() döngüsü sadece _tick() çağırır ve GUI'yi besler.
- _pump_preview() YALNIZCA run() içinden, _tick() SONRASINDA çağrılır.
- Hiçbir state handler kendi içinde camera.show() çağırmaz;
  bunun yerine self._display_frame / self._display_overlay set eder.
  _pump_preview() bu frame'i gösterir.
- Non-blocking: hiçbir state handler uzun süre bloklamaz.
  Kalkış ve RTL gibi zorunlu bekleme gerektiren durumlar
  kendi mini döngüsünde _pump_preview() çağırır.
"""

import logging
import statistics
import time

import cv2
import numpy as np

from config import (
    ALIGN_CMD_INTERVAL_S,
    ALIGN_SPEED_MPS,
    ALIGN_ON_TARGET_TIMEOUT_S,
    ALTITUDE_TOLERANCE_M,
    APPROACH_SPEED_MPS,
    CAMERA_HEIGHT_PX,
    CAMERA_HORIZONTAL_FOV_DEG,
    CAMERA_WIDTH_PX,
    CAMERA_VERTICAL_FOV_DEG,
    COMPETITION_CORNERS,
    ALIGNMENT_DESCENT_STEP_M,
    ALIGNED_FRAMES_REQUIRED,
    CAMERA_TO_MECHANISM_OFFSET_M,
    DROP_OFFSET_M,
    MISSION_TIMEOUT_S,
    MAX_PICKUP_ATTEMPTS,
    MAX_PAYLOAD_SIZE_FACTOR,
    OFFSET_CORRECTION_WAIT_S,
    PAYLOAD_ALIGN_TIMEOUT_S,
    PAYLOAD_ALIGN_ALTITUDE_M,
    PAYLOAD_CONTACT_ALTITUDE,
    PAYLOAD_CORNER_PICK_ALT_M,
    PAYLOAD_DESCENT_STEP_M,
    PAYLOAD_HEIGHT,
    PAYLOAD_PICKUP_VISION_MIN_S,
    PAYLOAD_PIXEL_TOLERANCE_PX,
    PIXEL_ALIGNMENT_TOLERANCE_PX,
    PAYLOAD_DROP_ALIGN_TIMEOUT_S,
    PAYLOAD_DROP_ALTITUDE,
    PAYLOAD_RELEASE_ALTITUDE,
    PAYLOAD_SEARCH_ALTITUDE,
    PAYLOAD_SEARCH_TIMEOUT,
    PAYLOAD_SERVO_WAIT_S,
    PAYLOAD_VERIFY_ALTITUDE,
    PAYLOAD_WIDTH,
    PREVIEW_DISPLAY_WIDTH_PX,
    RTL_ARRIVAL_TIMEOUT_S,
    RTL_DISARM_TIMEOUT_S,
    RTL_PREVIEW_HZ,
    SCAN_LINE_SPACING,
    SEARCH_MARGIN,
    SEARCH_SPEED_MPS,
    STATE_TIMEOUT,
    TARGET_APPROACH_ALTITUDE,
    TAKEOFF_ALTITUDE,
    TAKEOFF_TIMEOUT,
    VERIFY_ALT_TOLERANCE,
    VERIFY_STABILIZE_S,
    VERIFY_BAND_LOW,
    VERIFY_BAND_HIGH,
    VERIFY_FALLBACK_CONFIDENCE,
    VERTICAL_SPEED_MPS,
)
from flight.navigation import Navigation, haversine_m
from flight.position_controller import PositionController
from flight.rtl import RTLController
from models import (
    Detection,
    MissionData,
    MissionState,
    Target,
    TargetColor,
    TargetType,
)
from payload.payload_controller import PayloadController
from search.coverage_planner import (
    CompetitionField,
    LawnmowerPlanner,
    SearchPoint,
    reorder_search_from_drone,
)
from vision.camera import CameraLike
from vision.geometry import CameraModel, expected_payload_pixels
from vision.payload_detector import PayloadDetector
from vision.payload_tracker import PayloadTracker
from vision.target_detector import TargetDetector

# Referans area için kaç frame toplanacağı (median hesabı için)
_REF_SAMPLE_COUNT = 5

# Verification'da oy çokluğu için kaç frame toplanacağı
_VERIFY_VOTE_COUNT = 6


class MissionStateMachine:
    def __init__(
        self,
        navigation: Navigation,
        camera: CameraLike,
        payload: PayloadController,
        logger: logging.Logger,
        heartbeat_timeout_s: float | None = None,
    ):
        # MAVLink heartbeat watchdog — yalnızca REAL modda main.py bağlar.
        # None → devre dışı (simulation/hil davranışı değişmez).
        self._heartbeat_timeout_s = heartbeat_timeout_s
        self._heartbeat_triggered = False
        # pending nav target used when we first ascend to search altitude
        self._nav_pending_target: tuple[float, float] | None = None
        self.log = logger
        self.nav = navigation
        self.camera = camera
        self.payload = payload

        self.camera_model = CameraModel(
            CAMERA_WIDTH_PX,
            CAMERA_HEIGHT_PX,
            CAMERA_HORIZONTAL_FOV_DEG,
            CAMERA_VERTICAL_FOV_DEG,
        )
        self.target_detector = TargetDetector()
        self.payload_detector = PayloadDetector(self.camera_model)
        self.payload_tracker  = PayloadTracker()
        self.position_controller = PositionController(navigation, self.camera_model)
        self.rtl_controller = RTLController(navigation)

        self.data = MissionData()
        # Görev, ArduPilot pre-arm kontrolleri geçmeden BAŞLAMAZ. Bu state'te
        # kamera/vision/HUD/GStreamer canlı çalışır; yalnızca uçuş görevleri
        # (arm/takeoff/nav) kilitlidir. Pre-arm GOOD → INIT → mevcut akış.
        self.data.state = MissionState.WAITING_FOR_PREARM
        self._prearm_reason: str | None = None   # HUD'da gösterilecek son neden

        # ---- Lawnmower tarama ----
        self._search_points: list = []
        self._search_idx: int = 0
        self._search_count: int = 0   # 1. / 2. search loglarını ayırt etmek için
        # Yarışma alanı frame'i — _init_search'te COMPETITION_CORNERS'ten kurulur.
        self._search_field: CompetitionField | None = None
        self._search_origin_lat: float | None = None
        self._search_origin_lon: float | None = None

        # ---- Non-blocking irtifa bekleme ----
        self._alt_target: float | None = None
        self._alt_deadline: float | None = None
        self._alt_waiting: bool = False

        # ---- Hedef hizalama fazı ----
        self._align_phase: str = "HORIZONTAL"  # "HORIZONTAL" | "DESCENDING" | "DROPPING" | "DROP_ALIGN"
        self._last_align_status_log_t: float = 0.0  # periyodik durum logu
        # 1m'de opsiyonel ek hizalama (PAYLOAD_DROP_ALIGN_TIMEOUT_S) bitiş zamanı
        self._drop_align_deadline: float | None = None

        # ---- Hedef ilk görüldüğünde kaydedilen kaba GPS ----
        # Hedef kadrajdan çıksa bile bu konuma dönebiliriz.
        self._target_rough_lat: float | None = None
        self._target_rough_lon: float | None = None

        # ---- Align hız sınırlayıcı ----
        # Her step'te goto_body_offset göndermemek için son komut zamanı.
        self._last_align_cmd_t: float = 0.0

        # ---- Recover throttle: aynı konuma sürekli goto gönderme ----
        self._last_recover_t: float = 0.0
        self._RECOVER_INTERVAL: float = 1.5  # saniye

        # ---- Tespit kayıp sayacı: kaç tick üst üste tespit yok ----
        self._no_detection_count: int = 0
        self._NO_DETECTION_LIMIT: int = 25  # ~0.5s @ 50Hz — komut işleme süresi için tolerans

        # ---- Hedef kilidi: ilk kararlı tespitte belirlenir ----
        # Görev boyunca aynı türe odaklanmak için kullanılır.
        # İki hedef aynı anda görünürse ilk kararlı tespit kazanır.
        self._locked_color: TargetColor | None = None
        self._locked_shape: str | None = None

        # ---- GPS kalite biriktirici ----
        # Hizalama döngüsünde "aligned=True" her anında GPS + confidence kaydedilir.
        # TARGET_ALIGNED → _do_save_gps en yüksek confidence'lı koordinatı alır.
        # (lat, lon, confidence) tuple listesi
        self._gps_candidates: list[tuple[float, float, float]] = []

        # ---- Payload pickup servo bekleme ----
        # _do_pickup servo komutunu gönderince bu zaman damgası set edilir.
        self._pickup_servo_start: float | None = None
        self._pickup_servo_wait:  float        = PAYLOAD_SERVO_WAIT_S

        # ---- Payload hassas iniş state'i ----
        # _payload_descent_alt: bir sonraki iniş hedefi; None = henüz başlamadı
        self._payload_descent_alt: float | None = None
        # Her HORIZONTAL fazının başlangıç zamanı — timeout için
        self._align_phase_start_t: float | None = None
        # Ardışık "hizalı" frame sayacı — tek frame'de merkezde görünmek
        # (jitter / tesadüf) hizalı sayılmasın; ALIGNED_FRAMES_REQUIRED ardışık
        # onay gereklidir. Hizasız frame sayaçı sıfırlar.
        self._aligned_frame_count: int = 0

        # ---- SEARCHING_PAYLOAD timeout ----
        # Payload bulunamazsa bu süre sonunda yukarı çıkıp yeniden merkezle.
        # Geçici frame kayıplarında (1-2 tick) tetiklenmez; sadece timeout dolunca.
        self._payload_search_start: float | None = None
        self._offset_correction_start: float | None = None
        self._offset_phase: str | None = None

        # ---- Hedef üzerinde hizalama (pickup öncesi) ----
        # ALIGNING_ON_TARGET_FOR_PICKUP state'inde kullanılır
        self._align_for_pickup_deadline: float | None = None
        # ALIGNING_ON_TARGET_FOR_PICKUP fazının başlangıç zamanı — minimum
        # görüntü işleme süresi (PAYLOAD_PICKUP_VISION_MIN_S) için
        self._align_for_pickup_start: float | None = None

        # ---- Gösterilecek frame (sadece _pump_preview yazar ekrana) ----
        # State handler'lar _pending_display'e atar; _pump_preview gösterir.
        self._pending_display: np.ndarray | None = None

        # ---- Payload 1m referans area (pre-pickup) ----
        # ALIGNING_PAYLOAD DESCENDING fazında irtifa PAYLOAD_VERIFY_ALTITUDE'a
        # (±tolerans) ilk indiğinde birkaç frame'den median area kaydedilir.
        # VERIFYING_PAYLOAD sırasında bu değerle karşılaştırma yapılır.
        self._ref_area_samples: list[float] = []   # ham sample'lar (mediana alınır)
        self._ref_area_recorded: bool = False       # referans tamamlandı mı?
        self._ref_area_px: float | None = None      # final referans değeri
        self._verify_votes: list[str] = []          # verification oy listesi
        self._verify_stabilize_start: float | None = None  # stabilizasyon sayacı

        # ---- Görev zamanlayıcısı ----
        # Kalkış tamamlanınca set edilir. None → görev henüz başlamadı.
        # time.monotonic() tabanlı — sistem saatinden bağımsız.
        self._mission_start_t: float | None = None
        # Timeout tetiklendikten sonra tekrar RTL gönderilmesini engeller.
        self._mission_timeout_triggered: bool = False

    # ==================================================================== #
    #  Ana döngü
    # ==================================================================== #

    def run(self) -> None:
        while self.data.state != MissionState.COMPLETE:
            self._pending_display = None          # her tick öncesi temizle
            self._tick()
            self._pump_preview()                  # tek nokta: ekranı güncelle
            time.sleep(0.02)                      # ~50 Hz

    def _tick(self) -> None:
        # ── MAVLink heartbeat watchdog (yalnızca REAL mod) ────────────────
        # Bağlantı koparsa görevin YENİ adımları engellenir ve mevcut RTL
        # abstraction'ı (rtl_controller) üzerinden dönüş başlatılır.
        # RTL/COMPLETE'te çalışmaz — Cube Orange zaten kendi ArduPilot
        # failsafe'ini yürütür (birincil güvenlik katmanı).
        # Not: Bağlantı gerçekten kopmuşsa nav.rtl() komutunun ulaşacağı
        # garanti edilemez; bu durumda FC'nin kendi failsafe ayarları devrededir.
        if (
            self._heartbeat_timeout_s is not None
            and not self._heartbeat_triggered
            and self.data.state not in (MissionState.RTL, MissionState.COMPLETE)
        ):
            if not self.nav.heartbeat_ok(self._heartbeat_timeout_s):
                self._heartbeat_triggered = True
                self.log.error(
                    "MAVLINK BAĞLANTI KAYBI: %.1f sn içinde telemetri alınamadı. "
                    "Yeni görev adımları durduruluyor → RTL başlatılıyor.",
                    self.nav.last_message_age_s,
                )
                self.data.mission_failed = True
                self._do_start_rtl()
                return

        # ── Görev zamanlayıcısı watchdog ─────────────────────────────────
        # Her tick'te kontrol edilir. Timer başlamışsa ve süre dolmuşsa
        # hangi state'te olursa olsun RTL'ye geçilir.
        # INIT, TAKEOFF, RTL ve COMPLETE state'lerinde çalışmaz.
        if (
            self._mission_start_t is not None
            and not self._mission_timeout_triggered
            and self.data.state not in (
                MissionState.INIT,
                MissionState.TAKEOFF,
                MissionState.RTL,
                MissionState.COMPLETE,
            )
        ):
            elapsed = time.monotonic() - self._mission_start_t
            if elapsed >= MISSION_TIMEOUT_S:
                self._mission_timeout_triggered = True
                mm = int(elapsed) // 60
                ss = int(elapsed) % 60
                self.log.warning(
                    "GÖREV ZAMANAŞIMI: %d:%02d geçti (sınır=%d:%02d). "
                    "Mevcut state=%s → RTL başlatılıyor.",
                    mm, ss,
                    MISSION_TIMEOUT_S // 60, MISSION_TIMEOUT_S % 60,
                    self.data.state.value,
                )
                self.data.mission_failed = True
                self._do_start_rtl()
                return

        s = self.data.state
        if s == MissionState.WAITING_FOR_PREARM:
            self._do_wait_prearm()
        elif s == MissionState.INIT:
            self._do_init()
        elif s == MissionState.TAKEOFF:
            self._do_takeoff()
        elif s in (MissionState.SEARCHING_FIRST_TARGET,
                   MissionState.SEARCHING_SECOND_TARGET):
            self._do_search_target()
        elif s == MissionState.TARGET_DETECTED:
            self._do_classify()
        elif s == MissionState.TARGET_CLASSIFIED:
            self._goto(MissionState.APPROACHING_TARGET)
        elif s == MissionState.APPROACHING_TARGET:
            self._do_approach()
        elif s == MissionState.ALIGNING_TARGET:
            self._do_align_target()
        elif s == MissionState.TARGET_ALIGNED:
            self._do_save_gps()
        elif s == MissionState.RELEASING_PAYLOAD:
            self._do_release()
        elif s == MissionState.ASCENDING_AFTER_FIRST_RELEASE:
            self._do_ascend_after_first_release()
        elif s == MissionState.NAVIGATING_TO_FIRST_TARGET:
            self._do_navigate_to_first()
        elif s == MissionState.ALIGNING_ON_TARGET_FOR_PICKUP:
            self._do_align_on_target_for_pickup()
        elif s == MissionState.SEARCHING_PAYLOAD:
            self._do_search_payload()
        elif s == MissionState.ASCENDING_FOR_PAYLOAD_RETRY:
            self._do_ascend_for_payload_retry()
        elif s == MissionState.ALIGNING_PAYLOAD:
            self._do_align_payload()
        elif s == MissionState.OFFSET_CORRECTION:
            self._do_offset_correction()
        elif s == MissionState.PICKING_UP_PAYLOAD:
            self._do_pickup()
        elif s == MissionState.ASCENDING_FOR_VERIFICATION:
            self._do_ascend_for_verification()
        elif s == MissionState.VERIFYING_PAYLOAD:
            self._do_verify()
        elif s == MissionState.TASK_2_COMPLETE:
            self._do_after_task2()
        elif s == MissionState.RETURNING_PAYLOAD:
            self._do_return_payload()
        elif s == MissionState.RELEASING_RETURNED_PAYLOAD:
            self._do_release_returned_payload()
        elif s == MissionState.TASK_2_FAILED:
            self._do_start_rtl()
        elif s == MissionState.RTL:
            self._do_wait_rtl()
        else:
            raise RuntimeError(f"Unhandled state: {s}")

    # ==================================================================== #
    #  Görüntü gösterme — TEK NOKTA
    # ==================================================================== #

    def _pump_preview(self) -> None:
        """
        run() döngüsünde _tick() SONRASI bir kez çağrılır.
        _pending_display set edildiyse onu gösterir.
        Set edilmediyse son kamera frame'ini ham gösterir.
        Hiçbir state handler doğrudan camera.show() çağırmaz.
        """
        if self._pending_display is not None:
            disp = self._scale_for_display(self._pending_display)
        else:
            frame = self.camera.latest()
            if frame is None:
                self.camera.pump()
                return
            # Önce küçült, sonra HUD çiz — imshow + çizim maliyeti ~4x düşer.
            disp = self._base_overlay(self._scale_for_display(frame))

        # Timeout banner — timeout olunca her frame'de göster
        if self._mission_timeout_triggered:
            disp = self._draw_timeout_banner(disp)

        self.camera.show(disp)

    # ------------------------------------------------------------------ #
    @staticmethod
    def _scale_for_display(frame: np.ndarray) -> np.ndarray:
        """
        Yalnızca PENCERE gösterimi için frame'i küçültür.
        Görüntü işleme/tespit TAM çözünürlükte yapılır — bu fonksiyon
        yalnızca gösterim yolunda çağrılır, hiçbir detektörü etkilemez.
        """
        h, w = frame.shape[:2]
        if w <= PREVIEW_DISPLAY_WIDTH_PX:
            return frame
        nw = PREVIEW_DISPLAY_WIDTH_PX
        nh = max(1, int(round(h * nw / w)))
        return cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_AREA)

    # ------------------------------------------------------------------ #
    @staticmethod
    def _draw_hud(
        frame: np.ndarray,
        line1: str,
        line2: str = "",
        line1_color: tuple = (0, 230, 0),
        line2_color: tuple = (0, 210, 255),
    ) -> np.ndarray:
        """
        Şeffaf HUD bandı çiz.
        Üst banda iki satır bilgi yaz — bant %55 opak, görüntü arkadan okunuyor.
        Font ve marker boyutları frame çözünürlüğüne göre otomatik ölçeklenir.
        """
        out = frame.copy()
        h, w = out.shape[:2]

        # Çözünürlüğe bağlı ölçek faktörü.
        # 640'ı baz alarak 1920px'de ~3x, 1280px'de ~2x ölçek verir.
        ui_scale = w / 640.0

        band_h = int(56 * ui_scale)

        # Yarı saydam bant
        overlay = out.copy()
        cv2.rectangle(overlay, (0, 0), (w, band_h), (10, 10, 10), -1)
        cv2.addWeighted(overlay, 0.55, out, 0.45, 0, out)

        # İnce ayırıcı çizgi
        cv2.line(out, (0, band_h), (w, band_h), (60, 60, 60), 1)

        font_scale1 = 0.65 * ui_scale
        font_scale2 = 0.58 * ui_scale
        thickness1  = max(1, int(2 * ui_scale))
        thickness2  = max(1, int(1 * ui_scale))
        y1 = int(22 * ui_scale)
        y2 = int(46 * ui_scale)

        cv2.putText(out, line1,
                    (10, y1), cv2.FONT_HERSHEY_SIMPLEX, font_scale1, line1_color,
                    thickness1, cv2.LINE_AA)
        if line2:
            cv2.putText(out, line2,
                        (10, y2), cv2.FONT_HERSHEY_SIMPLEX, font_scale2, line2_color,
                        thickness2, cv2.LINE_AA)

        # Merkez artı işareti
        cx, cy = w // 2, h // 2
        marker_size = int(24 * ui_scale)
        cv2.drawMarker(out, (cx, cy), (255, 255, 255),
                       cv2.MARKER_CROSS, marker_size, max(1, int(ui_scale)), cv2.LINE_AA)
        return out

    @staticmethod
    def _draw_timeout_banner(frame: np.ndarray) -> np.ndarray:
        """
        Timeout gerçekleştikten sonra her frame'e kırmızı bir banner çizer.
        Boyutlar çözünürlüğe göre otomatik ölçeklenir.
        """
        out = frame.copy()
        h, w = out.shape[:2]
        ui_scale = w / 640.0
        bh = int(38 * ui_scale)
        by = h // 2 - bh - int(10 * ui_scale)
        x0, x1 = w // 4, 3 * w // 4
        overlay = out.copy()
        cv2.rectangle(overlay, (x0, by), (x1, by + bh), (0, 0, 180), -1)
        cv2.addWeighted(overlay, 0.75, out, 0.25, 0, out)
        cv2.putText(
            out, "MISSION TIMEOUT - RTL",
            (x0 + int(16 * ui_scale), by + int(26 * ui_scale)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.80 * ui_scale,
            (255, 255, 255), max(1, int(2 * ui_scale)), cv2.LINE_AA,
        )
        return out

    def _mission_timer_str(self) -> str:
        """
        Geçen görev süresini MM:SS formatında döndür.
        Timer başlamadıysa '--:--', timeout olduysa '11:00 TIMEOUT' gösterir.
        """
        if self._mission_start_t is None:
            return "--:--"
        elapsed = int(time.monotonic() - self._mission_start_t)
        elapsed = min(elapsed, MISSION_TIMEOUT_S)   # 11:00'dan yukarı taşmaz
        mm = elapsed // 60
        ss = elapsed % 60
        base = f"{mm:02d}:{ss:02d}"
        if self._mission_timeout_triggered:
            return f"{base} TIMEOUT"
        # Son 60 saniyede kırmızı uyarı için string'e işaret ekle (renk _draw_hud'da)
        if elapsed >= MISSION_TIMEOUT_S - 60:
            return f"{base} !"
        return base

    def _hud_lines(self, extra: str = "") -> tuple[str, str]:
        """Standart HUD satırları: STATE + irtifa / heading / batarya / GPS."""
        pos = self.nav.position()
        line1 = (
            f"{self.data.state.value}"
            + (f"  |  {extra}" if extra else "")
        )
        # Batarya alanları her kaynakta olmayabilir (mock/test/yaşlı telemetri) —
        # güvenli varsayılanla düş.
        try:
            battery_v = float(pos.battery_voltage_v)
        except (TypeError, ValueError):
            battery_v = 0.0
        try:
            battery_pct = int(pos.battery_percent)
        except (TypeError, ValueError):
            battery_pct = -1

        if battery_v > 0:
            bat_str = f"Bat {battery_v:.1f}V"
        else:
            bat_str = "Bat --.-V"
        if battery_pct >= 0:
            bat_str += f" {battery_pct}%"
        line2 = (
            f"Alt {pos.relative_altitude:.1f}m  "
            f"Hdg {pos.heading_deg:.0f}°  "
            f"{bat_str}  "
            f"({pos.latitude:.5f}, {pos.longitude:.5f})"
            f"  T:{self._mission_timer_str()}"
        )
        pre_str = self._prearm_hud_str()
        if pre_str:
            line2 = f"{pre_str}  |  {line2}"
        return line1, line2

    def _prearm_hud_str(self) -> str:
        """Pre-arm bekleme state'inde HUD satırı; diğer state'lerde boş.

        Örn: 'PRE-ARM: WAITING | GPS 1: Bad fix'  veya  'PRE-ARM: GOOD'.
        """
        state = getattr(getattr(self, "data", None), "state", None)
        if not (isinstance(state, MissionState)
                and state == MissionState.WAITING_FOR_PREARM):
            return ""
        try:
            good = self.nav.prearm_good()
        except Exception:
            good = None
        if good is True:
            return "PRE-ARM: GOOD"
        reason = getattr(self, "_prearm_reason", None) or ""
        base = "PRE-ARM: WAITING"
        return f"{base} | {reason}" if reason else base

    def _base_overlay(self, frame: np.ndarray) -> np.ndarray:
        l1, l2 = self._hud_lines()
        return self._draw_hud(frame, l1, l2)

    def _set_display_detections(self, frame: np.ndarray, detections: list) -> None:
        """Tespit overlay hazırla ve _pending_display'e koy."""
        disp = TargetDetector.draw_detections(frame, detections)
        det_str = "  ".join(
            f"{d.color.value[0]}{d.shape[0]} {d.confidence:.2f}"
            for d in detections[:3]
        ) if detections else "tespit yok"
        l1, l2 = self._hud_lines(extra=det_str)
        self._pending_display = self._draw_hud(disp, l1, l2)

    def _set_display_payload(self, frame: np.ndarray, results: list, label: str) -> None:
        """Payload overlay hazırla."""
        disp = frame.copy()
        h, w = disp.shape[:2]
        # Çözünürlüğe bağlı ölçek — üst HUD ile aynı (640 bazlı); sabit/küçük
        # boyutlar okunmuyordu.
        ui_scale = w / 640.0
        font_scale = 0.60 * ui_scale
        thickness  = max(1, int(2 * ui_scale))
        circle_r = int(14 * ui_scale)
        for r in results:
            tx, ty = int(r.center_x), int(r.center_y)
            cv2.circle(disp, (tx, ty), circle_r, (0, 255, 255),
                       max(1, int(3 * ui_scale)), cv2.LINE_AA)
            cv2.line(disp, (w // 2, h // 2), (tx, ty), (0, 200, 200),
                     max(1, int(ui_scale)), cv2.LINE_AA)
            bw = int(r.width_px)
            bh = int(r.height_px)
            cv2.rectangle(disp, (tx - bw // 2, ty - bh // 2),
                          (tx + bw // 2, ty + bh // 2), (0, 255, 255), thickness)
            cv2.putText(disp, f"PL {r.confidence:.2f} {bw}x{bh}px",
                        (tx + int(16 * ui_scale), ty - int(10 * ui_scale)),
                        cv2.FONT_HERSHEY_SIMPLEX, font_scale,
                        (0, 255, 200), thickness, cv2.LINE_AA)

        pl_str = f"{len(results)} pl" if results else "pl yok"
        l1, l2 = self._hud_lines(extra=f"{label}  {pl_str}")
        self._pending_display = self._draw_hud(disp, l1, l2)

    def _set_display_offset(self, label: str) -> None:
        """Bırakma OFFSET fazı bildirimi — görüntü işleme penceresinde
        payload alımındaki gibi HUD satırı olarak gösterilir.
        Görüntü işleme kapalıyken son kare üzerine çizilir (read-only)."""
        frame = self.camera.latest()
        if frame is None:
            return
        l1, l2 = self._hud_lines(extra=label)
        self._pending_display = self._draw_hud(frame, l1, l2)

    # ==================================================================== #
    #  Non-blocking irtifa yardımcıları
    # ==================================================================== #

    def _send_altitude(self, target: float, timeout: float = 35.0) -> None:
        self.nav.set_relative_altitude(target)
        self._alt_target = target
        self._alt_deadline = time.monotonic() + timeout
        self._alt_waiting = True

    def _altitude_ok(self) -> bool | None:
        """True=ulaşıldı  False=bekleniyor  None=timeout"""
        if not self._alt_waiting:
            return True
        pos = self.nav.position()
        if (self._alt_target is not None
                and abs(pos.relative_altitude - self._alt_target) <= ALTITUDE_TOLERANCE_M):
            self._alt_waiting = False
            return True
        if self._alt_deadline is not None and time.monotonic() > self._alt_deadline:
            self._alt_waiting = False
            self.log.warning("İrtifa timeout! Hedef=%.1fm Mevcut=%.1fm",
                             self._alt_target, pos.relative_altitude)
            return None
        return False

    def _reset_altitude_state(self) -> None:
        self._alt_waiting = False
        self._alt_target = None
        self._alt_deadline = None

    # ==================================================================== #
    #  Lawnmower tarama
    # ==================================================================== #

    def _init_search(self, reference_lat=None, reference_lon=None) -> None:
        """Yarışma alanı frame'inde dinamik search rotası kurar.

        reference_lat/lon verilirse O GPS referans alınır — ikinci search,
        launch konumu yerine İLK PAYLOAD BIRAKMA konumundan başlar. Verilmezse
        İHA'nın o anki GPS'i (launch) kullanılır. Her iki durumda da aynı
        CompetitionField + LawnmowerPlanner + reorder_search_from_drone kullanılır.

        1. Referans GPS → competition frame'e çevrilir.
        2. Referans alan dışındaysa search başlatılmaz → hata + RTL.
        3. Lawnmower planner competition frame'de çalıştırılır (pattern aynı).
        4. Rota, referansa EN YAKIN search hattından başlatılır (zigzag korunur).
        """
        self._search_count += 1
        if reference_lat is None or reference_lon is None:
            pos = self.nav.fresh_position(timeout=1.0)
            ref_lat, ref_lon = pos.latitude, pos.longitude
        else:
            ref_lat, ref_lon = reference_lat, reference_lon
        self._search_origin_lat = ref_lat
        self._search_origin_lon = ref_lon

        field = CompetitionField(COMPETITION_CORNERS)
        lx, ly = field.gps_to_local(ref_lat, ref_lon)
        if not field.contains_local(lx, ly):
            self.log.error(
                "Search referansı competition alanı dışında: "
                "GPS=(%.7f, %.7f) local=(%.1f, %.1f) — alan [0..%.0f]x[0..%.0f] m",
                ref_lat, ref_lon, lx, ly,
                field.width_m, field.length_m,
            )
            self.data.mission_failed = True
            self._goto(MissionState.RTL)
            return

        planner = LawnmowerPlanner(
            field.width_m, field.length_m, SCAN_LINE_SPACING, SEARCH_MARGIN
        )
        points = planner.generate()
        # Planner ortalanmış x eksenini ([-w/2, w/2]) competition frame'e (0..w)
        # kaydır; y ekseni zaten competition frame ile aynıdır (alt kenar tabanlı).
        points = [SearchPoint(p.x_m + field.width_m / 2.0, p.y_m) for p in points]
        # Referansa en yakın search hattından başla — zigzag sırası korunur.
        self._search_points = reorder_search_from_drone(points, lx, ly)
        self._search_field = field
        self._search_idx = 0
        # SEARCH_SPEED_MPS — yalnızca search rotasında geçerli (config'ten).
        self.nav.set_max_speed(SEARCH_SPEED_MPS, VERTICAL_SPEED_MPS)
        start = self._search_points[0] if self._search_points else None
        self.log.info(
            "SEARCH #%d başlatıldı: %d nokta, alan %.0fx%.0f m | "
            "referans GPS=(%.7f, %.7f) local=(%.1f, %.1f) | "
            "ilk waypoint local=(%.1f, %.1f) | hız=%.1f m/s",
            self._search_count, len(self._search_points),
            field.width_m, field.length_m,
            ref_lat, ref_lon, lx, ly,
            start.x_m if start else 0.0, start.y_m if start else 0.0,
            SEARCH_SPEED_MPS,
        )

    def _advance_search(self) -> None:
        if not self._search_points or self._search_field is None:
            return
        if self._search_idx >= len(self._search_points):
            self._search_idx = 0
            self.log.warning("Lawnmower tamamlandı, başa dönüyor.")
        sp = self._search_points[self._search_idx]
        lat, lon = self._search_field.local_to_gps(sp.x_m, sp.y_m)
        self.nav.goto_global(lat, lon, TAKEOFF_ALTITUDE)
        pos = self.nav.position()
        if haversine_m(pos.latitude, pos.longitude, lat, lon) < 3.0:
            self._search_idx += 1
            self.log.debug("Tarama noktası %d/%d geçildi.",
                           self._search_idx, len(self._search_points))

    # ==================================================================== #
    #  State handler'lar
    # ==================================================================== #

    # ------------------------------------------------------------------ #
    def _do_wait_prearm(self) -> None:
        """
        ArduPilot pre-arm kontrolleri geçene kadar görevi beklet.

        NON-BLOCKING: kamera frame'i alınır, vision çalışır, HUD hazırlanır
        (run() → _pump_preview üzerinden ekrana + GStreamer'a gider) ve
        telemetri (GPS/batarya/heading) güncellenir. Uçuş komutu GÖNDERİLMEZ.

        Sinyal: Navigation.prearm_good() — SYS_STATUS sağlık bitleri
        (AHRS + GPS + PREARM_CHECK). STATUSTEXT'teki 'PreArm: ...' mesajı
        HUD'da neden olarak gösterilir. Timeout YOK — gerektiği kadar bekler.
        """
        # Vision canlı: kareyi işle ve display frame'i hazırla
        # (GStreamer dahil tüm çıktılar _pump_preview üzerinden güncellenir).
        frame = self.camera.latest()
        if frame is not None:
            detections = self.target_detector.detect(frame)
            self._set_display_detections(frame, detections)

        self._prearm_reason = self.nav.prearm_reason()
        good = self.nav.prearm_good()

        if good is None:
            # Henüz SYS_STATUS yok / bayat — bekle (mission başlatma).
            return
        if not good:
            # Pre-arm geçmedi — neden HUD'da; bekle.
            return

        self.log.info(
            "Pre-arm GOOD — görev başlatılıyor (son neden: %s).",
            self._prearm_reason or "-",
        )
        self._goto(MissionState.INIT)

    # ------------------------------------------------------------------ #
    def _do_init(self) -> None:
        self.nav.wait_for_position()
        self._goto(MissionState.TAKEOFF)

    # ------------------------------------------------------------------ #
    def _do_takeoff(self) -> None:
        self.nav.takeoff(TAKEOFF_ALTITUDE)
        deadline = time.monotonic() + TAKEOFF_TIMEOUT
        while time.monotonic() < deadline:
            pos = self.nav.position()
            if abs(pos.relative_altitude - TAKEOFF_ALTITUDE) <= 0.4:
                break
            self._pending_display = None
            frame = self.camera.latest()
            if frame is not None:
                self._pending_display = self._base_overlay(
                    self._scale_for_display(frame)
                )
            self._pump_preview()
            time.sleep(0.1)
        else:
            self.log.error("Kalkış timeout!")
            self.data.mission_failed = True
            self._goto(MissionState.RTL)
            return

        self.log.info("Kalkış tamamlandı. İrtifa=%.1fm",
                      self.nav.position().relative_altitude)
        # Görev zamanlayıcısını başlat — yalnızca bir kez set edilir.
        if self._mission_start_t is None:
            self._mission_start_t = time.monotonic()
            self.log.info("Görev zamanlayıcısı başlatıldı. Süre sınırı: %d:%02d",
                          MISSION_TIMEOUT_S // 60, MISSION_TIMEOUT_S % 60)
        self._init_search()
        self._goto(MissionState.SEARCHING_FIRST_TARGET)

    # ------------------------------------------------------------------ #
    def _do_search_target(self) -> None:
        frame = self.camera.latest()
        if frame is None:
            self._advance_search()
            return

        # Frame, kamera katmanından BGR olarak gelir. Detector içinde B↔R takası
        # yapmayız; Gazebo dahil tüm kaynaklarda renk sınıflandırması stabil kalır.
        detections = self.target_detector.detect(frame)

        # İkinci hedef ararken birincisini filtrele
        if (self.data.state == MissionState.SEARCHING_SECOND_TARGET
                and self.data.first_target is not None):
            known = self.data.first_target
            detections = [
                d for d in detections
                if not (d.color == known.color
                        and d.shape == known.shape.value)
            ]
            # İkinci hedef için kilidi sıfırla (birinciye ait kilit kalmış olabilir)
            if self._locked_shape == known.shape.value and self._locked_color == known.color:
                self._locked_color = None
                self._locked_shape = None
                self.target_detector.history.clear()
                self.target_detector.reset_time_lock()

        self._set_display_detections(frame, detections)

        # Zaman bazlı kilit güncelle (1 sn görünürlük → kilit)
        self.target_detector.update_time_lock(detections)
        if (self._locked_color is None
                and self.target_detector.time_locked_color is not None):
            self._locked_color = self.target_detector.time_locked_color
            self._locked_shape = self.target_detector.time_locked_shape
            self.log.info(
                "Zaman kilidi aktif: renk=%s şekil=%s (%.1fs görüntede kaldı)",
                self._locked_color.value, self._locked_shape,
                1.0,  # TARGET_LOCK_DURATION_S
            )

        # Kararlı tespit — kilitli türe göre (kilit yoksa ilk kararlı seçimi al)
        stable = self.target_detector.stable_detection(
            detections,
            locked_color=self._locked_color,
            locked_shape=self._locked_shape,
            search_mode=True,
        )

        if stable is not None:
            # İlk kez kilitleniyorsa kaydet
            if self._locked_color is None:
                self._locked_color = stable.color
                self._locked_shape = stable.shape
                self.log.info(
                    "Hedef kilitleniyor: renk=%s şekil=%s",
                    stable.color.value, stable.shape,
                )

            self.log.info(
                "Hedef kararlı tespit: renk=%s şekil=%s güven=%.2f merkez=(%.0f,%.0f)",
                stable.color.value, stable.shape, stable.confidence,
                stable.center_x, stable.center_y,
            )
            # *** Kritik: hedef görüldüğünde anında GPS konumu kaydet ***
            rough = self.nav.position()
            self._target_rough_lat = rough.latitude
            self._target_rough_lon = rough.longitude
            self.data.pending_detection = stable
            self._goto(MissionState.TARGET_DETECTED)
            return

        # Tespit var ama henüz kararlı değil — dur, taramayı ilerletme
        if detections:
            return

        self._advance_search()

    # ------------------------------------------------------------------ #
    def _do_classify(self) -> None:
        d = self.data.pending_detection
        if d is None:
            self._return_search()
            return
        valid = (
            (d.color == TargetColor.BLUE and d.shape == "HEXAGON")
            or (d.color == TargetColor.RED and d.shape == "TRIANGLE")
        )
        if valid:
            self.log.info("Hedef geçerli: %s %s", d.color.value, d.shape)
            self._goto(MissionState.TARGET_CLASSIFIED)
        else:
            self.log.warning("Geçersiz kombinasyon %s+%s, yoksayıldı.",
                             d.color.value, d.shape)
            self.data.pending_detection = None
            self._locked_color = None
            self._locked_shape = None
            self.target_detector.history.clear()
            self.target_detector.reset_time_lock()
            self._return_search()

    # ------------------------------------------------------------------ #
    def _do_approach(self) -> None:
        """
        Kaba yatay yaklaşma — irtifa değiştirilmez.
        Önce yatay hizalama (ALIGNING), ardından kademeli iniş yapılır.
        """
        pos = self.nav.position()
        self.nav.set_max_speed(APPROACH_SPEED_MPS, VERTICAL_SPEED_MPS)
        if self._target_rough_lat is not None and self._target_rough_lon is not None:
            self.nav.goto_global(
                self._target_rough_lat,
                self._target_rough_lon,
                pos.relative_altitude,
            )
        self._reset_altitude_state()
        self._align_phase = "HORIZONTAL"
        self._last_align_cmd_t = 0.0
        self._no_detection_count = 0
        self.nav.set_max_speed(ALIGN_SPEED_MPS, VERTICAL_SPEED_MPS)
        self._goto(MissionState.ALIGNING_TARGET)

    # ------------------------------------------------------------------ #
    def _enter_drop_align(self) -> None:
        """
        1m (PAYLOAD_DROP_ALTITUDE) inişi tamamlanınca çağrılır — opsiyonel
        görüntü tabanlı ek hizalama fazı başlatır (PAYLOAD_DROP_ALIGN_TIMEOUT_S).
        Süre 0 ise bu metot çağrılmaz (eski davranış: doğrudan bırakma).
        """
        self._align_phase = "DROP_ALIGN"
        self._drop_align_deadline = time.monotonic() + PAYLOAD_DROP_ALIGN_TIMEOUT_S
        self._no_detection_count = 0
        self.log.info(
            "%.1fm'de ek görüntü hizalaması: %.1f sn süre tanındı "
            "(tespit olmazsa mevcut konumdan bırakılır).",
            PAYLOAD_DROP_ALTITUDE, PAYLOAD_DROP_ALIGN_TIMEOUT_S,
        )

    def _drop_align_tick(self, frame: np.ndarray, detection) -> bool:
        """
        DROP_ALIGN fazı ilerlemesi (non-blocking).
        True → bırakma yapılabilir (hizalı VEYA süre doldu).
        False → bekle (tespit yok / düzeltme komutu gönderildi).
        """
        if self._drop_align_deadline is None:
            return True
        if time.monotonic() > self._drop_align_deadline:
            self.log.info(
                "Ek hizalama süresi doldu (%.1f sn) — mevcut konumdan bırakılıyor.",
                PAYLOAD_DROP_ALIGN_TIMEOUT_S,
            )
            return True
        if detection is None:
            return False
        now = time.monotonic()
        if now - self._last_align_cmd_t < ALIGN_CMD_INTERVAL_S:
            return False
        pos = self.nav.position()
        aligned = self.position_controller.align_detection(
            detection.center_x,
            detection.center_y,
            frame.shape[1],
            frame.shape[0],
            pos.relative_altitude,
        )
        self._last_align_cmd_t = now
        if aligned:
            self.log.info("1m ek hizalama tamamlandı — yük bırakılıyor.")
            return True
        return False

    # ------------------------------------------------------------------ #
    def _do_align_target(self) -> None:
        """
        Kademeli yük bırakma yaklaşımı:
          yatay ortala → in → ortala → ... → 2.5m'de son ortalama
          → OFFSET düzeltmesi (İHA ileri — mekanizma hedefin ortasına)
          → görüntü işleme KAPALI → dik iniş 1m → yük bırakma.
        """
        # ── Görüntü işleme GEREKTİRMEYEN fazlar ──────────────────────────────
        # OFFSET: 2.5m'de ortalama bitti; İHA, hedef merkez doğrultusu boyunca
        # DROP_OFFSET_M (0.5m) İLERİ hareket eder (yük mekanizması arkada →
        # hedefin ortasına denk gelir). Bu adımda görüntü işleme ÇALIŞMAZ.
        if self._align_phase == "OFFSET":
            if self._offset_correction_start is None:
                self._offset_correction_start = time.monotonic()
                self.log.info(
                    "OFFSET (bırakma): İleri %.0fcm gidiliyor...",
                    DROP_OFFSET_M * 100,
                )
                self.nav.goto_body_offset(
                    forward_m=DROP_OFFSET_M, right_m=0.0
                )
                # Görüntü işleme penceresinde bildirim göster (payload alımındaki gibi)
                self._set_display_offset(
                    f"OFFSET: hedefe {DROP_OFFSET_M:.1f}m ileri gidiliyor"
                )
                return
            elapsed = time.monotonic() - self._offset_correction_start
            if elapsed < OFFSET_CORRECTION_WAIT_S:
                # Bekleme süresince bildirim güncel kalsın
                self._set_display_offset(
                    f"OFFSET: hedefe {DROP_OFFSET_M:.1f}m ileri  "
                    f"({elapsed:.1f}/{OFFSET_CORRECTION_WAIT_S:.0f}s)"
                )
                return
            # Offset tamam → dik iniş: 2.5m → 1m (yalnızca irtifa değişir)
            self._offset_correction_start = None
            self.log.info(
                "OFFSET bitti. Dik iniş: → %.1fm (yük bırakılacak).",
                PAYLOAD_DROP_ALTITUDE,
            )
            self._reset_altitude_state()
            self._align_phase = "DROPPING"
            self._no_detection_count = 0
            self._send_altitude(PAYLOAD_DROP_ALTITUDE, timeout=20.0)
            return

        # DROPPING: görüntü işleme OLMADAN dik iniş → 1m'ye varınca bırakma.
        if self._align_phase == "DROPPING":
            result = self._altitude_ok()
            if result is False:
                return  # hâlâ iniyor
            self._reset_altitude_state()
            self.log.info(
                "Sabit iniş tamamlandı (%.1fm). Yük bırakılıyor.",
                self.nav.position().relative_altitude,
            )
            self._align_phase = "HORIZONTAL"
            self._goto(MissionState.TARGET_ALIGNED)
            return

        # ── Görüntü işleme fazları (HORIZONTAL / DESCENDING) ──────────────────
        frame = self.camera.latest()
        if frame is None:
            return

        pos = self.nav.position()

        if pos.relative_altitude < 0.5:
            self.log.warning("İrtifa tehlikeli düşük %.1fm, geri çekiliyor.", pos.relative_altitude)
            self._reset_altitude_state()
            self._safe_reacquire()
            return

        now_status = time.monotonic()
        if now_status - self._last_align_status_log_t > 2.0:
            self._last_align_status_log_t = now_status
            self.log.info(
                "ALIGN aktif: faz=%s irtifa=%.1fm kayıp_sayac=%d kilit=%s/%s",
                self._align_phase, pos.relative_altitude, self._no_detection_count,
                self._locked_color.value if self._locked_color else "-",
                self._locked_shape or "-",
            )

        detections = self._filter_active_detections(self.target_detector.detect(frame))
        self._set_display_detections(frame, detections)

        if detections:
            picked = self._pick_locked_detection(detections)
            self.data.pending_detection = picked
            self._no_detection_count = 0
            p = self.nav.position()
            self._target_rough_lat = p.latitude
            self._target_rough_lon = p.longitude
        else:
            self._no_detection_count += 1

        # ---------- DESCENDING fazı ----------
        if self._align_phase == "DESCENDING":
            result = self._altitude_ok()
            if result is False:
                return
            reached = self._alt_target
            self._reset_altitude_state()
            self._align_phase = "HORIZONTAL"
            self._no_detection_count = 0
            self._aligned_frame_count = 0   # yeni faz: hizalı onay sayacı temiz
            self.log.info("İrtifa %.1fm'ye ulaşıldı → yatay hizalamaya devam.", reached or 0)
            return

        # ---------- HORIZONTAL fazı ----------
        detection = self.data.pending_detection

        # Tespit kayıp sayacı: hem tespit yok hem de limit dolmuşsa recover et
        if self._no_detection_count > self._NO_DETECTION_LIMIT:
            if self._is_over_rough_target():
                # İHA hedefin üzerinde — tespit kaybı NORMAL: hedef görüntüyü
                # dolduruyor, kontur/şekil doğrulaması bozuluyor. Yeniden kovalamak
                # (kaba GPS'e dönüp tekrar yaklaşmak) sonsuz döngü/daire üretir;
                # hedefi merkezde kabul edip inişe geçmek doğrusu.
                self.log.info(
                    "Tespit kaybı ama hedef kaba GPS'e yakın — hedef üzerinde "
                    "kabul ediliyor, inişe geçiliyor."
                )
                self.data.pending_detection = self._synthetic_center_detection(frame)
                detection = self.data.pending_detection
                self._no_detection_count = 0
            else:
                self.log.warning(
                    "ALIGN: %d tick üst üste tespit yok, kaba GPS'e dönülüyor.",
                    self._no_detection_count,
                )
                self._no_detection_count = 0
                self._recover_to_rough_position()
                return

        if detection is None:
            return

        now = time.monotonic()
        if now - self._last_align_cmd_t < ALIGN_CMD_INTERVAL_S:
            return

        # Hedef görüntüyü büyük ölçüde dolduruyorsa (İHA hedefe yeterince yakın)
        # centroid sinyali güvenilmez: sürekli düzeltme → hedef etrafında dönme.
        # İHA hedef üzerinde kabul edilip inişe geçilir (kovalama biter).
        # Eşik %50 — hassas ortalama yerine hızlı geçiş.
        if detection.bbox_width_px >= 0.5 * frame.shape[1]:
            self.log.info(
                "Hedef çok yakın (bbox=%.0fpx / frame=%dpx) — "
                "hedef üzerinde kabul edildi, inişe geçiliyor.",
                detection.bbox_width_px, frame.shape[1],
            )
            aligned = True
        else:
            aligned = self.position_controller.align_detection(
                detection.center_x,
                detection.center_y,
                frame.shape[1],
                frame.shape[0],
                pos.relative_altitude,
            )
        self._last_align_cmd_t = now

        # Ardışık frame onayı: tek frame'de merkezde görünmek (jitter / tesadüfi
        # geçiş) "hizalı" sayılmaz. ALIGNED_FRAMES_REQUIRED ardışık onay gerekir;
        # araya hizasız frame girerse sayaç sıfırlanır.
        if aligned:
            self._aligned_frame_count += 1
        else:
            self._aligned_frame_count = 0
        aligned = self._aligned_frame_count >= ALIGNED_FRAMES_REQUIRED

        dx = detection.center_x - frame.shape[1] / 2
        dy = detection.center_y - frame.shape[0] / 2
        heading = self.nav.position().heading_deg
        self.log.info(
            "ALIGN HORIZONTAL: merkez=(%.0f,%.0f) dx=%.0f dy=%.0f heading=%.0f° "
            "onay=%d/%d → hizalı=%s",
            detection.center_x, detection.center_y, dx, dy, heading,
            self._aligned_frame_count, ALIGNED_FRAMES_REQUIRED, aligned,
        )

        if not aligned:
            return

        # aligned=True: bu an GPS'i biriktir (en iyi hizalama anı)
        # non-blocking: position() kullan, blocking bekleme yok
        pos_now = self.nav.position()
        self._gps_candidates.append(
            (pos_now.latitude, pos_now.longitude, detection.confidence)
        )
        self.log.debug(
            "GPS aday kaydedildi: lat=%.7f lon=%.7f güven=%.2f (toplam=%d)",
            pos_now.latitude, pos_now.longitude, detection.confidence,
            len(self._gps_candidates),
        )

        next_alt = self._next_step_down_altitude(
            pos.relative_altitude, PAYLOAD_RELEASE_ALTITUDE,
        )
        if next_alt is not None:
            self.log.info(
                "Yatay hizalama tamam (%.1fm). Kademeli iniş: %.1fm",
                pos.relative_altitude, next_alt,
            )
            self.nav.set_max_speed(ALIGN_SPEED_MPS, VERTICAL_SPEED_MPS)
            self._align_phase = "DESCENDING"
            self._no_detection_count = 0
            self._aligned_frame_count = 0   # iniş başladı: onay sayacı temiz
            self._send_altitude(next_alt, timeout=30.0)
        else:
            # Son irtifada (PAYLOAD_RELEASE_ALTITUDE = 2.5m) ve tam merkezde.
            # GPS'i kaydet, ardından kamera-mekanizma OFFSET düzeltmesine geç
            # (mekanizma arkada → İHA ileri), sonra dik inişle 1m'ye inip bırak.
            pos_now = self.nav.position()
            self._gps_candidates.append(
                (pos_now.latitude, pos_now.longitude, detection.confidence)
            )
            self.log.info(
                "Son hizalama tamamlandı (%.1fm). GPS kaydedildi: lat=%.7f lon=%.7f "
                "güven=%.2f (toplam aday=%d). Kamera-mekanizma offset düzeltmesi "
                "başlıyor (%.0fcm ileri).",
                pos.relative_altitude,
                pos_now.latitude, pos_now.longitude, detection.confidence,
                len(self._gps_candidates),
                DROP_OFFSET_M * 100,
            )
            self.nav.set_max_speed(ALIGN_SPEED_MPS, VERTICAL_SPEED_MPS)
            self._align_phase = "OFFSET"
            self._offset_correction_start = None
            self._no_detection_count = 0
            self._aligned_frame_count = 0   # OFFSET fazına geçiş: onay sayacı temiz

    # ------------------------------------------------------------------ #
    def _is_over_rough_target(self) -> bool:
        """İHA, hedefin kaba GPS konumuna (ilk görüldüğü yer) yakın mı?
        Hedef büyük (4m çap) olduğundan ~2m içi 'hedef üzerinde' sayılır."""
        if self._target_rough_lat is None or self._target_rough_lon is None:
            return False
        pos = self.nav.position()
        return haversine_m(
            pos.latitude, pos.longitude, self._target_rough_lat, self._target_rough_lon
        ) < 2.0

    def _synthetic_center_detection(self, frame: np.ndarray) -> Detection:
        """Tespit kaybında hedefi görüntü merkezinde kabul et.
        align_detection merkeze göre hizalı döner → iniş akışı başlar."""
        return Detection(
            color=self._locked_color if self._locked_color is not None else TargetColor.BLUE,
            shape=self._locked_shape or "HEXAGON",
            center_x=frame.shape[1] / 2.0,
            center_y=frame.shape[0] / 2.0,
            area_px=1.0,
            bbox_width_px=1.0,
            bbox_height_px=1.0,
            confidence=0.5,
        )

    # ------------------------------------------------------------------ #
    def _recover_to_rough_position(self) -> None:
        """
        Hizalama sırasında hedef kadrajdan çıktı.
        Kaba GPS konumuna dön (mevcut irtifayı koru).
        Komut throttle'lı — aynı konuma sürekli goto gönderme.
        """
        if self._target_rough_lat is None or self._target_rough_lon is None:
            self._safe_reacquire()
            return
        now = time.monotonic()
        if now - self._last_recover_t < self._RECOVER_INTERVAL:
            return  # henüz erken
        self._last_recover_t = now
        pos = self.nav.position()
        self.log.info("Hedef kadrajdan çıktı, kaba GPS'e dönülüyor (%.6f, %.6f).",
                      self._target_rough_lat, self._target_rough_lon)
        self.nav.goto_global(
            self._target_rough_lat,
            self._target_rough_lon,
            pos.relative_altitude,
        )

    # ------------------------------------------------------------------ #
    def _do_save_gps(self) -> None:
        """
        GPS en iyi hizalama anında kaydedilir (non-blocking).
        Biriktirilen adaylar arasından en yüksek confidence'lıyı seç.
        Aday yoksa son bilinen pozisyonu kullan.
        """
        detection = self.data.pending_detection
        if detection is None:
            self._gps_candidates.clear()
            self._return_search()
            return

        # En iyi GPS adayını seç
        if self._gps_candidates:
            best_lat, best_lon, best_conf = max(
                self._gps_candidates, key=lambda c: c[2]
            )
            self.log.info(
                "GPS seçildi: %d aday arasından en iyi → lat=%.7f lon=%.7f güven=%.2f",
                len(self._gps_candidates), best_lat, best_lon, best_conf,
            )
        else:
            # Fallback: son bilinen pozisyon (non-blocking)
            pos = self.nav.position()
            best_lat, best_lon, best_conf = pos.latitude, pos.longitude, detection.confidence
            self.log.warning(
                "GPS adayı bulunamadı, son bilinen pozisyon kullanılıyor: lat=%.7f lon=%.7f",
                best_lat, best_lon,
            )
        self._gps_candidates.clear()

        shape = TargetType.HEXAGON if detection.shape == "HEXAGON" else TargetType.TRIANGLE
        target = Target(
            target_id=1 if self.data.first_target is None else 2,
            shape=shape,
            color=detection.color,
            latitude=best_lat,
            longitude=best_lon,
            confidence=best_conf,
        )

        if target.target_id == 1:
            self.data.first_target = target
        else:
            self.data.second_target = target

        self.log.info(
            "TARGET %d kaydedildi: %s %s  lat=%.7f lon=%.7f güven=%.2f",
            target.target_id, target.shape.value, target.color.value,
            target.latitude, target.longitude, target.confidence,
        )
        self._goto(MissionState.RELEASING_PAYLOAD)

    # ------------------------------------------------------------------ #
    def _do_release(self) -> None:
        # Hangi hedef henüz yük bırakmadı?
        if (self.data.second_target is not None
                and not self.data.second_target.payload_released):
            target = self.data.second_target
        elif (self.data.first_target is not None
              and not self.data.first_target.payload_released):
            target = self.data.first_target
        else:
            self.log.error("Yük bırakılacak hedef bulunamadı!")
            self.data.mission_failed = True
            self._goto(MissionState.TASK_2_FAILED)
            return

        result = self.payload.release_payload(self.payload_for_target(target))
        target.payload_released = result.success
        self.data.release_count += 1
        if not result.success:
            self.log.error("Payload bırakma başarısız: %s", result.message)

        self.log.info(
            "Payload bırakıldı: %s → hedef %d",
            self.payload_for_target(target).value, target.target_id,
        )

        self.data.pending_detection = None
        self.target_detector.history.clear()
        self._reset_altitude_state()
        self._target_rough_lat = None
        self._target_rough_lon = None
        self._gps_candidates.clear()
        # Kilidi sıfırla — bir sonraki hedef için temiz başla
        self._locked_color = None
        self._locked_shape = None
        self.target_detector.reset_time_lock()

        if self.data.second_target is None:
            # Birinci hedef tamamlandı → önce TAKEOFF_ALTITUDE'a çık,
            # irtifaya ulaşınca ikinci hedefi ara.
            # Doğrudan aramaya girmek yük bırakma irtifasında (düşük) arama yapar.
            self.log.info(
                "Birinci yük bırakıldı. %.1fm'ye çıkılıyor, ardından ikinci hedef aranacak.",
                TAKEOFF_ALTITUDE,
            )
            self._goto(MissionState.ASCENDING_AFTER_FIRST_RELEASE)
        else:
            # Her iki hedef de tamamlandı → Görev 2
            self._goto(MissionState.NAVIGATING_TO_FIRST_TARGET)

    # ------------------------------------------------------------------ #
    def _do_ascend_after_first_release(self) -> None:
        """
        Hata 1: Birinci yükü bıraktıktan sonra arama başlamadan önce
        TAKEOFF_ALTITUDE'a çık. Non-blocking: irtifaya ulaşınca ikinci
        hedef aramasına geç.
        """
        if not self._alt_waiting:
            pos = self.nav.position()
            self.log.info(
                "Yük sonrası yükselme: %.1fm → %.1fm",
                pos.relative_altitude, TAKEOFF_ALTITUDE,
            )
            self.nav.set_max_speed(APPROACH_SPEED_MPS, VERTICAL_SPEED_MPS)
            self._send_altitude(TAKEOFF_ALTITUDE, timeout=30.0)
            return

        result = self._altitude_ok()
        if result is False:
            return  # hâlâ çıkıyor

        self._reset_altitude_state()
        self.log.info(
            "%.1fm irtifaya ulaşıldı. İkinci hedef aranıyor.",
            self.nav.position().relative_altitude,
        )
        if self.data.first_target is None:
            self.log.error(
                "İkinci search için ilk hedef (drop) konumu yok! Search başlatılamıyor."
            )
            self.data.mission_failed = True
            self._goto(MissionState.RTL)
            return
        # İkinci search, launch konumu yerine İLK PAYLOAD BIRAKMA konumundan başlar.
        self._init_search(
            self.data.first_target.latitude,
            self.data.first_target.longitude,
        )
        self._goto(MissionState.SEARCHING_SECOND_TARGET)

    # ------------------------------------------------------------------ #
    def _do_navigate_to_first(self) -> None:
        """
        İlk hedefin GPS konumuna git.
        Davranış (güncellendi):
          1) Eğer mevcut irtifa < PAYLOAD_SEARCH_ALTITUDE (2m) ise önce bu irtifaya yüksel.
          2) Yükselme tamamlanınca hedefin GPS konumuna git (o irtifada).
          3) Varınca hedef merkezleme başlat.

        Bu sayede payload arama 2m'de başlatılacak, hedefe yaklaşıldığında
        kademeli inişle 1m'ye gelince payload araması başlatılacaktır.
        """
        target = self.data.first_target
        if target is None:
            self.data.mission_failed = True
            self._goto(MissionState.TASK_2_FAILED)
            return

        self.nav.set_max_speed(APPROACH_SPEED_MPS, VERTICAL_SPEED_MPS)
        pos_now = self.nav.position()
        self.log.info(
            "Görev 2: İlk hedefe gitme planı: önce irtifa kontrol, sonra GPS->lat=%.7f lon=%.7f",
            target.latitude, target.longitude,
        )

        # Eğer yeterli irtifada değilsek önce SEARCH altitude'a yüksel
        if pos_now.relative_altitude < PAYLOAD_SEARCH_ALTITUDE - ALTITUDE_TOLERANCE_M:
            self.log.info(
                "Mevcut irtifa %.2fm < %.2fm. Önce %.2fm'ye yükselebilmek için komut veriliyor.",
                pos_now.relative_altitude, PAYLOAD_SEARCH_ALTITUDE, PAYLOAD_SEARCH_ALTITUDE,
            )
            # Set pending nav target so that after ascend we navigate to target
            self._nav_pending_target = (target.latitude, target.longitude)
            self._send_altitude(PAYLOAD_SEARCH_ALTITUDE, timeout=30.0)
            return

        # Eğer _alt_waiting aktif ise yükselme devam ediyor — bekle
        if self._alt_waiting:
            res = self._altitude_ok()
            if res is False:
                # hala yükseliyor
                return
            if res is None:
                # timeout: fallback -> try navigating anyway
                self.log.warning("İrtifa yükselme timeout; doğrudan hedefe gitmeye çalışılıyor.")
                self._reset_altitude_state()
            else:
                # başarılı yükseldi — devam
                self._reset_altitude_state()

        # Eğer bir nav pending target varsa şimdi bu irtifada GPS'e git
        if self._nav_pending_target is not None:
            lat, lon = self._nav_pending_target
            self._nav_pending_target = None
            self.log.info("Yükselme tamamlandı; hedef GPS'e bu irtifada gidiliyor: lat=%.7f lon=%.7f",
                          lat, lon)
            self.nav.goto_global(lat, lon, self.nav.position().relative_altitude)
        else:
            # Normal akış: mevcut irtifada GPS'e git
            self.log.info(
                "Görev 2: İlk hedef GPS'e gidiliyor (mevcut irtifa): lat=%.7f lon=%.7f",
                target.latitude, target.longitude,
            )
            pos_now = self.nav.position()
            self.nav.goto_global(target.latitude, target.longitude, pos_now.relative_altitude)

        # Non-blocking bekleme ile hedefe ulaşınca merkezleme başlat
        nav_deadline = time.monotonic() + STATE_TIMEOUT
        while time.monotonic() < nav_deadline:
            pos = self.nav.position()
            dist = haversine_m(pos.latitude, pos.longitude,
                               target.latitude, target.longitude)
            self._pending_display = None
            frame = self.camera.latest()
            if frame is not None:
                self._pending_display = self._base_overlay(
                    self._scale_for_display(frame)
                )
            self._pump_preview()
            if dist < 3.0:
                break
            time.sleep(0.1)
        else:
            self.log.error("İlk hedef konumuna ulaşılamadı (timeout).")
            self.data.mission_failed = True
            self._goto(MissionState.TASK_2_FAILED)
            return

        # GPS konumuna varıldı — irtifayı DÜŞÜRME, mevcut irtifada görüntü işlemeyi başlat.
        self._reset_altitude_state()
        self.log.info(
            "GPS konumuna ulaşıldı (irtifa=%.1fm). Mevcut irtifada hedef merkezleme başlıyor.",
            self.nav.position().relative_altitude,
        )
        self._align_for_pickup_deadline = time.monotonic() + ALIGN_ON_TARGET_TIMEOUT_S
        self._align_for_pickup_start = time.monotonic()
        self._aligned_frame_count = 0   # yeni faz: hizalı onay sayacı temiz
        self._last_align_cmd_t = 0.0
        self._payload_descent_alt = None
        self._goto(MissionState.ALIGNING_ON_TARGET_FOR_PICKUP)

    # ------------------------------------------------------------------ #
    def _do_align_on_target_for_pickup(self) -> None:
        """
        Hata 9b: SEARCHING_PAYLOAD'dan önce İHA kendini hedef şeklinin
        tam merkezine hizalar. Bu sayede yük araması hedefin tam ortasından
        (aşağıya bakarak) yapılır.

        Zaman aşımı dolunca veya hizalama başarılıysa SEARCHING_PAYLOAD'a geç.
        Hedef görüntüde yoksa GPS'te beklemeye devam et.
        """
        frame = self.camera.latest()
        pos = self.nav.position()

        # Zaman aşımı kontrolü — her halükarda payload aramaya geç
        if (self._align_for_pickup_deadline is not None
                and time.monotonic() > self._align_for_pickup_deadline):
            self.log.info(
                "Hedef merkezleme zaman aşımı (%.0fs). Payload aramaya geçiliyor.",
                ALIGN_ON_TARGET_TIMEOUT_S,
            )
            self._align_for_pickup_deadline = None
            self._last_align_cmd_t = 0.0
            self._payload_descent_alt = None
            self._goto(MissionState.SEARCHING_PAYLOAD)
            return

        if frame is None:
            return

        target = self.data.first_target
        if target is None:
            self._goto(MissionState.TASK_2_FAILED)
            return

        # Hedef şeklini kamerada ara (hedefin rengi + şekli)
        detections = self.target_detector.detect(frame)
        target_detections = [
            d for d in detections
            if d.color == target.color and d.shape == target.shape.value
        ]
        self._set_display_detections(frame, target_detections)

        if not target_detections:
            # Hedef görünmüyor — GPS'te bekle, taşınma.
            # Görünmezlik sırasında İHA kayabilir: onay sayacını sıfırla.
            self._aligned_frame_count = 0
            return

        best = target_detections[0]
        now = time.monotonic()
        if now - self._last_align_cmd_t < ALIGN_CMD_INTERVAL_S:
            return

        aligned = self.position_controller.align_detection(
            best.center_x, best.center_y,
            frame.shape[1], frame.shape[0],
            pos.relative_altitude,
        )
        self._last_align_cmd_t = now

        self.log.info(
            "Hedef merkezleme: renk=%s şekil=%s merkez=(%.0f,%.0f) hizalı=%s",
            best.color.value, best.shape,
            best.center_x, best.center_y, aligned,
        )

        # Ardışık frame onayı: tek frame'de tesadüfi hizalı görünüm (jitter)
        # İHA'yı daha hedefi ortalamadan SEARCHING'e göndermesin.
        if aligned:
            self._aligned_frame_count += 1
        else:
            self._aligned_frame_count = 0
        aligned_confirmed = self._aligned_frame_count >= ALIGNED_FRAMES_REQUIRED

        if aligned_confirmed and self._align_for_pickup_start is not None:
            # Minimum görüntü işleme süresi: faz en az PAYLOAD_PICKUP_VISION_MIN_S
            # sürmeden SEARCHING'e geçilmez — İHA hedefi gerçekten ortalamalı.
            phase_elapsed = time.monotonic() - self._align_for_pickup_start
            if phase_elapsed < PAYLOAD_PICKUP_VISION_MIN_S:
                self.log.info(
                    "Hedef hizalı; minimum görüntü işleme süresi için "
                    "%.0fs daha bekleniyor...",
                    PAYLOAD_PICKUP_VISION_MIN_S - phase_elapsed,
                )
                return
            self.log.info(
                "Hedef merkezleme tamamlandı (%.1fm, %.0fs). "
                "İHA hedefin tam üzerinde → payload aranıyor.",
                pos.relative_altitude, phase_elapsed,
            )
            self._align_for_pickup_deadline = None
            self._align_for_pickup_start = None
            self._last_align_cmd_t = 0.0
            self._payload_descent_alt = None
            self._aligned_frame_count = 0
            self._goto(MissionState.SEARCHING_PAYLOAD)

    # ------------------------------------------------------------------ #
    def _do_search_payload(self) -> None:
        target = self.data.first_target
        if target is None:
            self._goto(MissionState.TASK_2_FAILED)
            return

        frame = self.camera.latest()
        pos = self.nav.position()

        # ── Timeout sayacını başlat (ilk girişte) ─────────────────────
        if self._payload_search_start is None:
            self._payload_search_start = time.monotonic()

        elapsed = time.monotonic() - self._payload_search_start

        # ── Timeout: PAYLOAD_SEARCH_TIMEOUT kadar bulunamadıysa recovery ─
        if elapsed >= PAYLOAD_SEARCH_TIMEOUT:
            self.log.warning(
                "Payload %.0fs içinde bulunamadı. "
                "%.1fm'ye çıkılıp hedef yeniden ortalanacak.",
                PAYLOAD_SEARCH_TIMEOUT,
                PAYLOAD_SEARCH_ALTITUDE,
            )
            self._payload_search_start = None   # sayacı sıfırla
            self._reset_altitude_state()
            self._align_for_pickup_deadline = time.monotonic() + ALIGN_ON_TARGET_TIMEOUT_S
            self._align_for_pickup_start = time.monotonic()
            self._aligned_frame_count = 0   # yeni faz: hizalı onay sayacı temiz
            self._last_align_cmd_t = 0.0
            self._payload_descent_alt = None

            # Önce arama irtifasına çık, sonra ALIGNING_ON_TARGET_FOR_PICKUP
            # Non-blocking: yükselme state'ini kullan
            self._send_altitude(PAYLOAD_SEARCH_ALTITUDE, timeout=20.0)
            self._goto(MissionState.ASCENDING_FOR_PAYLOAD_RETRY)
            return

        if frame is None:
            return

        # Renk sırası burada değiştirilmez. Beklenen renk, ilk kaydedilen hedefe
        # göre payload_for_target() ile belirlenir: mavi altıgen→kırmızı, kırmızı üçgen→mavi.
        color = self.payload_for_target(target)
        results = self.payload_detector.detect(frame, pos.relative_altitude, color)
        frame_for_display = frame

        # Ekrana kalan süreyi de göster
        remaining = max(0.0, PAYLOAD_SEARCH_TIMEOUT - elapsed)
        self._set_display_payload(
            frame_for_display, results,
            f"PAYLOAD ARA ({color.value}) timeout:{remaining:.0f}s"
        )

        if not results:
            self.payload_tracker.update([], frame.shape)
            return

        # Boyut filtresi: hedef boyutundaki renk tespitlerini at
        filtered = self._filter_payload_results(results, pos.relative_altitude)

        if not filtered:
            self.log.debug("Payload araması: tüm tespitler boyut filtresinde reddedildi.")
            self.payload_tracker.update([], frame.shape)
            return

        # Tracker ile en güvenilir candidate'ı seç
        best = self.payload_tracker.update(filtered, frame.shape)
        if best is None:
            return

        self.log.info(
            "Payload bulundu: renk=%s merkez=(%.0f,%.0f) güven=%.2f "
            "boyut=%.0fx%.0fpx alan=%.0fpx² track_id=%s",
            color.value, best.center_x, best.center_y, best.confidence,
            best.width_px, best.height_px, best.area_px,
            self.payload_tracker.locked_track_id,
        )
        # Başarılı tespit — sayacı sıfırla (bir sonraki arama için temiz başla)
        self._payload_search_start = None
        self.data.pending_detection = Detection(
            color=best.color, shape="PAYLOAD",
            center_x=best.center_x, center_y=best.center_y,
            area_px=best.area_px,
            bbox_width_px=best.width_px, bbox_height_px=best.height_px,
            confidence=best.confidence,
        )
        self._last_align_cmd_t = 0.0
        self._align_phase_start_t = None   # ALIGNING_PAYLOAD yeni faz temiz başlasın
        self._aligned_frame_count = 0      # hizalı onay sayacı temiz
        self._goto(MissionState.ALIGNING_PAYLOAD)

    # ------------------------------------------------------------------ #
    def _do_ascend_for_payload_retry(self) -> None:
        """
        SEARCHING_PAYLOAD timeout sonrası recovery:
          1. PAYLOAD_SEARCH_ALTITUDE'a yükselt (non-blocking)
          2. İrtifaya varınca ALIGNING_ON_TARGET_FOR_PICKUP'a geç
             → hedef yeniden ortalanır → payload araması yeniden başlar
        """
        if not self._alt_waiting:
            # Henüz irtifa komutu verilmedi — ver
            pos = self.nav.position()
            if pos.relative_altitude >= PAYLOAD_SEARCH_ALTITUDE - ALTITUDE_TOLERANCE_M:
                # Zaten yeterli irtifadayız, direkt merkezlemeye geç
                self.log.info(
                    "Payload retry: irtifa zaten %.1fm ≥ %.1fm. "
                    "Hedef merkezlemeye geçiliyor.",
                    pos.relative_altitude, PAYLOAD_SEARCH_ALTITUDE,
                )
                self._goto(MissionState.ALIGNING_ON_TARGET_FOR_PICKUP)
                return
            self.log.info(
                "Payload retry: %.1fm → %.1fm yükseltiliyor.",
                pos.relative_altitude, PAYLOAD_SEARCH_ALTITUDE,
            )
            self.nav.set_max_speed(APPROACH_SPEED_MPS, VERTICAL_SPEED_MPS)
            self._send_altitude(PAYLOAD_SEARCH_ALTITUDE, timeout=25.0)
            return

        result = self._altitude_ok()
        if result is False:
            return  # hâlâ çıkıyor

        self._reset_altitude_state()
        self.log.info(
            "Payload retry: %.1fm'ye ulaşıldı. Hedef merkezleme başlıyor.",
            self.nav.position().relative_altitude,
        )
        self._goto(MissionState.ALIGNING_ON_TARGET_FOR_PICKUP)

    # ------------------------------------------------------------------ #
    def _do_align_payload(self) -> None:
        """
        10cm'lik yük için hassas kademeli iniş + hizalama.

        Mimari (hepsi non-blocking, tek state içinde yönetilir):
          _align_phase == "HORIZONTAL"  → yatay hizalama; hizalınca DESCENDING'e geç
          _align_phase == "DESCENDING"  → basamak inişi bekle; inince HORIZONTAL'a dön
          Basamak hedefi PAYLOAD_CONTACT_ALTITUDE'a ulaşınca → PICKING_UP_PAYLOAD

        Böylece:
          PAYLOAD_SEARCH_ALTITUDE (2m)
            → hizala → 2-0.4=1.6m in
            → hizala → 1.6-0.4=1.2m in
            → ...
            → hizala → 0.4m in
            → hizala → 0m in  → pickup
        Donma yok: blocking bekleme ÇAĞRILMIYOR, sadece position() kullanılıyor.
        """
        frame = self.camera.latest()
        pos = self.nav.position()

        # ---------- DESCENDING fazı: basamak inişini non-blocking bekle ----------
        if self._align_phase == "DESCENDING":
            if frame is not None:
                target_obj = self.data.first_target
                if target_obj is not None:
                    color = self.payload_for_target(target_obj)
                    results = self.payload_detector.detect(
                        frame, pos.relative_altitude, color
                    )
                    results = self._filter_payload_results(results, pos.relative_altitude)
                    self._set_display_payload(
                        frame, results,
                        f"İNİŞ {pos.relative_altitude:.2f}m→{self._payload_descent_alt:.2f}m"
                        if self._payload_descent_alt is not None else "İNİŞ"
                    )
                    # ── 1m referans area toplama ──────────────────────────
                    # Henüz kayıt tamamlanmadıysa ve irtifa PAYLOAD_VERIFY_ALTITUDE
                    # bandındaysa birkaç frame'den sample biriktir.
                    if (not self._ref_area_recorded
                            and results
                            and abs(pos.relative_altitude - PAYLOAD_VERIFY_ALTITUDE)
                                <= ALTITUDE_TOLERANCE_M):
                        # Merkeze en yakın candidate'ı kullan — büyük hedef dışlanır
                        ref_cand = self._nearest_to_last_known(results)
                        if ref_cand is not None:
                            self._ref_area_samples.append(ref_cand.area_px)
                        # REF_SAMPLE_COUNT frame birikmişse median'a al
                        if len(self._ref_area_samples) >= _REF_SAMPLE_COUNT:
                            self._ref_area_px = statistics.median(self._ref_area_samples)
                            self._ref_area_recorded = True
                            self.log.info(
                                "[PAYLOAD REF] altitude=%.2fm  "
                                "area=%.0fpx²  samples=%d",
                                pos.relative_altitude,
                                self._ref_area_px,
                                len(self._ref_area_samples),
                            )
            result = self._altitude_ok()
            if result is False:
                return  # hâlâ iniyor — döngü bloklama yok
            # İrtifaya ulaşıldı veya timeout
            reached = pos.relative_altitude
            self._reset_altitude_state()

            if reached <= PAYLOAD_CONTACT_ALTITUDE + ALTITUDE_TOLERANCE_M:
                # Yere ulaştık → pickup
                self.log.info(
                    "Temas irtifasına ulaşıldı (%.2fm). Pickup başlıyor.", reached
                )
                self._align_phase = "HORIZONTAL"
                self._payload_descent_alt = None
                self._goto(MissionState.PICKING_UP_PAYLOAD)
                return

            # Bir sonraki basamak için yatay hizalamaya dön
            self._align_phase = "HORIZONTAL"
            self._align_phase_start_t = None   # yeni faz sayacını sıfırla
            self._no_detection_count = 0
            self._aligned_frame_count = 0   # yeni faz: hizalı onay sayacı temiz
            self.log.info(
                "Basamak inişi tamamlandı (%.2fm). Yatay hizalamaya devam.", reached
            )
            return

        # ---------- HORIZONTAL fazı: yatay hizalama ----------
        if frame is None:
            return

        # Faz başlangıç zamanı — hizalama timeout'u ve minimum görüntü işleme
        # süresi (PAYLOAD_PICKUP_VISION_MIN_S) için; ilk tick'te set edilir.
        if self._align_phase_start_t is None:
            self._align_phase_start_t = time.monotonic()

        target = self.data.first_target
        if target is None:
            self._goto(MissionState.TASK_2_FAILED)
            return

        color = self.payload_for_target(target)
        results = self.payload_detector.detect(frame, pos.relative_altitude, color)
        # Boyut filtresi: hedef boyutundaki tespitleri at
        filtered = self._filter_payload_results(results, pos.relative_altitude)

        # Tracker: frame'ler arası tutarlı candidate seç
        best = self.payload_tracker.update(filtered, frame.shape)

        # Güncellenen etiket: irtifa ve sonraki hedef
        next_step = max(
            PAYLOAD_CONTACT_ALTITUDE,
            pos.relative_altitude - PAYLOAD_DESCENT_STEP_M,
        )
        self._set_display_payload(
            frame, filtered,
            f"HİZALA {pos.relative_altitude:.2f}m → sonraki:{next_step:.2f}m"
        )

        if best is not None:
            self._no_detection_count = 0
            self.data.pending_detection = Detection(
                color=best.color, shape="PAYLOAD",
                center_x=best.center_x, center_y=best.center_y,
                area_px=best.area_px,
                bbox_width_px=best.width_px, bbox_height_px=best.height_px,
                confidence=best.confidence,
            )
        else:
            # Yük bu irtifada görünmüyor — hafif kayma olabilir, GPS'te bekle.
            # ANCAK: İHA hedefi (şekli) henüz ortalamadıysa payload görünmez;
            # görüntü işleme fazı en az PAYLOAD_PICKUP_VISION_MIN_S sürmeli,
            # bu süre dolmadan SEARCHING'e dönmek yanlış alarm üretir (döngü).
            self._no_detection_count += 1
            vision_elapsed = time.monotonic() - self._align_phase_start_t
            if (self._no_detection_count > self._NO_DETECTION_LIMIT
                    and vision_elapsed >= PAYLOAD_PICKUP_VISION_MIN_S):
                self.log.warning(
                    "Payload görünmüyor (%d tick, %.0fs). "
                    "Yük arama adımına dönülüyor.",
                    self._no_detection_count, vision_elapsed,
                )
                self._no_detection_count = 0
                self._payload_descent_alt = None
                self._payload_search_start = None
                self.payload_tracker.reset()
                self._reset_altitude_state()
                self._goto(MissionState.SEARCHING_PAYLOAD)
            return

        detection = self.data.pending_detection
        if detection is None:
            return

        now = time.monotonic()
        if now - self._last_align_cmd_t < ALIGN_CMD_INTERVAL_S:
            return

        cx, cy = detection.center_x, detection.center_y
        fw, fh = frame.shape[1], frame.shape[0]
        dx = abs(cx - fw / 2)
        dy = abs(cy - fh / 2)

        # Dinamik tolerans: 1m üstünde GPS bu hassasiyete ulaşamaz → gevşek eşik
        # 1m altında: yakın mesafe, sıkı eşik mantıklı
        effective_tol = (
            PAYLOAD_PIXEL_TOLERANCE_PX
            if pos.relative_altitude <= 1.0
            else PIXEL_ALIGNMENT_TOLERANCE_PX
        )
        aligned_tight = dx <= effective_tol and dy <= effective_tol

        # Ardışık frame onayı: tek frame'de merkezde görünmek (jitter / tesadüfi
        # geçiş) "hizalı" sayılmaz. ALIGNED_FRAMES_REQUIRED ardışık onay gerekir;
        # araya hizasız frame girerse sayaç sıfırlanır.
        if aligned_tight:
            self._aligned_frame_count += 1
        else:
            self._aligned_frame_count = 0
        aligned_tight = self._aligned_frame_count >= ALIGNED_FRAMES_REQUIRED

        # Her durumda pozisyon komutu gönder — stuck önleme
        self.position_controller.align_detection(cx, cy, fw, fh, pos.relative_altitude)
        self._last_align_cmd_t = now

        self.log.info(
            "PAYLOAD HİZALA: merkez=(%.0f,%.0f) dx=%.0f dy=%.0f "
            "irtifa=%.2fm tol=%.0fpx onay=%d/%d hizalı=%s",
            cx, cy, dx, dy, pos.relative_altitude, effective_tol,
            self._aligned_frame_count, ALIGNED_FRAMES_REQUIRED, aligned_tight,
        )

        # Hizalama timeout: belirli süre hizalanamıyorsa zorla bir basamak in.
        # Daha aşağıda payload daha büyük görünür ve hizalama kolaylaşır.
        align_elapsed = now - self._align_phase_start_t

        if not aligned_tight and align_elapsed < PAYLOAD_ALIGN_TIMEOUT_S:
            return   # Henüz timeout dolmadı, pozisyon komutları çalışıyor olabilir

        if not aligned_tight:
            self.log.warning(
                "Hizalama timeout (%.1fs) irtifa=%.2fm dx=%.0f dy=%.0f → "
                "zorla basamak inişi, daha yakında daha iyi görülür.",
                align_elapsed, pos.relative_altitude, dx, dy,
            )

        # Hizalandı VEYA timeout → bir sonraki basamağa in
        # Eğer zaten son hizalama irtifasına (~0.4m) indiysek offset correction'a geç
        if pos.relative_altitude <= PAYLOAD_ALIGN_ALTITUDE_M + ALTITUDE_TOLERANCE_M:
            self.log.info(
                "Payload ~%.1fm'de hizalandı. Kamera-mekanizma offset düzeltmesine "
                "(%.0fcm ileri) geçiliyor.",
                PAYLOAD_ALIGN_ALTITUDE_M, CAMERA_TO_MECHANISM_OFFSET_M * 100,
            )
            self._offset_correction_start = None
            self._goto(MissionState.OFFSET_CORRECTION)
            return

        next_alt = max(PAYLOAD_CONTACT_ALTITUDE,
                       pos.relative_altitude - PAYLOAD_DESCENT_STEP_M)
        self._payload_descent_alt = next_alt
        self._align_phase = "DESCENDING"
        self._align_phase_start_t = None   # bir sonraki HORIZONTAL fazı için sıfırla
        self._no_detection_count = 0
        self._aligned_frame_count = 0   # iniş başladı: onay sayacı temiz
        self.log.info(
            "Payload hizalama → basamak inişi: %.2fm → %.2fm",
            pos.relative_altitude, next_alt,
        )
        self.nav.set_max_speed(ALIGN_SPEED_MPS, VERTICAL_SPEED_MPS)
        self._send_altitude(next_alt, timeout=20.0)

    # ------------------------------------------------------------------ #
    def _do_offset_correction(self) -> None:
        """
        Kamera payload'u 0.4m'de ortaladı. 
        Kamera ile görev mekanizması arasında CAMERA_TO_MECHANISM_OFFSET_M (8cm)
        offset var (kamera önde, mekanizma arkada). Dolayısıyla İHA'nın, payload
        mekanizmasını payload'un üzerine getirmesi için +0.08m (8cm ileri) gitmesi
        ve ardından yük alım irtifasına (0.0m) inmesi gerekir.
        """
        now = time.monotonic()
        pos = self.nav.position()

        if self._offset_correction_start is None:
            self._offset_correction_start = now
            self.log.info(
                "OFFSET: İleri %.2fm gidiliyor...", CAMERA_TO_MECHANISM_OFFSET_M
            )
            # Sadece offset gönderiyoruz, irtifa değişmeyecek
            self.nav.goto_body_offset(
                forward_m=CAMERA_TO_MECHANISM_OFFSET_M, right_m=0.0
            )
            self._offset_phase = "WAITING_OFFSET"
            return

        elapsed = now - self._offset_correction_start

        if self._offset_phase == "WAITING_OFFSET":
            # 8cm için birkaç saniye bekleme güvenli ve yeterlidir.
            if elapsed < OFFSET_CORRECTION_WAIT_S:
                return

            # Offset bitti, şimdi inelim
            self.log.info("OFFSET bitti. Temas irtifasına (0.0m) iniliyor.")
            self._send_altitude(PAYLOAD_CONTACT_ALTITUDE, timeout=10.0)
            self._offset_phase = "WAITING_DESCENT"
            return

        if self._offset_phase == "WAITING_DESCENT":
            # İrtifaya ulaşmayı kontrol edelim
            result = self._altitude_ok()
            if result is False and elapsed < OFFSET_CORRECTION_WAIT_S + 10.0:
                return  # hala iniyor ve toplam zaman var

            self._reset_altitude_state()
            self.log.info(
                "OFFSET VE İNİŞ tamamlandı (%.2fs). Mevcut irtifa: %.2fm. Pickup başlıyor.",
                elapsed, pos.relative_altitude,
            )
            self._goto(MissionState.PICKING_UP_PAYLOAD)

    # ------------------------------------------------------------------ #
    def _do_pickup(self) -> None:
        """
        Non-blocking servo bekleme.
        İlk çağrıda servo komutunu gönder; sonraki tick'lerde
        bekleme süresi doldu mu diye kontrol et.

        Bekleme süresi: payload.servo_wait_s → pickup bekleme süresi
        (LocalRPiBackend/RemoteRPiBackend — pickup_wait_s, varsayılan 3.0s;
        STEP MOTOR kullanılmaz, mekanizma sabittir).
        None dönerse simulation için config.py PAYLOAD_SERVO_WAIT_S kullanılır.
        """
        target = self.data.first_target
        if target is None:
            self._goto(MissionState.TASK_2_FAILED)
            return

        if self._pickup_servo_start is None:
            payload_type = self.payload_for_target(target)
            self.payload.send_pickup_command(payload_type)
            self._pickup_servo_start = time.monotonic()
            # Pickup bekleme süresini backend'den al, yoksa simulation değeri
            wait = self.payload.servo_wait_s
            if wait is None:
                wait = PAYLOAD_SERVO_WAIT_S
            self._pickup_servo_wait = wait
            self.log.info(
                "Pickup komutu gönderildi (%s). %.1fs bekleniyor...",
                payload_type.value, self._pickup_servo_wait,
            )
            return

        elapsed = time.monotonic() - self._pickup_servo_start
        wait    = getattr(self, "_pickup_servo_wait", PAYLOAD_SERVO_WAIT_S)

        if elapsed < wait:
            frame = self.camera.latest()
            if frame is not None:
                # Simulation: kalan süreyi göster. Gerçek donanım: sadece normal HUD.
                is_sim = (self.payload.servo_wait_s is None)
                if is_sim:
                    extra = f"PICKUP BEKLE {elapsed:.1f}/{wait:.0f}s"
                    l1, l2 = self._hud_lines(extra=extra)
                    self._pending_display = self._draw_hud(
                        frame, l1, l2, line1_color=(0, 200, 255))
                else:
                    self._pending_display = self._base_overlay(frame)
            return

        self._pickup_servo_start = None
        result = self.payload.finish_pickup(self.payload_for_target(target))
        if result.success:
            self.log.info("Pickup tamamlandı. Doğrulama irtifasına çıkılıyor.")
            self._reset_altitude_state()
            self._goto(MissionState.ASCENDING_FOR_VERIFICATION)
        else:
            self.data.pickup_attempts += 1
            self._retry_or_fail()

    # ------------------------------------------------------------------ #
    def _do_ascend_for_verification(self) -> None:
        pos = self.nav.position()

        # Faz 1: Henüz altitude komutu verilmediyse ver
        if not self._alt_waiting and self._verify_stabilize_start is None:
            self.log.info("Doğrulama irtifasına çıkılıyor: %.1fm", PAYLOAD_VERIFY_ALTITUDE)
            self.nav.set_max_speed(APPROACH_SPEED_MPS, VERTICAL_SPEED_MPS)
            self._send_altitude(PAYLOAD_VERIFY_ALTITUDE, timeout=30.0)
            return

        # Faz 2: Altitude komutu verildi, irtifaya ulaşılmasını bekle
        if self._alt_waiting:
            result = self._altitude_ok()
            if result is False:
                return  # hâlâ çıkıyor
            # True veya None (timeout): irtifaya ulaşıldı ya da zaman aşımı
            self._reset_altitude_state()
            # Stabilizasyon sayacını başlat
            self._verify_stabilize_start = time.monotonic()
            self.log.info(
                "Doğrulama irtifasına ulaşıldı (%.2fm). %.1fs stabilizasyon bekleniyor.",
                pos.relative_altitude, VERIFY_STABILIZE_S,
            )
            return

        # Faz 3: Stabilizasyon bekleniyor
        elapsed = time.monotonic() - self._verify_stabilize_start
        in_band = abs(pos.relative_altitude - PAYLOAD_VERIFY_ALTITUDE) <= VERIFY_ALT_TOLERANCE

        if not in_band:
            # İrtifa kaydı — nadiren olur, sadece logla
            self.log.debug(
                "Stabilizasyon sırasında irtifa bant dışı (%.2fm), bekleniyor.",
                pos.relative_altitude,
            )

        if elapsed < VERIFY_STABILIZE_S:
            return  # stabilizasyon süresi dolmadı

        self.log.info(
            "Stabilizasyon tamamlandı (%.2fm, %.1fs). Validation başlıyor.",
            pos.relative_altitude, elapsed,
        )
        self._verify_stabilize_start = None
        self._goto(MissionState.VERIFYING_PAYLOAD)

    # ------------------------------------------------------------------ #
    def _do_verify(self) -> None:
        """
        Pickup doğrulaması — basit median area karşılaştırması.

        Akış:
          1. VERIFYING_PAYLOAD'a girildiğinde _VERIFY_COLLECT_COUNT frame toplanır.
          2. Toplanan area değerlerinin medyanı hesaplanır.
          3. Medyan, referans alanın [ref * VERIFY_LOW, ref * VERIFY_HIGH]
             bandındaysa → payload hâlâ yerde → FAIL.
          4. Bandin dışındaysa veya hiç tespit yoksa → pickup başarılı → SUCCESS.

        Referans (_ref_area_px): iniş sırasında PAYLOAD_VERIFY_ALTITUDE'da
        kaydedilen birkaç frame'in medyanı. Yoksa fallback.
        """
        target = self.data.first_target
        if target is None:
            self._goto(MissionState.TASK_2_FAILED)
            return

        frame = self.camera.latest()
        if frame is None:
            return

        color = self.payload_for_target(target)
        pos   = self.nav.position()
        raw   = self.payload_detector.detect(frame, pos.relative_altitude, color)
        candidates = self._filter_payload_results(raw, pos.relative_altitude)

        # Validation'da confidence sıralaması yerine önceki hizalamadan bilinen
        # payload merkezine en yakın contour'u seç.
        # Aynı renkteki büyük hedef veya başka nesneler merkez uzaksa dışlanır.
        best_candidate = self._nearest_to_last_known(candidates)

        ref = self._ref_area_px

        # ── Referans yoksa: basit fallback ─────────────────────────────────
        if ref is None:
            self._set_display_payload(frame, candidates, "DOĞRULA (ref yok)")
            if best_candidate is None or best_candidate.confidence < VERIFY_FALLBACK_CONFIDENCE:
                self.log.info("[PAYLOAD VALIDATION] result=SUCCESS (fallback: tespit yok/düşük güven)")
                self._reset_ref()
                self._goto(MissionState.TASK_2_COMPLETE)
            else:
                self.data.pickup_attempts += 1
                self.log.warning(
                    "[PAYLOAD VALIDATION] result=FAIL (fallback: güven=%.2f) Deneme %d/%d",
                    best_candidate.confidence, self.data.pickup_attempts, MAX_PICKUP_ATTEMPTS,
                )
                self._retry_or_fail()
            return

        # ── Frame toplama ──────────────────────────────────────────────────
        in_band = abs(pos.relative_altitude - PAYLOAD_VERIFY_ALTITUDE) <= VERIFY_ALT_TOLERANCE
        if not in_band:
            self.log.debug(
                "[PAYLOAD VALIDATION] irtifa bant dışı (%.2fm), frame atlandı.",
                pos.relative_altitude,
            )
            self._set_display_payload(
                frame, candidates,
                f"DOĞRULA (irtifa bekleniyor: {pos.relative_altitude:.2f}m/{PAYLOAD_VERIFY_ALTITUDE:.1f}m)",
            )
            return

        # Merkeze en yakın candidate'ın area'sını kaydet, yoksa 0
        area_this_frame = best_candidate.area_px if best_candidate is not None else 0.0
        self._verify_votes.append(area_this_frame)

        cur_str = f"{area_this_frame:.0f}" if area_this_frame else "—"
        near_str = (
            f" center=({best_candidate.center_x:.0f},{best_candidate.center_y:.0f})"
            if best_candidate is not None else ""
        )
        self.log.info(
            "[PAYLOAD VALIDATION] altitude=%.2fm  reference_area=%.0fpx²  "
            "current_area=%s%s  frame=%d/%d",
            pos.relative_altitude, ref, cur_str, near_str,
            len(self._verify_votes), _VERIFY_VOTE_COUNT,
        )
        self._set_display_payload(
            frame, ([best_candidate] if best_candidate else []),
            f"DOĞRULA ref={ref:.0f} cur={cur_str} {len(self._verify_votes)}/{_VERIFY_VOTE_COUNT}",
        )

        if len(self._verify_votes) < _VERIFY_VOTE_COUNT:
            return  # Henüz yeterli frame yok

        # ── Karar ─────────────────────────────────────────────────────────
        areas = self._verify_votes[:]
        self._verify_votes.clear()

        detected = [a for a in areas if a > 0]
        median_area = statistics.median(detected) if detected else 0.0

        # Payload hâlâ yerde bandı: referansın %60 ile %160 arasında
        low  = ref * VERIFY_BAND_LOW
        high = ref * VERIFY_BAND_HIGH

        still_there = (detected and low <= median_area <= high)

        self.log.info(
            "[PAYLOAD VALIDATION] Final: ref=%.0fpx²  median=%.0fpx²  "
            "band=[%.0f, %.0f]  detected=%d/%d  still_there=%s",
            ref, median_area, low, high,
            len(detected), _VERIFY_VOTE_COUNT, still_there,
        )

        if still_there:
            self.data.pickup_attempts += 1
            self.log.warning(
                "[PAYLOAD VALIDATION] result=FAIL (payload hâlâ yerde). Deneme %d/%d",
                self.data.pickup_attempts, MAX_PICKUP_ATTEMPTS,
            )
            self._retry_or_fail()
        else:
            self.log.info(
                "[PAYLOAD VALIDATION] result=SUCCESS "
                "(median=%.0fpx² bant dışı veya tespit yok). Görev 2 tamamlandı.",
                median_area,
            )
            self._reset_ref()
            self._goto(MissionState.TASK_2_COMPLETE)

    def _reset_ref(self) -> None:
        """Referans area, sample listelerini ve oy listesini temizle."""
        self._ref_area_px       = None
        self._ref_area_recorded = False
        self._ref_area_samples.clear()
        self._verify_votes.clear()

    # ------------------------------------------------------------------ #
    def _do_after_task2(self) -> None:
        """
        Görev 2 başarıyla tamamlandı.
        Alınan yük 2. hedefin (Görev 1'de 2. yükün bırakıldığı) GPS konumuna götürülüp
        orada bırakılır. Ardından RTL başlar.

        Görev tanımı:
          first_target  → 1. hedefin üzerinden yük alındı
          second_target → alınan yük buraya bırakılacak
        """
        target = self.data.second_target
        if target is None:
            self.log.error("2. hedef GPS bilgisi yok, RTL'ye geçiliyor.")
            self._do_start_rtl()
            return
        self.log.info(
            "Görev 2 tamamlandı. Alınan yük 2. hedefe götürülüyor: "
            "%s %s  lat=%.7f lon=%.7f",
            target.shape.value, target.color.value,
            target.latitude, target.longitude,
        )
        self._reset_altitude_state()
        self._goto(MissionState.RETURNING_PAYLOAD)

    # ------------------------------------------------------------------ #
    def _do_return_payload(self) -> None:
        """
        Alınan yükü 2. hedefin GPS konumuna götür.
        Adım 1: TAKEOFF_ALTITUDE'da GPS'e uç.
        Adım 2: Hedefe varınca TARGET_APPROACH_ALTITUDE'a in.
        Adım 3: RELEASING_RETURNED_PAYLOAD'a geç.
        """
        target = self.data.second_target
        if target is None:
            self.log.error("_do_return_payload: second_target yok, RTL.")
            self._do_start_rtl()
            return

        # Faz 1: GPS'e yüksekte git
        if not self._alt_waiting:
            pos = self.nav.position()
            if pos.relative_altitude < TARGET_APPROACH_ALTITUDE - ALTITUDE_TOLERANCE_M:
                self.log.info(
                    "Dönüş öncesi irtifa artırılıyor → %.1fm", TARGET_APPROACH_ALTITUDE
                )
                self.nav.set_max_speed(APPROACH_SPEED_MPS, VERTICAL_SPEED_MPS)
                self._send_altitude(TARGET_APPROACH_ALTITUDE, timeout=30.0)
                return

        if self._alt_waiting:
            result = self._altitude_ok()
            if result is False:
                return
            self._reset_altitude_state()

        # İrtifaya ulaşıldı — 2. hedefe git
        pos = self.nav.position()
        dist = haversine_m(
            pos.latitude, pos.longitude,
            target.latitude, target.longitude,
        )
        self.nav.goto_global(target.latitude, target.longitude, TARGET_APPROACH_ALTITUDE)

        if dist < 3.0:
            self.log.info(
                "2. hedef konumuna ulaşıldı (lat=%.7f lon=%.7f). Yük bırakılacak.",
                target.latitude, target.longitude,
            )
            self._reset_altitude_state()
            self._align_phase = "HORIZONTAL"
            self._last_align_cmd_t = 0.0
            self._target_rough_lat = target.latitude
            self._target_rough_lon = target.longitude
            self.nav.set_max_speed(ALIGN_SPEED_MPS, VERTICAL_SPEED_MPS)
            self._goto(MissionState.RELEASING_RETURNED_PAYLOAD)

    # ------------------------------------------------------------------ #
    def _do_release_returned_payload(self) -> None:
        """
        Alınan yükü 2. hedefin merkezine kademeli iniş + yatay hizalama ile bırak.
        Görüntü işleme: second_target'ın renk + şeklini ara.
        _align_phase: HORIZONTAL | DESCENDING | DROPPING | DROP_ALIGN | RELEASING
        """
        frame = self.camera.latest()
        pos = self.nav.position()

        # --- RELEASING ---
        if self._align_phase == "RELEASING":
            target = self.data.second_target
            if target is not None:
                if self.data.first_target is not None:
                    picked_up_from = self.data.first_target
                    payload_type = MissionStateMachine.payload_for_target(picked_up_from)
                else:
                    payload_type = MissionStateMachine.payload_for_target(target)
                result = self.payload.release_returned_payload(payload_type)
                self.log.info(
                    "Geri getirilen yük 2. hedefte bırakıldı: %s (hedef=%s %s) — başarı=%s (2 MAGNET OFF)",
                    payload_type.value,
                    target.shape.value, target.color.value,
                    result.success,
                )
            self._align_phase = "HORIZONTAL"
            self._reset_altitude_state()
            self.log.info("Görev 2 yük bırakma tamamlandı → RTL başlatılıyor.")
            self._do_start_rtl()
            return

        # --- DROPPING: hizalama sonrası görüntü işleme olmadan sabit iniş ---
        if self._align_phase == "DROPPING":
            if frame is not None:
                self._pending_display = self._base_overlay(frame)
            result = self._altitude_ok()
            if result is False:
                return
            self._reset_altitude_state()
            self.log.info(
                "Geri yük bırakma: sabit iniş tamamlandı (%.1fm).",
                self.nav.position().relative_altitude,
            )
            # Opsiyonel: 1m'de 2. hedefe göre ek hizalama (timeout'lu)
            if PAYLOAD_DROP_ALIGN_TIMEOUT_S > 0:
                self._enter_drop_align()
                return
            self._align_phase = "RELEASING"
            return

        # --- DROP_ALIGN: 1m'de opsiyonel ek hizalama (2. hedef renk+şekil) ---
        if self._align_phase == "DROP_ALIGN":
            if frame is None:
                return
            detections = self.target_detector.detect(frame)
            target_obj = self.data.second_target
            if target_obj is not None:
                detections = [
                    d for d in detections
                    if d.color == target_obj.color
                    and d.shape == target_obj.shape.value
                ]
            best = detections[0] if detections else None
            if self._drop_align_tick(frame, best):
                self._align_phase = "RELEASING"
            return

        # --- DESCENDING ---
        if self._align_phase == "DESCENDING":
            if frame is not None:
                self._pending_display = self._base_overlay(frame)
            result = self._altitude_ok()
            if result is False:
                return
            reached = self._alt_target
            self._reset_altitude_state()
            self._align_phase = "HORIZONTAL"
            self.log.info(
                "Geri yük bırakma: %.1fm'ye ulaşıldı → yatay hizalamaya devam.",
                reached or 0,
            )
            return

        # --- HORIZONTAL: 2. hedefe göre görüntü işleme ---
        if frame is None:
            return

        target_obj = self.data.second_target
        detections = self.target_detector.detect(frame)
        if target_obj is not None:
            # Yalnızca 2. hedefe ait renk + şekli ara
            detections = [
                d for d in detections
                if d.color == target_obj.color
                and d.shape == target_obj.shape.value
            ]
        self._set_display_detections(frame, detections)

        if not detections:
            # Hedef görüntüden çıkmış — kaba GPS'e dön
            now = time.monotonic()
            if now - self._last_recover_t >= self._RECOVER_INTERVAL:
                self._last_recover_t = now
                if self._target_rough_lat is not None:
                    self.nav.goto_global(
                        self._target_rough_lat,
                        self._target_rough_lon,
                        pos.relative_altitude,
                    )
            return

        best = detections[0]
        now = time.monotonic()
        if now - self._last_align_cmd_t < ALIGN_CMD_INTERVAL_S:
            return

        aligned = self.position_controller.align_detection(
            best.center_x, best.center_y,
            frame.shape[1], frame.shape[0],
            pos.relative_altitude,
        )
        self._last_align_cmd_t = now
        if not aligned:
            return

        next_alt = self._next_step_down_altitude(
            pos.relative_altitude, PAYLOAD_RELEASE_ALTITUDE,
        )
        if next_alt is not None:
            self.log.info(
                "Geri yük bırakma: yatay hizalama tamam (%.1fm). Kademeli iniş: %.1fm",
                pos.relative_altitude, next_alt,
            )
            self._align_phase = "DESCENDING"
            self.nav.set_max_speed(ALIGN_SPEED_MPS, VERTICAL_SPEED_MPS)
            self._send_altitude(next_alt, timeout=25.0)
        else:
            self.log.info(
                "Geri yük bırakma: %.1fm'de 2. hedef merkezinde → %.1fm'ye sabit iniş başlıyor.",
                pos.relative_altitude, PAYLOAD_DROP_ALTITUDE,
            )
            self._align_phase = "DROPPING"
            self.nav.set_max_speed(ALIGN_SPEED_MPS, VERTICAL_SPEED_MPS)
            self._send_altitude(PAYLOAD_DROP_ALTITUDE, timeout=20.0)

    # ------------------------------------------------------------------ #
    def _do_start_rtl(self) -> None:
        self.log.info("Görev 3: RTL başlatılıyor. Başarısız=%s", self.data.mission_failed)
        self.rtl_controller.start()
        self._goto(MissionState.RTL)

    def _do_wait_rtl(self) -> None:
        # 1) Önce aracı home'a getirmek için makul bir süre bekle
        arrival_deadline = time.monotonic() + RTL_ARRIVAL_TIMEOUT_S  # home'a varış (5 dk)
        arrived = False
        last_preview_t = 0.0
        preview_interval = 1.0 / RTL_PREVIEW_HZ  # akıcı ama ucuz (küçültülmüş gösterim)
        while time.monotonic() < arrival_deadline:
            # Önizlemeyi RTL_PREVIEW_HZ hızında tazele
            now = time.monotonic()
            if now - last_preview_t >= preview_interval:
                self._pending_display = None
                frame = self.camera.latest()
                if frame is not None:
                    # Gösterim için önce küçült → HUD çizimi + imshow ucuz kalır
                    self._pending_display = self._base_overlay(
                        self._scale_for_display(frame)
                    )
                self._pump_preview()
                last_preview_t = now

            pos = self.nav.position()
            if (self.nav.home_lat is not None
                    and self.nav.home_lon is not None
                    and haversine_m(pos.latitude, pos.longitude,
                                    self.nav.home_lat, self.nav.home_lon) < 2.0
                    and pos.relative_altitude < 1.5):
                arrived = True
                self.log.info("RTL: home'a ulaşıldı (%.2fm). Disarm bekleniyor.", pos.relative_altitude)
                break
            time.sleep(0.02)  # kısa uyku — önizleme hızını nicemlemez

        if not arrived:
            self.log.error("RTL arrival timeout (home'a ulaşılmadı). Görev COMPLETE'a geçiliyor.")
            self._goto(MissionState.COMPLETE)
            return

        # 2) Home'a ulaşıldı — şimdi motorların disarm olmasını bekle
        disarm_deadline = time.monotonic() + RTL_DISARM_TIMEOUT_S  # ek 5 dakika
        try:
            master = getattr(self.nav, 'master', None)
            if master is None:
                self.log.warning("Navigation master erişilemedi; doğrudan COMPLETE'a geçiliyor.")
                self._goto(MissionState.COMPLETE)
                return

            last_preview_t = 0.0
            while time.monotonic() < disarm_deadline:
                now = time.monotonic()
                if now - last_preview_t >= preview_interval:
                    self._pending_display = None
                    frame = self.camera.latest()
                    if frame is not None:
                        self._pending_display = self._base_overlay(
                            self._scale_for_display(frame)
                        )
                    self._pump_preview()
                    last_preview_t = now

                try:
                    armed = master.motors_armed()
                except Exception:
                    # Eğer master ile iletişimde anlık hata olursa tekrar dene
                    armed = True

                if not armed:
                    self.log.info("Araç disarm oldu. RTL tamamlandı.")
                    self._goto(MissionState.COMPLETE)
                    return

                time.sleep(0.02)

            # Timeout — disarm gerçekleşmedi
            self.log.error("RTL disarm bekleme timeout. COMPLETE'a geçiliyor.")
            self._goto(MissionState.COMPLETE)
            return
        except Exception:
            self.log.exception("RTL disarm beklenirken beklenmeyen hata.")
            self._goto(MissionState.COMPLETE)
            return

    # ==================================================================== #
    #  Yardımcı metodlar
    # ==================================================================== #

    def _goto(self, new_state: MissionState) -> None:
        self.log.info("STATE: %s → %s", self.data.state.value, new_state.value)
        self.data.state = new_state

    @staticmethod
    def payload_for_target(target) -> TargetColor:
        if target.color == TargetColor.BLUE and target.shape == TargetType.HEXAGON:
            return TargetColor.RED
        elif target.color == TargetColor.RED and target.shape == TargetType.TRIANGLE:
            return TargetColor.BLUE
        return TargetColor.RED if target.color == TargetColor.BLUE else TargetColor.BLUE

    def _retry_or_fail(self) -> None:
        self._pickup_servo_start = None    # servo bekleme sayacını sıfırla
        self._payload_search_start = None  # payload arama sayacını sıfırla
        self.payload_tracker.reset()       # tracking geçmişini temizle
        self._reset_ref()                  # referans area temizle
        self._verify_stabilize_start = None
        if self.data.pickup_attempts < MAX_PICKUP_ATTEMPTS:
            self.payload.clear_attached()
            self.data.pending_detection = None
            self._reset_altitude_state()
            self.log.info("Pickup yeniden denenecek (%d/%d).",
                          self.data.pickup_attempts, MAX_PICKUP_ATTEMPTS)
            self._goto(MissionState.SEARCHING_PAYLOAD)
        else:
            self.log.error("Maksimum pickup denemesi aşıldı. Görev 2 başarısız.")
            self.data.mission_failed = True
            self._goto(MissionState.TASK_2_FAILED)

    def _safe_reacquire(self) -> None:
        """Hedef tamamen kayboldu — güvenli irtifaya çık ve aramaya dön."""
        self.log.warning("Hedef tamamen kaybedildi. Güvenli irtifaya çıkılıyor.")
        self._reset_altitude_state()
        self.target_detector.history.clear()
        self.target_detector.reset_time_lock()
        self.data.pending_detection = None
        self._gps_candidates.clear()
        self._locked_color = None
        self._locked_shape = None
        self._align_phase = "HORIZONTAL"
        self._last_align_cmd_t = 0.0
        self._no_detection_count = 0
        self._last_recover_t = 0.0
        self._last_align_status_log_t = 0.0
        self.nav.set_max_speed(APPROACH_SPEED_MPS, VERTICAL_SPEED_MPS)
        self._send_altitude(TARGET_APPROACH_ALTITUDE, timeout=20.0)
        self._return_search()

    def _next_step_down_altitude(self, current_alt: float, floor_alt: float) -> float | None:
        """Kademeli iniş: yatay hizalamadan sonra bir sonraki basamak irtifası."""
        if current_alt <= floor_alt + ALTITUDE_TOLERANCE_M:
            return None
        step_target = current_alt - ALIGNMENT_DESCENT_STEP_M
        return max(floor_alt, step_target)

    def _filter_payload_results(self, results: list, altitude_m: float) -> list:
        """
        Payload tespitlerini filtrele ve gerçek payload'a yönlen:
          1. Beklenen boyutun MAX_PAYLOAD_SIZE_FACTOR katından büyükleri at (bunlar hedef).
          2. Görüntünün dış kenarlarına (köşelerine) çok yakın olanları ele (gürültü).
          3. Kalanları ALANA GÖRE KÜÇÜKTEN BÜYÜĞE sırala.
        """
        exp_w, exp_h = expected_payload_pixels(
            PAYLOAD_WIDTH, PAYLOAD_HEIGHT, altitude_m, self.position_controller.camera
        )
        max_w = exp_w * MAX_PAYLOAD_SIZE_FACTOR
        max_h = exp_h * MAX_PAYLOAD_SIZE_FACTOR

        expected_area = exp_w * exp_h
        min_area = expected_area * 0.10
        max_area = expected_area * (MAX_PAYLOAD_SIZE_FACTOR ** 2)

        # Kamera çözünürlüğü (merkeze uzaklık ve köşe/kenar filtresi için)
        cw, ch = self.camera_model.width_px, self.camera_model.height_px
        margin_x = cw * 0.15  # %15 kenar marjı
        margin_y = ch * 0.15

        # PAYLOAD_CORNER_PICK_ALT_M (0.5m) altında köşe/kenar filtresi UYGULANMAZ:
        # bu kadar alçakta İHA payload'ı ortalayamayabilir ve payload köşelere/
        # kenarlara kayabilir. Köşedeki tespitler de geçerli adaydır.
        low_alt = altitude_m <= PAYLOAD_CORNER_PICK_ALT_M

        filtered = [
            r for r in results
            if (r.width_px <= max_w
                and r.height_px <= max_h
                and min_area <= r.area_px <= max_area
                and (low_alt
                     or (margin_x < r.center_x < (cw - margin_x)
                         and margin_y < r.center_y < (ch - margin_y))))
        ]

        if len(filtered) < len(results):
            self.log.debug(
                "Payload filtresi: %d/%d tespit eşikleri geçti (kenar ve boyut)",
                len(filtered), len(results),
            )

        # PAYLOAD_CORNER_PICK_ALT_M altı: köşedeki adaylar dahil — "mavi puan
        # (color_purity) + alan" skoru EN YÜKSEK olan gerçek payload kabul edilir.
        if low_alt and filtered:
            def _low_score(r):
                return r.color_purity + min(
                    1.0, r.area_px / max(1.0, expected_area)
                )

            best = max(filtered, key=_low_score)
            self.log.info(
                "[PAYLOAD PICK] Düşük irtifa (<%.1fm): köşe adaylar dahil — "
                "skor=%.2f alan=%.0fpx² mavi=%.2f merkez=(%.0f,%.0f)",
                PAYLOAD_CORNER_PICK_ALT_M, _low_score(best), best.area_px,
                best.color_purity, best.center_x, best.center_y,
            )
            return [best]

        # Eğer irtifa PAYLOAD_VERIFY_ALTITUDE bandındaysa, bilinen 1m beklenen
        # payload piksel boyutuna göre en uygun adayı seç (area + merkez yakınlığı).
        try:
            from config import (
                PAYLOAD_VERIFY_ALTITUDE,
                VERIFY_ALT_TOLERANCE,
                PAYLOAD_EXPECTED_W_PX_AT_1M,
                PAYLOAD_EXPECTED_H_PX_AT_1M,
            )
        except Exception:
            # Eğer config değerleri yoksa normal filtreyi dön
            return filtered

        if abs(altitude_m - PAYLOAD_VERIFY_ALTITUDE) <= VERIFY_ALT_TOLERANCE and filtered:
            expected_area = float(PAYLOAD_EXPECTED_W_PX_AT_1M) * float(PAYLOAD_EXPECTED_H_PX_AT_1M)
            # referans merkez: görüntü merkezi (approach sırasında hedef ortada olur)
            ref_x = cw / 2.0
            ref_y = ch / 2.0
            # normalize factor for distances: image diagonal
            diag = (cw * cw + ch * ch) ** 0.5

            def _metric(r):
                area_err = abs(r.area_px - expected_area) / max(expected_area, 1.0)
                dx = r.center_x - ref_x
                dy = r.center_y - ref_y
                dist_norm = ((dx * dx + dy * dy) ** 0.5) / max(diag, 1.0)
                # combine — area is primary, center proximity secondary
                return area_err + 0.5 * dist_norm

            best = min(filtered, key=_metric)
            self.log.info(
                "[PAYLOAD PICK] Verify-alt: selecting best candidate by metric: area=%.0f exp=%.0f center=(%.0f,%.0f)",
                best.area_px, expected_area, best.center_x, best.center_y,
            )
            return [best]

        return filtered

    def _nearest_to_last_known(self, candidates: list) -> object | None:
        """
        Candidate listesinden önceki hizalamadan bilinen payload merkezine
        en yakın olanı döndür. Eğer merkez bilgisi yoksa, kamera merkezine en yakın olanı seçer,
        böylece ekranın köşelerindeki gürültü / hedefleri almaktan kaçınır.
        """
        if not candidates:
            return None

        ref_det = self.data.pending_detection
        if ref_det is None:
            # Merkez bilgisi yok — fallback: Görüntü merkezini kullan
            ref_x = self.camera_model.width_px / 2.0
            ref_y = self.camera_model.height_px / 2.0
        else:
            ref_x, ref_y = ref_det.center_x, ref_det.center_y

        def dist(c):
            return (c.center_x - ref_x) ** 2 + (c.center_y - ref_y) ** 2

        return min(candidates, key=dist)

    def _filter_active_detections(self, detections: list) -> list:
        """İşlenmiş hedefleri çıkar; kilit varsa yalnızca kilitli türe odaklan."""
        processed: set[tuple] = set()
        if self.data.first_target is not None and self.data.first_target.payload_released:
            processed.add((self.data.first_target.color, self.data.first_target.shape.value))
        if self.data.second_target is not None and self.data.second_target.payload_released:
            processed.add((self.data.second_target.color, self.data.second_target.shape.value))

        filtered = [
            d for d in detections
            if (d.color, d.shape) not in processed
        ]

        if self._locked_color is not None and self._locked_shape is not None:
            locked = [
                d for d in filtered
                if d.color == self._locked_color and d.shape == self._locked_shape
            ]
            if locked:
                return locked
        return filtered

    def _pick_locked_detection(self, detections: list) -> Detection:
        """Kilitli hedef türüne uygun tespiti seç."""
        if self._locked_color is not None and self._locked_shape is not None:
            for d in detections:
                if d.color == self._locked_color and d.shape == self._locked_shape:
                    return d
        return detections[0]

    def _return_search(self) -> None:
        self.target_detector.history.clear()
        self.data.pending_detection = None
        # Yaklaşma (APPROACH) / hizalama (ALIGN) fazından dönüşte düşük hız
        # search'te kalmasın — her iki search de her zaman SEARCH_SPEED_MPS
        # ile çalışmalı (yalnızca konum referansı farklıdır).
        if self.data.first_target is None:
            self.nav.set_max_speed(SEARCH_SPEED_MPS, VERTICAL_SPEED_MPS)
            self._goto(MissionState.SEARCHING_FIRST_TARGET)
        elif self.data.second_target is None:
            self.nav.set_max_speed(SEARCH_SPEED_MPS, VERTICAL_SPEED_MPS)
            self._goto(MissionState.SEARCHING_SECOND_TARGET)
        else:
            self._goto(MissionState.SEARCHING_PAYLOAD)
