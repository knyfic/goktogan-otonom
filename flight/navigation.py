import logging
import math
import threading
import time

from pymavlink import mavutil

from models import VehiclePosition
from config import LAND_DISARM_DELAY_S

# Mission logger'ıyla aynı ad — setup_logging() ile aynı akışa/format'a yazılır.
_log = logging.getLogger("iha_mission")

# Başlangıçta bu kadar heartbeat ham alanlarıyla INFO seviyesinde yazdırılır
# (sahada "hangi cihaz ne gönderiyor" teşhisi için — sonra yalnızca değişimler).
_HB_STARTUP_LOG_N = 15


class Navigation:
    """ArduPilot MAVLink adapter. DISARM operasyonu içermez."""

    def __init__(
        self, connection_string: str, baud: int | None = None, flow_control: bool = False
    ):
        # Serial bağlantı (Cube Orange TELEM2 → RPi UART) için baudrate gerekir;
        # UDP/TCP bağlantılarında (simülasyon/HIL) kullanılmaz ve iletilmez.
        if baud is not None:
            # Serial (Cube Orange TELEM2): MAVLink 2 sabit — ArduPilot tarafında
            # SERIAL2_PROTOCOL=2 (MAVLink2) + SERIAL2_BAUD=921 (921600) olmalı.
            self.master = mavutil.mavlink_connection(
                connection_string, baud=baud, force_mavlink2=True, write=True
            )
        else:
            self.master = mavutil.mavlink_connection(connection_string)

        # TELEM2 RTS/CTS bağlıysa ve config'te flow_control: true ise Pi tarafında
        # donanım akış kontrolünü aç — FC'nin TX'i 921600'de kilitlenmesin.
        if baud is not None and flow_control:
            try:
                self.master.port.rtscts = True
            except Exception:
                pass

        # ArduPilot telemetri stream'lerini (SRx_*) yalnızca GCS algıladığında
        # başlatır. Serial bağlantıda her sn GCS heartbeat gönderilir — aksi
        # halde yalnızca 1 Hz HEARTBEAT gelir, pozisyon/telemetri gelmez.
        # (Uçuş komutu değildir; MAVLink el sıkışmasıdır.)
        if baud is not None:
            self._hb_stop = threading.Event()

            def _hb_loop() -> None:
                while not self._hb_stop.is_set():
                    try:
                        self.master.mav.heartbeat_send(
                            mavutil.mavlink.MAV_TYPE_GCS,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                            0, 0, 0,
                        )
                    except Exception:
                        pass
                    self._hb_stop.wait(1.0)

            self._hb_thread = threading.Thread(target=_hb_loop, daemon=True)
            self._hb_thread.start()

        self.master.wait_heartbeat(timeout=30)
        self.system_id = self.master.target_system
        self.component_id = self.master.target_component
        self._position = VehiclePosition()
        self.home_lat = None
        self.home_lon = None
        # Heartbeat watchdog zamanı: son alınan telemetri (monotonic).
        # _update_position() her GLOBAL_POSITION_INT'te tazelenir — bu mesaj
        # uçuş sırasında ~10 Hz akar; link ölürse yaş büyür ve heartbeat_ok()
        # False döner. Mesaj tüketmez (alma buffer'ını kirletmez).
        self._last_message_s = time.monotonic()

        # ---- Pilot flight mode takibi (güvenlik) ----
        # HEARTBEAT custom_mode'dan beslenir (_update_mode). Görev YALNIZCA
        # GUIDED'da uçar; pilot RC'den başka bir mod seçerse mission durur ve
        # Python artık uçuş komutu göndermez (bkz. state_machine PILOT_CONTROL).
        # Mod bilgisi tüketici kodun paylaştığı telemetri akışından (position())
        # alınır — ayrı/blocking bir read eklenmez, mission loop kilitlenmez.
        self._mode_custom: int | None = None
        self._mode_name: str | None = None
        self._mode_last_t: float = 0.0
        # Teşhis: kaç heartbeat işlendi (ilk N tanesi INFO loglanır).
        self._hb_seen: int = 0
        # Armed durumu (HEARTBEAT base_mode SAFETY_ARMED biti — _update_mode tazeler).
        self._armed: bool = False
        # LAND öncesi saklanan LAND_DISARMDELAY orijinal değeri (restore için).
        self._land_disarmdelay_original: float | None = None
        # Asenkron parametre okuma — NON-BLOCKING (mission loop'u kilitlemez,
        # kamera/HUD donmaz). start_param_read() istek gönderir, yanıt telemetri
        # akışında (_dispatch_telemetry → _update_param_value) işlenir,
        # poll_param()/param_read_active() ile sonuç her tick'te okunur.
        self._param_request: str | None = None
        self._param_value: float | None = None
        self._param_request_deadline: float = 0.0

        # ---- Pre-arm / SYS_STATUS / STATUSTEXT durumu ----
        # ArduPilot'un SYS_STATUS sağlık bitleri (onboard_control_sensors_health)
        # üzerinden "arm edilebilir mi?" takip edilir; STATUSTEXT'teki
        # 'PreArm: ...' mesajları son nedeni HUD'da göstermek için saklanır.
        self._sys_status_health = 0
        self._sys_status_present = 0
        self._sys_status_last_t = 0.0
        self._status_text = ""
        self._prearm_reason: str | None = None
        self._prearm_reason_t = 0.0

    # ------------------------------------------------------------------ #
    #  Pozisyon
    # ------------------------------------------------------------------ #

    def wait_for_position(self, timeout: float = 15.0) -> VehiclePosition:
        """Geçerli bir GPS konumu gelene kadar bekler."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.master.recv_match(blocking=True, timeout=1)
            if msg is None:
                continue
            self._dispatch_telemetry(msg)
            if self._position.latitude != 0.0:
                return self._position
        raise TimeoutError("GLOBAL_POSITION_INT alınamadı.")

    def _update_position(self, msg) -> None:
        self._position.latitude = msg.lat / 1e7
        self._position.longitude = msg.lon / 1e7
        self._position.relative_altitude = msg.relative_alt / 1000.0
        self._position.heading_deg = msg.hdg / 100.0
        # GLOBAL_POSITION_INT.vz (cm/s, NED — pozitif aşağı) → dikey hız (m/s,
        # pozitif yükseliş). ArduPilot LAND'de "landed" doğrulamasında kullanılır.
        vz = getattr(msg, "vz", 0) or 0
        self._position.vertical_speed_mps = -float(vz) / 100.0
        # Bağlantı canlılık işareti — GLOBAL_POSITION_INT uçuşta sürekli akar.
        self._last_message_s = time.monotonic()

        if self.home_lat is None and self._position.latitude != 0.0:
            self.home_lat = self._position.latitude
            self.home_lon = self._position.longitude

    def _update_battery_status(self, msg) -> None:
        """BATTERY_STATUS: hücre voltajlarının (mV) toplamı → toplam voltaj.

        65535 = kullanılmayan hücre. Birden çok pil varsa ilk görülen
        (genellikle battery_id=0) güncel kalır.
        """
        cells = [v / 1000.0 for v in msg.voltages if v not in (0xFFFF, 0)]
        if cells:
            self._position.battery_voltage_v = sum(cells)
        if getattr(msg, "battery_remaining", -1) != -1:
            self._position.battery_percent = msg.battery_remaining

    def _update_sys_status(self, msg) -> None:
        """SYS_STATUS: sağlık bitleri + yüzde (pre-arm takibi için).

        ArduPilot'un onboard_control_sensors_health maskesi, "şu an arm
        edilebilir mi?" sorusunun resmî cevabıdır (GCS'ler bunu kullanır):
          • MAV_SYS_STATUS_AHRS          (bit 21) — AHRS/EKF sağlıklı
          • MAV_SYS_STATUS_GPS           (bit 5)  — GPS 3D fix
          • MAV_SYS_STATUS_PREARM_CHECK  (bit 28) — pre-arm kontrolleri geçti
        """
        self._sys_status_present = getattr(msg, "onboard_control_sensors_present", 0)
        self._sys_status_health = getattr(msg, "onboard_control_sensors_health", 0)
        self._sys_status_last_t = time.monotonic()
        if getattr(msg, "battery_remaining", -1) != -1:
            self._position.battery_percent = msg.battery_remaining

    @staticmethod
    def accept_autopilot_heartbeat(msg) -> tuple[bool, str]:
        """HEARTBEAT gerçek ArduPilot autopilot'undan mı? (mod kaynağı kabulü)

        _update_mode() ile tests/hardware/test_real_mode_telemetry.py'nin
        PAYLAŞTIĞI TEK kaynak — pilot takeover tespiti ve saha teşhisi aynı
        mantığı kullanır (test kendi basitleştirilmiş filtre icat etmez).

        Kabul şartları:
          1) autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA (3)
          2) base_mode'da MAV_MODE_FLAG_CUSTOM_MODE_ENABLED biti set
             (custom_mode yalnızca bu bitle anlamlıdır)
          3) type != MAV_TYPE_GCS

        GCS / RC telemetri / diğer cihaz heartbeat'leri custom_mode=0 taşıyıp
        yanlış 'STABILIZE' okumasına yol açabilir (real modda gözlemlendi).

        Returns: (accepted, reason) — accepted=True iken reason boştur.
        """
        custom = getattr(msg, "custom_mode", None)
        if custom is None:
            return False, "custom_mode yok (HEARTBEAT değil)"
        autopilot = getattr(msg, "autopilot", 0)
        base_mode = getattr(msg, "base_mode", 0)
        mav_type = getattr(msg, "type", 0)
        if autopilot != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            return (
                False,
                f"autopilot={autopilot} (ARDUPILOTMEGA="
                f"{mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA} değil)",
            )
        if not (base_mode & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED):
            return (
                False,
                f"base_mode=0x{base_mode:02X} CUSTOM_MODE_ENABLED biti yok",
            )
        if mav_type == mavutil.mavlink.MAV_TYPE_GCS:
            return False, f"type={mav_type} MAV_TYPE_GCS"
        return True, ""

    def _update_mode(self, msg) -> None:
        """HEARTBEAT → mevcut flight mode (pilot takeover tespiti).

        Kabul mantığı Navigation.accept_autopilot_heartbeat() ile PAYLAŞILIR —
        yalnızca gerçek ArduPilot autopilot heartbeat'i modu taşır. Başka
        kaynakların (RC telemetri modülü, başka GCS) custom_mode=0 heartbeat'i
        yanlış 'STABILIZE' okumasına yol açabilir (real modda gözlemlendi —
        SITL'de tek cihaz olduğu için görünmez).
        Mod adı, ArduPilot mode_mapping'in tersinden çözülür (ör. 4 → 'GUIDED',
        9 → 'LAND'). Mapping çözülemezse ad None kalır (mod bilinmiyor).
        """
        custom = getattr(msg, "custom_mode", None)
        if custom is None:
            return
        src_sys = msg.get_srcSystem() if hasattr(msg, "get_srcSystem") else None
        src_comp = msg.get_srcComponent() if hasattr(msg, "get_srcComponent") else None
        mav_type = getattr(msg, "type", 0)
        autopilot = getattr(msg, "autopilot", 0)
        base_mode = getattr(msg, "base_mode", 0)

        self._hb_seen = getattr(self, "_hb_seen", 0) + 1
        first = self._hb_seen <= _HB_STARTUP_LOG_N

        accepted, reason = self.accept_autopilot_heartbeat(msg)

        if first:
            # Başlangıç teşhisi: hangi cihazlar heartbeat gönderiyor, içeriği ne?
            _log.info(
                "HB#%d src=%s/%s type=%s autopilot=%s base=0x%02X custom=%s → %s",
                self._hb_seen, src_sys, src_comp, mav_type, autopilot,
                base_mode, int(custom),
                "kabul (mod kaynağı)" if accepted else f"YOKSAYILDI ({reason})",
            )
        else:
            _log.debug(
                "HB src=%s/%s type=%s autopilot=%s base=0x%02X custom=%s → %s",
                src_sys, src_comp, mav_type, autopilot, base_mode, int(custom),
                "kabul" if accepted else f"yoksay ({reason})",
            )

        if not accepted:
            return

        # Armed durumu: HEARTBEAT base_mode SAFETY_ARMED biti (7) — LAND sonrası
        # "disarm olursa asla TAKEOFF/re-arm yapma" güvenlik kontrolünde kullanılır.
        self._armed = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        self._mode_custom = int(custom)
        self._mode_last_t = time.monotonic()
        try:
            mapping = self.master.mode_mapping() or {}
            rev = {v: k for k, v in mapping.items()}
            name = rev.get(int(custom))
        except Exception:
            name = None
        if name is None:
            self._mode_name = None
            _log.warning(
                "Autopilot heartbeat custom_mode=%s ad çözülemedi (mode_mapping boş)",
                int(custom),
            )
            return
        if name != self._mode_name:
            _log.info("Flight mode: %s -> %s (custom=%s)", self._mode_name, name, int(custom))
        self._mode_name = name

    def _update_status_text(self, msg) -> None:
        """STATUSTEXT: son mesaj + 'PreArm:' ön ekli mesajları sakla.

        ArduPilot pre-arm hatalarını 'PreArm: <neden>' biçiminde
        MAV_SEVERITY_CRITICAL ile yayınlar — neden HUD'da gösterilir.
        """
        text = getattr(msg, "text", b"")
        if isinstance(text, bytes):
            text = text.decode(errors="replace")
        text = str(text).strip()
        if not text:
            return
        self._status_text = text
        if "prearm" in text.lower():
            self._prearm_reason = text
            self._prearm_reason_t = time.monotonic()

    def _dispatch_telemetry(self, msg) -> None:
        """Gelen telemetri mesajını türüne göre işler (pozisyon + batarya + durum)."""
        t = msg.get_type()
        if t == "GLOBAL_POSITION_INT":
            self._update_position(msg)
        elif t == "BATTERY_STATUS":
            self._update_battery_status(msg)
        elif t == "SYS_STATUS":
            self._update_sys_status(msg)
        elif t == "STATUSTEXT":
            self._update_status_text(msg)
        elif t == "HEARTBEAT":
            self._update_mode(msg)
        elif t == "PARAM_VALUE":
            self._update_param_value(msg)

    def _update_param_value(self, msg) -> None:
        """PARAM_VALUE → beklenen asenkron parametre okuma isteğini doldurur.

        start_param_read() ile istenen paramın yanıtı gelirse _param_value'ya
        yazılır; poll_param() bunu döner. Bekleyen istek yoksa yoksayılır.
        """
        if getattr(self, "_param_request", None) is None:
            return
        pid = getattr(msg, "param_id", b"")
        if isinstance(pid, bytes):
            pid = pid.decode(errors="replace").rstrip("\x00")
        if pid == self._param_request:
            self._param_value = float(getattr(msg, "param_value", 0.0))

    def position(self) -> VehiclePosition:
        """
        Son bilinen pozisyonu döndürür.
        Yeni MAVLink mesajları varsa önce onları işler (non-blocking) —
        pozisyon ve batarya telemetrisi buradan canlı güncellenir.
        """
        # Her çağrıda buffer'dan en fazla 10 mesaj işle — hem pozisyon hem
        # batarya (BATTERY_STATUS ~1-2 Hz) gecikmesiz işlensin.
        for _ in range(10):
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break
            self._dispatch_telemetry(msg)
        return self._position

    def fresh_position(self, timeout: float = 1.0) -> VehiclePosition:
        """
        Yeni bir GLOBAL_POSITION_INT mesajını bekler.
        Kritik komutlardan önce (goto_global, set_relative_altitude) çağrılır.
        timeout içinde gelmezse son bilinen pozisyonu döndürür.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.master.recv_match(blocking=True, timeout=0.1)
            if msg is None:
                continue
            self._dispatch_telemetry(msg)
            if msg.get_type() == "GLOBAL_POSITION_INT":
                return self._position
        return self._position

    # ------------------------------------------------------------------ #
    #  Bağlantı canlılığı (heartbeat watchdog — yalnızca REAL mod)
    # ------------------------------------------------------------------ #

    @property
    def last_message_age_s(self) -> float:
        """Son alınan MAVLink telemetrisinden bu yana geçen süre (saniye)."""
        return time.monotonic() - self._last_message_s

    @property
    def current_flight_mode(self) -> str | None:
        """Son bilinen flight mode adı ('GUIDED', 'LAND', ...). None = bilinmiyor."""
        return self._mode_name

    @property
    def armed(self) -> bool:
        """Araç şu an armed mı? (HEARTBEAT base_mode SAFETY_ARMED biti — ~1Hz taze)

        LAND sonrası "disarm olursa ASLA TAKEOFF/re-arm yapma" güvenlik
        kontrolünde kullanılır. Bayat olabilir (en fazla ~1s) — kritik kararlar
        (TAKEOFF) için ayrıca mode/fresh doğrulaması yapılır.
        """
        return bool(getattr(self, "_armed", False))

    def flight_mode_fresh(self, timeout_s: float = 5.0) -> bool:
        """Mod bilgisi bu süre içinde tazelendi mi?

        Bayat mod bilgisiyle karar verilmez (ör. telemetri kesildi, mod adı
        eski kaldı) — pilot takeover tespiti yalnızca taze modla yapılır.
        """
        return self._mode_last_t > 0.0 and (time.monotonic() - self._mode_last_t) <= timeout_s

    def heartbeat_ok(self, timeout_s: float) -> bool:
        """
        MAVLink bağlantısı canlı mı? (non-blocking, mesaj tüketmez)

        Uçuş sırasında GLOBAL_POSITION_INT ~10 Hz akar; _update_position()
        her mesajda _last_message_s'i tazeler. timeout_s içinde hiç mesaj
        gelmediyse bağlantının koptuğu varsayılır.

        Not: Ham HEARTBEAT paketi değil, telemetri akışı takip edilir — görev
        sırasında bağlantı sağlığının en güvenilir göstergesidir.
        """
        return self.last_message_age_s <= timeout_s

    # ------------------------------------------------------------------ #
    #  Pre-arm durumu (ArduPilot)
    # ------------------------------------------------------------------ #

    # MAV_SYS_STATUS bitleri (pymavlink mavlink enums ile aynı değerler)
    _SYS_AHRS         = 1 << 21   # AHRS / EKF sağlıklı
    _SYS_GPS          = 1 << 5    # GPS 3D fix
    _SYS_PREARM_CHECK = 1 << 28   # pre-arm kontrolleri geçti

    # "Arm edilebilir" için gereken sağlık bitleri. PREARM_CHECK biti
    # ArduPilot'un kendi arming-check özetidir; AHRS + GPS ayrıca istenir
    # (ikisi de bu görev için zorunlu — search GPS'e dayanır).
    _PREARM_REQUIRED_HEALTH = _SYS_AHRS | _SYS_GPS | _SYS_PREARM_CHECK

    # SYS_STATUS ~1-2 Hz akar; bu süre içinde tazelenmediyse veri bayat kabul edilir.
    _SYS_STATUS_FRESH_S = 5.0

    def prearm_good(self) -> bool | None:
        """
        ArduPilot şu an ARM komutunu kabul edecek kadar sağlıklı mı?

          True  → pre-arm kontrolleri geçti (arm edilebilir)
          False → geçmedi (bir engel var — prearm_reason() ile nedeni gör)
          None  → henüz güvenilir veri yok (SYS_STATUS gelmedi / bayat)

        Kırılgan "son PreArm mesajı kesildi → GOOD" mantığı KULLANILMAZ;
        sinyal, ArduPilot'un SYS_STATUS sağlık bitleridir (GCS standardı).
        """
        if self._sys_status_last_t == 0.0:
            return None
        if time.monotonic() - self._sys_status_last_t > self._SYS_STATUS_FRESH_S:
            return None   # bayat — güvenme
        return (
            (self._sys_status_health & self._PREARM_REQUIRED_HEALTH)
            == self._PREARM_REQUIRED_HEALTH
        )

    def prearm_reason(self) -> str | None:
        """Son bilinen 'PreArm: ...' STATUSTEXT mesajı (HUD için). None = yok."""
        return self._prearm_reason

    # ------------------------------------------------------------------ #
    #  Mod & Arm
    # ------------------------------------------------------------------ #

    def set_mode(self, mode: str) -> None:
        mapping = self.master.mode_mapping()
        if mode not in mapping:
            raise RuntimeError(f"ArduPilot mode desteklenmiyor: {mode}")
        self.master.set_mode(mapping[mode])

    def arm(self) -> None:
        self.master.arducopter_arm()
        self.master.motors_armed_wait()

    # ------------------------------------------------------------------ #
    #  Uçuş komutları
    # ------------------------------------------------------------------ #

    def takeoff(self, altitude_m: float) -> None:
        self.set_mode("GUIDED")
        if not self.master.motors_armed():
            self.arm()

        self.master.mav.command_long_send(
            self.system_id,
            self.component_id,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            altitude_m,
        )

    def set_max_speed(self, horizontal_mps: float, vertical_mps: float | None = None) -> None:
        """
        ArduPilot GUIDED mod hız parametrelerini günder.
        MAV_CMD_DO_CHANGE_SPEED ile yatay hız sınırı belirlenir.
        speed_type=1 = ground speed.
        """
        self.master.mav.command_long_send(
            self.system_id,
            self.component_id,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,
            1,               # speed_type: 1=ground speed
            horizontal_mps,  # hedef hız
            -1,              # throttle: -1=değiştirme
            0, 0, 0, 0,
        )
        if vertical_mps is not None:
            # WPNAV_SPEED_UP / DOWN parametreleri üzerinden ayarlamak için
            # param set kullanırız
            self.master.mav.param_set_send(
                self.system_id,
                self.component_id,
                b"WPNAV_SPEED_UP",
                vertical_mps * 100,  # cm/s
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
            self.master.mav.param_set_send(
                self.system_id,
                self.component_id,
                b"WPNAV_SPEED_DN",
                vertical_mps * 100,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )

    def goto_global(
        self,
        latitude: float,
        longitude: float,
        altitude_m: float,
        yaw_deg: float | None = None,
    ) -> None:
        """
        Belirtilen GPS konumuna ve irtifaya git.
        MAVLink SET_POSITION_TARGET_GLOBAL_INT — yalnızca pozisyon aktif.
        type_mask 0b0000111111111000 = sadece lat/lon/alt geçerli,
        hız/ivme/yaw yoksayılır.

        yaw_deg verilirse: yaw AKTİF yapılır ve bu heading SABİT tutulur.
        Hizalama sırasında İHA burnunun komutlar arasında dönmesi, düzeltme
        vektörlerinin gecikmeli heading'le hesaplanmasına ve hedef etrafında
        dönmeye (daire) yol açar; yaw kilidi bunu önler.

        NOT (güvenlik): Bu fonksiyon GUIDED modu ASLA yeniden göndermez.
        Pilot RC'den başka bir mod seçtiyse ArduPilot bu pozisyon komutunu
        zaten yoksayar (GUIDED dışı modlarda SET_POSITION_TARGET işlenmez) —
        pilotun seçtiği mod asla ezilmez.
        """
        type_mask = 0b0000111111111000
        yaw_rad = 0.0
        if yaw_deg is not None:
            # bit 10 (yaw ignore) = 0 → yaw aktif, mevcut heading verilir
            type_mask &= ~(1 << 10)
            yaw_rad = math.radians(yaw_deg)

        self.master.mav.set_position_target_global_int_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            self.system_id,
            self.component_id,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            int(latitude * 1e7),
            int(longitude * 1e7),
            altitude_m,
            0, 0, 0,
            0, 0, 0,
            yaw_rad,  # yaw — yaw_deg verildiyse sabitlenir
            0,        # yaw_rate
        )

    def goto_body_offset(self, forward_m: float, right_m: float) -> None:
        """
        Aracın gövde ekseninde ileri/sağ yönünde pozisyon offseti gönderir.

        MAV_FRAME_BODY_OFFSET_NED (LOCAL_NED) ArduPilot'ta güvenilir değildir:
        offset, mevcut konuma değil önceki position target'e eklenir ve yerleşik
        (settled) araçta çoğu zaman hiç hareket üretmez. Bu yüzden burada
        kanıtlanmış yöntem kullanılır — hizalama döngüsünün kullandığı
        goto_global yolu:

          body offset → heading ile NED (north/east) → GPS hedefi → goto_global

        İrtifa korunur (mevcut relative_altitude hedef olarak verilir).
        """
        pos = self.position()
        if pos.latitude == 0.0 and pos.longitude == 0.0:
            return  # GPS henüz yok — komut gönderilemez

        yaw_rad = math.radians(pos.heading_deg)
        north_m = forward_m * math.cos(yaw_rad) - right_m * math.sin(yaw_rad)
        east_m = forward_m * math.sin(yaw_rad) + right_m * math.cos(yaw_rad)

        # NED offset → GPS hedefi (küresel yaklaşık dönüşüm)
        earth_radius = 6378137.0
        d_lat = north_m / earth_radius
        d_lon = east_m / (earth_radius * math.cos(math.radians(pos.latitude)))
        target_lat = pos.latitude + math.degrees(d_lat)
        target_lon = pos.longitude + math.degrees(d_lon)

        self.goto_global(target_lat, target_lon, pos.relative_altitude)

    def set_relative_altitude(self, altitude_m: float) -> None:
        """
        Mevcut yatay konum ve yaw'ı koruyarak yeni irtifaya git.
        Güncel GPS konumunu alır; geçersizse komutu atlar.

        Yaw sabitlenir: iniş sırasında drone burnunu döndürmez.
        type_mask'te yaw alanı aktif bırakılır ve mevcut heading verilir.
        """
        pos = self.position()   # non-blocking — fresh_position gibi blocking değil
        if pos.latitude == 0.0 and pos.longitude == 0.0:
            return

        # NOT (güvenlik): GUIDED mod yeniden gönderilmez — pilotun RC modu
        # asla ezilmez. GUIDED dışı modda bu komut ArduPilot tarafından yoksayılır.

        # type_mask: sadece lat/lon/alt + yaw aktif; hız/ivme yoksayılır
        # bit 10 (yaw) = 0 → aktif (mevcut heading'i koru)
        type_mask = (
            (1 << 3)   # vx yoksay
            | (1 << 4) # vy yoksay
            | (1 << 5) # vz yoksay
            | (1 << 6) # ax yoksay
            | (1 << 7) # ay yoksay
            | (1 << 8) # az yoksay
            # bit 0,1,2 (x,y,z pos) = 0 → aktif
            # bit 10 (yaw) = 0 → aktif, mevcut değer verilecek
            # bit 11 (yaw_rate) = 1 → yoksay
            | (1 << 11)
        )

        yaw_rad = math.radians(pos.heading_deg)

        self.master.mav.set_position_target_global_int_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            self.system_id,
            self.component_id,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            int(pos.latitude * 1e7),
            int(pos.longitude * 1e7),
            altitude_m,
            0, 0, 0,    # vx, vy, vz
            0, 0, 0,    # ax, ay, az
            yaw_rad,    # yaw — mevcut heading'i koru
            0,          # yaw_rate
        )

    def rtl(self) -> None:
        """Return To Launch. DISARM komutu içermez."""
        self.master.mav.command_long_send(
            self.system_id,
            self.component_id,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0, 0, 0, 0, 0, 0, 0,
        )

    def land(self) -> None:
        """ArduPilot native LAND (payload alma inişi).

        MAV_CMD_NAV_LAND — ArduCopter bu komutta LAND moduna geçer ve kendi
        LAND controller'ı ile kontrollü iniş yapar. Mod değiştirir; DISARM
        içermez (LAND_DISARMDELAY=LAND_DISARM_DELAY_S garanti edilmişse yerde
        yalnızca 15 sn sonra güvenlik disarm'ı olur — normal pickup akışı bu
        sürede TAKEOFF'a kalkar).
        """
        self.master.mav.command_long_send(
            self.system_id,
            self.component_id,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0, 0,
        )

    def takeoff_only(self, altitude_m: float) -> None:
        """Yalnızca MAV_CMD_NAV_TAKEOFF gönderir — mod DEĞİŞTİRMEZ, ARM ETMEZ.

        Çağıran; GUIDED aktif + araç armed olduğunu DOĞRULAMIŞ olmalıdır.
        Güvenlik: disarm sonrası otomatik re-arm KESİNLİKLE yapılmaz
        (Navigation.takeoff()'un arm() davranışından kasıtlı olarak ayrıdır).
        """
        self.master.mav.command_long_send(
            self.system_id,
            self.component_id,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            altitude_m,
        )

    def start_param_read(self, name: str, timeout_s: float = 2.0) -> bool:
        """PARAM_REQUEST_READ gönderir — NON-BLOCKING.

        Yanıt telemetri akışında işlenir (_update_param_value); mission loop'u
        kilitlemez. Sonuç poll_param() / param_read_active() ile okunur.
        Başarılı gönderimse True döner (yanıt garantisi YOKTUR — FC yanıt
        vermeyebilir; timeout sonrası poll_param (False, None) döner).
        """
        try:
            self.master.mav.param_request_read_send(
                self.system_id, self.component_id, name.encode(), -1
            )
        except Exception:
            return False
        self._param_request = name
        self._param_value = None
        self._param_request_deadline = time.monotonic() + timeout_s
        return True

    def param_read_active(self) -> bool:
        """Bekleyen bir parametre okuma isteği var mı? (timeout dolmadıysa True) """
        return getattr(self, "_param_request", None) is not None

    def poll_param(self) -> tuple[bool, float | None]:
        """Beklenen parametre yanıtını döner — NON-BLOCKING, her tick çağrılır.

        (True, değer)  → yanıt geldi (isteği sıfırlar).
        (False, None)  → henüz yok VEYA timeout doldu. Ayrım için
        param_read_active(): True ise hâlâ bekleniyor, False ise zaman aşımı.
        """
        if getattr(self, "_param_request", None) is None:
            return False, None
        if self._param_value is not None:
            self._param_request = None
            return True, self._param_value
        if time.monotonic() > self._param_request_deadline:
            self._param_request = None
            return False, None
        return False, None

    def send_param(self, name: str, value: float) -> bool:
        """PARAM_SET gönderir — NON-BLOCKING. Gönderim başarılıysa True."""
        try:
            self.master.mav.param_set_send(
                self.system_id,
                self.component_id,
                name.encode(),
                value,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
            return True
        except Exception:
            return False

    def restore_land_disarmdelay(self, original: float | None = None) -> bool:
        """LAND_DISARMDELAY'i LAND öncesindeki orijinal değerine geri yükler.

        original verilmezse kendi sakladığı değeri kullanır (eski çağrılar).
        Başarısızsa False (log uyarısı — güvenli degrade: hedef değer kalırsa
        sonraki RTL disarm beklemesi zaman aşımına düşüp COMPLETE olur).
        """
        if original is None:
            original = self._land_disarmdelay_original
        if original is None:
            return True
        try:
            self.master.mav.param_set_send(
                self.system_id,
                self.component_id,
                b"LAND_DISARMDELAY",
                original,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
            _log.info("LAND_DISARMDELAY orijinal değerine döndürüldü: %.1f", original)
            self._land_disarmdelay_original = None
            return True
        except Exception:
            _log.error("LAND_DISARMDELAY geri yüklenemedi (%.1f) — uyarıyı inceleyin.",
                       self._land_disarmdelay_original)
            return False

    # ------------------------------------------------------------------ #
    #  Bekleme yardımcıları
    # ------------------------------------------------------------------ #

    def wait_until_altitude(
        self, target: float, tolerance: float, timeout: float
    ) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if abs(self.position().relative_altitude - target) <= tolerance:
                return True
            time.sleep(0.1)
        return False

    def wait_until_near_global(
        self, lat: float, lon: float, tolerance_m: float, timeout: float
    ) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            pos = self.position()
            if haversine_m(pos.latitude, pos.longitude, lat, lon) <= tolerance_m:
                return True
            time.sleep(0.2)
        return False

    def wait_for_rtl_completion(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            pos = self.position()
            if self.home_lat is not None and self.home_lon is not None:
                d = haversine_m(
                    pos.latitude, pos.longitude,
                    self.home_lat, self.home_lon,
                )
                if d < 2.0 and pos.relative_altitude < 1.5:
                    return True
            time.sleep(0.5)
        return False


# ------------------------------------------------------------------ #

def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """İki GPS noktası arasındaki mesafeyi metre cinsinden hesaplar."""
    radius = 6378137.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = (
        math.sin(dp / 2) ** 2
        + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    )
    return 2 * radius * math.asin(math.sqrt(a))
