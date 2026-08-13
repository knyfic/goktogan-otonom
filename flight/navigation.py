import math
import time

from pymavlink import mavutil

from models import VehiclePosition


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
                connection_string, baud=baud, force_mavlink2=True
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

    # ------------------------------------------------------------------ #
    #  Pozisyon
    # ------------------------------------------------------------------ #

    def wait_for_position(self, timeout: float = 15.0) -> VehiclePosition:
        """Geçerli bir GPS konumu gelene kadar bekler."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.master.recv_match(
                type="GLOBAL_POSITION_INT",
                blocking=True,
                timeout=1,
            )
            if msg is None:
                continue
            self._update_position(msg)
            if self._position.latitude != 0.0:
                return self._position
        raise TimeoutError("GLOBAL_POSITION_INT alınamadı.")

    def _update_position(self, msg) -> None:
        self._position.latitude = msg.lat / 1e7
        self._position.longitude = msg.lon / 1e7
        self._position.relative_altitude = msg.relative_alt / 1000.0
        self._position.heading_deg = msg.hdg / 100.0
        # Bağlantı canlılık işareti — GLOBAL_POSITION_INT uçuşta sürekli akar.
        self._last_message_s = time.monotonic()

        if self.home_lat is None and self._position.latitude != 0.0:
            self.home_lat = self._position.latitude
            self.home_lon = self._position.longitude

    def position(self) -> VehiclePosition:
        """
        Son bilinen pozisyonu döndürür.
        Yeni MAVLink mesajı varsa önce onu işler (non-blocking).
        """
        msg = self.master.recv_match(
            type="GLOBAL_POSITION_INT",
            blocking=False,
        )
        if msg is not None:
            self._update_position(msg)
        return self._position

    def fresh_position(self, timeout: float = 1.0) -> VehiclePosition:
        """
        Yeni bir GLOBAL_POSITION_INT mesajını bekler.
        Kritik komutlardan önce (goto_global, set_relative_altitude) çağrılır.
        timeout içinde gelmezse son bilinen pozisyonu döndürür.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.master.recv_match(
                type="GLOBAL_POSITION_INT",
                blocking=True,
                timeout=0.1,
            )
            if msg is not None:
                self._update_position(msg)
                return self._position
        return self._position

    # ------------------------------------------------------------------ #
    #  Bağlantı canlılığı (heartbeat watchdog — yalnızca REAL mod)
    # ------------------------------------------------------------------ #

    @property
    def last_message_age_s(self) -> float:
        """Son alınan MAVLink telemetrisinden bu yana geçen süre (saniye)."""
        return time.monotonic() - self._last_message_s

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

    def _ensure_guided(self) -> None:
        """Zaten GUIDED modundaysa tekrar komut gönderme."""
        try:
            mode_id = self.master.mode_mapping().get("GUIDED")
            if mode_id is not None:
                # Mevcut modu kontrol et
                hb = self.master.recv_match(type="HEARTBEAT", blocking=False)
                if hb is not None and hb.custom_mode == mode_id:
                    return
        except Exception:
            pass
        self.set_mode("GUIDED")

    def goto_global(self, latitude: float, longitude: float, altitude_m: float) -> None:
        """
        Belirtilen GPS konumuna ve irtifaya git.
        MAVLink SET_POSITION_TARGET_GLOBAL_INT — yalnızca pozisyon aktif.
        type_mask 0b0000111111111000 = sadece lat/lon/alt geçerli,
        hız/ivme/yaw yoksayılır.
        """
        self._ensure_guided()
        type_mask = 0b0000111111111000

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
            0, 0,
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

        self._ensure_guided()

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
