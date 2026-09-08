#!/usr/bin/env python3
"""
Direk (pole) konumu ölçümü — Raspberry Pi 5 + Cube Orange, READ-ONLY.

  python tests/hardware/test_real_mode_position.py [--duration 10] [--timeout 10]

KULLANIM ŞEKLİ:
  İHA fiziksel olarak statik direğin hemen YANINA konur. İHA'nın okunan global
  GPS koordinatı, direğin koordinatı olarak kabul edilir (kamera/mesafe/bearing
  hesabı YOK — İHA zaten direğin yanında).

YAPILAN İŞ:
  1. REAL modda MAVLink bağlantısı kur (GCS heartbeat el sıkışması — komut değil).
  2. GLOBAL_POSITION_INT → ArduPilot EKF fused latitude/longitude oku
     (mission'ın Navigation.position() ile kullandığı BİREBİR aynı kaynak).
  3. İHA sabitken POSITION_SAMPLE_DURATION_S boyunca örnek topla.
  4. GPS jitter'ını azaltmak için 3σ outlier eleme + ortalama (robust) uygula;
     median referans olarak gösterilir.
  5. Sonucu açık şekilde yazdır:

       Pole position:
       Latitude : 37.9331872 deg
       Longitude: 34.6177522 deg

  Ayrıca örnek dağılımı (spread) ve tahmini konum sapması (std) gösterilir.

KULLANILMAYANLAR:
  ✗ LOCAL_POSITION_NED / HOME_POSITION / EKF-origin dönüşümleri
  ✗ kamera, mesafe, bearing, kamera geometrisi
  ✗ fence / Mission Planner entegrasyonu / collision avoidance
  ✗ state_machine değişikliği
  ✗ ArduPilot'a HİÇBİR komut (arm/takeoff/goto/land/fence/param YOK)
  ✓ yalnızca telemetri okuma (GPS_RAW_INT yalnızca kalite bilgisidir — ham
    GPS koordinatı EKF konumunun yerine KONULMAZ)

pytest olarak da çalışır — gerçek donanım testi Raspberry Pi dışında SKIP,
saf hesaplama testleri her yerde koşar:
  python -m pytest tests/hardware/test_real_mode_position.py -v
"""
from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import pytest

from tests.hardware import hw_common as hw

# GPS→metre dönüşümü — yalnızca örnek dağılımını (spread/std) metre cinsinden
# göstermek için kullanılır (search/coverage_planner mevcut helper'ı).
from search.coverage_planner import _north_east_m as _gps_delta_north_east_m
from search.coverage_planner import local_to_gps as _gps_from_north_east_m

# Testin hiçbir yerinde komut gönderilmediğini doğrulamak için yasak API listesi
# (yalnızca dokümantasyon amaçlı — bu modülde bu çağrılar bulunmamalı).
_FORBIDDEN = ("command_long_send", "set_mode", "arducopter_arm", "motors_armed_wait",
              "set_position_target", "set_servo", "rc_channels_override")

# ── Ölçüm sabitleri (script başında kolayca değiştirilir) ────────────────────
POSITION_SAMPLE_DURATION_S = 10.0     # örnek toplama süresi
POSITION_SAMPLE_INTERVAL_S = 0.2      # örnek aralığı (5 Hz)
MIN_POSITION_SAMPLES = 10             # altında → FAIL
POSITION_ACQUIRE_TIMEOUT_S = 15.0     # geçerli EKF konumu bekleme
POSITION_DATA_TIMEOUT_S = 2.0         # bu süre konum mesajı gelmezse "kesinti"
OUTLIER_STD_MULT = 3.0                # outlier eşiği: ortalama etrafı 3σ
OUTLIER_MIN_M = 1.5                   # outlier için mutlak min eşik (metre)
SPREAD_WARN_M = 2.0                   # yayılım bu değeri aşarsa WARNING

# ArduPilot GPS fix tipleri (GPS_FIX_TYPE) — yalnızca kalite bilgisi
_FIX_NAMES = {0: "NO GPS", 1: "NO FIX", 2: "2D FIX", 3: "3D FIX",
              4: "3D DGPS", 5: "RTK FLOAT", 6: "RTK FIXED"}
_MIN_GOOD_FIX = 3

# SYS_STATUS GPS sağlık biti (GPS_RAW_INT akışta yoksa fix doğrulaması için)
_SYS_GPS = 1 << 5

_HB_INTERVAL_S = 1.0   # GCS heartbeat aralığı (el sıkışması — komut değil)

_PRINT_SAMPLE_LIMIT = 4   # terminalde gösterilen örnek satırı (ilk + son)


# ══════════════════════════════════════════════════════════════════════════ #
#  Saf hesaplama fonksiyonları (unit test edilebilir — donanım gerektirmez)
# ══════════════════════════════════════════════════════════════════════════ #

def _mean(xs: list[float]) -> float:
    return sum(xs) / len(xs) if xs else 0.0


def _median(xs: list[float]) -> float:
    return statistics.median(xs) if xs else 0.0


def _estimate_position(
    samples: list[tuple[float, float]],
    outlier_std_mult: float = OUTLIER_STD_MULT,
    outlier_min_m: float = OUTLIER_MIN_M,
) -> dict:
    """lat/lon örneklerinden robust konum tahmini (direk koordinatı).

    Yöntem:
      1) Başlangıç referansı = basit ortalama (küçük alan — yeterli).
      2) Her örneği referansa göre (north, east) metreye çevir.
      3) Ortalama offset + radyal std; eşik = max(3σ, outlier_min_m).
      4) Eşiği aşan örnekleri outlier işaretle, kalanla yeniden ortalama.
      5) Final konum = referans + ortalama offset.

    Dönen dict: lat, lon, median_lat, median_lon, inlier_n, outlier_n,
                spread_m (inlier'ların final konuma max uzaklığı), std_m
    """
    if not samples:
        raise ValueError("örnek yok")

    n = len(samples)
    ref_lat = _mean([s[0] for s in samples])
    ref_lon = _mean([s[1] for s in samples])

    offsets = [_gps_delta_north_east_m(ref_lat, ref_lon, lat, lon)
               for lat, lon in samples]
    mean_n = _mean([o[0] for o in offsets])
    mean_e = _mean([o[1] for o in offsets])
    dists = [math.hypot(o[0] - mean_n, o[1] - mean_e) for o in offsets]

    std = statistics.pstdev(dists) if n > 1 else 0.0
    threshold = max(outlier_std_mult * std, float(outlier_min_m))
    inlier_idx = [i for i, d in enumerate(dists) if d <= threshold]
    if not inlier_idx:              # aşırı dağınık GPS — hepsini koru
        inlier_idx = list(range(n))

    in_n = [offsets[i][0] for i in inlier_idx]
    in_e = [offsets[i][1] for i in inlier_idx]
    fin_n = _mean(in_n)
    fin_e = _mean(in_e)
    est_lat, est_lon = _gps_from_north_east_m(ref_lat, ref_lon, fin_n, fin_e)

    fin_dists = [
        math.hypot(*_gps_delta_north_east_m(est_lat, est_lon, lat, lon))
        for lat, lon in [samples[i] for i in inlier_idx]
    ]
    spread = max(fin_dists) if fin_dists else 0.0
    fin_std = statistics.pstdev(fin_dists) if len(fin_dists) > 1 else 0.0

    return {
        "lat": est_lat,
        "lon": est_lon,
        "median_lat": _median([s[0] for s in samples]),
        "median_lon": _median([s[1] for s in samples]),
        "inlier_n": len(inlier_idx),
        "outlier_n": n - len(inlier_idx),
        "spread_m": spread,
        "std_m": fin_std,
    }


# ══════════════════════════════════════════════════════════════════════════ #
#  Donanım ölçümü — READ-ONLY MAVLink telemetri
# ══════════════════════════════════════════════════════════════════════════ #

def _dispatch(msg, state: dict) -> None:
    """Gelen mesajı işler. Yalnızca telemetri okur — komut göndermez."""
    t = msg.get_type()
    if t == "GLOBAL_POSITION_INT":
        # ArduPilot EKF fused global konum — mission'ın kullandığı TEK kaynak
        state["lat"] = msg.lat / 1e7
        state["lon"] = msg.lon / 1e7
        state["pos_t"] = time.monotonic()
    elif t == "GPS_RAW_INT":
        # Ham GPS yalnızca KALİTE bilgisi — koordinat olarak kullanılmaz
        state["fix_type"] = getattr(msg, "fix_type", 0)
        sats = getattr(msg, "satellites_visible", 0)
        eph = getattr(msg, "eph", 0)                # ArduPilot: HDOP × 100
        state["sats_list"].append(int(sats))
        state["hdop_list"].append(float(eph) / 100.0)
    elif t == "SYS_STATUS":
        # GPS_RAW_INT akışta yoksa fix doğrulaması için (GPS sağlık biti)
        state["sys_health"] = getattr(msg, "onboard_control_sensors_health", 0)
        state["sys_t"] = time.monotonic()


def _gps_ok(state: dict) -> tuple[bool, str]:
    """GPS fix yeterli mi? (GPS_RAW_INT fix_type veya SYS_STATUS GPS biti)."""
    if state.get("fix_type") is not None and state["fix_type"] >= _MIN_GOOD_FIX:
        return True, _FIX_NAMES.get(state["fix_type"], f"FIX {state['fix_type']}")
    if state.get("sys_t"):
        if state["sys_health"] & _SYS_GPS:
            return True, "3D FIX (SYS_STATUS)"
        return False, "GPS sağlıksız (SYS_STATUS GPS biti yok)"
    return False, "GPS fix doğrulanamıyor (GPS_RAW_INT + SYS_STATUS yok)"


def run_position_test(
    device: str,
    baud: int,
    duration_s: float = POSITION_SAMPLE_DURATION_S,
    timeout_s: float = 10.0,
    flow_control: bool = False,
    interval_s: float = POSITION_SAMPLE_INTERVAL_S,
    acquire_timeout_s: float = POSITION_ACQUIRE_TIMEOUT_S,
) -> bool:
    """Direk (pole) konumunu ölçer ve yazdırır. Başarılıysa True döner."""
    print("=" * 56)
    print("      POLE POSITION MEASUREMENT")
    print("=" * 56)
    print("  (İHA direğin yanında sabit — okunan konum = direk konumu)")

    try:
        master, hb = hw.open_mavlink(
            device, baud, timeout_s=timeout_s, flow_control=flow_control
        )
    except TimeoutError as exc:
        print(f"[FAIL] MAVLink bağlantısı  ({exc})")
        return False
    except Exception as exc:
        print(f"[FAIL] MAVLink bağlantısı  ({exc})")
        return False

    mode = hw.flight_mode_name(master, hb)
    print(f"MAVLink              : CONNECTED  (mode={mode})")

    state = {
        "lat": 0.0, "lon": 0.0, "pos_t": 0.0,
        "fix_type": None, "sats_list": [], "hdop_list": [],
        "sys_health": 0, "sys_t": 0.0,
    }

    try:
        # ── Geçerli EKF konumu bekle (GLOBAL_POSITION_INT) ────────────────
        acquire_deadline = time.monotonic() + acquire_timeout_s
        ekf_valid = False
        while time.monotonic() < acquire_deadline:
            msg = master.recv_match(blocking=True, timeout=0.2)
            if msg is not None:
                _dispatch(msg, state)
            if state["pos_t"] > 0.0 and state["lat"] != 0.0:
                ekf_valid = True
                break
            hw.send_gcs_heartbeat(master)

        if not ekf_valid:
            print("[FAIL] No valid EKF position received (GLOBAL_POSITION_INT).")
            return False

        ok_gps, gps_txt = _gps_ok(state)
        if not ok_gps:
            print(f"[FAIL] GPS fix is not valid.  ({gps_txt})")
            return False

        print(f"EKF POSITION         : VALID  (GLOBAL_POSITION_INT)")
        print(f"GPS FIX              : {gps_txt}")
        if state["sats_list"]:
            print(f"SATELLITES           : {round(_mean(state['sats_list']))}")
        else:
            print("SATELLITES           : n/a (GPS_RAW_INT akışı yok — SR2_RAW_SENSORS)")
        if state["hdop_list"]:
            print(f"HDOP                 : {_mean(state['hdop_list']):.2f}")
        else:
            print("HDOP                 : n/a (GPS_RAW_INT akışı yok)")
        print()

        # ── Örnek toplama ─────────────────────────────────────────────────
        print(f"Collecting GPS samples ({duration_s:.0f}s @ {1.0/interval_s:.0f} Hz)...")
        samples: list[tuple[float, float]] = []
        gaps = 0
        last_sample_t = time.monotonic()
        deadline = time.monotonic() + duration_s
        last_hb_t = 0.0

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_hb_t >= _HB_INTERVAL_S:
                hw.send_gcs_heartbeat(master)   # el sıkışması — komut değil
                last_hb_t = now

            msg = master.recv_match(blocking=True, timeout=0.05)
            if msg is not None:
                _dispatch(msg, state)

            if now - last_sample_t >= interval_s:
                last_sample_t = now
                # Veri kesintisi kontrolü: son konum mesajı çok eski mi?
                if state["pos_t"] <= 0.0 or now - state["pos_t"] > POSITION_DATA_TIMEOUT_S:
                    gaps += 1
                    continue
                samples.append((state["lat"], state["lon"]))

        # ── Hesapla ───────────────────────────────────────────────────────
        if len(samples) < MIN_POSITION_SAMPLES:
            print(f"[FAIL] Yetersiz örnek: {len(samples)} (< {MIN_POSITION_SAMPLES})."
                  f" Veri kesintisi: {gaps}")
            return False
        if gaps:
            print(f"WARNING: {gaps} örnek aralığında konum verisi kesildi (timeout "
                  f">{POSITION_DATA_TIMEOUT_S:.0f}s).")

        est = _estimate_position(samples)

        # ── Terminal çıktısı ──────────────────────────────────────────────
        n = len(samples)
        show_head = samples[:_PRINT_SAMPLE_LIMIT]
        show_tail = samples[-1:] if n > _PRINT_SAMPLE_LIMIT else []
        for i, s in enumerate(show_head, start=1):
            print(f"[{i:02d}] Lat: {s[0]:.7f}  Lon: {s[1]:.7f}")
        if show_tail:
            print("...")
            print(f"[{n:02d}] Lat: {show_tail[0][0]:.7f}  Lon: {show_tail[0][1]:.7f}")
        print()

        warn = est["spread_m"] > SPREAD_WARN_M
        print(f"Samples collected    : {n}  (inlier={est['inlier_n']}, "
              f"outlier={est['outlier_n']})")
        print(f"Position spread      : {est['spread_m']:.2f} m")
        print(f"Std deviation        : {est['std_m']:.2f} m")
        if warn:
            print(f"WARNING: Ölçüm yayılımı {est['spread_m']:.2f} m — "
                  f"{SPREAD_WARN_M:.1f} m üzerinde. GPS gürültüsü yüksek; "
                  "konum ortalama olarak güvenilirdir.")
        print()

        print("=" * 56)
        print("        POLE POSITION")
        print("=" * 56)
        print()
        print(f"Latitude             : {est['lat']:.7f} deg")
        print(f"Longitude            : {est['lon']:.7f} deg")
        print()
        print(f"  (median ref       : {est['median_lat']:.7f} / "
              f"{est['median_lon']:.7f})")
        print(f"  (spread           : {est['spread_m']:.2f} m, "
              f"std {est['std_m']:.2f} m)")
        print()
        print(f"STATUS               : {'PASS (yayılım uyarısı)' if warn else 'PASS'}")
        print("=" * 56)
        print("No flight commands were sent (read-only position measurement).")
        return True
    finally:
        try:
            master.close()
        except Exception:
            pass


# ══════════════════════════════════════════════════════════════════════════ #
#  CLI
# ══════════════════════════════════════════════════════════════════════════ #

def main() -> int:
    ap = argparse.ArgumentParser(description="Direk konumu ölçümü (read-only, Pi)")
    ap.add_argument("--device", default=None, help="Seri cihaz (varsayılan: config)")
    ap.add_argument("--baud", type=int, default=None, help="Baudrate (varsayılan: config)")
    ap.add_argument("--duration", type=float, default=POSITION_SAMPLE_DURATION_S,
                    help=f"Örnek toplama süresi, sn (varsayılan: {POSITION_SAMPLE_DURATION_S})")
    ap.add_argument("--interval", type=float, default=POSITION_SAMPLE_INTERVAL_S,
                    help=f"Örnek aralığı, sn (varsayılan: {POSITION_SAMPLE_INTERVAL_S})")
    ap.add_argument("--timeout", type=float, default=10.0,
                    help="Heartbeat bekleme süresi, sn (varsayılan: 10.0)")
    ap.add_argument("--acquire", type=float, default=POSITION_ACQUIRE_TIMEOUT_S,
                    help=f"Konum bekleme süresi, sn (varsayılan: {POSITION_ACQUIRE_TIMEOUT_S})")
    args = ap.parse_args()

    if not hw.is_raspberry_pi():
        print("Bu test Raspberry Pi 5 + Cube Orange gerektirir (gerçek MAVLink).")
        print("Bu makine Raspberry Pi değil — test çalıştırılamaz.")
        return 2

    cfg = hw.load_payload_config()
    conn = cfg.get("flight_controller", {}).get("connection", {}) or {}
    device = args.device or conn.get("device", "/dev/serial0")
    baud = args.baud or conn.get("baudrate", 921600)
    flow_control = bool(conn.get("flow_control", False))

    ok = run_position_test(
        device, baud,
        duration_s=args.duration,
        timeout_s=args.timeout,
        flow_control=flow_control,
        interval_s=args.interval,
        acquire_timeout_s=args.acquire,
    )
    return 0 if ok else 1


# ══════════════════════════════════════════════════════════════════════════ #
#  Testler
# ══════════════════════════════════════════════════════════════════════════ #

def test_estimate_single_cluster():
    """Küçük bir alana yayılmış örnekler → outlier yok, doğru merkez."""
    lat, lon = 37.9331871, 34.6177524
    samples = []
    for di in range(-10, 11):
        for dj in range(-1, 2):
            samples.append((lat + di * 1e-7, lon + dj * 1e-7))
    est = _estimate_position(samples)
    assert est["outlier_n"] == 0
    assert est["inlier_n"] == len(samples)
    north, east = _gps_delta_north_east_m(est["lat"], est["lon"], lat, lon)
    assert math.hypot(north, east) < 0.05


def test_estimate_rejects_outliers():
    """Merkezden ~100 m uzağa atılmış 2 örnek outlier sayılmalı ve final
    konumu etkilememeli."""
    lat, lon = 37.9331871, 34.6177524
    cluster = [(lat + i * 1e-7, lon) for i in range(-20, 21)]
    far1 = _gps_from_north_east_m(lat, lon, 100.0, 0.0)
    far2 = _gps_from_north_east_m(lat, lon, -100.0, 50.0)
    est = _estimate_position(cluster + [far1, far2])
    assert est["outlier_n"] == 2
    assert est["inlier_n"] == len(cluster)
    north, east = _gps_delta_north_east_m(est["lat"], est["lon"], lat, lon)
    assert math.hypot(north, east) < 0.05


def test_estimate_single_sample():
    """Tek örnek → spread 0, çökme yok."""
    est = _estimate_position([(37.9331871, 34.6177524)])
    assert est["inlier_n"] == 1
    assert est["outlier_n"] == 0
    assert est["spread_m"] == 0.0


def test_gps_ok_helpers():
    """GPS_RAW_INT fix_type + SYS_STATUS GPS biti mantığı."""
    state_fix = {"fix_type": 3, "sys_health": 0, "sys_t": 0.0}
    ok, txt = _gps_ok(state_fix)
    assert ok and "3D" in txt

    state_sys = {"fix_type": None, "sys_health": _SYS_GPS, "sys_t": 1.0}
    ok, txt = _gps_ok(state_sys)
    assert ok and "SYS_STATUS" in txt

    state_bad = {"fix_type": 1, "sys_health": 0, "sys_t": 0.0}
    ok, _ = _gps_ok(state_bad)
    assert not ok


def test_real_mode_position_reads_ekf_position():
    """Cube Orange'dan EKF konumu okuyup direk konumu olarak yazdırır
    (READ-ONLY). Raspberry Pi dışında otomatik SKIP."""
    if not hw.is_raspberry_pi():
        pytest.skip("Raspberry Pi + Cube Orange gerekli")

    cfg = hw.load_payload_config()
    conn = cfg["flight_controller"]["connection"]
    ok = run_position_test(
        conn["device"], conn["baudrate"],
        duration_s=6.0, timeout_s=10.0,
        flow_control=bool(conn.get("flow_control", False)),
    )
    assert ok, "EKF konumu ölçülemedi (test yalnızca telemetri okur — komut yok)"


if __name__ == "__main__":
    sys.exit(main())
