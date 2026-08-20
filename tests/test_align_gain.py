"""
PositionController.align_detection uyarlanabilir kazanç testleri.

Overshoot kök nedeni: yüksek irtifada aynı piksel hatası çok daha büyük bir
fiziksel adıma karşılık gelir (mpp ∝ irtifa); İHA komut aralığında
(ALIGN_CMD_INTERVAL_S) adımı bitiremezse hareket halinde alınan yeni ölçümle
hedef aşılır → sağ-sol salınımı.

Davranış modu (config eşikleri): ALIGN_ALT_MODE_LO_M ve ALTINDA ESKİ/stabil
hesap (sabit ALIGN_GAIN, adım sınırı YOK); eşikler arasında iki hesap yumuşakça
karıştırılır; ALIGN_ALT_MODE_HI_M ve ÜSTÜNDE tam adaptive hesap.

Test edilen davranışlar:
  1. Mod eşiğinin ALTINDA eski davranış DEĞİŞMEZ (tam ALIGN_GAIN, clamp yok).
  2. Eşiğin ÜSTÜNDE (11.5m) adaptive davranış (hata oranı + irtifa + adaptive step).
  3. Geçiş bandında adım monoton küçülür (süreksizlik yok).
  4. 11.5m'de büyük piksel hatası büyük adım kullanır.
  5. Hedef yaklaştıkça adım küçülür.
  6. Deadband (tolerans) korunur.
"""
import math
from unittest.mock import MagicMock

import pytest

from config import (
    ALIGN_ALT_GAIN_REF_M,
    ALIGN_ALT_MODE_HI_M,
    ALIGN_ALT_MODE_LO_M,
    ALIGN_GAIN,
    ALIGN_GAIN_FULL_ERROR_PX,
    ALIGN_MAX_STEP_LARGE_M,
    ALIGN_MAX_STEP_M,
    ALIGN_STEP_FULL_ERROR_PX,
    PIXEL_ALIGNMENT_TOLERANCE_PX,
)
from flight.position_controller import PositionController
from vision.geometry import CameraModel

_CAM = CameraModel(1920, 704, 66.0, 41.0)
_FW, _FH = 1920, 704
_CX, _CY = _FW / 2.0, _FH / 2.0
_EARTH_R = 6378137.0


def _command_step(dx_px: float, dy_px: float, alt_m: float) -> float:
    """
    align_detection'a TEK çağrı yap, gönderilen goto_global hedefinin metre
    büyüklüğünü (heading=0 → north=forward, east=right) döndür.
    """
    nav = MagicMock()
    nav.position.return_value = MagicMock(
        latitude=41.0, longitude=28.9, heading_deg=0.0,
    )
    pc = PositionController(nav, _CAM)
    pc.align_detection(_CX + dx_px, _CY + dy_px, _FW, _FH, alt_m)
    args = nav.goto_global.call_args[0]
    lat, lon = args[0], args[1]
    north = (lat - 41.0) * math.pi / 180.0 * _EARTH_R
    east = (lon - 28.9) * math.pi / 180.0 * _EARTH_R * math.cos(math.radians(41.0))
    return math.hypot(north, east)


def _mpp_x(alt_m: float) -> float:
    """11.5m dahil — CameraModel ile birebir aynı geometri."""
    w = 2.0 * alt_m * math.tan(math.radians(66.0) / 2.0)
    return w / _FW


def _legacy_step(err_px: float, alt_m: float) -> float:
    """ESKİ (mod eşiği altı) hesap: sabit ALIGN_GAIN, adım sınırı yok."""
    return err_px * _mpp_x(alt_m) * ALIGN_GAIN


# ── 1) Hata küçüldükçe adım küçülür (oransal kazanç) ──────────────────────── #
def test_smaller_error_smaller_step():
    """Aynı irtifada 80px hata, 150px hatadan KÜÇÜK adım üretmeli.
    (60px deadband'in üzerinde — her ikisi de komut gönderir.)"""
    step_150 = _command_step(150.0, 0.0, 11.5)
    step_80 = _command_step(80.0, 0.0, 11.5)
    assert step_80 < step_150, "hedefe yaklaştıkça komut adımı küçülmeli"


def test_very_small_error_tiny_step():
    """Deadband'in hemen üstündeki küçük hata → çok küçük hareket."""
    # Yarı çözünürlük: deadband 60→30px; "hemen üstü" de 65→35px olur.
    step = _command_step(35.0, 0.0, 11.5)   # 30px deadband üstü
    assert step < 0.06, f"küçük hata adımı çok küçük olmalı (step={step:.4f}m)"


# ── 2) Yüksek irtifa adımı PATLATMAZ (irtifa kazancı) ──────────────────────── #
def test_high_altitude_keeps_step_bounded():
    """100px hata @11.5m: adım, irtifa kazancı olmadan verilecek NAİF adımdan
    (0.8·mpp·100 ≈ 0.62m) çok küçük olmalı — aksi halde İHA hedefi aşar."""
    step = _command_step(100.0, 0.0, 11.5)
    naive = 100.0 * _mpp_x(11.5) * ALIGN_GAIN
    assert step < 0.3 * naive, \
        f"irtifa kazancı adımı küçültmeli (step={step:.3f}m, naif={naive:.3f}m)"
    assert step <= ALIGN_MAX_STEP_M * 1.001


# ── 2b) Mod eşiğinin ALTINDA ESKİ davranış birebir korunur ────────────────── #
def test_below_mode_threshold_uses_legacy_behavior():
    """Mod eşiği (ALIGN_ALT_MODE_LO_M) ALTINDA eski hesap: sabit ALIGN_GAIN,
    adım sınırı YOK — büyük hata clamp'sız büyük adım üretir."""
    alt = ALIGN_ALT_MODE_LO_M - 1.0   # eşiğin 1m altı
    for err in (150.0, 500.0):
        expected = _legacy_step(err, alt)
        got = _command_step(err, 0.0, alt)
        assert got == pytest.approx(expected, rel=0.01), \
            f"{alt:.1f}m'de eski hesap bozuldu (err={err:.0f}px)"
    # Büyük hata adımı üst sınırı aşar → clamp'ın OLMADIĞINI kanıtlar
    assert _command_step(500.0, 0.0, alt) > ALIGN_MAX_STEP_LARGE_M, \
        "clamp olmamalı (eski davranış)"


def test_at_mode_threshold_uses_legacy_behavior():
    """Eşiğin KENDİSİNDE (ALIGN_ALT_MODE_LO_M) de eski davranış: tam gain, clamp yok."""
    expected = _legacy_step(500.0, ALIGN_ALT_MODE_LO_M)
    got = _command_step(500.0, 0.0, ALIGN_ALT_MODE_LO_M)
    assert got == pytest.approx(expected, rel=0.01), \
        "eşikte eski hesap bozuldu — adaptive devreye GİRMEMELİ"


def test_mode_boundary_no_jump():
    """Mod sınırının HEMEN altı ve üstünde adımda ani sıçrama yoktur."""
    lo = ALIGN_ALT_MODE_LO_M
    step_below = _command_step(500.0, 0.0, lo - 0.1)
    step_at = _command_step(500.0, 0.0, lo)
    # ikisi de eski hesap (w=0) → adımlar yakın (yalnızca mpp farkı)
    assert step_below == pytest.approx(step_at, rel=0.05), \
        "mod sınırında davranış sıçraması olmamalı"


def test_transition_band_monotonic():
    """Geçiş bandında (LO→HI) adım monoton küçülür (süreksizlik yok)."""
    s_lo = _command_step(500.0, 0.0, ALIGN_ALT_MODE_LO_M)
    s_mid = _command_step(500.0, 0.0, (ALIGN_ALT_MODE_LO_M + ALIGN_ALT_MODE_HI_M) / 2.0)
    s_hi = _command_step(500.0, 0.0, ALIGN_ALT_MODE_HI_M)
    assert s_hi < s_mid < s_lo, "geçiş bandında adım kademeli küçülmeli"
    # HI eşiğinde tam adaptive: adım eski hesabın belirgin altında
    assert s_hi < _legacy_step(500.0, ALIGN_ALT_MODE_HI_M) * 0.5


def test_high_altitude_uses_adaptive():
    """10m ve 11.5m'de tam adaptive: büyük hata adımı eski hesabın çok altında
    ve üst sınıra (ALIGN_MAX_STEP_LARGE_M) bağlı."""
    for alt in (10.0, 11.5):
        step = _command_step(500.0, 0.0, alt)
        assert step < _legacy_step(500.0, alt) * 0.5, \
            f"{alt}m'de adaptive davranış uygulanmalı (step={step:.3f}m)"
        assert step <= ALIGN_MAX_STEP_LARGE_M * 1.001


# ── 2c) 11.5m'de büyük hata yavaşlamasın (küçük clamp kalktı) ─────────────── #
def test_11_5m_large_error_not_slow():
    """11.5m'de 500px hata ALIGN_MAX_STEP_M'yi AŞAN büyük adım kullanır —
    küçük sabit clamp yüzünden aşırı yavaşlama YOK."""
    step = _command_step(500.0, 0.0, 11.5)
    assert step > ALIGN_MAX_STEP_M, \
        f"büyük hata büyük adım kullanmalı (step={step:.3f}m, alt sınır {ALIGN_MAX_STEP_M}m)"


def test_altitude_gain_scales_with_ref_ratio():
    """50px @11.5m: kazanç = ALIGN_GAIN·(hata/eşik)·(ref/alt) — oransal."""
    # ratio_gain: 50px < 75px → 50/75 (yarı çözünürlükte eşik 150→75px)
    expected_gain = ALIGN_GAIN * (50.0 / ALIGN_GAIN_FULL_ERROR_PX) * (
        ALIGN_ALT_GAIN_REF_M / 11.5
    )
    expected = 50.0 * _mpp_x(11.5) * expected_gain
    got = _command_step(50.0, 0.0, 11.5)
    assert got == pytest.approx(expected, rel=0.01)


# ── 3) Adım sınırı (adaptive — yalnızca yüksek irtifa) ───────────────────── #
def test_step_never_exceeds_max():
    """11.5m'de çok büyük hata tek komutta ALIGN_MAX_STEP_LARGE_M'yi aşamaz."""
    step = _command_step(5000.0, 0.0, 11.5)
    assert step <= ALIGN_MAX_STEP_LARGE_M * 1.001, \
        f"adım sınırı aşıldı: {step:.3f}m > {ALIGN_MAX_STEP_LARGE_M}m"


def test_large_step_clamped_in_2d():
    """Bileşik (dx,dy) büyük hata da aynı üst sınıra tabidir."""
    step = _command_step(400.0, 300.0, 11.5)
    assert step <= ALIGN_MAX_STEP_LARGE_M * 1.001


def test_adaptive_step_scales_with_error():
    """11.5m'de adım piksel hatasıyla büyür: 500px > 300px > 150px."""
    s150 = _command_step(150.0, 0.0, 11.5)
    s300 = _command_step(300.0, 0.0, 11.5)
    s500 = _command_step(500.0, 0.0, 11.5)
    assert s150 < s300 < s500, "uzak hedef için daha büyük adım kullanılmalı"


def test_large_error_uses_large_step():
    """Çok büyük hata (2000px) büyük adım kullanır — ALIGN_MAX_STEP_LARGE_M doygunluğu."""
    step = _command_step(2000.0, 0.0, 11.5)
    assert step == pytest.approx(ALIGN_MAX_STEP_LARGE_M, rel=0.01), \
        "çok büyük hata ~ALIGN_MAX_STEP_LARGE_M adıma ulaşmalı (sabit küçük clamp değil)"


def test_small_error_below_small_step_limit():
    """Küçük hata adımı ALIGN_MAX_STEP_M'nin altında kalır (clamp yok — gain küçük)."""
    step = _command_step(80.0, 0.0, 11.5)
    assert step < ALIGN_MAX_STEP_M, \
        "küçük hata → sınırın bile altında küçük adım (hassas yaklaşma)"


def test_step_limit_linear_region():
    """Eşiğin altında sınır doğrusal büyür: 150px'te sınır, 300px'tekinden küçük."""
    # 500px clamp'a takılır (0.5); 300px gain çıktısı ~0.49; 150px ~0.24
    s150 = _command_step(150.0, 0.0, 11.5)
    assert s150 < ALIGN_MAX_STEP_M * 1.5, \
        f"orta-küçük hata adımı hâlâ kontrollü (s150={s150:.3f}m)"


# ── 4) Alçak irtifa + büyük hata → tam ALIGN_GAIN ──────────────────────────── #
def test_low_altitude_full_gain_for_large_error():
    """3m altında ve hata ≥ eşik: kazanç = ALIGN_GAIN (mevcut davranış korunur)."""
    error_px = ALIGN_GAIN_FULL_ERROR_PX   # 150px → ratio_gain = 1.0
    expected = error_px * _mpp_x(2.0) * ALIGN_GAIN
    got = _command_step(error_px, 0.0, 2.0)
    assert got == pytest.approx(expected, rel=0.01)
    # Adım sınırına takılmamalı (0.16m < 0.25m)
    assert got < ALIGN_MAX_STEP_M


# ── 5) Deadband korunur ────────────────────────────────────────────────────── #
def test_deadband_returns_aligned_no_command():
    """Tolerans içindeki hata → komut GÖNDERİLMEZ, True döner."""
    nav = MagicMock()
    nav.position.return_value = MagicMock(
        latitude=41.0, longitude=28.9, heading_deg=0.0,
    )
    pc = PositionController(nav, _CAM)
    ok = pc.align_detection(
        _CX + PIXEL_ALIGNMENT_TOLERANCE_PX - 1.0,
        _CY + PIXEL_ALIGNMENT_TOLERANCE_PX - 1.0,
        _FW, _FH, 11.5,
    )
    assert ok is True
    nav.goto_global.assert_not_called()


def test_alignment_commands_still_sent_outside_deadband():
    """Tolerans dışında komut gönderilmeye devam eder (False döner)."""
    nav = MagicMock()
    nav.position.return_value = MagicMock(
        latitude=41.0, longitude=28.9, heading_deg=0.0,
    )
    pc = PositionController(nav, _CAM)
    ok = pc.align_detection(
        _CX + PIXEL_ALIGNMENT_TOLERANCE_PX + 30.0,
        _CY, _FW, _FH, 11.5,
    )
    assert ok is False
    nav.goto_global.assert_called_once()
