"""
vision_camera_fps_test.py — kamera yakalama FPS ölçüm aracı testleri.

Gerçek kamera/donanım gerekmez; sahte kamera ile measure_camera/_frame_hash
davranışı, config değerlerinin kullanımı ve güvenlik (flight/payload/MAVLink
import yok) doğrulanır.
"""
import logging
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

import vision_camera_fps_test as vcf


# ── sahte kamera / frame üretici ──────────────────────────────────────────── #

def _frame(seed: int) -> np.ndarray:
    """İçeriği seed'e göre değişen küçük frame (8x8 BGR)."""
    f = np.zeros((8, 8, 3), dtype=np.uint8)
    f[0, 0] = seed & 0xFF
    return f


class _FakeCam:
    """latest() davranışı kontrol edilebilen sahte kamera.

    frames:  list — None içerirse latest() o konumda None döndürür (sona
                     ulaşınca başa döner).
    """

    def __init__(self, frames):
        self._frames = list(frames)
        self._i = -1
        self.calls = 0

    def latest(self):
        self.calls += 1
        if not self._frames:
            return None
        self._i = (self._i + 1) % len(self._frames)
        frame = self._frames[self._i]
        return None if frame is None else frame


# ── güvenlik ──────────────────────────────────────────────────────────────── #

def test_module_does_not_import_flight_payload_state_machine():
    """Güvenlik: fps-test aracı flight/payload/state_machine/pymavlink çekmez.

    sys.modules kontrolü temiz bir alt-süreçte yapılır — full suite içinde
    başka testler bu modülleri zaten import etmiş olabilir (yanlış pozitif).
    """
    root = str(Path(__file__).resolve().parent.parent)
    code = (
        "import sys; import vision_camera_fps_test; "
        "bad = sorted({m.split('.')[0] for m in sys.modules} "
        "& {'flight','payload','state_machine','pymavlink'}); "
        "print('BAD=' + ','.join(bad))"
    )
    out = subprocess.run(
        [sys.executable, "-c", code],
        capture_output=True, text=True, cwd=root, timeout=60,
    )
    assert out.returncode == 0, out.stderr
    assert out.stdout.strip() == "BAD=", \
        f"vision_camera_fps_test şunları import ediyor: {out.stdout.strip()}"


# ── _frame_hash ───────────────────────────────────────────────────────────── #

def test_frame_hash_identical_for_same_frame():
    assert vcf._frame_hash(_frame(7)) == vcf._frame_hash(_frame(7))


def test_frame_hash_differs_for_different_frames():
    hashes = {vcf._frame_hash(_frame(s)) for s in range(1, 50)}
    assert len(hashes) >= 49, "farklı frame'ler farklı hash üretmeli (çarpışma yok)"


# ── measure_camera ────────────────────────────────────────────────────────── #

def test_measure_camera_counts_unique_frames():
    """Her latest() yeni frame üretirse benzersiz sayı çağrı sayısına eşittir."""
    cam = _FakeCam([_frame(i) for i in range(10)])
    stats = vcf.measure_camera(cam, duration_s=0.15)
    assert stats["stalled"] is False
    assert stats["unique"] == stats["calls"] >= 30
    assert stats["shape"] == (8, 8, 3)
    assert stats["fps"] > 0


def test_measure_camera_same_frame_not_double_counted():
    """Kamera aynı frame'i tekrar veriyorsa (yakalama döngüden yavaş) benzersiz 1 olur."""
    cam = _FakeCam([_frame(3)])
    stats = vcf.measure_camera(cam, duration_s=0.1)
    assert stats["unique"] == 1
    assert stats["stalled"] is False
    assert stats["calls"] > 1


def test_measure_camera_stalled_when_no_frames():
    """Hiç frame gelmiyorsa stalled=True ve fps=0 olur."""
    cam = _FakeCam([])
    stats = vcf.measure_camera(cam, duration_s=0.1)
    assert stats["stalled"] is True
    assert stats["unique"] == 0
    assert stats["fps"] == 0.0
    assert stats["shape"] is None


def test_measure_camera_respects_duration():
    """Ölçüm süresi istenen süreden kısa olmamalı (zaman sınırı çalışıyor)."""
    cam = _FakeCam([_frame(i) for i in range(5)])
    start = __import__("time").monotonic()
    vcf.measure_camera(cam, duration_s=0.2)
    elapsed = __import__("time").monotonic() - start
    assert elapsed >= 0.2 - 0.02


def test_measure_camera_interval_statistics():
    """Yeni frame'lerin aralık istatistikleri doldurulur (en az 2 benzersiz frame)."""
    cam = _FakeCam([_frame(1), _frame(2)])
    stats = vcf.measure_camera(cam, duration_s=0.2)
    assert len(stats["intervals"]) >= 1
    assert all(t > 0 for t in stats["intervals"])


# ── _fmt_interval ─────────────────────────────────────────────────────────── #

def test_fmt_interval_empty():
    assert vcf._fmt_interval([]) == "-"


def test_fmt_interval_avg_min_max():
    s = vcf._fmt_interval([0.001, 0.002, 0.003])
    assert "2.0 ms" in s
    assert "min 1.0" in s
    assert "max 3.0" in s


# ── _build_camera (config kullanımı) ──────────────────────────────────────── #

def test_build_camera_uses_config_values(monkeypatch):
    """Kamera config'teki çözünürlük/FPS/kamera no ile açılır, headless'tır."""
    from unittest.mock import MagicMock

    cam = MagicMock()
    captured = {}

    def fake_rpicam3(**kwargs):
        captured.update(kwargs)
        return cam

    monkeypatch.setattr(vcf, "RPiCam3", fake_rpicam3)
    c = vcf._build_camera(30, logging.getLogger("fps-test"))
    assert c is cam
    assert captured["width"] == vcf.CAMERA_WIDTH_PX
    assert captured["height"] == vcf.CAMERA_HEIGHT_PX
    assert captured["fps"] == 30
    assert captured["camera_num"] == vcf.RPI_CAMERA_NUM
    assert captured["preview"] is False  # headless


def test_build_camera_clamps_fps_to_min_1(monkeypatch):
    from unittest.mock import MagicMock

    captured = {}

    def fake_rpicam3(**kwargs):
        captured.update(kwargs)
        return MagicMock()

    monkeypatch.setattr(vcf, "RPiCam3", fake_rpicam3)
    vcf._build_camera(0, logging.getLogger("fps-test"))
    assert captured["fps"] == 1
