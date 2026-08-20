"""
frame_gate modülü testleri — gyro tabanlı frame kalite kapısı.

Senaryolar:
  * düşük gyro → kabul
  * yüksek gyro → reject
  * gyro verisi yok → config'e göre davranış (require_gyro_data)
"""
import pytest

from config import VISION_FRAME_GATE
from vision.frame_gate import FrameGate


@pytest.fixture
def gate():
    return FrameGate(VISION_FRAME_GATE)


def test_low_gyro_accepted(gate):
    assert gate.accept(0.0) is True
    assert gate.accept(10.0) is True


def test_high_gyro_rejected(gate):
    max_gyro = float(VISION_FRAME_GATE["max_gyro_deg_s"])
    assert gate.accept(max_gyro + 1.0) is False


def test_boundary_accepted(gate):
    max_gyro = float(VISION_FRAME_GATE["max_gyro_deg_s"])
    assert gate.accept(max_gyro) is True


def test_missing_gyro_allowed_when_not_required(gate):
    """require_gyro_data=False → gyro yokluğu frame'i engellemez."""
    assert VISION_FRAME_GATE["require_gyro_data"] is False
    assert gate.accept(None) is True


def test_missing_gyro_blocked_when_required():
    cfg = dict(VISION_FRAME_GATE)
    cfg["require_gyro_data"] = True
    g = FrameGate(cfg)
    assert g.accept(None) is False
    assert g.accept(5.0) is True


def test_disabled_gate_accepts_everything():
    cfg = dict(VISION_FRAME_GATE)
    cfg["enabled"] = False
    g = FrameGate(cfg)
    assert g.accept(None) is True
    assert g.accept(999.0) is True


def test_config_used_for_threshold():
    assert FrameGate(VISION_FRAME_GATE)._max_gyro == float(
        VISION_FRAME_GATE["max_gyro_deg_s"]
    )
