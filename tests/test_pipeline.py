"""
pipeline modülü testleri — tam zincir entegrasyonu.

Senaryolar:
  * tam pipeline: frame → mask → contour → shape → size → projection
  * irtifa verilince GSD tabanlı boyut filtresi devreye girer
  * irtifa yokken mutlak eşiklerle çalışır (eski davranış — state machine uyumu)
  * gyro frame gate pipeline'ı eleyebilir
  * pose verilince dünya koordinatı üretilir
  * false positive: daire/kare/karışık blob elenir
  * iki hedef aynı frame'de bağımsız değerlendirilir
"""
import numpy as np
import pytest

from config import (
    VISION_CAMERA_GEOMETRY,
    VISION_COLOR_PURITY,
    VISION_CONFIDENCE_WEIGHTS,
    VISION_DECOY_REJECTION,
    VISION_DETECT_WIDTH_PX,
    VISION_FRAME_GATE,
    VISION_GEO_PROJECTION,
    VISION_MORPHOLOGY,
    VISION_SHAPE_SCORING,
    VISION_SIZE_FILTER,
    VISION_SQUARE,
    VISION_TEMPORAL,
    VISION_TARGETS,
    VISION_VERTEX_EPSILONS,
)
from models import TargetColor
from vision.geo_projection import VehiclePose
from vision.pipeline import VisionPipeline

H, W = 704, 1920


def _pipeline() -> VisionPipeline:
    return VisionPipeline(
        targets_cfg=VISION_TARGETS,
        morphology_cfg=VISION_MORPHOLOGY,
        scoring_cfg=VISION_SHAPE_SCORING,
        vertex_epsilons=VISION_VERTEX_EPSILONS,
        size_filter_cfg=VISION_SIZE_FILTER,
        geometry_cfg=VISION_CAMERA_GEOMETRY,
        frame_gate_cfg=VISION_FRAME_GATE,
        geo_cfg=VISION_GEO_PROJECTION,
        temporal_cfg=VISION_TEMPORAL,
        purity_cfg=VISION_COLOR_PURITY,
        confidence_weights=VISION_CONFIDENCE_WEIGHTS,
        square_cfg=VISION_SQUARE,
        decoy_cfg=VISION_DECOY_REJECTION,
        work_width_px=VISION_DETECT_WIDTH_PX,
        min_detection_confidence=0.60,
    )


def _mk() -> np.ndarray:
    return np.zeros((H, W, 3), dtype=np.uint8)


def _flat_hex(cx=700, cy=450, s=258) -> np.ndarray:
    """Gerçek 2m DÜZGÜN altıgen: 11.5m'de kenar ≈ 129px @960 = 258px @1920."""
    return np.array(
        [
            [cx - s, cy], [cx - s // 2, cy - int(s * 0.866)],
            [cx + s // 2, cy - int(s * 0.866)], [cx + s, cy],
            [cx + s // 2, cy + int(s * 0.866)], [cx - s // 2, cy + int(s * 0.866)],
        ],
        dtype=np.int32,
    )


def _tri(cx=1350, top=250, half=64) -> np.ndarray:
    """Gerçek 1m eşkenar üçgen: 11.5m'de kenar ≈ 64px @960 = 128px @1920."""
    base = top + int(half * 1.732)  # eşkenar yükseklik ≈ half·√3
    return np.array([[cx, top], [cx - half, base], [cx + half, base]], dtype=np.int32)


def _two_targets_img() -> np.ndarray:
    import cv2
    img = _mk()
    cv2.fillPoly(img, [_flat_hex()], (255, 0, 0))
    cv2.fillPoly(img, [_tri()], (0, 0, 255))
    return img


def test_full_pipeline_detects_both_targets():
    p = _pipeline()
    dets = p.process(_two_targets_img())
    assert any(d.color == TargetColor.BLUE and d.shape == "HEXAGON" for d in dets)
    assert any(d.color == TargetColor.RED and d.shape == "TRIANGLE" for d in dets)


def test_pipeline_works_without_altitude():
    """İrtifa verilmezse pipeline yine çalışır (state machine detect(frame))."""
    p = _pipeline()
    dets = p.process(_two_targets_img())
    assert len(dets) >= 2


def test_size_filter_uses_gsd_when_altitude_given():
    """İrtifa verilince fiziksel boyut filtresi devreye girer: 11.5m'de hedefler
    beklenen alanda olduğu için kabul edilir."""
    p = _pipeline()
    dets = p.process(_two_targets_img(), altitude_m=11.5)
    assert any(d.color == TargetColor.BLUE for d in dets)
    assert any(d.color == TargetColor.RED for d in dets)


def test_size_filter_rejects_wrong_altitude_size():
    """Çok yakın irtifada (hedef beklenenden çok büyük) boyut filtresi eler."""
    # Hedefler 11.5m'ye göre çizildi; 2m'de (GSD küçük) beklenen alan 33x büyür
    # → çizilen alan (11.5m için doğru) 2m beklentisinin çok altında → reddedilir.
    p = _pipeline()
    dets = p.process(_two_targets_img(), altitude_m=2.0)
    assert dets == [], "2m'de 11.5m'lik alan çok küçük → fiziksel filtre elemeli"


def test_frame_gate_rejects_high_gyro():
    """Yüksek gyro → pipeline hiçbir şey döndürmez."""
    p = _pipeline()
    max_gyro = float(VISION_FRAME_GATE["max_gyro_deg_s"])
    dets = p.process(_two_targets_img(), gyro_deg_s=max_gyro + 10.0)
    assert dets == []


def test_frame_gate_passes_low_gyro():
    p = _pipeline()
    dets = p.process(_two_targets_img(), gyro_deg_s=5.0)
    assert len(dets) >= 2


def test_pipeline_produces_world_coordinates_with_pose():
    """Pose verilince Detection.world_lat/lon doldurulur."""
    p = _pipeline()
    pose = VehiclePose(latitude=40.0, longitude=29.0, yaw_deg=0.0)
    dets = p.process(_two_targets_img(), altitude_m=11.5, pose=pose)
    assert dets
    assert all(d.world_lat is not None for d in dets)
    assert all(d.world_lon is not None for d in dets)


def test_world_coordinates_none_without_pose():
    p = _pipeline()
    dets = p.process(_two_targets_img())
    assert dets
    assert all(d.world_lat is None for d in dets)


def test_false_positives_rejected():
    """Daire/kare/karışık blob hedef üretmez."""
    import cv2

    p = _pipeline()
    for draw in (
        lambda im: cv2.ellipse(im, (960, 460), (120, 120), 0, 0, 360, (255, 0, 0), -1),
        lambda im: cv2.rectangle(im, (860, 360), (1060, 560), (255, 0, 0), -1),
    ):
        img = _mk()
        draw(img)
        assert p.process(img) == [], "yanlış şekil detection üretmemeli"

    # Karışık renk blob (yarım kırmızı yarım mavi altıgen).
    img = _mk()
    hex_pts = _flat_hex(cx=960, cy=460, s=170)
    cv2.fillPoly(img, [hex_pts], (255, 0, 0))
    half = np.zeros((H, W), dtype=np.uint8)
    cv2.fillPoly(half, [hex_pts[:4]], 255)
    img[half == 255] = (0, 0, 255)
    assert p.process(img) == [], "karışık renk blob elenmeli"


def test_output_coordinates_in_original_resolution():
    """Detection koordinatları giriş frame'i (1920) çözünürlüğündedir."""
    p = _pipeline()
    dets = p.process(_two_targets_img())
    for d in dets:
        assert 0 <= d.center_x <= W
        assert 0 <= d.center_y <= H


def test_empty_scene():
    p = _pipeline()
    assert p.process(_mk()) == []


def test_processing_time_measured():
    p = _pipeline()
    p.process(_two_targets_img())
    assert p.last_frame_ms > 0
