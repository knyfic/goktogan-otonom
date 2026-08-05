from vision.geometry import CameraModel, expected_payload_pixels


def test_ground_footprint_grows_with_altitude():
    camera = CameraModel(1280, 720, 78, 60)
    low = camera.ground_footprint(1.0)
    high = camera.ground_footprint(10.0)
    assert high[0] > low[0]
    assert high[1] > low[1]


def test_pixel_to_body_direction():
    camera = CameraModel(1280, 720, 78, 60)
    forward, right = camera.pixel_offset_to_body_offset(100, 100, 10)
    assert right > 0
    assert forward < 0


def test_payload_pixel_size():
    camera = CameraModel(1280, 720, 78, 60)
    w, h = expected_payload_pixels(0.2, 0.1, 1.0, camera)
    assert w > 0
    assert h > 0
