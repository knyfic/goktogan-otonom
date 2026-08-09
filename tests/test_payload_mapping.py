from models import Target, TargetColor, TargetType
from state_machine import MissionStateMachine


def test_blue_hexagon_maps_to_red_payload():
    target = Target(
        target_id=1,
        shape=TargetType.HEXAGON,
        color=TargetColor.BLUE,
        latitude=0.0, longitude=0.0,
        confidence=1.0,
    )
    result = MissionStateMachine.payload_for_target(target)
    assert result == TargetColor.RED, (
        f"MAVİ ALTİGEN → Kırmızı payload bırakılmalı. Sonuç: {result.value}"
    )


def test_red_triangle_maps_to_blue_payload():
    target = Target(
        target_id=1,
        shape=TargetType.TRIANGLE,
        color=TargetColor.RED,
        latitude=0.0, longitude=0.0,
        confidence=1.0,
    )
    result = MissionStateMachine.payload_for_target(target)
    assert result == TargetColor.BLUE, (
        f"KIRMIZI ÜÇGEN → Mavi payload bırakılmalı. Sonuç: {result.value}"
    )


def test_second_target_blue_hexagon_maps_to_red():
    target = Target(
        target_id=2,
        shape=TargetType.HEXAGON,
        color=TargetColor.BLUE,
        latitude=0.0, longitude=0.0,
        confidence=1.0,
    )
    result = MissionStateMachine.payload_for_target(target)
    assert result == TargetColor.RED


def test_second_target_red_triangle_maps_to_blue():
    target = Target(
        target_id=2,
        shape=TargetType.TRIANGLE,
        color=TargetColor.RED,
        latitude=0.0, longitude=0.0,
        confidence=1.0,
    )
    result = MissionStateMachine.payload_for_target(target)
    assert result == TargetColor.BLUE
