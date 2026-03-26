"""Tests for the AprilTag semantic-overlay helpers in the live Carter demo."""

import importlib.util
from pathlib import Path


def _load_module(module_path: Path, module_name: str):
    spec = importlib.util.spec_from_file_location(module_name, str(module_path))
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load module spec: {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_apriltag_semantic_box_center_uses_fixed_world_position():
    demo = _load_module(
        Path(__file__).resolve().parents[1] / "examples" / "sdk_hospital_carter_live_demo.py",
        "sdk_hospital_carter_live_demo_apriltag_helpers",
    )

    center = demo._semantic_box_center_for_tag(1)

    assert center == (-47.780966508350595, 9.253047326267392, 0.5390588048650494)


def test_apriltag_semantic_box_center_uses_office_scene_profile():
    demo = _load_module(
        Path(__file__).resolve().parents[1] / "examples" / "sdk_hospital_carter_live_demo.py",
        "sdk_hospital_carter_live_demo_apriltag_helpers_office",
    )

    center = demo._semantic_box_center_for_tag(1, "office")

    assert center == (4.910504642109061, 30.97664047071059, 0.4973964158136842)


def test_apriltag_semantic_box_center_returns_none_for_unknown_tag():
    demo = _load_module(
        Path(__file__).resolve().parents[1] / "examples" / "sdk_hospital_carter_live_demo.py",
        "sdk_hospital_carter_live_demo_apriltag_helpers_unknown",
    )

    assert demo._semantic_box_center_for_tag(999) is None


def test_apriltag_person_semantic_id_is_stable():
    demo = _load_module(
        Path(__file__).resolve().parents[1] / "examples" / "sdk_hospital_carter_live_demo.py",
        "sdk_hospital_carter_live_demo_apriltag_id_helpers",
    )

    assert demo._apriltag_person_semantic_id(7) == "apriltag_person_7"
    assert demo.DEFAULT_APRILTAG_SEMANTIC_BOX_SIZE == (0.6, 0.6, 1.7)
