"""Tests for the AprilTag semantic-overlay helpers in the live Carter demo."""

import importlib.util
import math
from pathlib import Path


def _load_module(module_path: Path, module_name: str):
    spec = importlib.util.spec_from_file_location(module_name, str(module_path))
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load module spec: {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_apriltag_semantic_box_center_projects_one_meter_forward():
    demo = _load_module(
        Path(__file__).resolve().parents[1] / "examples" / "sdk_hospital_carter_live_demo.py",
        "sdk_hospital_carter_live_demo_apriltag_helpers",
    )

    pose = demo._RobotPose2D(frame_id="map", x=2.0, y=-1.5, yaw=math.pi / 2.0)
    center = demo._semantic_box_center_from_robot_pose(pose, 1.0)

    assert center == (2.0, -0.5, 0.0)


def test_apriltag_person_semantic_id_is_stable():
    demo = _load_module(
        Path(__file__).resolve().parents[1] / "examples" / "sdk_hospital_carter_live_demo.py",
        "sdk_hospital_carter_live_demo_apriltag_id_helpers",
    )

    assert demo._apriltag_person_semantic_id(7) == "apriltag_person_7"
    assert demo.DEFAULT_APRILTAG_SEMANTIC_BOX_SIZE == (0.6, 0.6, 1.7)
