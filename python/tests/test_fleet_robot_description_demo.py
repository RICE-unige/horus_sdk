from argparse import Namespace
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

EXAMPLES_DIR = Path(__file__).resolve().parents[1] / "examples"
if str(EXAMPLES_DIR) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_DIR))

from fleet_robot_description_registration import (
    DEFAULT_MESH_ROOT,
    FLEET,
    build_robot,
)


def _demo_args(**overrides):
    values = {
        "source": "local",
        "mesh_root": Path(DEFAULT_MESH_ROOT),
        "body_mesh_mode": "",
        "visual_mesh_triangle_budget": 0,
        "no_meshes": False,
        "dry_run": True,
    }
    values.update(overrides)
    return Namespace(**values)


def test_fleet_robot_description_demo_uses_prefixed_frames():
    args = _demo_args()

    assert len(FLEET) == 5
    assert {spec.name for spec in FLEET} == {"g1", "h1", "anymal_c", "spot", "jackal"}
    assert all(spec.body_mesh_mode == "runtime_high_mesh" for spec in FLEET)

    for spec in FLEET:
        robot = build_robot(spec, args)

        binding = robot.get_ros_binding()
        assert binding["tf_mode"] == "prefixed"
        assert binding["tf_prefix"] == spec.name
        assert binding["base_frame"] == spec.base_frame
        assert robot.resolve_tf_frame() == f"{spec.name}/{spec.base_frame}"


def test_fleet_robot_description_demo_hides_unbacked_controls():
    args = _demo_args()
    robot = build_robot(next(spec for spec in FLEET if spec.name == "jackal"), args)

    manager = robot.get_metadata("robot_manager_config")
    assert manager["enabled"] is True
    assert manager["sections"] == {
        "status": True,
        "data_viz": True,
        "teleop": False,
        "tasks": False,
    }


def test_fleet_robot_description_demo_uses_local_urdf_assets_by_default():
    args = _demo_args(dry_run=False)
    robot = build_robot(next(spec for spec in FLEET if spec.name == "jackal"), args)

    description = robot.get_metadata("robot_description_config")
    assert description["source"] == "local"
    assert description["urdf_path"].endswith("jackal.urdf")
    assert description["robot_description_topic"] == "/jackal/robot_description"
    assert description["mesh_root"] == str(DEFAULT_MESH_ROOT)


def test_fleet_robot_description_demo_can_use_live_robot_description_topic():
    args = _demo_args(source="topic", dry_run=False)
    robot = build_robot(next(spec for spec in FLEET if spec.name == "jackal"), args)

    description = robot.get_metadata("robot_description_config")
    assert description["source"] == "topic"
    assert description["urdf_path"] == ""
    assert description["robot_description_topic"] == "/jackal/robot_description"


def test_fleet_robot_description_demo_has_showroom_floor_offsets():
    for spec in FLEET:
        assert spec.z >= 0.0

    spot = next(spec for spec in FLEET if spec.name == "spot")
    assert spot.base_frame == "body"
    assert spot.urdf_root_frame == "body"
    assert spot.z > 0.6

    humanoids = {spec.name: spec for spec in FLEET if spec.name in {"g1", "h1"}}
    assert humanoids["g1"].base_frame == "pelvis"
    assert humanoids["g1"].urdf_root_frame == "pelvis"
    assert humanoids["g1"].z > 0.7
    assert humanoids["h1"].base_frame == "pelvis"
    assert humanoids["h1"].urdf_root_frame == "pelvis"
    assert humanoids["h1"].z > 0.9


def test_fleet_robot_description_demo_anchors_actual_urdf_roots():
    assets_dir = EXAMPLES_DIR / ".local_assets" / "robot_descriptions"

    for spec in FLEET:
        urdf_path = assets_dir / spec.urdf_name
        assert urdf_path.is_file(), f"missing showroom URDF for {spec.name}: {urdf_path}"

        root = ET.parse(urdf_path).getroot()
        links = {element.get("name") for element in root.findall("link")}
        child_links = {
            child.get("link")
            for joint in root.findall("joint")
            for child in [joint.find("child")]
            if child is not None
        }
        root_links = links - child_links

        assert spec.urdf_root_frame in root_links, (
            f"{spec.name} anchor must target an actual URDF root; "
            f"configured={spec.urdf_root_frame!r}, roots={sorted(root_links)!r}"
        )
