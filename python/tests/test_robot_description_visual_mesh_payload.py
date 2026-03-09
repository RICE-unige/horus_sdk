"""Tests for baked visual mesh payloads in robot descriptions."""

import os
import tempfile
from pathlib import Path

from horus.description.robot_description_resolver import RobotDescriptionResolver
from horus.robot import Robot, RobotType


def _write_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content.strip() + "\n", encoding="utf-8")


def test_visual_mesh_payload_is_baked_from_package_obj():
    with tempfile.TemporaryDirectory() as tmp_dir:
        package_root = Path(tmp_dir) / "tiny_bot_description"
        _write_text(
            package_root / "package.xml",
            "<package><name>tiny_bot_description</name></package>",
        )
        _write_text(
            package_root / "meshes" / "panel.obj",
            """
o panel
v 0 0 0
v 1 0 0
v 0 1 0
vn 0 0 1
f 1//1 2//1 3//1
            """,
        )
        urdf_path = package_root / "urdf" / "tiny_bot.urdf"
        _write_text(
            urdf_path,
            """
<robot name="tiny_bot">
  <link name="base_link">
    <visual>
      <origin xyz="0.1 0.2 0.3" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://tiny_bot_description/meshes/panel.obj"/>
      </geometry>
    </visual>
    <collision>
      <geometry><box size="0.2 0.2 0.1"/></geometry>
    </collision>
  </link>
</robot>
            """,
        )

        robot = Robot(name="tiny_bot", robot_type=RobotType.WHEELED)
        robot.configure_robot_description(
            urdf_path=str(urdf_path),
            base_frame="base_link",
            include_visual_meshes=True,
            visual_mesh_triangle_budget=2000,
        )
        resolver = RobotDescriptionResolver()
        artifact = resolver.resolve_for_robot(robot)

        assert artifact is not None
        manifest = artifact.manifest
        assert manifest.version == "v2"
        assert manifest.supports_visual_meshes is True
        assert manifest.mesh_asset_count == 1
        assert manifest.mesh_asset_encoded_bytes > 0

        payload = artifact.payload_dict
        assert len(payload["mesh_assets"]) == 1
        assert len(payload["visual_links"]) == 1
        mesh_asset = payload["mesh_assets"][0]
        visual_link = payload["visual_links"][0]
        assert mesh_asset["vertex_count"] == 3
        assert mesh_asset["triangle_count"] == 1
        assert mesh_asset["positions_b64"]
        assert mesh_asset["normals_b64"]
        assert mesh_asset["indices_b64"]
        assert visual_link["mesh_id"] == mesh_asset["mesh_id"]

