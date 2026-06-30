"""Tests for baked visual mesh payloads in robot descriptions."""

import base64
import os
import struct
import tempfile
from pathlib import Path

import pytest

from horus.description.robot_description_resolver import RobotDescriptionResolver
from horus.robot import Robot, RobotType


def _write_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content.strip() + "\n", encoding="utf-8")


def _decode_float32(base64_text: str) -> list[float]:
    raw = base64.b64decode(base64_text)
    return list(struct.unpack("<" + ("f" * (len(raw) // 4)), raw))


def _decode_color32(base64_text: str) -> list[tuple[int, int, int, int]]:
    raw = base64.b64decode(base64_text)
    values = list(raw)
    return [
        (values[index], values[index + 1], values[index + 2], values[index + 3])
        for index in range(0, len(values), 4)
    ]


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
        assert mesh_asset["colors_b64"]
        assert visual_link["mesh_id"] == mesh_asset["mesh_id"]
        assert visual_link["frame_id"] == "base_link"


def test_visual_mesh_payload_is_baked_from_package_dae():
    with tempfile.TemporaryDirectory() as tmp_dir:
        package_root = Path(tmp_dir) / "tiny_bot_description"
        _write_text(
            package_root / "package.xml",
            "<package><name>tiny_bot_description</name></package>",
        )
        _write_text(
            package_root / "meshes" / "panel.dae",
            """
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <library_effects>
    <effect id="panel-material-effect">
      <profile_COMMON>
        <technique sid="common">
          <lambert>
            <diffuse><color>0.2 0.4 0.7 1</color></diffuse>
          </lambert>
        </technique>
      </profile_COMMON>
    </effect>
  </library_effects>
  <library_materials>
    <material id="panel-material" name="panel">
      <instance_effect url="#panel-material-effect"/>
    </material>
  </library_materials>
  <library_geometries>
    <geometry id="panel-mesh" name="panel">
      <mesh>
        <source id="panel-positions">
          <float_array id="panel-positions-array" count="9">0 0 0 1 0 0 0 1 0</float_array>
          <technique_common>
            <accessor source="#panel-positions-array" count="3" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="panel-vertices">
          <input semantic="POSITION" source="#panel-positions"/>
        </vertices>
        <triangles count="1">
          <input semantic="VERTEX" source="#panel-vertices" offset="0"/>
          <p>0 1 2</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="Scene">
      <node id="panel-node">
        <matrix sid="transform">0 -1 0 0 1 0 0 0 0 0 1 0 0 0 0 1</matrix>
        <instance_geometry url="#panel-mesh">
          <bind_material>
            <technique_common>
              <instance_material symbol="panel-material" target="#panel-material"/>
            </technique_common>
          </bind_material>
        </instance_geometry>
      </node>
    </visual_scene>
  </library_visual_scenes>
</COLLADA>
            """,
        )
        urdf_path = package_root / "urdf" / "tiny_bot.urdf"
        _write_text(
            urdf_path,
            """
<robot name="tiny_bot">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://tiny_bot_description/meshes/panel.dae"/>
      </geometry>
    </visual>
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
        artifact = RobotDescriptionResolver().resolve_for_robot(robot)

        assert artifact is not None
        assert artifact.manifest.supports_visual_meshes is True
        assert artifact.manifest.mesh_asset_count == 1
        mesh_asset = artifact.payload_dict["mesh_assets"][0]
        assert mesh_asset["vertex_count"] == 3
        assert mesh_asset["triangle_count"] == 1
        positions = _decode_float32(mesh_asset["positions_b64"])
        xs = positions[0::3]
        ys = positions[1::3]
        assert min(xs) == pytest.approx(-1.0)
        assert max(xs) == pytest.approx(0.0)
        assert min(ys) == pytest.approx(0.0)
        assert max(ys) == pytest.approx(1.0)
        assert mesh_asset["colors_b64"]
        assert mesh_asset["color_rgb"] == pytest.approx([0.2, 0.4, 0.7], rel=1e-6, abs=1e-6)


def test_visual_mesh_payload_preserves_dae_triangle_material_colors():
    with tempfile.TemporaryDirectory() as tmp_dir:
        package_root = Path(tmp_dir) / "tiny_bot_description"
        _write_text(
            package_root / "package.xml",
            "<package><name>tiny_bot_description</name></package>",
        )
        _write_text(
            package_root / "meshes" / "panel.dae",
            """
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <library_effects>
    <effect id="red-effect">
      <profile_COMMON><technique sid="common"><lambert>
        <diffuse><color>1 0 0 1</color></diffuse>
      </lambert></technique></profile_COMMON>
    </effect>
    <effect id="blue-effect">
      <profile_COMMON><technique sid="common"><lambert>
        <diffuse><color>0 0 1 1</color></diffuse>
      </lambert></technique></profile_COMMON>
    </effect>
  </library_effects>
  <library_materials>
    <material id="red-material" name="red"><instance_effect url="#red-effect"/></material>
    <material id="blue-material" name="blue"><instance_effect url="#blue-effect"/></material>
  </library_materials>
  <library_geometries>
    <geometry id="panel-mesh" name="panel">
      <mesh>
        <source id="panel-positions">
          <float_array id="panel-positions-array" count="12">0 0 0 1 0 0 0 1 0 1 1 0</float_array>
          <technique_common>
            <accessor source="#panel-positions-array" count="4" stride="3"/>
          </technique_common>
        </source>
        <vertices id="panel-vertices"><input semantic="POSITION" source="#panel-positions"/></vertices>
        <triangles material="red-symbol" count="1">
          <input semantic="VERTEX" source="#panel-vertices" offset="0"/>
          <p>0 1 2</p>
        </triangles>
        <triangles material="blue-symbol" count="1">
          <input semantic="VERTEX" source="#panel-vertices" offset="0"/>
          <p>2 1 3</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="Scene">
      <node id="panel-node">
        <instance_geometry url="#panel-mesh">
          <bind_material>
            <technique_common>
              <instance_material symbol="red-symbol" target="#red-material"/>
              <instance_material symbol="blue-symbol" target="#blue-material"/>
            </technique_common>
          </bind_material>
        </instance_geometry>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#Scene"/></scene>
</COLLADA>
            """,
        )
        urdf_path = package_root / "urdf" / "tiny_bot.urdf"
        _write_text(
            urdf_path,
            """
<robot name="tiny_bot">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://tiny_bot_description/meshes/panel.dae"/>
      </geometry>
    </visual>
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
        artifact = RobotDescriptionResolver().resolve_for_robot(robot)

        assert artifact is not None
        mesh_asset = artifact.payload_dict["mesh_assets"][0]
        encoded_colors = set(_decode_color32(mesh_asset["colors_b64"]))
        assert (255, 0, 0, 255) in encoded_colors
        assert (0, 0, 255, 255) in encoded_colors


def test_visual_mesh_payload_uses_dae_texture_average_color():
    image_module = pytest.importorskip("PIL.Image")

    with tempfile.TemporaryDirectory() as tmp_dir:
        package_root = Path(tmp_dir) / "tiny_bot_description"
        _write_text(
            package_root / "package.xml",
            "<package><name>tiny_bot_description</name></package>",
        )
        texture_path = package_root / "meshes" / "panel.png"
        texture_path.parent.mkdir(parents=True, exist_ok=True)
        image = image_module.new("RGB", (2, 1))
        image.putpixel((0, 0), (128, 0, 0))
        image.putpixel((1, 0), (0, 0, 128))
        image.save(texture_path)
        _write_text(
            package_root / "meshes" / "panel.dae",
            """
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <library_effects>
    <effect id="panel-effect">
      <profile_COMMON>
        <newparam sid="panel-surface">
          <surface type="2D"><init_from>panel-image</init_from></surface>
        </newparam>
        <newparam sid="panel-sampler">
          <sampler2D><source>panel-surface</source></sampler2D>
        </newparam>
        <technique sid="common">
          <lambert><diffuse><texture texture="panel-sampler" texcoord="UVMap"/></diffuse></lambert>
        </technique>
      </profile_COMMON>
    </effect>
  </library_effects>
  <library_images>
    <image id="panel-image" name="panel"><init_from>panel.png</init_from></image>
  </library_images>
  <library_materials>
    <material id="panel-material" name="panel"><instance_effect url="#panel-effect"/></material>
  </library_materials>
  <library_geometries>
    <geometry id="panel-mesh" name="panel">
      <mesh>
        <source id="panel-positions">
          <float_array id="panel-positions-array" count="9">0 0 0 1 0 0 0 1 0</float_array>
          <technique_common>
            <accessor source="#panel-positions-array" count="3" stride="3"/>
          </technique_common>
        </source>
        <vertices id="panel-vertices"><input semantic="POSITION" source="#panel-positions"/></vertices>
        <triangles material="panel-material" count="1">
          <input semantic="VERTEX" source="#panel-vertices" offset="0"/>
          <p>0 1 2</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="Scene">
      <node id="panel-node">
        <instance_geometry url="#panel-mesh">
          <bind_material>
            <technique_common>
              <instance_material symbol="panel-material" target="#panel-material"/>
            </technique_common>
          </bind_material>
        </instance_geometry>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#Scene"/></scene>
</COLLADA>
            """,
        )
        urdf_path = package_root / "urdf" / "tiny_bot.urdf"
        _write_text(
            urdf_path,
            """
<robot name="tiny_bot">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://tiny_bot_description/meshes/panel.dae"/>
      </geometry>
    </visual>
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
        artifact = RobotDescriptionResolver().resolve_for_robot(robot)

        assert artifact is not None
        mesh_asset = artifact.payload_dict["mesh_assets"][0]
        expected_channel = (((128.0 / 255.0 + 0.055) / 1.055) ** 2.4) * 0.5
        assert mesh_asset["color_rgb"] == pytest.approx([expected_channel, 0.0, expected_channel], abs=0.01)
        encoded_colors = set(_decode_color32(mesh_asset["colors_b64"]))
        expected_byte = round(expected_channel * 255.0)
        assert (expected_byte, 0, expected_byte, 255) in encoded_colors


def test_visual_mesh_payload_preserves_obj_mtl_diffuse_color_hint():
    with tempfile.TemporaryDirectory() as tmp_dir:
        package_root = Path(tmp_dir) / "tiny_bot_description"
        _write_text(
            package_root / "package.xml",
            "<package><name>tiny_bot_description</name></package>",
        )
        _write_text(
            package_root / "meshes" / "panel.mtl",
            """
newmtl panel_blue
Kd 0.1 0.3 0.8
            """,
        )
        _write_text(
            package_root / "meshes" / "panel.obj",
            """
mtllib panel.mtl
usemtl panel_blue
v 0 0 0
v 1 0 0
v 0 1 0
f 1 2 3
            """,
        )
        urdf_path = package_root / "urdf" / "tiny_bot.urdf"
        _write_text(
            urdf_path,
            """
<robot name="tiny_bot">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://tiny_bot_description/meshes/panel.obj"/>
      </geometry>
    </visual>
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
        mesh_asset = artifact.payload_dict["mesh_assets"][0]
        assert mesh_asset["colors_b64"]
        assert mesh_asset["color_rgb"] == pytest.approx([0.1, 0.3, 0.8], rel=1e-6, abs=1e-6)


def test_visual_mesh_payload_resolves_named_urdf_material_color_and_mesh_id():
    with tempfile.TemporaryDirectory() as tmp_dir:
        package_root = Path(tmp_dir) / "tiny_bot_description"
        _write_text(
            package_root / "package.xml",
            "<package><name>tiny_bot_description</name></package>",
        )
        _write_text(
            package_root / "meshes" / "panel.obj",
            """
v 0 0 0
v 1 0 0
v 0 1 0
f 1 2 3
            """,
        )

        def resolve_with_color(color: str):
            urdf_path = package_root / f"tiny_bot_{color.replace(' ', '_')}.urdf"
            _write_text(
                urdf_path,
                f"""
<robot name="tiny_bot">
  <material name="body_color">
    <color rgba="{color} 1"/>
  </material>
  <link name="base_link">
    <visual>
      <material name="body_color"/>
      <geometry>
        <mesh filename="package://tiny_bot_description/meshes/panel.obj"/>
      </geometry>
    </visual>
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
            return RobotDescriptionResolver().resolve_for_robot(robot)

        green_artifact = resolve_with_color("0.0 0.8 0.0")
        orange_artifact = resolve_with_color("1.0 0.42 0.04")

        assert green_artifact is not None
        assert orange_artifact is not None
        green_asset = green_artifact.payload_dict["mesh_assets"][0]
        orange_asset = orange_artifact.payload_dict["mesh_assets"][0]
        assert green_asset["color_rgb"] == pytest.approx([0.0, 0.8, 0.0], rel=1e-6, abs=1e-6)
        assert orange_asset["color_rgb"] == pytest.approx([1.0, 0.42, 0.04], rel=1e-6, abs=1e-6)
        assert green_asset["mesh_id"] != orange_asset["mesh_id"]


def test_visual_mesh_payload_uses_gazebo_link_material_as_fallback():
    with tempfile.TemporaryDirectory() as tmp_dir:
        package_root = Path(tmp_dir) / "tiny_bot_description"
        _write_text(
            package_root / "package.xml",
            "<package><name>tiny_bot_description</name></package>",
        )
        _write_text(
            package_root / "meshes" / "panel.obj",
            """
v 0 0 0
v 1 0 0
v 0 1 0
f 1 2 3
            """,
        )
        urdf_path = package_root / "tiny_bot.urdf"
        _write_text(
            urdf_path,
            """
<robot name="tiny_bot">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://tiny_bot_description/meshes/panel.obj"/>
      </geometry>
    </visual>
  </link>
  <gazebo reference="base_link">
    <material>Gazebo/DarkGrey</material>
  </gazebo>
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
        artifact = RobotDescriptionResolver().resolve_for_robot(robot)

        assert artifact is not None
        mesh_asset = artifact.payload_dict["mesh_assets"][0]
        assert mesh_asset["color_rgb"] == pytest.approx([0.15, 0.15, 0.15], abs=1e-6)


def test_visual_mesh_payload_does_not_inherit_parent_gazebo_material_fallback():
    with tempfile.TemporaryDirectory() as tmp_dir:
        package_root = Path(tmp_dir) / "tiny_bot_description"
        _write_text(
            package_root / "package.xml",
            "<package><name>tiny_bot_description</name></package>",
        )
        _write_text(
            package_root / "meshes" / "panel.obj",
            """
v 0 0 0
v 1 0 0
v 0 1 0
f 1 2 3
            """,
        )
        urdf_path = package_root / "tiny_bot.urdf"
        _write_text(
            urdf_path,
            """
<robot name="tiny_bot">
  <link name="base_link"/>
  <link name="child_link">
    <visual>
      <geometry>
        <mesh filename="package://tiny_bot_description/meshes/panel.obj"/>
      </geometry>
    </visual>
  </link>
  <joint name="base_to_child" type="revolute">
    <parent link="base_link"/>
    <child link="child_link"/>
    <axis xyz="0 0 1"/>
  </joint>
  <gazebo reference="base_link">
    <material>Gazebo/DarkGrey</material>
  </gazebo>
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
        artifact = RobotDescriptionResolver().resolve_for_robot(robot)

        assert artifact is not None
        mesh_asset = artifact.payload_dict["mesh_assets"][0]
        assert "color_rgb" not in mesh_asset


def test_visual_mesh_payload_splits_movable_groups_from_fixed_chains():
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
f 1 2 3
            """,
        )
        urdf_path = package_root / "urdf" / "tiny_bot.urdf"
        _write_text(
            urdf_path,
            """
<robot name="tiny_bot">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://tiny_bot_description/meshes/panel.obj"/>
      </geometry>
    </visual>
  </link>
  <link name="sensor_link">
    <visual>
      <geometry>
        <mesh filename="package://tiny_bot_description/meshes/panel.obj"/>
      </geometry>
    </visual>
  </link>
  <link name="wheel_link">
    <visual>
      <geometry>
        <mesh filename="package://tiny_bot_description/meshes/panel.obj"/>
      </geometry>
    </visual>
  </link>
  <joint name="fixed_sensor" type="fixed">
    <parent link="base_link"/>
    <child link="sensor_link"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
  </joint>
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
            """,
        )

        robot = Robot(name="tiny_bot", robot_type=RobotType.WHEELED)
        robot.configure_robot_description(
            urdf_path=str(urdf_path),
            base_frame="base_link",
            include_visual_meshes=True,
            visual_mesh_triangle_budget=3000,
        )
        resolver = RobotDescriptionResolver()
        artifact = resolver.resolve_for_robot(robot)

        assert artifact is not None
        payload = artifact.payload_dict
        visual_links = sorted(payload["visual_links"], key=lambda entry: entry["frame_id"])
        assert [entry["frame_id"] for entry in visual_links] == ["base_link", "wheel_link"]
        visual_origins = {entry["frame_id"]: entry["origin_xyz"] for entry in visual_links}
        assert visual_origins["base_link"] == pytest.approx([0.0, 0.0, 0.0], abs=1e-6)
        assert visual_origins["wheel_link"] == pytest.approx([1.0, 0.0, 0.0], abs=1e-6)
        assert len(payload["mesh_assets"]) == 2


def test_collision_only_mode_omits_visual_mesh_assets():
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
f 1 2 3
            """,
        )
        urdf_path = package_root / "urdf" / "tiny_bot.urdf"
        _write_text(
            urdf_path,
            """
<robot name="tiny_bot">
  <link name="base_link">
    <visual>
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
            body_mesh_mode="collision_only",
        )
        resolver = RobotDescriptionResolver()
        artifact = resolver.resolve_for_robot(robot)

        assert artifact is not None
        assert artifact.manifest.supports_visual_meshes is False
        assert artifact.payload_dict.get("mesh_assets", []) == []
        assert artifact.payload_dict.get("visual_links", []) == []


def test_runtime_high_mesh_mode_preserves_full_triangle_count():
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
v 1 1 0
v 0 1 0
v 2 0 0
v 2 1 0
f 1 2 3
f 1 3 4
f 2 5 6
f 2 6 3
            """,
        )
        urdf_path = package_root / "urdf" / "tiny_bot.urdf"
        _write_text(
            urdf_path,
            """
<robot name="tiny_bot">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://tiny_bot_description/meshes/panel.obj"/>
      </geometry>
    </visual>
  </link>
</robot>
            """,
        )

        robot = Robot(name="tiny_bot", robot_type=RobotType.WHEELED)
        robot.configure_robot_description(
            urdf_path=str(urdf_path),
            base_frame="base_link",
            body_mesh_mode="runtime_high_mesh",
            visual_mesh_triangle_budget=2,
        )
        resolver = RobotDescriptionResolver()
        artifact = resolver.resolve_for_robot(robot)

        assert artifact is not None
        mesh_assets = artifact.payload_dict["mesh_assets"]
        assert len(mesh_assets) == 1
        assert mesh_assets[0]["triangle_count"] == 4
        assert mesh_assets[0]["colors_b64"]
