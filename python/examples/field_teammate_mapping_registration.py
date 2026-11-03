#!/usr/bin/env python3
"""Register the live Field teammate, travelled path and full-detail nvblox maps.

Run through ./field-demo to start the real sensor, trajectory and mesh adapters.
This entry point alone registers existing streams; it does not start mapping.
"""
import argparse

from horus.robot import RobotDimensions, WorkspaceVisualization, is_registration_cancelled, register_entities
from field_teammate_registration import build_field_teammate, build_parser
from tools.field_teammate_scene import scene_frames
from examples.tools.field_map_source import resolve_map_source
from horus.dataviz.mesh_display import mesh_display_options


def build_entities(name="field_teammate_1", mapping=True, estimated=False, adaptive=False, map_source=None,
                   mesh_render_profile="off"):
    if mesh_render_profile not in ("off", "lod", "spacewarp", "combined"):
        raise ValueError("Unknown mesh render profile")
    source = resolve_map_source(estimated, map_source)
    arguments = ["--name", name]
    if mapping:
        arguments += ["--human-model-source", "head_only"]
    args = build_parser().parse_args(arguments)
    teammate = build_field_teammate(args)
    floor_frame, _, camera_frame = scene_frames(name)
    if mapping:
        teammate.dimensions = RobotDimensions(length=0.38, width=0.34, height=0.38)
        teammate.configure_ros_binding(tf_prefix=f"{name}_scene", topic_mode="flat", base_frame="camera")
        teammate.configure_field_teammate(base_frame=camera_frame, camera_frame=camera_frame,
            body_model="head_only", profile_height_m=args.profile_height, profile_sex=args.profile_sex)
        teammate.get_sensor("fpv_camera").frame_id = camera_frame
        teammate.add_metadata("representation", "native_camera_head_marker_not_full_body")
    entities = [teammate]
    visualizations = [teammate.create_dataviz()]
    if mapping:
        visualizations[0].add_tf_tree(frame_id=floor_frame)
        visualizations[0].add_robot_local_path(name, f"/{name}/trajectory", frame_id=floor_frame,
            render_options={"color": "#FFC247", "line_width": 2})
        map_name = f"field_{source}{'_adaptive' if adaptive else ''}_map"
        source_topic = "/horus/field/estimated_mesh" if source == "estimated" else "/horus/field/mesh"
        resource = WorkspaceVisualization(map_name)
        resource.add_metadata("map_source", source)
        resource.add_metadata("provisional", source == "estimated")
        resource.add_metadata("original_source_topic", source_topic)
        dataviz = resource.create_dataviz(map_name)
        render_options = {"max_upload_milliseconds_per_frame": 2.5, "display": mesh_display_options({
            "lod_enabled": mesh_render_profile in ("lod", "combined"),
            "spacewarp_enabled": mesh_render_profile in ("spacewarp", "combined"),
        })}
        if adaptive:
            dataviz.add_adaptive_mesh(topic="/horus/field/adaptive_mesh", frame_id=floor_frame,
                interests=[
                    {"id": "teammate", "shape": "sphere", "frame": camera_frame,
                     "size": [2, 2, 2], "priority": 2.0, "min_fraction": .25},
                    {"id": "inspection", "shape": "box", "center": [0, 0, 1],
                     "size": [2, 2, 2], "priority": 1.0, "min_fraction": .25},
                ], render_options=render_options)
            resource.add_metadata("adaptive_source", source)
            resource.add_metadata("source_voxel_size_m", .01)
            resource.add_metadata("context_detail_target_m", .03)
        else:
            dataviz.add_nvblox_mesh(topic=source_topic, frame_id=floor_frame,
                render_options=render_options)
        entities.append(resource)
        visualizations.append(dataviz)
    return entities, visualizations


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", default="field_teammate_1")
    parser.add_argument("--receive-only", action="store_true")
    parser.add_argument("--estimated-depth", action="store_true")
    parser.add_argument("--adaptive", action="store_true")
    parser.add_argument("--mesh-render-profile", choices=("off", "lod", "spacewarp", "combined"), default="off")
    parser.add_argument("--map-source", choices=("measured", "estimated"),
                        help="Workspace map to display; default follows --estimated-depth")
    args = parser.parse_args()
    try:
        entities, visualizations = build_entities(args.name, not args.receive_only,
                                                 args.estimated_depth, args.adaptive, args.map_source,
                                                 args.mesh_render_profile)
    except ValueError as error:
        parser.error(str(error))
    success, result = register_entities(entities, datavizs=visualizations,
        workspace_scale=0.1, compass_enabled=False, keep_alive=True)
    if not success and not is_registration_cancelled(result):
        raise SystemExit(f"Field registration failed: {result}")


if __name__ == "__main__":
    main()
