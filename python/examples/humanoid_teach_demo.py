#!/usr/bin/env python3
"""Synthetic, hardware-free HORUS Teach registration and recording demo."""

import argparse
from pathlib import Path
import tempfile

from horus.robot import Robot, RobotType, register_robots, is_registration_cancelled
from horus.sensors import Camera
from horus.teaching import TeachAction, TeachConfig, TeachCamera, TeachStream, TeachPortal
from horus.teaching.runtime import TeachRuntime
from horus import ControllerHint, ControllerHintProfile, ControllerHints, TeleopHintMode
from tools.teach_synthetic import NAME, PREFIX, JOINT_NAMES, HANDS, SyntheticHumanoid, SyntheticSource, description


def make_registration(urdf_path, recording_root, input_schema_version=1, engagement_mode="hold_to_run", front_portal=None, task_actions=False):
    robot = Robot(NAME, RobotType.HUMANOID, dimensions=(.8, .9, 1.65))
    robot.configure_ros_binding(tf_prefix=NAME, topic_mode="flat", base_frame="base_footprint")
    robot.configure_robot_description(source="local", urdf_path=str(urdf_path), base_frame="base_footprint",
        body_mesh_mode="runtime_high_mesh", visual_link_pose_source="tf", visual_mesh_triangle_budget=20000)
    robot.configure_teleop(enabled=False)
    robot.configure_navigation_tasks(go_to_point_enabled=False, waypoint_enabled=False)
    robot.configure_robot_manager(status=True, data_viz=True, teleop=True, tasks=False)
    streams = [TeachStream("joints", PREFIX + "/joint_states", "sensor_msgs/msg/JointState", role="measured_state",
                           stamp_semantics="simulation"),
               TeachStream("tf", PREFIX + "/tf", "tf2_msgs/msg/TFMessage", role="tf"),
               TeachStream("scheduled_targets", PREFIX + "/scheduled_targets", "std_msgs/msg/String",
                           role="accepted_command", required=False)]
    cameras = []
    view = robot.create_dataviz()
    view.add_robot_transform(NAME, PREFIX + "/tf", frame_id="base_footprint")
    for name, position, width in (("front", (0., 1.65, 1.1), .7),
                                   ("left", (-.5, 1.1, .85), .42), ("right", (.5, 1.1, .85), .42)):
        portal = TeachPortal(mode=front_portal) if name == "front" and front_portal else None
        stereo = portal is not None and portal.mode != "mono"
        estimated = portal is not None and portal.mode == "estimated"
        sensor = Camera(name, f"{NAME}/{name}_camera_optical", f"{PREFIX}/{name}/image_raw",
            resolution=(320, 240), encoding="rgb8", streaming_type="ros", minimap_streaming_type="ros",
            teleop_streaming_type="ros", minimap_image_type="raw",
            teleop_image_type="compressed" if estimated else "raw", is_stereo=stereo,
            minimap_topic=f"{PREFIX}/{name}/image_raw",
            teleop_topic=f"{PREFIX}/front/portal_sbs/compressed" if estimated else f"{PREFIX}/{name}/image_raw",
            stereo_layout="side_by_side" if estimated else "dual_topic" if stereo else "mono",
            right_topic=f"{PREFIX}/front/right/image_raw" if stereo and not estimated else "")
        robot.add_sensor(sensor)
        view.add_sensor_visualization(sensor, NAME)
        streams.append(TeachStream(name, sensor.topic, "sensor_msgs/msg/Image", role="camera",
            source_identity="synthetic-" + name, stamp_semantics="simulation"))
        cameras.append(TeachCamera(name, role=name, position=position, width_m=width,
                                  stream_id=name, required_for_control=True, portal=portal))
    if front_portal == "stereo":
        streams.append(TeachStream("front_right", PREFIX + "/front/right/image_raw", "sensor_msgs/msg/Image",
            role="camera", source_identity="synthetic-front-right", stamp_semantics="simulation"))
    robot.configure_teaching(TeachConfig(str(Path(recording_root).expanduser().resolve()),
        streams=tuple(streams), cameras=tuple(cameras), provenance="fixture", workflow="collection",
        adapter_id="synthetic_cartesian", joint_names=JOINT_NAMES,
        joint_groups={h: tuple(j for j in JOINT_NAMES if j.startswith(h + "_")) for h in HANDS},
        command_frame="base_footprint", calibration_id="synthetic-v1",
        task="Synthetic reach and grasp", task_presets=("Synthetic reach and grasp", "Synthetic bimanual handover"),
        task_management=task_actions, episode_counts=task_actions,
        actions=(TeachAction("check_fixture", "Check fixture"),) if task_actions else (),
        input_schema_version=input_schema_version,
        engagement_mode=engagement_mode,
        required_input_features=("head_pose", "left_stick", "right_stick", "left_buttons", "right_buttons") if input_schema_version == 2 else (),
        clutch_binding="both_primary" if input_schema_version == 2 else "both_grips",
        grasp_bindings=({"left_grasp": "left_grip", "right_grasp": "right_grip", "left_pinch": "left_trigger", "right_pinch": "right_trigger"}
                        if input_schema_version == 2 else {"left_grasp": "left_trigger", "right_grasp": "right_trigger"})))
    # These labels describe the same grasp_bindings used by this synthetic adapter.
    bindings = robot.get_metadata("teaching_config")["grasp_bindings"]
    robot.configure_controller_hints(ControllerHints((ControllerHintProfile(TeleopHintMode.TEACH,
        tuple(ControllerHint(control, action, "Pinch" if "pinch" in action else "Grasp")
              for action, control in sorted(bindings.items())), "synthetic_hands"),)))
    return robot, view


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording-root", type=Path, default=Path.home() / "horus-data" / "teach-synthetic")
    parser.add_argument("--input-schema-version", type=int, choices=(1, 2), default=1)
    parser.add_argument("--continuous", action="store_true", help="Use v2 input with countdown engagement and X+A disengage")
    parser.add_argument("--task-actions", action="store_true", help="Show editable tasks, episode counts and a custom fixture action")
    parser.add_argument("--front-portal", choices=("mono", "stereo", "estimated"),
                        help="Opt-in front portal; estimated requires the separate camera_portal converter")
    args = parser.parse_args()
    if args.continuous:
        args.input_schema_version = 2
    # Only the temporary generated URDF is removed. Recordings are never temporary.
    with tempfile.TemporaryDirectory(prefix="horus-teach-model-") as temporary:
        path = Path(temporary) / "synthetic.urdf"
        urdf = description()
        path.write_text(urdf)
        robot, view = make_registration(path, args.recording_root, args.input_schema_version,
                                        "continuous" if args.continuous else "hold_to_run", args.front_portal, args.task_actions)
        model = SyntheticHumanoid()
        print("HORUS Teach synthetic demo: no hardware commands. Recordings:", args.recording_root)
        print("Connect HORUS, create a workspace, then Robot Manager > Teleop > Teach.")
        if args.front_portal == "estimated":
            print("Start the estimated preview converter: /teach_synthetic/front/image_raw -> /teach_synthetic/front/portal_sbs/compressed")
        if args.continuous:
            print("Align, Record, Engage; control starts after the 3-second countdown. X+A or Disengage stops control.")
            print("Grips close the grippers. Head pose, sticks, buttons and pinch triggers are recorded.")
        elif args.input_schema_version == 2:
            print("Align, Record, Engage; release then hold both primary buttons (X/A). Grips close the grippers.")
            print("Head pose, sticks, buttons and pinch triggers are recorded; this stationary fixture does not walk.")
        else:
            print("Align, Record, Engage; release then hold both grips. Triggers close the grippers.")
        if not args.continuous:
            print("Release the clutch to Hold.")
        print("Right secondary (B) also stops control. Select outcome, Finish & Save, then Record again.")
        def check_fixture(context):
            return {"accepted": context.is_current(), "message": "Synthetic fixture is ready", "reason": "Session changed"}
        def saved(episode):
            print(f"Saved episode {episode.episode_id}: {episode.task} ({episode.quality}), {episode.path}")
        try:
            with SyntheticSource(model, urdf, stereo_front=args.front_portal == "stereo"), TeachRuntime(robot, adapter=model.adapter(),
                    action_handlers={"check_fixture": check_fixture} if args.task_actions else None,
                    after_save=saved if args.task_actions else None):
                ok, result = register_robots([robot], datavizs=[view], keep_alive=True, compass_enabled=False)
                if not ok and not is_registration_cancelled(result):
                    raise SystemExit(f"Teach registration failed: {result}")
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
