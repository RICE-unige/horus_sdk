#!/usr/bin/env python3
"""Observation/recording template. Set these declarations to your robot driver.

This example does not command hardware. Supply a qualified TeachAdapter to
TeachRuntime and change the workflow explicitly to enable teaching motion.
"""

from pathlib import Path

from horus.robot import Robot, RobotType, register_robots, is_registration_cancelled
from horus.sensors import Camera
from horus.teaching import TeachConfig, TeachStream, TeachCamera
from horus.teaching.runtime import TeachRuntime


def make_registration():
    robot = Robot("teaching_robot", RobotType.HUMANOID, dimensions=(0.6, 0.6, 1.5))
    robot.configure_ros_binding(tf_prefix="teaching_robot", topic_mode="flat", base_frame="base_link")
    robot.configure_robot_description(source="topic", robot_description_topic="/teaching_robot/robot_description",
        base_frame="base_link", body_mesh_mode="runtime_high_mesh", visual_link_pose_source="tf")
    robot.configure_teleop(enabled=False)
    robot.configure_navigation_tasks(go_to_point_enabled=False, waypoint_enabled=False)
    front = Camera("front", "teaching_robot/camera_optical", "/teaching_robot/camera/image_raw",
        resolution=(640, 480), encoding="rgb8", streaming_type="ros", minimap_streaming_type="ros",
        teleop_streaming_type="ros", minimap_image_type="raw", teleop_image_type="raw")
    robot.add_sensor(front)
    robot.configure_teaching(TeachConfig(
        recording_root=str(Path.home() / "horus-data" / "teaching"),
        task="Observe the task", workflow="observe_record",
        task_presets=("Pick and place", "Two-arm transfer", "Pause and re-anchor"),
        streams=(
            TeachStream("joints", "/teaching_robot/joint_states", "sensor_msgs/msg/JointState", role="measured_state"),
            TeachStream("front", front.topic, "sensor_msgs/msg/Image", role="camera"),
            TeachStream("tf", "/tf", "tf2_msgs/msg/TFMessage", role="tf", required=False),
        ), cameras=(TeachCamera("front", stream_id="front"),),
    ))
    view = robot.create_dataviz()
    view.add_robot_transform(robot.name, "/tf", frame_id="base_link")
    view.add_sensor_visualization(front, robot.name)
    return robot, view


def main():
    robot, view = make_registration()
    with TeachRuntime(robot):
        ok, result = register_robots([robot], datavizs=[view], keep_alive=True, compass_enabled=False)
        if not ok and not is_registration_cancelled(result):
            raise SystemExit(f"Teach registration failed: {result}")


if __name__ == "__main__":
    main()
