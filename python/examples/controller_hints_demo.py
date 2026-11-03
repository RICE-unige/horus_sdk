#!/usr/bin/env python3
"""Hardware-free controller hints: virtual driving plus a B-button inspection lamp."""
import math
import threading
import time

from horus import ControllerHint, ControllerHintContext, ControllerHintProfile, ControllerHints, TeleopHintMode
from horus.robot import Robot, RobotType, register_robots, is_registration_cancelled
from horus.sensors import Camera


def make_registration():
    robot = Robot('hint_demo', RobotType.WHEELED, dimensions=(.5, .3, .25))
    robot.configure_ros_binding(tf_prefix=robot.name, topic_mode='flat', base_frame='base_link')
    robot.configure_navigation_tasks(go_to_point_enabled=False, waypoint_enabled=False)
    robot.configure_teleop(command_topic='/hint_demo/cmd_vel', raw_input_topic='/hint_demo/joy')
    hints = ControllerHints(tuple(ControllerHintProfile(mode, (
        ControllerHint('left_stick_y', 'move_forward', 'Forward/back'),
        ControllerHint('right_stick_x', 'turn', 'Turn'),
        ControllerHint('right_secondary', 'lamp', 'Toggle lamp'),
    ), 'inspection') for mode in (TeleopHintMode.MINIMAP, TeleopHintMode.IMMERSIVE, TeleopHintMode.FOLLOW_LEADER)),
        state_topic='/hint_demo/controller_hints')
    robot.configure_controller_hints(hints)
    view = robot.create_dataviz()
    view.add_robot_transform(robot.name, '/hint_demo/tf', frame_id='base_link')
    camera = Camera('front', 'hint_demo/camera', '/hint_demo/camera/image_raw', resolution=(160, 120),
        encoding='rgb8', streaming_type='ros', minimap_streaming_type='ros', teleop_streaming_type='ros',
        minimap_image_type='raw', teleop_image_type='raw')
    robot.add_sensor(camera); view.add_sensor_visualization(camera, robot.name)
    return robot, view, hints


def main():
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from geometry_msgs.msg import Twist, TransformStamped
    from sensor_msgs.msg import Joy, Image
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    robot, view, hints = make_registration()
    context = Context(); rclpy.init(context=context)

    class VirtualInspection(Node):
        def __init__(self):
            super().__init__('horus_controller_hint_fixture', context=context)
            self.lamp = self.pressed = False
            self.current = None; self.revision = 0
            self.x = self.y = self.yaw = self.forward = self.turn = 0.
            self.last_velocity = 0.
            self.output = self.create_publisher(String, hints.state_topic, 1)
            self.tf = self.create_publisher(TFMessage, '/hint_demo/tf', 1)
            self.image = self.create_publisher(Image, '/hint_demo/camera/image_raw', 1)
            self.create_subscription(String, hints.context_topic, self.on_context, 1)
            self.create_subscription(Joy, '/hint_demo/joy', self.on_joy, 1)
            self.create_subscription(Twist, '/hint_demo/cmd_vel', self.on_velocity, 1)
            self.create_timer(.05, self.tick)
            self.create_timer(.2, self.camera)

        def on_context(self, message):
            try: self.current = ControllerHintContext.from_json(message.data)
            except (ValueError, TypeError): return
            self.send_labels()

        def on_joy(self, message):
            pressed = len(message.buttons) > 1 and bool(message.buttons[1])
            if pressed and not self.pressed:
                self.lamp = not self.lamp; self.send_labels()
            self.pressed = pressed

        def on_velocity(self, message):
            self.forward, self.turn = message.linear.x, message.angular.z
            self.last_velocity = time.monotonic()

        def send_labels(self):
            if self.current is None: return
            self.revision += 1
            try:
                update = hints.reply(self.current, robot_name=robot.name,
                    labels={'lamp': 'Lamp on' if self.lamp else 'Lamp off'}, revision=self.revision)
            except ValueError: return
            self.output.publish(String(data=update.to_json()))

        def tick(self):
            if time.monotonic() - self.last_velocity > .3: self.forward = self.turn = 0.
            self.yaw += self.turn * .05
            self.x += self.forward * math.cos(self.yaw) * .05
            self.y += self.forward * math.sin(self.yaw) * .05
            t = TransformStamped(); t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'; t.child_frame_id = 'hint_demo/base_link'
            t.transform.translation.x = self.x; t.transform.translation.y = self.y
            t.transform.rotation.z = math.sin(self.yaw / 2); t.transform.rotation.w = math.cos(self.yaw / 2)
            c = TransformStamped(); c.header.stamp = t.header.stamp
            c.header.frame_id = 'hint_demo/base_link'; c.child_frame_id = 'hint_demo/camera'
            c.transform.translation.x = .2; c.transform.translation.z = .25
            c.transform.rotation.x = -.5; c.transform.rotation.y = .5; c.transform.rotation.z = -.5; c.transform.rotation.w = .5
            self.tf.publish(TFMessage(transforms=[t, c]))

        def camera(self):
            msg = Image(); msg.header.stamp = self.get_clock().now().to_msg(); msg.header.frame_id = 'hint_demo/camera'
            msg.width = 160; msg.height = 120; msg.encoding = 'rgb8'; msg.step = 480
            color = bytes((210, 180, 40) if self.lamp else (35, 65, 100))
            msg.data = color * (160 * 120); self.image.publish(msg)

    node = VirtualInspection(); executor = SingleThreadedExecutor(context=context); executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True); thread.start()
    try:
        ok, result = register_robots([robot], datavizs=[view], keep_alive=True, compass_enabled=False)
        if not ok and not is_registration_cancelled(result): raise SystemExit(str(result))
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(); thread.join(timeout=2); node.destroy_node(); context.shutdown()


if __name__ == '__main__': main()
