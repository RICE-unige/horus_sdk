"""Deterministic kinematic humanoid and diagnostic cameras, never robot hardware.

This is a fixture for HORUS interaction and recording, not a dynamics simulator
or training dataset. Every output is confined to the teach_synthetic namespace.
"""

from copy import deepcopy
import json
import math
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

NAME = "teach_synthetic"
PREFIX = "/" + NAME
HANDS = ("left", "right")
ARM_JOINTS = ("shoulder_yaw", "shoulder_pitch", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw", "gripper")
JOINT_NAMES = tuple(f"{hand}_{joint}" for hand in HANDS for joint in ARM_JOINTS)
SHOULDERS = {"left": np.array([0., .24, 1.30]), "right": np.array([0., -.24, 1.30])}
LENGTHS = (.29, .27)


def inverse_arm(position, rotation, hand):
    delta = np.asarray(position, dtype=float) - SHOULDERS[hand]
    reach = float(np.linalg.norm(delta))
    if not abs(LENGTHS[0] - LENGTHS[1]) + .015 <= reach <= sum(LENGTHS) - .015:
        raise ValueError("Synthetic arm reach exceeded; hold and re-anchor")
    yaw = math.atan2(delta[1], delta[0])
    elbow = math.acos(np.clip((reach**2 - sum(x*x for x in LENGTHS)) / (2 * LENGTHS[0] * LENGTHS[1]), -1, 1))
    pitch = math.atan2(-delta[2], math.hypot(delta[0], delta[1])) - math.atan2(
        LENGTHS[1] * math.sin(elbow), LENGTHS[0] + LENGTHS[1] * math.cos(elbow))
    forearm = Rotation.from_euler("z", yaw) * Rotation.from_euler("y", pitch + elbow)
    wrist = (forearm.inv() * Rotation.from_quat(rotation)).as_euler("XYZ")
    return np.array([yaw, pitch, elbow, *wrist])


def forward_arm(joints, hand):
    shoulder = Rotation.from_euler("z", joints[0]) * Rotation.from_euler("y", joints[1])
    elbow = shoulder * Rotation.from_euler("y", joints[2])
    position = SHOULDERS[hand] + shoulder.apply([LENGTHS[0], 0, 0]) + elbow.apply([LENGTHS[1], 0, 0])
    orientation = elbow * Rotation.from_euler("XYZ", joints[3:6])
    return dict(position=position.tolist(), rotation=orientation.as_quat().tolist())


def description():
    """Self-authored colored primitives; no downloads or vendor mesh licenses."""
    root = ET.Element("robot", name=NAME)
    cube = Path(__file__).resolve().parents[1] / "assets/teach_synthetic/cube.obj"

    def link(name, size=None, center=(0, 0, 0), color="0.72 0.77 0.82 1"):
        node = ET.SubElement(root, "link", name=name)
        if size is not None:
            visual = ET.SubElement(node, "visual")
            ET.SubElement(visual, "origin", xyz=" ".join(map(str, center)))
            ET.SubElement(ET.SubElement(visual, "geometry"), "mesh", filename=str(cube), scale=" ".join(map(str, size)))
            ET.SubElement(ET.SubElement(visual, "material", name=name + "_color"), "color", rgba=color)

    def joint(name, parent, child, xyz=(0, 0, 0), axis=None, rpy=(0, 0, 0), prismatic=False):
        node = ET.SubElement(root, "joint", name=name, type="fixed" if axis is None else "prismatic" if prismatic else "revolute")
        ET.SubElement(node, "parent", link=parent)
        ET.SubElement(node, "child", link=child)
        ET.SubElement(node, "origin", xyz=" ".join(map(str, xyz)), rpy=" ".join(map(str, rpy)))
        if axis is not None:
            ET.SubElement(node, "axis", xyz=" ".join(map(str, axis)))
            ET.SubElement(node, "limit", lower="0" if prismatic else "-3.141593", upper="0.04" if prismatic else "3.141593", effort="1", velocity="2")

    link("base_footprint")
    link("torso", (.26, .38, .46), color="0.15 0.38 0.75 1")
    joint("torso_fixed", "base_footprint", "torso", (0, 0, 1.1))
    link("head", (.20, .20, .22), color="0.80 0.83 0.87 1")
    joint("head_fixed", "base_footprint", "head", (0, 0, 1.48))
    link("face", (.015, .15, .07), color="0.04 0.10 0.15 1")
    joint("face_fixed", "head", "face", (.107, 0, .02))
    for hand, sign, color in (("left", 1, "0.1 0.65 0.60 1"), ("right", -1, "0.85 0.3 0.15 1")):
        link(hand + "_leg", (.13, .14, .76), color="0.3 0.35 0.42 1")
        joint(hand + "_leg_fixed", "base_footprint", hand + "_leg", (0, sign * .12, .48))
        link(hand + "_foot", (.28, .16, .10))
        joint(hand + "_foot_fixed", "base_footprint", hand + "_foot", (.05, sign * .12, .05))
        link(hand + "_shoulder")
        joint(hand + "_shoulder_yaw", "base_footprint", hand + "_shoulder", SHOULDERS[hand], (0, 0, 1))
        link(hand + "_upper_arm", (LENGTHS[0], .09, .09), (LENGTHS[0]/2, 0, 0), color)
        joint(hand + "_shoulder_pitch", hand + "_shoulder", hand + "_upper_arm", axis=(0, 1, 0))
        link(hand + "_forearm", (LENGTHS[1], .075, .075), (LENGTHS[1]/2, 0, 0), color)
        joint(hand + "_elbow", hand + "_upper_arm", hand + "_forearm", (LENGTHS[0], 0, 0), (0, 1, 0))
        parent = hand + "_forearm"
        for index, (part, axis) in enumerate((("roll", (1, 0, 0)), ("pitch", (0, 1, 0)), ("yaw", (0, 0, 1)))):
            child = hand + "_wrist_" + part + "_link"
            link(child, (.07, .07, .06) if index == 2 else None, color=color)
            joint(hand + "_wrist_" + part, parent, child, (LENGTHS[1], 0, 0) if index == 0 else (0, 0, 0), axis)
            parent = child
        link(hand + "_finger", (.08, .015, .04), (.04, 0, 0), color)
        joint(hand + "_gripper", parent, hand + "_finger", (.03, .015, 0), (0, 1, 0), prismatic=True)
        link(hand + "_finger_fixed", (.08, .015, .04), (.04, 0, 0), color)
        joint(hand + "_finger_fixed_joint", parent, hand + "_finger_fixed", (.03, -.025, 0))
        link(hand + "_camera_optical")
        joint(hand + "_camera_fixed", parent, hand + "_camera_optical", (.06, 0, .05), rpy=(-math.pi/2, 0, -math.pi/2))
    link("front_camera_optical")
    joint("front_camera_fixed", "head", "front_camera_optical", (.12, 0, .025), rpy=(-math.pi/2, 0, -math.pi/2))
    return ET.tostring(root, encoding="unicode")


class SyntheticHumanoid:
    """Bounded, authority-scoped kinematics with an independent servo watchdog."""

    def __init__(self):
        self.lock = threading.RLock()
        self.joints = {h: np.r_[inverse_arm([.32, SHOULDERS[h][1], 1.06], [0, 0, 0, 1], h), .025] for h in HANDS}
        self.target = deepcopy(self.joints)
        self.lease = {}
        self.lease_deadline = 0.
        self.active_authority = None
        self.feedback_time = time.monotonic()
        self.last_command = 0.
        self.mode = "holding"
        self.accepted = 0
        self.last_bundle = None

    def update_lease(self, owner, generation, remaining):
        with self.lock:
            self.lease = dict(owner=owner, authority_generation=generation)
            self.lease_deadline = time.monotonic() + min(max(remaining, 0), 3)

    def feedback(self):
        with self.lock:
            return dict(ready=True, received_monotonic=self.feedback_time,
                        poses={h: forward_arm(self.joints[h], h) for h in HANDS})

    def publish(self, bundle):
        with self.lock:
            authority = bundle["authority"]
            if authority.get("robot_name") != NAME or time.monotonic() >= self.lease_deadline or any(
                    authority.get(k) != v for k, v in self.lease.items()) or not self.lease.get("owner"):
                raise RuntimeError("Synthetic controller authority expired or changed")
            targets = {h: np.r_[inverse_arm(bundle["targets"][h]["position"], bundle["targets"][h]["rotation"], h),
                               .04 * (1 - bundle.get("tools", {}).get(h + "_grasp", 0.))] for h in HANDS}
            self.target = targets
            self.active_authority = deepcopy(authority)
            self.last_command = time.monotonic()
            self.mode = "tracking"
            self.accepted += 1
            self.last_bundle = deepcopy(bundle)
            return dict(ok=True, boundary="scheduled", execution="unconfirmed", provenance="fixture")

    def hold(self, context):
        with self.lock:
            identity = {k: context.get(k) for k in ("robot_name", "owner", "session_id", "authority_generation")}
            if self.active_authority is not None and identity != self.active_authority:
                return False
            self.target = deepcopy(self.joints)
            self.active_authority = None
            self.mode = "holding"
            return True

    def step(self, dt):
        with self.lock:
            now = time.monotonic()
            if now - self.last_command > .25 or now >= self.lease_deadline:
                self.target = deepcopy(self.joints)
                self.mode = "holding"
            for hand in HANDS:
                change = self.target[hand] - self.joints[hand]
                self.joints[hand] += np.clip(change, -2 * dt, 2 * dt)
            self.feedback_time = now

    def snapshot(self):
        with self.lock:
            return dict(zip(JOINT_NAMES, [float(v) for h in HANDS for v in self.joints[h]]))

    def adapter(self):
        from horus.teaching.adapter import AdapterCapabilities
        from horus.teaching.cartesian import CartesianAdapter, CartesianLimits
        cap = AdapterCapabilities("synthetic_cartesian", "simulation", JOINT_NAMES, "base_footprint",
            "synthetic-v1", "cartesian_pose_absolute", "independent 250ms synthetic servo watchdog",
            profile_revision="1", command_boundary="scheduled", hold_description="freeze synthetic joint targets",
            authority_contract="controller verifies owner/generation TTL and scopes hold to original session",
            joint_units={j: "m" if j.endswith("gripper") else "rad" for j in JOINT_NAMES})
        return CartesianAdapter(cap, CartesianLimits((-.4, -.8, .55), (.7, .8, 1.9), 3., 12.),
            feedback=self.feedback, publish=self.publish, hold=self.hold)


def camera_image(model, view, frame):
    """Moving, distinct RGB diagnostic views of the synthetic hands and targets."""
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    image[:] = (24, 29, 38)
    image[::24, :, :] = (45, 55, 68)
    image[:, ::32, :] = (45, 55, 68)
    image[155:220, 24:296] = (75, 84, 100)
    image[175:198, 143:167] = (230, 196, 70)
    poses = model.feedback()["poses"]
    for hand, color in (("left", (35, 205, 175)), ("right", (235, 90, 45))):
        x, y, z = poses[hand]["position"]
        horizontal = -y if view == "front" else x - .3
        u, v = int(160 + horizontal * 190), int(125 - (z - 1.1) * 140)
        u, v = max(12, min(307, u)), max(12, min(227, v))
        image[v-10:v+10, u-10:u+10] = color
    image[6:12, 8:8 + (frame % 300)] = {"front": (120, 160, 255), "left": (35, 205, 175), "right": (235, 90, 45)}[view]
    return image


def stereo_portal_fixture(frame):
    """Rectified pinhole pair of three known-depth targets; diagnostic only."""
    images = []
    for eye_x in (-.032, .032):
        image = np.full((240, 320, 3), (25, 32, 40), np.uint8)
        for x, y, z, size, color in ((-.55, -.2, 3., .6, (65, 150, 235)),
                                     (.35, .1, 1.8, .35, (225, 180, 60)),
                                     (.07 * math.sin(frame * .04), 0., .8, .12, (55, 210, 170))):
            u, v = int(160 + 240 * (x - eye_x) / z), int(120 + 240 * y / z)
            half = int(120 * size / z)
            image[max(0, v-half):min(240, v+half), max(0, u-half):min(320, u+half)] = color
        images.append(image)
    return images


class SyntheticSource:
    def __init__(self, model, urdf, stereo_front=False):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from sensor_msgs.msg import Image, JointState, CameraInfo
        from tf2_msgs.msg import TFMessage
        from std_msgs.msg import String
        self.model, self.urdf = model, ET.fromstring(urdf)
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node("horus_teach_synthetic_source", context=self.context)
        self.tf = self.node.create_publisher(TFMessage, PREFIX + "/tf", 5)
        self.joints = self.node.create_publisher(JointState, PREFIX + "/joint_states", 5)
        self.images = {v: self.node.create_publisher(Image, PREFIX + f"/{v}/image_raw", 2) for v in ("front", *HANDS)}
        self.stereo_right = self.node.create_publisher(Image, PREFIX + "/front/right/image_raw", 2) if stereo_front else None
        self.info = {v: self.node.create_publisher(CameraInfo, PREFIX + f"/{v}/camera_info", 2) for v in self.images}
        self.commands = self.node.create_publisher(String, PREFIX + "/scheduled_targets", 5)
        desc = self.node.create_publisher(String, PREFIX + "/robot_description",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        desc.publish(String(data=urdf))
        self.node.create_subscription(String, "/horus/multi_operator/control_lease_state", self._lease, 10)
        self.frame = 0
        self.last_published_command = 0
        self.last_step = time.monotonic()
        self.node.create_timer(1/30, self._tick)
        self.node.create_timer(.1, self._cameras)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self._spin, name="horus-synthetic-robot")

    def _spin(self):
        from rclpy.executors import ExternalShutdownException
        from rclpy._rclpy_pybind11 import RCLError
        try:
            self.executor.spin()
        except ExternalShutdownException:
            pass
        except RCLError:
            if self.context.ok(): raise

    def _lease(self, message):
        data = json.loads(message.data)
        lease = next((v for v in data.get("leases", []) if v.get("robot_name") == NAME), {})
        self.model.update_lease(lease.get("holder_app_id", ""), lease.get("authority_generation", ""), lease.get("remaining_ms", 0) / 1000)

    def _tick(self):
        from geometry_msgs.msg import TransformStamped
        from sensor_msgs.msg import JointState
        from tf2_msgs.msg import TFMessage
        from std_msgs.msg import String
        now = time.monotonic()
        self.model.step(min(now - self.last_step, .05))
        self.last_step = now
        positions = self.model.snapshot()
        stamp = self.node.get_clock().now().to_msg()
        root = TransformStamped(child_frame_id=NAME + "/base_footprint")
        root.header.frame_id, root.header.stamp = "map", stamp
        root.transform.rotation.w = 1.
        transforms = [root]
        for joint in self.urdf.findall("joint"):
            message = TransformStamped(child_frame_id=NAME + "/" + joint.find("child").get("link"))
            message.header.frame_id = NAME + "/" + joint.find("parent").get("link")
            message.header.stamp = stamp
            origin = joint.find("origin")
            xyz = np.fromstring(origin.get("xyz"), sep=" ")
            rotation = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy"), sep=" "))
            value = positions.get(joint.get("name"), 0.)
            axis = joint.find("axis")
            if axis is not None:
                vector = np.fromstring(axis.get("xyz"), sep=" ")
                if joint.get("type") == "prismatic": xyz += rotation.apply(vector * value)
                else: rotation = rotation * Rotation.from_rotvec(vector * value)
            message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = map(float, xyz)
            q = rotation.as_quat()
            message.transform.rotation.x, message.transform.rotation.y, message.transform.rotation.z, message.transform.rotation.w = map(float, q)
            transforms.append(message)
        self.tf.publish(TFMessage(transforms=transforms))
        joints = JointState(name=list(positions), position=list(positions.values()))
        joints.header.stamp = stamp
        self.joints.publish(joints)
        with self.model.lock:
            if self.model.accepted != self.last_published_command:
                self.commands.publish(String(data=json.dumps(self.model.last_bundle)))
                self.last_published_command = self.model.accepted

    def _cameras(self):
        from sensor_msgs.msg import Image, CameraInfo
        self.frame += 1
        for view, publisher in self.images.items():
            pair = stereo_portal_fixture(self.frame) if view == "front" and self.stereo_right is not None else None
            image = Image(width=320, height=240, encoding="rgb8", step=960,
                          data=(pair[0] if pair is not None else camera_image(self.model, view, self.frame)).tobytes())
            image.header.frame_id = f"{NAME}/{view}_camera_optical"
            image.header.stamp = self.node.get_clock().now().to_msg()
            publisher.publish(image)
            if pair is not None:
                right = Image(width=320, height=240, encoding="rgb8", step=960, data=pair[1].tobytes())
                right.header.stamp = image.header.stamp
                right.header.frame_id = f"{NAME}/front_right_camera_optical"
                self.stereo_right.publish(right)
            calibration = CameraInfo(width=320, height=240, distortion_model="plumb_bob",
                d=[0.] * 5, k=[240., 0., 160., 0., 240., 120., 0., 0., 1.],
                r=[1., 0., 0., 0., 1., 0., 0., 0., 1.],
                p=[240., 0., 160., 0., 0., 240., 120., 0., 0., 0., 1., 0.])
            calibration.header = image.header
            self.info[view].publish(calibration)

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *args):
        self.executor.shutdown()
        if self.thread.is_alive(): self.thread.join()
        self.node.destroy_node()
        self.context.try_shutdown()
