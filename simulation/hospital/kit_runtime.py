"""Local hospital sensor/control adapter, executed inside the shared Isaac Sim."""

import json
import math
import os
import time
from pathlib import Path

import numpy as np

ROBOTS = ("carter1", "carter2", "carter3")
CAMERA_OFFSET = (0.35, 0.0, 0.58)
LASER_OFFSET = (0.0, 0.0, 0.64)
HEADLIGHT_OFFSET = (0.20, 0.0, 0.60)
HEADLIGHT_PITCH_DEG = -85.0
WHEEL_RADIUS = 0.14
WHEEL_TRACK = 0.413


def stamp(seconds):
    from builtin_interfaces.msg import Time
    ns = max(0, round(seconds * 1_000_000_000))
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def transform(parent, child, xyz, quat, timestamp):
    from geometry_msgs.msg import TransformStamped
    message = TransformStamped()
    message.header.frame_id = parent
    message.header.stamp = timestamp
    message.child_frame_id = child
    message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = map(float, xyz)
    message.transform.rotation.w, message.transform.rotation.x, message.transform.rotation.y, message.transform.rotation.z = map(float, quat)
    return message


def yaw_of(quat):
    w, x, y, z = quat
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def quaternion(yaw):
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


def multiply_quaternions(first, second):
    w,x,y,z = first
    a,b,c,d = second
    return (w*a-x*b-y*c-z*d, w*b+x*a+y*d-z*c,
            w*c-x*d+y*a+z*b, w*d+x*c-y*b+z*a)


def local_physics_pose(position, orientation, origin):
    """Express the physical chassis pose in its known spawn's odometry frame."""
    ox,oy,heading = origin
    x,y,z = position
    cosine,sine = math.cos(heading),math.sin(heading)
    local = (cosine*(x-ox)+sine*(y-oy), -sine*(x-ox)+cosine*(y-oy), z)
    return local, multiply_quaternions(quaternion(-heading), orientation)


def vector_in_body(vector, orientation):
    inverse = (orientation[0], -orientation[1], -orientation[2], -orientation[3])
    rotated = multiply_quaternions(multiply_quaternions(inverse, (0,*vector)), orientation)
    return rotated[1:]


def attach_headlight(stage, root):
    """A real scene light follows the physical chassis, not a copied pose."""
    from pxr import Gf, UsdGeom, UsdLux

    light = UsdLux.SphereLight.Define(stage, root+"/chassis_link/horus_headlight")
    light.CreateRadiusAttr(.025)
    light.CreateIntensityAttr(2000000.0)
    light.CreateNormalizeAttr(False)
    light.CreateColorAttr(Gf.Vec3f(1.0, .98, .94))
    shape = UsdLux.ShapingAPI.Apply(light.GetPrim())
    shape.CreateShapingConeAngleAttr(50.0)
    shape.CreateShapingConeSoftnessAttr(.35)
    shape.CreateShapingFocusAttr(1.0)
    xform = UsdGeom.Xformable(light.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*HEADLIGHT_OFFSET))
    xform.AddRotateYOp().Set(HEADLIGHT_PITCH_DEG)
    return light


def camera_graph(stage, root, name, width=640, height=360):
    import omni.graph.core as og
    from isaacsim.sensors.experimental.rtx import RtxCamera
    from pxr import Gf, UsdGeom
    import usdrt.Sdf

    path = f"{root}/chassis_link/horus_camera"
    camera = RtxCamera(path, tick_rate=15.0, translations=[CAMERA_OFFSET],
                       orientations=[[0.5, 0.5, -0.5, -0.5]])
    optical = UsdGeom.Camera(stage.GetPrimAtPath(path))
    optical.CreateFocalLengthAttr(12.0)
    optical.CreateHorizontalApertureAttr(20.955)
    optical.CreateVerticalApertureAttr(20.955 * height / width)
    optical.CreateClippingRangeAttr(Gf.Vec2f(0.05, 50.0))
    keys = og.Controller.Keys
    nodes = [("Tick", "omni.graph.action.OnPlaybackTick"),
             ("Context", "isaacsim.ros2.bridge.ROS2Context"),
             ("Product", "isaacsim.core.nodes.IsaacCreateRenderProduct")]
    values = [("Product.inputs:cameraPrim", [usdrt.Sdf.Path(path)]),
              ("Product.inputs:width", width), ("Product.inputs:height", height)]
    connections = [("Tick.outputs:tick", "Product.inputs:execIn")]
    for node, mode, topic in (("Color", "rgb", "color/image_raw"),
                              ("Depth", "depth", "depth/image_raw"),
                              ("Info", None, "camera_info")):
        kind = "ROS2CameraInfoHelper" if mode is None else "ROS2CameraHelper"
        nodes.append((node, f"isaacsim.ros2.bridge.{kind}"))
        values += [(f"{node}.inputs:nodeNamespace", f"/{name}/camera"),
                   (f"{node}.inputs:frameId", f"{name}/camera_optical"),
                   (f"{node}.inputs:topicName", topic),
                   (f"{node}.inputs:resetSimulationTimeOnStop", True),
                   (f"{node}.inputs:queueSize", 2)]
        if mode:
            values.append((f"{node}.inputs:type", mode))
        connections += [("Product.outputs:execOut", f"{node}.inputs:execIn"),
                        ("Product.outputs:renderProductPath", f"{node}.inputs:renderProductPath"),
                        ("Context.outputs:context", f"{node}.inputs:context")]
    og.Controller.edit({"graph_path": f"/World/HorusCamera_{name}", "evaluator_name": "execution"},
                       {keys.CREATE_NODES: nodes, keys.SET_VALUES: values, keys.CONNECT: connections})
    return camera


class HospitalRuntime:
    def __init__(self, directory, domain=87):
        self.directory = Path(directory)
        self.domain = domain
        self.commands = {name: (0.0, 0.0, 0.0) for name in ROBOTS}
        self.odom = np.zeros((3, 3), dtype=float)
        self.encoder_odom = np.zeros((3, 3), dtype=float)
        self.last_tf = self.last_scan = self.last_status = -1.0
        self.wall_start = time.monotonic()
        self.errors = 0
        self.subscription = None
        self.physics_subscription = None
        self.cameras = []

    async def start(self, scene):
        import omni.kit.app
        import omni.physx
        import omni.usd
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan, JointState, Imu
        from rosgraph_msgs.msg import Clock
        from tf2_msgs.msg import TFMessage
        from std_msgs.msg import String
        from isaacsim.core.experimental.prims import Articulation
        from isaacsim.sensors.experimental.physics import IMU, IMUSensor
        import isaacsim.core.experimental.utils.app as app_utils
        from pxr import PhysxSchema, UsdGeom, UsdPhysics

        if not Path(scene).is_file():
            raise FileNotFoundError(scene)
        await omni.usd.get_context().open_stage_async(str(scene))
        from omni.kit.viewport.utility import get_active_viewport
        # This unobserved desktop viewport is not one of the three sensor images.
        get_active_viewport().set_texture_resolution((320, 180))
        get_active_viewport().updates_enabled = False
        self.stage = omni.usd.get_context().get_stage()
        self.stage.SetEditTarget(self.stage.GetRootLayer())
        self.stage.SetStartTimeCode(0)
        self.stage.SetEndTimeCode(1_000_000_000)
        self.stage.SetTimeCodesPerSecond(60)
        self.stage.SetEditTarget(self.stage.GetSessionLayer())
        roots = [f"/World/Nova_Carter_ROS_{i}" for i in (1, 2, 3)]
        self.initial = []
        for root in roots:
            matrix = UsdGeom.Xformable(self.stage.GetPrimAtPath(root)).ComputeLocalToWorldTransform(0)
            q = matrix.ExtractRotationQuat()
            self.initial.append((*matrix.ExtractTranslation()[:2], yaw_of((q.GetReal(), *q.GetImaginary()))))
        for prim in self.stage.Traverse():
            if prim.IsA(UsdPhysics.Scene):
                PhysxSchema.PhysxSceneAPI.Apply(prim).CreateTimeStepsPerSecondAttr(60)
            if os.environ.get("HORUS_HOSPITAL_CONTACT_TEST") == "1" and str(prim.GetPath()).startswith("/World/Nova_Carter_ROS_") and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        self.headlights = [attach_headlight(self.stage, root) for root in roots]
        self.cameras = [camera_graph(self.stage, root, name) for root, name in zip(roots, ROBOTS)]
        self.imus = [IMUSensor(IMU.create(root+"/chassis_link/horus_imu",
            translations=[[0,0,0]], orientations=[[1,0,0,0]])) for root in roots]
        self.invalid_imu_readings = 0
        self.imu_readings = []
        self.imu_ok = False
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context, domain_id=self.domain)
        self.node = Node("hospital_simulation", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.clock = self.node.create_publisher(Clock, "/clock", 2)
        self.tf = self.node.create_publisher(TFMessage, "/tf", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.static = self.node.create_publisher(TFMessage, "/tf_static", latched)
        self.odom_pubs = [self.node.create_publisher(Odometry, f"/{name}/odom", 5) for name in ROBOTS]
        self.encoder_pubs = [self.node.create_publisher(Odometry, f"/{name}/wheel_odom", 3) for name in ROBOTS]
        self.truth_pubs = [self.node.create_publisher(Odometry, f"/{name}/ground_truth/odom", 5) for name in ROBOTS]
        self.scan_pubs = [self.node.create_publisher(LaserScan, f"/{name}/scan", 5) for name in ROBOTS]
        self.joint_pubs = [self.node.create_publisher(JointState, f"/{name}/joint_states", 3) for name in ROBOTS]
        self.imu_pubs = [self.node.create_publisher(Imu, f"/{name}/imu", 3) for name in ROBOTS]
        self.local_tf = [self.node.create_publisher(TFMessage, f"/{name}/tf", 10) for name in ROBOTS]
        self.local_static = [self.node.create_publisher(TFMessage, f"/{name}/tf_static", latched) for name in ROBOTS]
        self.status = self.node.create_publisher(String, "/hospital/simulation/status", 2)
        self.command_subs = [self.node.create_subscription(Twist, f"/{name}/cmd_vel",
            lambda msg, robot=name: self.receive_command(robot, msg), 2) for name in ROBOTS]
        self.articulations = Articulation([root + "/chassis_link" for root in roots])
        app_utils.play(commit=True)
        for _ in range(12):
            await omni.kit.app.get_app().next_update_async()
        self.wheel_indices = self.articulations.get_dof_indices(["joint_wheel_left", "joint_wheel_right"]).numpy().tolist()
        self.articulations.set_dof_gains(stiffnesses=0.0, dampings=100000.0, dof_indices=self.wheel_indices)
        self.query = omni.physx.get_physx_scene_query_interface()
        self.publish_static()
        self.physics_subscription = omni.physx.get_physx_interface().subscribe_physics_step_events(self.physics_step)
        self.subscription = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self.update)
        print("HOSPITAL_READY", self.wheel_indices, self.initial)

    def receive_command(self, name, message):
        linear, angular = message.linear.x, message.angular.z
        if math.isfinite(linear) and math.isfinite(angular):
            self.commands[name] = (max(-0.6, min(0.6, linear)),
                                   max(-1.2, min(1.2, angular)), time.monotonic())

    def physics_step(self, dt):
        velocities = []
        now = time.monotonic()
        for name in ROBOTS:
            linear, angular, received = self.commands[name]
            if now - received > 0.4:
                linear = angular = 0.0
            velocities.append(((linear - angular * WHEEL_TRACK / 2) / WHEEL_RADIUS,
                               (linear + angular * WHEEL_TRACK / 2) / WHEEL_RADIUS))
        self.articulations.set_dof_velocity_targets(velocities, dof_indices=self.wheel_indices)
        measured = self.articulations.get_dof_velocities(dof_indices=self.wheel_indices).numpy()
        linear = WHEEL_RADIUS * measured.sum(axis=1) / 2
        self.imu_readings = [sensor.get_sensor_reading() for sensor in self.imus]
        if not all(value.is_valid for value in self.imu_readings):
            self.imu_ok = False
            self.invalid_imu_readings += 1
            self.articulations.set_dof_velocity_targets([[0,0]]*3, dof_indices=self.wheel_indices)
            return
        self.imu_ok = True
        # Use the simulator IMU's ideal attitude, rather than integrating solver
        # velocity jitter. This is an explicit ideal sensor, not SLAM localization.
        measured_heading = np.array([yaw_of((r.orientation_w, r.orientation_x,
            r.orientation_y, r.orientation_z))-origin[2] for r,origin in zip(self.imu_readings,self.initial)])
        change = np.arctan2(np.sin(measured_heading-self.encoder_odom[:,2]), np.cos(measured_heading-self.encoder_odom[:,2]))
        angular = change / dt
        heading = self.encoder_odom[:, 2] + change / 2
        self.encoder_odom[:, 0] += linear * np.cos(heading) * dt
        self.encoder_odom[:, 1] += linear * np.sin(heading) * dt
        self.encoder_odom[:, 2] += change
        self.measured_twists = np.stack((linear, angular), axis=1)

    def publish_static(self):
        from tf2_msgs.msg import TFMessage
        public = []
        for number, name in enumerate(ROBOTS):
            x, y, yaw = self.initial[number]
            local = [transform("global_odom", "odom", (x, y, 0), quaternion(yaw), stamp(0)),
                     transform("base_link", "laser", LASER_OFFSET, (1, 0, 0, 0), stamp(0)),
                     transform("base_link", "camera_optical", CAMERA_OFFSET, (0.5, -0.5, 0.5, -0.5), stamp(0))]
            self.local_static[number].publish(TFMessage(transforms=local))
            for value in local:
                value.child_frame_id = f"{name}/{value.child_frame_id}"
                if value.header.frame_id != "global_odom":
                    value.header.frame_id = f"{name}/{value.header.frame_id}"
                public.append(value)
        self.static.publish(TFMessage(transforms=public))

    def publish_state(self, seconds, positions, orientations):
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import JointState, Imu
        from tf2_msgs.msg import TFMessage
        timestamp = stamp(seconds)
        joint_positions = self.articulations.get_dof_positions().numpy()
        joint_velocities = self.articulations.get_dof_velocities().numpy()
        linear_velocities,angular_velocities = self.articulations.get_velocities()
        linear_velocities,angular_velocities = linear_velocities.numpy(),angular_velocities.numpy()
        public = []
        for index, name in enumerate(ROBOTS):
            position, orientation = local_physics_pose(positions[index],orientations[index],self.initial[index])
            self.odom[index] = (position[0],position[1],yaw_of(orientation))
            local = transform("odom", "base_link", position, orientation, timestamp)
            self.local_tf[index].publish(TFMessage(transforms=[local]))
            local.header.frame_id = f"{name}/odom"
            local.child_frame_id = f"{name}/base_link"
            public.append(local)
            message = Odometry()
            message.header = local.header
            message.child_frame_id = local.child_frame_id
            message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = map(float, position)
            message.pose.pose.orientation = local.transform.rotation
            body_linear = vector_in_body(linear_velocities[index],orientations[index])
            body_angular = vector_in_body(angular_velocities[index],orientations[index])
            message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.linear.z = map(float,body_linear)
            message.twist.twist.angular.x, message.twist.twist.angular.y, message.twist.twist.angular.z = map(float,body_angular)
            self.odom_pubs[index].publish(message)
            encoder = Odometry()
            encoder.header = message.header
            encoder.child_frame_id = message.child_frame_id
            encoder.pose.pose.position.x, encoder.pose.pose.position.y = map(float,self.encoder_odom[index,:2])
            q = quaternion(self.encoder_odom[index,2])
            encoder.pose.pose.orientation.w,encoder.pose.pose.orientation.x,encoder.pose.pose.orientation.y,encoder.pose.pose.orientation.z = q
            encoder.twist.twist.linear.x = float(self.measured_twists[index,0])
            encoder.twist.twist.angular.z = float(self.measured_twists[index,1])
            self.encoder_pubs[index].publish(encoder)
            truth = Odometry()
            truth.header.stamp = timestamp
            truth.header.frame_id = "global_odom"
            truth.child_frame_id = f"{name}/ground_truth"
            truth.pose.pose.position.x, truth.pose.pose.position.y, truth.pose.pose.position.z = map(float, positions[index])
            truth.pose.pose.orientation.w, truth.pose.pose.orientation.x, truth.pose.pose.orientation.y, truth.pose.pose.orientation.z = map(float, orientations[index])
            self.truth_pubs[index].publish(truth)
            joints = JointState()
            joints.header.stamp = timestamp
            joints.name = self.articulations.dof_names
            joints.position = joint_positions[index].astype(float).tolist()
            joints.velocity = joint_velocities[index].astype(float).tolist()
            self.joint_pubs[index].publish(joints)
            reading = self.imu_readings[index]
            imu = Imu()
            imu.header.stamp = stamp(reading.time)
            imu.header.frame_id = f"{name}/base_link"
            imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = (
                reading.orientation_w, reading.orientation_x, reading.orientation_y, reading.orientation_z)
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = (
                reading.angular_velocity_x, reading.angular_velocity_y, reading.angular_velocity_z)
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = (
                reading.linear_acceleration_x, reading.linear_acceleration_y, reading.linear_acceleration_z)
            self.imu_pubs[index].publish(imu)
        self.tf.publish(TFMessage(transforms=public))

    def publish_scan(self, seconds, positions, orientations):
        from sensor_msgs.msg import LaserScan
        for index, name in enumerate(ROBOTS):
            yaw = yaw_of(orientations[index])
            x, y, z = map(float, positions[index])
            ox, oy, oz = LASER_OFFSET
            origin = (x + ox * math.cos(yaw) - oy * math.sin(yaw),
                      y + ox * math.sin(yaw) + oy * math.cos(yaw), z + oz)
            scan = LaserScan()
            scan.header.stamp = stamp(seconds)
            scan.header.frame_id = f"{name}/laser"
            scan.angle_min = -math.pi
            scan.angle_increment = 2 * math.pi / 360
            scan.angle_max = scan.angle_min + 359 * scan.angle_increment
            scan.range_min = 0.1
            scan.range_max = 20.0
            scan.scan_time = 0.1
            ranges = []
            for beam in range(360):
                angle = yaw + scan.angle_min + beam * scan.angle_increment
                hit = self.query.raycast_closest(origin, (math.cos(angle), math.sin(angle), 0.0), scan.range_max)
                own_body = hit.get("rigidBody", "").startswith(f"/World/Nova_Carter_ROS_{index+1}/")
                ranges.append(float(hit["distance"]) if hit["hit"] and not own_body and hit["distance"] >= scan.range_min else float("inf"))
            scan.ranges = ranges
            self.scan_pubs[index].publish(scan)

    def update(self, _event):
        import rclpy
        from isaacsim.core.simulation_manager import SimulationManager
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String
        seconds = SimulationManager.get_simulation_time()
        if seconds <= 0 or not self.imu_ok or not hasattr(self, "measured_twists"):
            return
        try:
            for _ in range(6):
                self.executor.spin_once(timeout_sec=0)
            self.clock.publish(Clock(clock=stamp(seconds)))
            positions, orientations = self.articulations.get_world_poses()
            positions, orientations = positions.numpy(), orientations.numpy()
            if seconds - self.last_tf >= 1 / 30 - 1e-5:
                self.publish_state(seconds, positions, orientations)
                self.last_tf = seconds
            if seconds - self.last_scan >= 0.1 - 1e-5:
                self.publish_scan(seconds, positions, orientations)
                self.last_scan = seconds
            if seconds - self.last_status >= 2:
                payload = {"sim_time": seconds, "elapsed_wall": time.monotonic() - self.wall_start,
                           "robots": list(ROBOTS), "odometry": self.odom.tolist(),
                           "wheel_odometry": self.encoder_odom.tolist(),
                           "ground_truth": positions.tolist(), "errors": self.errors,
                           "invalid_imu_readings": self.invalid_imu_readings,
                           "odometry_source": "ideal_simulator_physics_pose",
                           "heading_source": "simulator_physics_pose"}
                self.status.publish(String(data=json.dumps(payload)))
                self.directory.mkdir(parents=True, exist_ok=True)
                (self.directory / "simulation.json").write_text(json.dumps(payload, indent=2))
                self.last_status = seconds
        except Exception:
            self.errors += 1
            import traceback
            traceback.print_exc()

    def stop(self):
        self.subscription = None
        self.physics_subscription = None
        try:
            self.articulations.set_dof_velocity_targets([[0, 0]] * 3, dof_indices=self.wheel_indices)
        except Exception as error:
            print(f"Hospital physics was already stopped: {error}")
        finally:
            self.node.destroy_node()
            self.executor.shutdown()
            self.context.shutdown()
