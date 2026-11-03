"""HORUS-owned ROS recording/session host. Start explicitly via a context manager."""

import json
import logging
import threading
import time
import os
from pathlib import Path

from .config import TeachConfig
from .recording import Sample
from .session import TeachSession

logger = logging.getLogger(__name__)


class TeachRuntime:
    def __init__(self, robot, *, adapter=None, action_handlers=None, after_save=None, catalog=None):
        self.robot = robot
        self.config = TeachConfig.from_dict(robot.get_metadata("teaching_config"))
        self.session = TeachSession(self.config, robot.name, adapter, action_handlers=action_handlers,
                                    after_save=after_save, catalog=catalog)
        self.stop = threading.Event()
        self.node = None
        self.executor = None
        self.thread = None
        self.worker = None
        self.instance_lock = None

    def __enter__(self):
        # Fail before registering a session when its required recording extra is absent.
        import mcap.writer
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
        from rosidl_runtime_py.utilities import get_message
        from rclpy.serialization import serialize_message
        from std_msgs.msg import String
        from .ros_schema import message_schema
        import fcntl

        self.context = Context()
        try:
            directory = Path(os.environ.get("XDG_RUNTIME_DIR", str(Path.home() / ".cache"))) / "horus-teach"
            directory.mkdir(parents=True, exist_ok=True)
            self.instance_lock = (directory / f"{os.environ.get('ROS_DOMAIN_ID', '0')}-{self.robot.name}.lock").open("a+")
            try:
                fcntl.flock(self.instance_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise RuntimeError("A Teach host for this robot/domain is already running") from exc
            rclpy.init(context=self.context)
            self.node = rclpy.create_node("horus_teach_" + self.robot.name, context=self.context)
            topics = self.config.registration(self.robot.name)
            self.publisher = self.node.create_publisher(String, topics["state_topic"], 10)
            self.node.create_subscription(String, topics["request_topic"], self._request, 10)
            self.node.create_subscription(String, topics["input_topic"], self._input,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            # Volatile prevents a transient old lease snapshot authorizing a new host.
            self.node.create_subscription(String, "/horus/multi_operator/control_lease_state", self._lease,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE))
            for stream in self.config.streams:
                message_type = get_message(stream.message_type)
                schema = message_schema(stream.message_type)

                def receive(message, stream=stream, schema=schema):
                    received_unix = time.time_ns()
                    received_mono = time.monotonic_ns()
                    self.session.note_stream(stream.stream_id)
                    if not stream.retained_calibration and self.session.capture != "recording":
                        return
                    stamp = getattr(getattr(message, "header", None), "stamp", None)
                    source = stamp.sec * 1_000_000_000 + stamp.nanosec if stamp is not None else None
                    keys = [t.child_frame_id for t in message.transforms] if stream.retained_calibration and hasattr(message, "transforms") else None
                    self.session.append(Sample(stream.stream_id, serialize_message(message),
                                           received_unix, received_mono, source_ns=source,
                                           schema=schema, schema_encoding="ros2msg"), keys)

                self.node.create_subscription(message_type, stream.topic, receive,
                    QoSProfile(depth=stream.queue_depth,
                        reliability=ReliabilityPolicy.RELIABLE if stream.reliability == "reliable" else ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL if stream.durability == "transient_local" else DurabilityPolicy.VOLATILE))
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            self.thread = threading.Thread(target=self._spin, name="horus-teach-ros", daemon=False)
            self.worker = threading.Thread(target=self._worker, name="horus-teach-control", daemon=False)
            self.thread.start()
            self.worker.start()
            return self
        except Exception:
            self.__exit__(None, None, None)
            raise

    def _publish(self, state):
        from std_msgs.msg import String
        if not self.context.ok(): return
        self.publisher.publish(String(data=json.dumps(state, allow_nan=False)))

    def _spin(self):
        from rclpy.executors import ExternalShutdownException
        from rclpy._rclpy_pybind11 import RCLError
        try:
            self.executor.spin()
        except ExternalShutdownException:
            pass
        except RCLError:
            if self.context.ok():
                logger.exception("Teach ROS worker failed")
        except Exception as exc:
            logger.exception("Teach observation worker failed; stopping capture/control")
            with self.session.lock:
                self.session.error = f"Observation worker failed: {type(exc).__name__}: {exc}"
                self.session.quality = "unacceptable"
        finally:
            self.stop.set()
            with self.session.lock:
                self.session._hold("ROS worker stopped")

    def _request(self, message):
        try:
            if len(message.data) > 65536:
                raise ValueError("Teach request exceeds 64 KiB")
            self._publish(self.session.request(json.loads(message.data)))
        except (ValueError, TypeError, KeyError) as exc:
            logger.warning("Rejected invalid Teach request: %s", exc)

    def _input(self, message):
        try:
            if len(message.data) > 65536:
                raise ValueError("Teach input exceeds 64 KiB")
            self.session.input(json.loads(message.data))
        except (ValueError, TypeError, KeyError) as exc:
            logger.warning("Rejected invalid Teach input: %s", exc)

    def _lease(self, message):
        try:
            root = json.loads(message.data)
            lease = next((v for v in root.get("leases", []) if v.get("robot_name") == self.robot.name), {})
            self.session.update_lease(lease.get("holder_app_id", ""), lease.get("authority_generation", ""),
                                      float(lease.get("remaining_ms", 0)) / 1000)
        except (ValueError, TypeError, KeyError):
            self.session.update_lease("", "", 0)

    def _worker(self):
        next_publish = 0.0
        try:
            while not self.stop.wait(0.01):
                self.session.tick()
                now = time.monotonic()
                if now >= next_publish:
                    self._publish(self.session.state())
                    next_publish = now + 0.05
        except Exception:
            if self.context.ok(): logger.exception("Teach control worker failed; requesting hold")
            with self.session.lock:
                self.session._hold("worker failure")

    def __exit__(self, *exc):
        self.stop.set()
        if self.worker is not None and self.worker.is_alive():
            self.worker.join()
        self.session.close()
        if self.executor is not None:
            self.executor.shutdown()
        if self.thread is not None and self.thread.is_alive():
            self.thread.join()
        if self.node is not None:
            self.node.destroy_node()
        if hasattr(self, "context"):
            self.context.try_shutdown()
        if self.instance_lock is not None:
            self.instance_lock.close()
            self.instance_lock = None
