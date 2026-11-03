"""Opt-out is explicit, per robot, and retained in the actual wire serializer."""
from horus.robot import Robot, RobotType
from horus.bridge.robot_registry import RobotRegistryClient


def payload(robot):
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    client.ros_initialized, client.node = False, None
    client._robot_description_resolver = None
    client._robot_description_by_robot = {}
    client._robot_description_by_id = {}
    return client._build_robot_config_dict(robot, robot.create_dataviz())['control']['tasks']['go_to_point']


def test_placement_default_is_bounded_for_legacy_registrations():
    robot = Robot(name='legacy', robot_type=RobotType.DRONE)
    assert payload(robot)['allow_unbounded_placement'] is False


def test_navigation_helper_opt_out_reaches_real_wire_payload():
    robot = Robot(name='simulated_drone', robot_type=RobotType.DRONE)
    robot.configure_navigation_tasks(allow_unbounded_placement=True)
    assert payload(robot)['allow_unbounded_placement'] is True


def test_single_task_helper_can_restore_bounded_authoring():
    robot = Robot(name='simulated_drone', robot_type=RobotType.DRONE)
    robot.configure_go_to_point_task(allow_unbounded_placement=True)
    assert payload(robot)['allow_unbounded_placement'] is True
    robot.configure_go_to_point_task(allow_unbounded_placement=False)
    assert payload(robot)['allow_unbounded_placement'] is False
