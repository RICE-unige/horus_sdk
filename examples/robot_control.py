from horus_sdk import Robot, Sensors

# Define a ROSBOT robot
my_robot = Robot(
    name="ROSBOT",
    robot_type="MOBILE",
    actions=["GO_TO_POINT"],
    sensors=Sensors.RGBD | Sensors.RGB | Sensors.DEPTH | Sensors.LIDAR
)

# Set preferred teleop to camera view
my_robot.set_preferred_teleop("CAMERA")

# Get robot information
print(my_robot.get_name())        # Output: ROSBOT
print(my_robot.get_supported_actions())  # Output: ['GO_TO_POINT']
print(my_robot.get_available_sensors())  # Output: (RGBD | RGB | DEPTH | LIDAR)
