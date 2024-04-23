class Robot:
    def __init__(self, name, robot_type, actions, sensors):
        self.name = name
        self.robot_type = robot_type  # e.g., 'MOBILE', 'ARM'
        self.actions = actions  # List of supported actions (strings)
        self.sensors = sensors  # Enum of supported sensor types

        # Additional robot-specific attributes (optional)
        self.preferred_teleop = None  # Default teleop method (e.g., 'CAMERA')

    def get_name(self):
        return self.name

    def get_type(self):
        return self.robot_type

    def get_supported_actions(self):
        return self.actions

    def get_available_sensors(self):
        return self.sensors

    def set_preferred_teleop(self, teleop_method):
        # Validate teleop_method against available options
        self.preferred_teleop = teleop_method

    def get_preferred_teleop(self):
        return self.preferred_teleop

    # Methods for sending commands, retrieving sensor data (implementation details)
    # ...
class CameraTeleop:
    def __init__(self, robot, vr_display):
        self.robot = robot
        self.vr_display = vr_display  # Interface to the VR headset functionalities

    def start(self):
        # Subscribe to relevant camera topics (implementation detail)
        # ...
        while True:
            vr_image = self.vr_display.get_camera_image(180)  # Assuming 180-degree view
            self.vr_display.display_image(vr_image)

            # Handle user input from VR controller (implementation detail)
            # ...
            controller_input = self.vr_display.get_controller_input()

            # Translate controller input to cmd_vel messages (implementation detail)
            # ...
            cmd_vel = process_controller_input(controller_input)
            self.robot.send_command(cmd_vel)  # Replace with actual command sending method

# Example usage within a main
