class HorusSDK:
    def __init__(self, config_path):
        self.config = ConfigLoader.load(config_path)
        self.ros_interface = RosInterface(self.config)
        self.mr_interface = MixedRealityInterface(self.config)

    def initialize_workspace(self):
        self.ros_interface.connect()
        self.mr_interface.setup_workspace()
        Logger.info("Workspace initialized successfully.")

    def send_command(self, robot_id, command):
        self.ros_interface.send_command(robot_id, command)
        Logger.info(f"Command sent to robot {robot_id}: {command}")

    def visualize_sensor_data(self, robot_id):
        data = self.ros_interface.get_sensor_data(robot_id)
        self.mr_interface.visualize_data(robot_id, data)
        Logger.info(f"Visualizing sensor data for robot {robot_id}.")
