import rospy

class RosInterface:
    def __init__(self, config):
        self.config = config

    def connect(self):
        rospy.init_node(self.config['ros_node_name'])
        Logger.info("Connected to ROS.")

    def send_command(self, robot_id, command):
        topic = self.config['robots'][robot_id]['command_topic']
        pub = rospy.Publisher(topic, String, queue_size=10)
        pub.publish(command)

    def get_sensor_data(self, robot_id):
        topic = self.config['robots'][robot_id]['sensor_topic']
        data = rospy.wait_for_message(topic, String)
        return data
