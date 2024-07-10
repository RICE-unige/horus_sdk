import rospy

class Robot:
    def __init__(self, name: str):
        self.name = name
        rospy.init_node(self.name)