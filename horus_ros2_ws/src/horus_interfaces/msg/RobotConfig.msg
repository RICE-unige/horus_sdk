# Complete robot configuration for registration
string name
string robot_type         # wheeled, legged, aerial
SensorConfig[] sensors
VisualizationConfig[] visualizations
string[] control_topics   # Topics for sending commands to robot
string[] status_topics    # Topics for monitoring robot status
string[] metadata_keys
string[] metadata_values
