# Real-time robot status updates
string robot_id
builtin_interfaces/Time timestamp
geometry_msgs/Pose pose
geometry_msgs/Twist velocity
float32 battery_level     # 0.0 - 1.0
string health_status      # healthy, warning, error, unknown
bool[] sensor_active      # Status of each registered sensor
string[] topic_status     # Status of each monitored topic
float32[] topic_rates     # Hz rates of each monitored topic
string status_message     # Human readable status
