# Commands sent from HORUS MR app to robots
string robot_id
builtin_interfaces/Time timestamp
string command_type       # teleop, goto, emergency_stop, start_mission, etc.
geometry_msgs/Twist velocity      # For teleop commands
geometry_msgs/Pose target_pose    # For goto commands
string mission_data       # JSON data for missions
string[] parameters       # Additional command parameters
uint32 command_id         # For tracking command completion
