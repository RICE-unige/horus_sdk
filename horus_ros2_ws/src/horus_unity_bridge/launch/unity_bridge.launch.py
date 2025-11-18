from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('horus_unity_bridge')
    config_file = os.path.join(pkg_dir, 'config', 'bridge_config.yaml')
    
    # Unity bridge node
    unity_bridge_node = Node(
        package='horus_unity_bridge',
        executable='horus_unity_bridge_node',
        name='horus_unity_bridge',
        output='screen',
        parameters=[config_file] if os.path.exists(config_file) else [],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        unity_bridge_node
    ])
