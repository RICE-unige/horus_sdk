# SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    tcp_port_arg = DeclareLaunchArgument(
        'tcp_port', default_value='8080', description='TCP port for SDK communication'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error)',
    )

    verbose_arg = DeclareLaunchArgument(
        'verbose', default_value='false', description='Enable verbose logging'
    )

    # Main backend node
    backend_node = Node(
        package='horus_backend',
        executable='horus_backend_node',
        name='horus_backend',
        output='screen',
        parameters=[
            {
                'tcp_port': LaunchConfiguration('tcp_port'),
                'log_level': LaunchConfiguration('log_level'),
            }
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        condition=IfCondition(
            PythonExpression(['"', LaunchConfiguration('verbose'), '" == "true"'])
        ),
    )

    # Alternative node for non-verbose mode
    backend_node_quiet = Node(
        package='horus_backend',
        executable='horus_backend_node',
        name='horus_backend',
        output='screen',
        parameters=[
            {
                'tcp_port': LaunchConfiguration('tcp_port'),
                'log_level': LaunchConfiguration('log_level'),
            }
        ],
        condition=IfCondition(
            PythonExpression(['"', LaunchConfiguration('verbose'), '" == "false"'])
        ),
    )

    return LaunchDescription(
        [
            tcp_port_arg,
            log_level_arg,
            verbose_arg,
            backend_node,
            backend_node_quiet,
        ]
    )
