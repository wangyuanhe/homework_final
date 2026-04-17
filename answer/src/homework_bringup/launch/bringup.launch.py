import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'serial',
        default_value='/dev/pts/2',
        description='Serial port name for worker node'
    )
    port_name = LaunchConfiguration('serial')

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode in worker node'
    )
    debug_name = LaunchConfiguration('debug')

    return LaunchDescription([
        port_arg,
        debug_arg,
        Node(
            package='worker',
            executable='worker_node',
            name='worker',
            output='screen',
            parameters=[
                {'serial': port_name},
                {'debug': debug_name},
                {'health_impact_factor':[1e7,1.0,0.7,0.2]}
            ]
        ),
        Node(
            package='debuger',
            executable='debuger_node',
            name='debuger',
            output='screen',
        ),
    ])