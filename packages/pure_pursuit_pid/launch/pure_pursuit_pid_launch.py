from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pure_pursuit_pid',
            executable='waypoint_publisher',  # your waypoint node executable name
            name='waypoint_publisher',
            output='screen',
            # parameters or remappings if needed
        ),
        Node(
            package='pure_pursuit_pid',
            executable='pure_pursuit_controller',
            name='pure_pursuit_controller',
            output='screen',
        ),
        Node(
            package='pure_pursuit_pid',
            executable='pid_controller',
            name='pid_controller',
            output='screen',
        ),
        Node(
            package='pure_pursuit_pid',
            executable='cmd_vel_combiner',
            name='cmd_vel_combiner',
            output='screen',
        ),
        Node(
            package='pure_pursuit_pid',
            executable='path_recorder',
            name='path_recorder',
            output='screen',
        )
    ])
