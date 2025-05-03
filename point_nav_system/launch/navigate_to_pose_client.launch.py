from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='point_nav_system',
            executable='client_navigate_to_pose',
            output='screen'
        )
    ])