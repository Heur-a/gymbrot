from symbol import parameters

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_real = LaunchConfiguration('robot_real', default='0')
    return LaunchDescription([
        Node(
            package='gymbrot_point_nav_system',
            executable='client_navigate_to_pose',
            output='screen',
            parameters=[
                {'robot_real': True}
            ]
        )
    ])
