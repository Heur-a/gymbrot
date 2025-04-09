import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml


def generate_launch_description():
    # Load waypoints from YAML
    package_dir = get_package_share_directory('point_nav_system')
    yaml_path = os.path.join(package_dir, 'config', 'waypoints.yaml')

    # try:
    #     with open(yaml_path, 'r') as file:
    #         config = yaml.safe_load(file)
    #         waypoints = config.get("waypoints", [])
    #
    #         # Convert numbers to strings for ROS2 parameter system
    #         str_waypoints = []
    #         for wp in waypoints:
    #             str_waypoints.append([str(val) for val in wp])
    #
    #         print(f"✅ Successfully loaded {len(str_waypoints)} waypoints")
    # except Exception as e:
    #     print(f"❌ Error loading YAML: {str(e)}")
    #     str_waypoints = []

    return LaunchDescription([
        Node(
            package='point_nav_system',
            executable='waypointsWebService',
            name='navigation_node',
            output='screen',
        ),
    ])