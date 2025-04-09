import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml


def generate_launch_description():
    package_dir = get_package_share_directory('point_nav_system')
    yaml_path = os.path.join(package_dir, 'config', 'waypoints.yaml')
    waypoints = []  # Valor por defecto si falla la carga

    try:
        with open(yaml_path, 'r') as file:
            config = yaml.safe_load(file)
            waypoints = config.get("waypoints", [])
            if not waypoints:
                raise ValueError("El fitxer YAML no contiene 'waypoints' o està vacio.")
    except FileNotFoundError:
        print(f"Error: No se ha encontrado el fichero YAML a {yaml_path}")
    except yaml.YAMLError as e:
        print(f"Error leyendo el YAML: {e}")
    except Exception as e:
        print(f"Error inesperado: {e}")

    return LaunchDescription([
        Node(
            package="point_nav_system",
            executable="waypointWebService",
            name="navigation_node",
            parameters=[{"waypoints": waypoints}],
            output="screen",
        ),
    ])