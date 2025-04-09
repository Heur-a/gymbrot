"""
ROS2 Navigation Node for handling goal requests and executing navigation actions.

This node subscribes to location goals and interfaces with Nav2's NavigateToPose action server
to execute autonomous navigation commands.

License: GNU 3.0
Copyright (c) 2025 Gymbrot Team
"""

import rclpy
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

##TODO:¡¡¡¡¡¡FALTA QUE LE LLEGUE EL SITIO AL QUE TIENE QUE IR POR WEB!!!!!!!
## Deberia de interrumpir la accion del Waypoint y hacer un NavToPose a la maquina :_D


class NavigationNode(Node):
    """ROS2 Node that handles navigation goals and executes them using Nav2's NavigateToPose action.

        Attributes:
            _action_client (ActionClient): Client for the followWaypoints action server
            subscription (Subscription): Subscriber for amcl poses
            """

    def __init__(self):
        """Initialize the navigation node and its components."""
        super().__init__('navigation_node')
        #Iniciar clientes y suscripciones
        self._action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        #Iniciar parametros propios de la clase que vamos a usar
        self.hasGoalRunning = False
        self.currentWP = 0

        #Iniciar los parametros que vamos a recibir
        #Basicamente solo van a ser los Waypoints asi que solo declaramos esto
        self.declare_parameter('waypoints', [])
        waypoints_params = self.get_parameter('waypoints').value
        frame_id = self.get_parameter('')
        if not waypoints_params:
            self.get_logger().error("No se han proporcionado waypoints. Verificar el archivo waypoints.yaml en ./config.")
            return  # Parar si no hay waypoints

        # Pasar waypoints en el .yaml a parametros a enviar
        self.waypoints = self.parsePosesStampedFromYaml(waypoints_params)

        self.send_goal(self.waypoints)

    def parsePosesStampedFromYaml(self, waypoints_params):
        """Parses the data from the yaml file to PosesStamped.
                Args:
                    waypoints_params (Any[]):
                Returns:
                    PoseStamped[]: Array of PoseStamped
        """
        waypoints = []
        for wp in waypoints_params:
            try:
                # El yaml del que parseamos debe ser asi
                # Si hay alguna clave mal, lo notificara y pasara de ella
                #    waypoints:
                #     - frame_id: "map"
                #       position: {x: 1.0, y: 2.0, z: 0.0}
                #       orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
                #     - frame_id: "map"
                #       position: {x: 3.0, y: 4.0, z: 0.0}
                #       orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
                pose = PoseStamped()
                pose.header.frame_id = wp.get("frame_id", "map")
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = wp["position"]["x"]
                pose.pose.position.y = wp["position"]["y"]
                pose.pose.position.z = wp["position"]["z"]
                pose.pose.orientation.x = wp["orientation"]["x"]
                pose.pose.orientation.y = wp["orientation"]["y"]
                pose.pose.orientation.z = wp["orientation"]["z"]
                pose.pose.orientation.w = wp["orientation"]["w"]
                waypoints.append(pose)
            except KeyError as e:
                self.get_logger().error(f"Waypoint mal formatado: falta el campo {e}")
        self.get_logger().info(f"Waypoints cargados: {len(waypoints)}")
        return waypoints

    def pose_callback(self, msg):
        #TODO: Hacer esto en la web (ahora imprime por pantalla solamente) si eso
        """Callback for sending position info.

                Args:
                    msg (PoseWithCovarianceStamped): Received goal coordinates with x and y positions
                """
        if self.hasGoalRunning:
            #TODO: Include distance remaining to next Waypoint
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.get_logger().info(f"Posicion x: {x:.2f}, y: {y:.2f}")
            self.get_logger().info(f"Ultimo Waypoint: {self.waypoints[self.currentWP]}")

    def send_goal(self, PosesStamped:PoseStamped):
        """Send a list of waypoints to the action server for sequential navigation.

        Args:
            poses_stamped (list of PoseStamped):

        Raises:
            RuntimeError: If action server is unavailable or input is empty.
        """

        goal_msg = FollowWaypoints().Goal()
        goal_msg.poses = PosesStamped

        self._action_client.wait_for_server()
        self.get_logger().info(f'Enviando lista Waypoints')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle response from action server after goal submission."""
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Lista de waypoints rechazada')
            return

        self.get_logger().info('Objetivo aceptado!')
        self._get_result_future = self.goal_handle.get_result_async()
        self.hasGoalRunning = True
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Process final result of navigation action."""
        try:
            result = future.result()

            # Mostrar feedback final si está disponible
            if result.missed_waypoints:
                str = "No se ha llegado a los waypoints: "
                for wp in result.missed_waypoints:
                    str = str + f"{wp} ,"
                self.get_logger().info(str)
        except Exception as e:
            self.get_logger().info(f"Error al obtener el result: {e}")
        finally:
            # Falle el result o no, volvemos a enviar los waypoints para que no se quede atascado
            # Es decir, que si falla, sigue funcionando
            self.send_goal(self.waypoints)

    def feedback_callback(self, feedback_msg):
        """Process ongoing navigation feedback.

                Args:
                    feedback_msg (FollowWaypoints): Navigation progress feedback
                """
        feedback = feedback_msg.feedback
        self.currentWP = feedback.current_waypoint


def main(args=None):
    """Main entry point for the navigation node."""
    rclpy.init(args=args)
    navigator = NavigationNode()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Navigation node shutting down...')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()