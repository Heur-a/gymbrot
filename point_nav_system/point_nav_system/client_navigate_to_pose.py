"""
ROS2 Navigation Node for handling goal requests and executing navigation actions.

This node subscribes to location goals and interfaces with Nav2's NavigateToPose action server
to execute autonomous navigation commands.

License: GNU 3.0
Copyright (c) 2025 Gymbrot Team
"""

import rclpy
import math

from action_msgs.msg import GoalStatus
from interfaces_gymbrot.msg import LocationGoal
from interfaces_gymbrot.srv import IrMaquina
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile, ReliabilityPolicy

feedback_period = 0.75


class NavigationNode(Node):
    """ROS2 Node that handles navigation goals and executes them using Nav2's NavigateToPose action.

        Attributes:
            _action_client (ActionClient): Client for the NavigateToPose action server
            subscription (Subscription): Subscriber for LocationGoal messages
            current_goal (Point): Currently active navigation goal coordinates
            last_sent_goal (Point): Last successfully sent goal coordinates
            is_active (bool): Flag indicating if a navigation action is currently in progress
            epsilon (float): Tolerance threshold for goal position comparisons
        """
    def __init__(self):
        """Initialize the navigation node and its components."""
        super().__init__('navigation_node')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.service = self.create_service(IrMaquina,'/ir_maquina', self.service_irq_callback)

        self.route_points = []
        #TODO: CAMBIAR DE PONER LOS PUNTOS A PELO A QUE LOS RECOJA DE LA BBDD
        self.patrol_points = {
            "PESAS1": Point(x=-4.0, y=-3.5, z=0.0),
            "PESAS2": Point(x=-4.0, y=-0.5, z=0.0),
            "PESAS3": Point(x=-4.0, y=2.5, z=0.0),
            "PESAS4": Point(x=1.5, y=4.0, z=0.0),
            "PESAS5": Point(x=5.0, y=4.0, z=0.0),
        }

        # GUARDAMOS LA CLAVE DEL PUNTO QUE SIGUE EN LA RUTA DE PATRULLA
        self.current_patrol_goal_key = list(self.patrol_points.keys())[0]

        self.current_goal = self.patrol_points[self.current_patrol_goal_key]
        self.last_sent_goal = None  # Almacena el último objetivo enviado
        self.is_active = False
        self.epsilon = 0.01  # Margen para comparación de coordenadas

        # Empezar bucle
        self.send_goal(self.current_goal)

    def next_point_controller(self):
        """
        Actualiza y/o devuelve el robot a su patrulla normal
        :return:
        """
        # Si hemos llegado al punto de patrulla,
        # Hay que ir al siguiente
        if (self._is_same_goal(self.current_goal, self.patrol_points[self.current_patrol_goal_key])):
            # Pasamos la clave del punto a index
            # Para poder hacer aritmetica
            # Y asignarle el siguiente punto
            current_patrol_index = list(self.patrol_points.keys()).index(self.current_patrol_goal_key)

            # Avanzamos al siguiente punto, en bucle
            next_patrol_index = (current_patrol_index + 1) % len(self.patrol_points)

            # Convertimos el index otra vez en key
            # Para poder buscar el punto en el diccionario
            self.current_patrol_goal_key = list(self.patrol_points.keys())[next_patrol_index]

            #Avisar de cambio a siguiente punto
            self.get_logger().info("Ruta a siguiente punto de patrulla actualizado")

        # Si hemos llegado se envia con el goal de patrulla actualizado
        # Sino, con el ultimo establecido
        self.get_logger().info("Siguiente punto, robot yendo a " + self.current_patrol_goal_key)
        self.send_goal(self.patrol_points[self.current_patrol_goal_key])
        return

    def service_irq_callback(self, req, res):
        """Non-blocking service handler with async cancellation

        Args:
            req (IrMaquina.Request): Service request with new coordinates
            res (IrMaquina.Response): Immediate service response

        Returns:
            IrMaquina.Response: Status acknowledgement
        """
        new_goal = Point(x=req.x, y=req.y, z=0.0)

        if self._is_same_goal(new_goal, self.current_goal):
            res.res = "Goal already active"
            res.codigo = 0
            return res

        # Store request context for async handling
        self.pending_req = (req, res)
        self.current_goal = new_goal

        if self.is_active:
            self.get_logger().info("Async cancellation initiated")
            self.goal_handle.cancel_goal_async()

        else:
            self.send_goal(new_goal)
            res.res = "New goal accepted"
            res.codigo = 1

        # Always return immediately with interim status
        res.res = "Request processing started"
        res.codigo = 2
        return res



    def send_goal(self, point):
        """Send a navigation goal to the action server.

                Args:
                    point (Point): Target position in map coordinates

                Raises:
                    RuntimeError: If action server is unavailable
                """
        # if self._is_same_goal(point, self.last_sent_goal):
        #     self.get_logger().info('Objetivo idéntico al anterior, ignorando...')
        #     return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position = point
        goal_msg.pose.pose.orientation.w = 1.0

        self._action_client.wait_for_server()
        self.get_logger().info(f'Enviando nuevo objetivo: X: {point.x:.2f}, Y: {point.y:.2f}')
        self.is_active = True
        self.last_sent_goal = point  # Actualizar último objetivo enviado
        self.current_goal = point

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def _is_same_goal(self, goal1, goal2) -> bool:
        """Compare two goals with position tolerance.

                Args:
                    goal1 (Point): First goal to compare
                    goal2 (Point): Second goal to compare

                Returns:
                    bool: True if goals are within epsilon tolerance, False otherwise
                """
        """Compara dos objetivos con margen de error epsilon"""
        if goal1 is None or goal2 is None:
            return False
        return (math.isclose(goal1.x, goal2.x, abs_tol=self.epsilon) and
                math.isclose(goal1.y, goal2.y, abs_tol=self.epsilon))

    def goal_response_callback(self, future):
        """Handle response from action server after goal submission."""
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Objetivo rechazado')
            self.is_active = False
            return

        self.get_logger().info('Objetivo aceptado!')
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Process final navigation result with proper error handling"""

        result = future.result().result
        status = future.result().status

        self.is_active = False

            # Valid final statuses: SUCCEEDED(4), CANCELED(2), ABORTED(5)
        if status == GoalStatus.STATUS_SUCCEEDED:
            # Si llegamos al final del punto, volvemos al bucle
                self.get_logger().info(f"Navigation succeeded to {self.current_pos.x:.2f},{self.current_pos.y:.2f}")
                self.next_point_controller()
        elif status == GoalStatus.STATUS_CANCELED or status == GoalStatus.STATUS_ABORTED:
            # Si hemos cancelado la accion o se ha abortado
            # Vamos a la current goal puesta
            # Esto se debe a que cancelamos la goal cuando llega un punto del servicio
                self.send_goal(self.current_goal)
        else:
                self.get_logger().warning(f"Navigation interrupted: {status}")
        return

    def feedback_callback(self, feedback_msg):
        """Process navigation feedback with rate limiting (0.5Hz).

        Args:
            feedback_msg (NavigateToPose.Feedback): Navigation progress feedback
        """
        # Rate limiting implementation
        current_time = self.get_clock().now()

        # Initialize last feedback time on first call
        if not hasattr(self, 'last_feedback_time'):
            self.last_feedback_time = current_time

        # Calculate time since last feedback display
        elapsed_time = (current_time - self.last_feedback_time).nanoseconds * 1e-9

        # Only process if 0.5 seconds have passed since last update
        if elapsed_time < feedback_period:
            return

        # Store current time for next rate limit check
        self.last_feedback_time = current_time

        # Extract and display feedback data
        feedback = feedback_msg.feedback
        self.current_pos = feedback.current_pose.pose.position
        self.get_logger().info(
            f'Progress: X: {self.current_pos.x:.2f}, Y: {self.current_pos.y:.2f} | '
            f'Remaining: {feedback.distance_remaining:.2f}m | '
            f'Duration: {feedback.navigation_time.sec}s'
        )


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