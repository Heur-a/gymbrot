"""
ROS2 Navigation Node for handling goal requests and executing navigation actions.

This node subscribes to location goals and interfaces with Nav2's NavigateToPose action server
to execute autonomous navigation commands.

License: GNU 3.0
Copyright (c) 2025 Gymbrot Team
"""

import rclpy
import math
from interfaces_gymbrot.msg import LocationGoal
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile, ReliabilityPolicy


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
        self.subscription = self.create_subscription(
            LocationGoal,
            '/locationGoal',
            self.location_goal_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.current_goal = None
        self.last_sent_goal = None  # Almacena el último objetivo enviado
        self.is_active = False
        self.epsilon = 0.01  # Margen para comparación de coordenadas

    def location_goal_callback(self, msg):
        """Callback for processing incoming navigation goals.

                Args:
                    msg (LocationGoal): Received goal coordinates with x and y positions
                """
        new_goal = Point()
        new_goal.x = msg.x
        new_goal.y = msg.y
        new_goal.z = 0.0

        # Solo actualizar si es un objetivo nuevo
        if not self._is_same_goal(new_goal, self.current_goal):
            self.current_goal = new_goal
            self.get_logger().info(f'Nuevo objetivo registrado: X: {msg.x:.2f}, Y: {msg.y:.2f}')

        # Enviar solo si no está activo y es diferente al último enviado
        if not self.is_active and not self._is_same_goal(new_goal, self.last_sent_goal):
            self.send_goal(new_goal)

    def send_goal(self, point):
        """Send a navigation goal to the action server.

                Args:
                    point (Point): Target position in map coordinates

                Raises:
                    RuntimeError: If action server is unavailable
                """
        if self._is_same_goal(point, self.last_sent_goal):
            self.get_logger().info('Objetivo idéntico al anterior, ignorando...')
            return

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
        """Process final result of navigation action."""
        try:
            result = future.result()
            status = result.status

            # Mostrar feedback final si está disponible
            if result.feedback:
                feedback = result.feedback
                final_pos = feedback.current_pose.pose.position
                self.get_logger().info(
                    f'Posición final: X: {final_pos.x:.2f}, Y: {final_pos.y:.2f}\n'
                    f'Distancia recorrida: {feedback.distance_remaining:.2f}m'
                )

        except Exception as e:
            self.get_logger().error(f'Error en resultado: {str(e)}')
        finally:
            self.is_active = False
            self.get_logger().info(f'Estado final de navegación: {status}')
            # Reenviar solo si hay nuevo objetivo diferente
            if self.current_goal and not self._is_same_goal(self.current_goal, self.last_sent_goal):
                self.send_goal(self.current_goal)

    def feedback_callback(self, feedback_msg):
        """Process ongoing navigation feedback.

                Args:
                    feedback_msg (NavigateToPose.Feedback): Navigation progress feedback
                """
        feedback = feedback_msg.feedback
        current_pos = feedback.current_pose.pose.position
        self.get_logger().info(
            f'Progreso: X: {current_pos.x:.2f}, Y: {current_pos.y:.2f} | '
            f'Distancia restante: {feedback.distance_remaining:.2f}m | '
            f'Tiempo navegación: {feedback.navigation_time.sec}s'
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