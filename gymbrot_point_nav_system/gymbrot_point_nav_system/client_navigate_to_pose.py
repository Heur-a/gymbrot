"""
ROS2 Navigation Node for handling goal requests and executing navigation actions.

This node subscribes to location goals and interfaces with Nav2's NavigateToPose action server
to execute autonomous navigation commands. Implements patrol route management and service-based
goal interruption capabilities.

License: GNU 3.0
Copyright (c) 2025 Gymbrot Team
"""

import rclpy
import math

from action_msgs.msg import GoalStatus
from interfaces_gymbrot.msg import LocationGoal
from gymbrot_interfaces.srv import IrMaquina
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile, ReliabilityPolicy

feedback_period = 0.75


class NavigationNode(Node):
    """ROS2 Node that handles navigation goals and executes them using Nav2's NavigateToPose action.

    Implements a state machine for managing patrol routes and handling external goal requests through
    a service interface. Maintains persistent connection to Nav2 action server and implements robust
    error handling for navigation tasks.

    Attributes:
        _action_client (ActionClient): Client for the NavigateToPose action server
        service (Service): Service server for handling external goal requests
        sim_time (bool): Flag indicating simulation vs real robot operation
        patrol_points (dict): Dictionary of predefined patrol points with string keys
        current_patrol_goal_key (str): Current target key in patrol_points dictionary
        current_goal (Point): Currently active navigation goal coordinates
        last_sent_goal (Point): Last successfully sent goal coordinates
        is_active (bool): Flag indicating active navigation action
        epsilon (float): Tolerance threshold for position comparisons
        route_points (list): Buffer for dynamic route planning (reserved for future use)
    """

    def __init__(self):
        """Initialize navigation node with configuration parameters and service endpoints.

        Sets up action client, service interface, and patrol point configuration based on
        simulation/reality mode. Starts initial navigation sequence.
        """
        super().__init__('navigation_node')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.service = self.create_service(IrMaquina, '/ir_maquina', self.service_irq_callback)

        self.declare_parameter('robot_real', False)
        self.sim_time = self.get_parameter('robot_real').get_parameter_value().bool_value

        self.route_points = []
        self._configure_patrol_points()
        self.current_patrol_goal_key = list(self.patrol_points.keys())[0]
        self.current_goal = self.patrol_points[self.current_patrol_goal_key]
        self.last_sent_goal = None
        self.is_active = False
        self.epsilon = 0.01

        self.send_goal(self.current_goal)

    def _configure_patrol_points(self):
        """Configure patrol points dictionary based on operation mode.

        Populates different coordinate sets for simulation and real-world environments.
        """
        if self.sim_time:
            self.patrol_points = {
                "ESQUINA2": Point(x=0.5, y=0.4, z=0.0),
                "ESQUINA3": Point(x=0.18, y=0.0, z=0.0),
                "ESQUINA1": Point(x=0.4, y=-0.4, z=0.0)
            }
        else:
            self.patrol_points = {
                "PESAS1": Point(x=-4.0, y=-3.5, z=0.0),
                "PESAS2": Point(x=-4.0, y=-0.5, z=0.0),
                "PESAS3": Point(x=-4.0, y=2.5, z=0.0),
                "PESAS4": Point(x=1.5, y=4.0, z=0.0),
                "PESAS5": Point(x=5.0, y=4.0, z=0.0),
            }

    def next_point_controller(self):
        """Manage patrol route progression and goal updates.

        Advances to next patrol point in sequence when current target is reached.
        Implements circular buffer behavior for continuous patrol routes.
        """
        if self._is_same_goal(self.current_goal, self.patrol_points[self.current_patrol_goal_key]):
            current_idx = list(self.patrol_points.keys()).index(self.current_patrol_goal_key)
            next_idx = (current_idx + 1) % len(self.patrol_points)
            self.current_patrol_goal_key = list(self.patrol_points.keys())[next_idx]
            self.get_logger().info("Ruta a siguiente punto de patrulla actualizado")

        self.get_logger().info(f"Siguiente punto, robot yendo a {self.current_patrol_goal_key}")
        self.send_goal(self.patrol_points[self.current_patrol_goal_key])

    def service_irq_callback(self, req, res):
        """Handle asynchronous goal requests via service interface.

        Args:
            req (IrMaquina.Request): Service request containing:
                - x: float32 target x-coordinate
                - y: float32 target y-coordinate
            res (IrMaquina.Response): Service response containing:
                - res: string status message
                - codigo: int8 status code

        Returns:
            IrMaquina.Response: Immediate response indicating request acceptance status
        """
        new_goal = Point(x=req.x, y=req.y, z=0.0)

        if self._is_same_goal(new_goal, self.current_goal):
            res.res = "Goal already active"
            res.codigo = 0
            return res

        self.pending_req = (req, res)
        self.current_goal = new_goal

        if self.is_active:
            self.get_logger().info("Async cancellation initiated")
            self.goal_handle.cancel_goal_async()
        else:
            self.send_goal(new_goal)
            res.res = "New goal accepted"
            res.codigo = 1

        res.res = "Request processing started"
        res.codigo = 2
        return res

    def send_goal(self, point):
        """Send navigation goal to action server with duplicate detection.

        Args:
            point (Point): Target coordinates in map frame

        Raises:
            RuntimeError: If action server becomes unavailable during request
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
        self.last_sent_goal = point
        self.current_goal = point

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def _is_same_goal(self, goal1, goal2) -> bool:
        """Compare two positions with tolerance threshold.

        Args:
            goal1 (Point): First position to compare
            goal2 (Point): Second position to compare

        Returns:
            bool: True if positions are within epsilon tolerance in both axes
        """
        if None in (goal1, goal2):
            return False
        return (math.isclose(goal1.x, goal2.x, abs_tol=self.epsilon) and
                math.isclose(goal1.y, goal2.y, abs_tol=self.epsilon))

    def goal_response_callback(self, future):
        """Handle action server response to goal submission.

        Args:
            future (concurrent.futures.Future): Async result container for goal acceptance status
        """
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Objetivo rechazado')
            self.is_active = False
            return

        self.get_logger().info('Objetivo aceptado!')
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Process final navigation result and manage state transitions.

        Args:
            future (concurrent.futures.Future): Async result container containing navigation outcome
        """
        result = future.result().result
        status = future.result().status
        self.is_active = False

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Navigation succeeded to {self.current_pos.x:.2f},{self.current_pos.y:.2f}")
            self.next_point_controller()
        elif status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
            self.send_goal(self.current_goal)
        else:
            self.get_logger().warning(f"Navigation interrupted: {status}")

    def feedback_callback(self, feedback_msg):
        """Process and display navigation feedback with rate limiting.

        Args:
            feedback_msg (NavigateToPose.Feedback): Navigation progress update containing:
                - current_pose: geometry_msgs/PoseStamped current robot position
                - distance_remaining: float32 remaining distance to goal
                - navigation_time: builtin_interfaces/Duration elapsed navigation time
        """
        current_time = self.get_clock().now()

        if not hasattr(self, 'last_feedback_time'):
            self.last_feedback_time = current_time

        elapsed_time = (current_time - self.last_feedback_time).nanoseconds * 1e-9
        if elapsed_time < feedback_period:
            return

        self.last_feedback_time = current_time
        feedback = feedback_msg.feedback
        self.current_pos = feedback.current_pose.pose.position
        self.get_logger().info(
            f'Progress: X: {self.current_pos.x:.2f}, Y: {self.current_pos.y:.2f} | '
            f'Remaining: {feedback.distance_remaining:.2f}m | '
            f'Duration: {feedback.navigation_time.sec}s'
        )


def main(args=None):
    """Main execution entry point for navigation node.

    Handles node lifecycle management and graceful shutdown procedures.

    Args:
        args (list, optional): Command-line arguments. Defaults to None.
    """
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