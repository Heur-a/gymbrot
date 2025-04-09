"""
ROS2 Navigation Node for handling goal requests and executing navigation actions.

This node subscribes to location goals and interfaces with Nav2's FollowWaypoints action server
to execute autonomous navigation commands.

License: GNU 3.0
Copyright (c) 2025 Gymbrot Team
"""

import rclpy
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class NavigationNode(Node):
    """ROS2 Node that handles navigation goals and executes them using Nav2's FollowWaypoints action."""

    def __init__(self):
        """Initialize the navigation node and its components."""
        super().__init__('navigation_node')

        # Initialize action client and subscribers
        self._action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        # Node state variables
        self.hasGoalRunning = False
        self.currentWP = 0
        self.waypoints = []

        # Declare parameters with proper types
        # self.declare_parameter(
        #     'waypoints',
        #     [],
        #     ParameterDescriptor(
        #         type=ParameterType.PARAMETER_STRING_ARRAY,
        #         description='List of waypoints as string arrays [x,y,z,qx,qy,qz,qw]'
        #     )
        # )
        # self.declare_parameter('frame_id', 'map')
        #
        # # Get parameters and convert to poses
        # waypoints_params = self.get_parameter('waypoints').value
        # frame_id = self.get_parameter('frame_id').value
        #
        # if not waypoints_params:
        #     self.get_logger().error("No waypoints provided. Check waypoints.yaml config file.")
        #     return
        #
        # self.waypoints = self._convert_to_poses(waypoints_params, frame_id)
        wp1 = PoseStamped()
        wp2 = PoseStamped()
        wp3 = PoseStamped()
        wp4 = PoseStamped()
        wp5 = PoseStamped()

        wp1.pose.position.x = -3.0
        wp1.pose.position.y = -3.8
        wp1.pose.orientation.w = 1.0
        wp1.header.frame_id = 'map'
        wp1.header.stamp = self.get_clock().now().to_msg()

        wp2.pose.position.x = -3.0
        wp2.pose.position.y = -0.7
        wp2.pose.orientation.w = 1.0
        wp2.header.frame_id = 'map'
        wp2.header.stamp = self.get_clock().now().to_msg()

        wp3.pose.position.x = -3.0
        wp3.pose.position.y = 2.0
        wp3.pose.orientation.w = 1.0
        wp3.header.frame_id = 'map'
        wp3.header.stamp = self.get_clock().now().to_msg()

        wp4.pose.position.x = 1.0
        wp4.pose.position.y = 3.0
        wp4.pose.orientation.w = 1.0
        wp4.header.frame_id = 'map'
        wp4.header.stamp = self.get_clock().now().to_msg()

        wp5.pose.position.x = 2.8
        wp5.pose.position.y = 3.0
        wp5.pose.orientation.w = 1.0
        wp5.header.frame_id = 'map'
        wp5.header.stamp = self.get_clock().now().to_msg()

        self.waypoints = [wp1,wp2,wp3,wp4,wp5]

        if self.waypoints:
            self.send_goal(self.waypoints)

    def _convert_to_poses(self, waypoints_params, frame_id):
        """Convert string parameters to PoseStamped messages."""
        poses = []
        for i, wp in enumerate(waypoints_params):
            try:
                # Convert string values to floats
                values = [float(val) for val in wp]

                if len(values) != 7:
                    self.get_logger().warn(f"Waypoint {i} skipped: expected 7 values, got {len(values)}")
                    continue

                pose = PoseStamped()
                pose.header.frame_id = frame_id
                pose.header.stamp = self.get_clock().now().to_msg()

                # Assign position
                pose.pose.position.x = values[0]
                pose.pose.position.y = values[1]
                pose.pose.position.z = values[2]

                # Assign orientation
                pose.pose.orientation.x = values[3]
                pose.pose.orientation.y = values[4]
                pose.pose.orientation.z = values[5]
                pose.pose.orientation.w = values[6]

                poses.append(pose)

            except (ValueError, IndexError) as e:
                self.get_logger().error(f"Error processing waypoint {i}: {str(e)}")

        self.get_logger().info(f"Successfully loaded {len(poses)} waypoints")
        return poses

    def pose_callback(self, msg):
        """Handle current pose updates."""
        if self.hasGoalRunning:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.get_logger().info(f"Current position: x: {x:.2f}, y: {y:.2f}")

            if self.currentWP < len(self.waypoints):
                target = self.waypoints[self.currentWP].pose.position
                self.get_logger().info(f"Current waypoint: x: {target.x:.2f}, y: {target.y:.2f}")

    def send_goal(self, poses_stamped):
        """Send waypoints to navigation server."""
        if not poses_stamped:
            self.get_logger().error("Empty waypoints list, cannot navigate")
            return

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses_stamped

        self._action_client.wait_for_server()
        self.get_logger().info(f'Sending {len(poses_stamped)} waypoints to server')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Waypoints rejected by server')
            return

        self.get_logger().info('Waypoints accepted! Navigating...')
        self.hasGoalRunning = True
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle navigation completion."""
        try:
            result = future.result().result
            if result.missed_waypoints:
                self.get_logger().warn(f"Missed waypoints: {result.missed_waypoints}")
        except Exception as e:
            self.get_logger().error(f"Error getting result: {str(e)}")
        finally:
            # Restart navigation
            self.send_goal(self.waypoints)

    def feedback_callback(self, feedback_msg):
        """Update current waypoint index."""
        self.currentWP = feedback_msg.feedback.current_waypoint


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    navigator = NavigationNode()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Shutting down navigation node...')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()