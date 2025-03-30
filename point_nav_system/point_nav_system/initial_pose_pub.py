#!/usr/bin/env python3
"""
ROS2 Node for publishing initial robot pose estimation.

This node publishes a single initial pose estimate for robot localization systems
like AMCL (Adaptive Monte Carlo Localization).

License: GNU 3.0
Copyright (c) 2023 Your Name/Organization
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class Publisher(Node):
    """ROS2 Node that publishes a single initial pose estimate for robot localization.

    Attributes:
        publisher_ (Publisher): ROS2 publisher for initial pose estimates
    """

    def __init__(self) -> None:
        """Initialize the initial pose publisher node and configure message."""
        super().__init__('initial_pose_pub_node')

        # Create publisher with QoS profile depth 1 (latched-like behavior)
        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            'initialpose',
            1  # QoS history depth
        )

        # Configure initial pose message
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'  # Coordinate frame for the pose
        msg.pose.pose.position.x = 2.0  # X position in meters
        msg.pose.pose.position.y = -0.5 # Y position in meters
        msg.pose.pose.orientation.w = 1.0  # Neutral orientation (no rotation)

        # Publish and log information
        self.get_logger().info(
            'Publishing initial pose (single shot):\n'
            'X: 0.2 m\n'
            'Y: 0.0 m\n'
            'Orientation (w): 1.0'
        )
        self.publisher_.publish(msg)


def main(args: list = None) -> None:
    """Main entry point for the initial pose publisher node.

    Args:
        args (list, optional): Command-line arguments. Defaults to None.
    """
    rclpy.init(args=args)

    try:
        publisher = Publisher()
        # Node exits after single publication
        rclpy.shutdown()
    except Exception as e:
        rclpy.logging.get_logger('initial_pose_pub').error(
            f'Node initialization failed: {str(e)}'
        )
        raise


if __name__ == '__main__':
    main()