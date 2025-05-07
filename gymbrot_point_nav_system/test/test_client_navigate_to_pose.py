#!/usr/bin/env python3
"""
Unit Tests for gymbrot_point_nav_system NavigationNode

Execute amb:
colcon test --packages-select gymbrot_point_nav_system --event-handlers console_direct+
"""

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from unittest.mock import MagicMock, patch
import rclpy
import pytest
from geometry_msgs.msg import Point
from gymbrot_interfaces.srv import IrMaquina
from rclpy.clock import Clock
from rclpy.time import Time as RclpyTime
from src.gymbrot.gymbrot_point_nav_system.gymbrot_point_nav_system.client_navigate_to_pose import NavigationNode


@pytest.fixture
def node():
    rclpy.init()
    try:
        with patch('rclpy.action.ActionClient'), \
                patch.object(NavigationNode, 'create_service'), \
                patch.object(NavigationNode, 'get_clock') as mock_get_clock, \
                patch.object(NavigationNode, 'send_goal') as mock_send_goal:

            real_clock = Clock()
            mock_get_clock.return_value.now.return_value.to_msg.return_value = real_clock.now().to_msg()

            node = NavigationNode()
            node._action_client = MagicMock()
            node._action_client.wait_for_server.return_value = True
            node.get_logger = MagicMock()

            yield node
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_initialization(node):
    """Verify node initializes with correct patrol points"""
    assert len(node.patrol_points) == 5
    assert node.current_patrol_goal_key == "PESAS1"
    assert node._action_client.wait_for_server.called


def test_goal_comparison(node):
    """Test coordinate comparison logic"""
    p1 = Point(x=1.0, y=2.0, z=0.0)
    p2 = Point(x=1.009, y=1.991, z=0.0)
    assert node._is_same_goal(p1, p2) is True

    p3 = Point(x=1.02, y=2.0, z=0.0)
    assert node._is_same_goal(p1, p3) is False


def test_service_duplicate_goal(node):
    """Test service request with existing goal"""
    node.current_goal = Point(x=2.0, y=3.0, z=0.0)
    req = IrMaquina.Request(x=2.0, y=3.0, z=0.0)
    res = node.service_irq_callback(req, IrMaquina.Response())
    assert res.codigo == 0
    assert "already active" in res.res


def test_service_new_goal(node):
    """Test valid new goal through service"""
    req = IrMaquina.Request(x=5.0, y=6.0, z=0.0)
    res = node.service_irq_callback(req, IrMaquina.Response())
    assert res.codigo == 2
    assert node.current_goal == Point(x=5.0, y=6.0, z=0.0)


def test_patrol_sequence(node):
    """Verify automatic patrol point rotation"""
    node.current_patrol_goal_key = "PESAS5"
    with patch.object(node, 'send_goal') as mock_send_goal:
        node.next_point_controller()
        assert node.current_patrol_goal_key == "PESAS1"
        mock_send_goal.assert_called_once()


def test_goal_handling(node):
    """Test full goal sending workflow"""
    test_point = Point(x=3.0, y=4.0, z=0.0)

    # Setup mocks
    node._action_client.send_goal_async = MagicMock()
    node._action_client.wait_for_server = MagicMock()
    node.get_logger = MagicMock()

    node.send_goal(test_point)

    # Verify action client call
    node._action_client.send_goal_async.assert_called_once()
    goal_msg = node._action_client.send_goal_async.call_args[0][0]
    assert goal_msg.pose.pose.position == test_point


@patch('rclpy.time.Time')
def test_feedback_rate_limiting(mock_time, node):
    """Test feedback rate limiting logic"""
    feedback = MagicMock()
    feedback.feedback.current_pose.pose.position = Point(x=1.0, y=2.0, z=0.0)
    feedback.feedback.distance_remaining = 1.5
    feedback.feedback.navigation_time.sec = 10

    # First call
    node.feedback_callback(feedback)
    assert node.get_logger.info.call_count == 1

    # Immediate second call
    node.feedback_callback(feedback)
    assert node.get_logger.info.call_count == 1

    # Simulate time passage
    node.last_feedback_time = node.last_feedback_time - rclpy.time.Duration(seconds=1)
    node.feedback_callback(feedback)
    assert node.get_logger.info.call_count == 2


def test_navigation_result_handling(node):
    """Test result processing for success case"""
    future = MagicMock()
    future.result.return_value.status = GoalStatus.STATUS_SUCCEEDED
    future.result.return_value.result = MagicMock()

    node.current_patrol_goal_key = "PESAS1"
    node.get_logger = MagicMock()
    node.send_goal = MagicMock()

    node.get_result_callback(future)
    assert node.current_patrol_goal_key == "PESAS2"
    # Ensure next goal is sent
    node.send_goal.assert_called_once()


if __name__ == '__main__':
    pytest.main()
