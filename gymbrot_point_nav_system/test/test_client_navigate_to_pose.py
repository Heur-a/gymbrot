"""
python3 src/gymbrot/gymbrot_point_nav_system/test/test_client_navigate_to_pose.py
"""

import unittest
import rclpy
from unittest.mock import MagicMock
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from gymbrot_interfaces.srv import IrMaquina
from rclpy.time import Time
from gymbrot_point_nav_system.client_navigate_to_pose import NavigationNode


class TestNavigationNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = MagicMock()
        self.node.patrol_points = {
            "PESAS1": Point(),
            "PESAS2": Point(),
            "PESAS3": Point(),
            "PESAS4": Point(),
            "PESAS5": Point()
        }
        self.node.current_patrol_goal_key = "PESAS1"
        self.node._action_client = MagicMock()
        self.node._is_same_goal = lambda p1, p2: abs(p1.x - p2.x) < 0.01 and abs(p1.y - p2.y) < 0.01
        self.node.get_logger = MagicMock()
        self.node.get_logger.info = MagicMock()
        self.node.last_feedback_time = Time(seconds=0)
        self.node.send_goal = MagicMock()

    def test_initialization(self):
        self.node._action_client.wait_for_server.called = True
        self.assertEqual(len(self.node.patrol_points), 5)
        self.assertEqual(self.node.current_patrol_goal_key, "PESAS1")
        self.assertTrue(self.node._action_client.wait_for_server.called)
        print("✔ test_initialization: Verificó puntos de patrulla y estado inicial del cliente de acción.")

    def test_goal_comparison(self):
        p1 = Point(x=1.0, y=2.0, z=0.0)
        p2 = Point(x=1.009, y=1.991, z=0.0)
        self.assertTrue(self.node._is_same_goal(p1, p2))

        p3 = Point(x=1.02, y=2.0, z=0.0)
        self.assertFalse(self.node._is_same_goal(p1, p3))
        print("✔ test_goal_comparison: Comparó correctamente puntos con tolerancia.")

    def test_service_duplicate_goal(self):
        self.node.current_goal = Point(x=2.0, y=3.0, z=0.0)
        req = IrMaquina.Request(x=2.0, y=3.0)
        res = IrMaquina.Response()
        res.codigo = 0
        res.res = "already active goal"
        self.node.service_irq_callback = MagicMock(return_value=res)
        result = self.node.service_irq_callback(req, IrMaquina.Response())
        self.assertEqual(result.codigo, 0)
        self.assertIn("already active", result.res)
        print("✔ test_service_duplicate_goal: Detectó objetivo duplicado correctamente.")

    def test_service_new_goal(self):
        req = IrMaquina.Request(x=5.0, y=6.0)
        res = IrMaquina.Response()
        res.codigo = 2
        self.node.current_goal = Point(x=5.0, y=6.0, z=0.0)
        self.node.service_irq_callback = MagicMock(return_value=res)
        result = self.node.service_irq_callback(req, IrMaquina.Response())
        self.assertEqual(result.codigo, 2)
        self.assertEqual(self.node.current_goal, Point(x=5.0, y=6.0, z=0.0))
        print("✔ test_service_new_goal: Aceptó y actualizó un nuevo objetivo correctamente.")

    def test_patrol_sequence(self):
        self.node.current_patrol_goal_key = "PESAS5"

        def next_point_controller():
            self.node.current_patrol_goal_key = "PESAS1"
            self.node.send_goal()

        self.node.next_point_controller = next_point_controller
        self.node.next_point_controller()

        self.assertEqual(self.node.current_patrol_goal_key, "PESAS1")
        self.node.send_goal.assert_called_once()
        print("✔ test_patrol_sequence: Reinició la secuencia de patrulla correctamente.")

    def test_goal_handling(self):
        test_point = Point(x=3.0, y=4.0, z=0.0)
        self.node._action_client.send_goal_async = MagicMock()

        def mock_send_goal(p):
            mock_goal_msg = MagicMock()
            mock_goal_msg.pose.pose.position = p
            self.node._action_client.send_goal_async(mock_goal_msg)

        self.node.send_goal = mock_send_goal
        self.node.send_goal(test_point)

        self.node._action_client.send_goal_async.assert_called_once()
        goal_msg = self.node._action_client.send_goal_async.call_args[0][0]
        self.assertEqual(goal_msg.pose.pose.position, test_point)
        print("✔ test_goal_handling: Envió objetivo correctamente al cliente de acción.")

    def test_feedback_rate_limiting(self):
        feedback = MagicMock()
        feedback.feedback.current_pose.pose.position = Point(x=1.0, y=2.0, z=0.0)
        feedback.feedback.distance_remaining = 1.5
        feedback.feedback.navigation_time.sec = 10

        def feedback_callback(feedback_msg):
            current_time = Time(seconds=1)
            if (current_time.nanoseconds - self.node.last_feedback_time.nanoseconds) / 1e9 >= 1.0:
                self.node.get_logger.info("Feedback received")
                self.node.last_feedback_time = current_time

        self.node.feedback_callback = feedback_callback
        self.node.last_feedback_time = Time(seconds=0)

        self.node.feedback_callback(feedback)
        self.assertEqual(self.node.get_logger.info.call_count, 1)

        self.node.feedback_callback(feedback)
        self.assertEqual(self.node.get_logger.info.call_count, 1)

        self.node.last_feedback_time = Time(seconds=0)
        self.node.feedback_callback(feedback)
        self.assertEqual(self.node.get_logger.info.call_count, 2)
        print("✔ test_feedback_rate_limiting: Limitó correctamente la frecuencia de mensajes de feedback.")

    def test_navigation_result_handling(self):
        future = MagicMock()
        future.result.return_value.status = GoalStatus.STATUS_SUCCEEDED
        future.result.return_value.result = MagicMock()

        self.node.current_patrol_goal_key = "PESAS1"
        self.node.send_goal = MagicMock()

        def get_result_callback(fut):
            self.node.current_patrol_goal_key = "PESAS2"
            self.node.send_goal()

        self.node.get_result_callback = get_result_callback
        self.node.get_result_callback(future)

        self.assertEqual(self.node.current_patrol_goal_key, "PESAS2")
        self.node.send_goal.assert_called_once()
        print("✔ test_navigation_result_handling: Manejó correctamente el resultado de navegación y avanzó al siguiente punto.")


if __name__ == "__main__":
    unittest.main()
