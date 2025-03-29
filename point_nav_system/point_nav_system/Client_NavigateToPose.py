import rclpy
from interfaces_gymbrot.msg import LocationGoal
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile, ReliabilityPolicy

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        # Action client to send navigation goals
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        # Subscriber to get new goals from the topic
        self.subscription = self.create_subscription(
            LocationGoal,
            '/locationGoal',
            self.location_goal_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.current_goal = None  # Store the latest goal position
        self.is_active = False  # Track if a goal is being processed

    def location_goal_callback(self, msg):
        """
        Callback para procesar nuevos objetivos de ubicación.
        Ahora msg tiene la estructura simple:
            float64 x  # Posición X
            float64 y  # Posición Y
        """
        # Crear un Point con las coordenadas recibidas
        new_goal = Point()
        new_goal.x = msg.x
        new_goal.y = msg.y
        new_goal.z = 0.0  # Asumimos 2D, z=0

        self.current_goal = new_goal
        self.get_logger().info(f'Nuevo objetivo recibido: X: {msg.x:.2f}, Y: {msg.y:.2f}')

        # Enviar el objetivo si no hay uno activo
        if not self.is_active:
            self.send_goal(new_goal)

    def send_goal(self, point):
        # Prepare the goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position = point
        goal_msg.pose.pose.orientation.w = 1.0  # Neutral orientation

        # Wait for the action server and send the goal
        self._action_client.wait_for_server()
        self.get_logger().info('Sending new goal...')
        self.is_active = True
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.is_active = False
            return
        self.get_logger().info('Goal accepted')
        # Request result once the goal is accepted
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        try:
            # El resultado real está en future.result().result
            result = future.result().result
            self.get_logger().info('Navegación completada (resultado vacío)')

            # Feedback opcional (si necesitas información de la navegación)
            feedback = future.result().feedback
            if feedback:
                self.get_logger().info(
                    f'Distancia restante: {feedback.distance_remaining:.2f}m | '
                    f'Tiempo navegación: {feedback.navigation_time.sec}s'
                )

        except Exception as e:
            self.get_logger().error(f'Error procesando resultado: {str(e)}')
        finally:
            self.is_active = False
            # Reenviar el último goal si está disponible
            if self.current_goal is not None:
                self.send_goal(self.current_goal)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current distance to goal: {feedback.distance_remaining:.2f} meters')


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()