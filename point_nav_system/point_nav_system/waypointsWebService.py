import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from interfaces_gymbrot.srv import GoToMachine


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._service = self.create_service(GoToMachine, 'go_to_machine', self.location_goal_callback)

        # Predefined waypoints for patrolling
        self.waypoints = self.get_patrol_waypoints()
        # Predefined machine positions (example coordinates)
        self.machines = [
            self.create_pose(5.0, 5.0),
            self.create_pose(6.0, 6.0),
            self.create_pose(7.0, 7.0),
            self.create_pose(8.0, 8.0)
        ]
        self.current_index = 0
        self.resume_index = 0
        self.is_interrupt = False
        self.goal_handle = None

        # Start patrolling
        self.send_next_waypoint()

    def get_patrol_waypoints(self):
        return [
            self.create_pose(1.0, 1.0),
            self.create_pose(2.0, 1.0),
            self.create_pose(2.0, 2.0),
            self.create_pose(1.0, 2.0),
        ]

    def create_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose

    def location_goal_callback(self, request, response):
        self.get_logger().info(f"Interrupting to go to machine {request.maquina}")
        self.resume_index = self.current_index



        # Send machine goal
        machine_pose = self.machines[request.maquina]
        self.is_interrupt = True
        self.send_goal(machine_pose)
        response.aceptado = True
        response.error = ''
        return response


    def send_next_waypoint(self):
        waypoint = self.waypoints[self.current_index]
        self.send_goal(waypoint)

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self._action_client.wait_for_server()
        self.get_logger().info(f'Sending goal: X={pose.pose.position.x:.2f}, Y={pose.pose.position.y:.2f}')

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        get_result_future = self.goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        self.process_after_navigation()

    def process_after_navigation(self):
        if self.is_interrupt:
            self.current_index = self.resume_index
            self.is_interrupt = False
        else:
            self.current_index = (self.current_index + 1) % len(self.waypoints)

        self.send_next_waypoint()


def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointFollower()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()