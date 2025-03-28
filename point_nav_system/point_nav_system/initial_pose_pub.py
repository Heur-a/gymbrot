import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class Publisher(Node):

    def __init__(self):
        super().__init__('initial_pose_pub_node')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)

        # Crear i publicar el missatge una sola vegada
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = 0.2
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0

        self.get_logger().info('Publicant posició inicial una sola vegada \n X= 0.2 \n Y=0.0 \n W = 1.0')
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    publisher = Publisher()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
