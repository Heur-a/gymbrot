import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ament_index_python.packages import get_package_share_directory
import os

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/image', 10)

        package_path = get_package_share_directory('gymbrot_capture_poses')
        video_path = os.path.join(package_path, 'resources', 'video_referencia.mp4')
        self.get_logger().info(f"Ruta al video: {video_path}")

        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir el video")
            return

        self.timer = self.create_timer(1.0, self.publish_image)

    def publish_image(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("No se pudo leer del video")
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(msg)
        self.get_logger().info("Imagen publicada")

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
