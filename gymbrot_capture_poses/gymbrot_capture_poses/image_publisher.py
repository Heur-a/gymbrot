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

        # Ruta configurable (puedes cambiar el archivo aquí)
        package_path = get_package_share_directory('gymbrot_capture_poses')
        file_path = os.path.join(package_path, 'resources', 'mid_071.png')  # Cambia a .png, .jpg, etc. si quieres usar imagen

        self.get_logger().info(f"📂 Archivo de entrada: {file_path}")

        if not os.path.exists(file_path):
            self.get_logger().error("❌ El archivo no existe")
            return

        self.file_path = file_path
        self.is_video = file_path.lower().endswith(('.mp4', '.avi', '.mov', '.mkv'))

        if self.is_video:
            self.cap = cv2.VideoCapture(file_path)
            if not self.cap.isOpened():
                self.get_logger().error("❌ No se pudo abrir el video")
                return
            self.timer = self.create_timer(1.0, self.publish_video_frame)  # 1 Hz
        else:
            self.image = cv2.imread(file_path)
            if self.image is None:
                self.get_logger().error("❌ No se pudo leer la imagen")
                return
            self.timer = self.create_timer(2.0, self.publish_static_image)  # Cada 2s para imagen fija

    def publish_video_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("🎞️ Fin del video o error de lectura")
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Reiniciar video
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(msg)
        self.get_logger().info("📤 Frame de video publicado")

    def publish_static_image(self):
        msg = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
        self.publisher.publish(msg)
        self.get_logger().info("📷 Imagen estática publicada")

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
