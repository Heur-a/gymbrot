import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from ament_index_python.packages import get_package_share_directory
import os

class TemplateMatchingNode(Node):
    def __init__(self):
        super().__init__('template_matching_node')

        # Obtener ruta del paquete
        package_share_dir = get_package_share_directory('gymbrot_capture_image')

        # Declarar y obtener parámetros
        self.declare_parameter("threshold", 0.6)
        self.declare_parameter("scales", [1.0])  # Ejemplo: [0.8, 1.0, 1.2]

        self.threshold = self.get_parameter("threshold").value
        self.scales = self.get_parameter("scales").value

        # Rutas absolutas de las plantillas
        template_paths = [
            os.path.join(package_share_dir, 'plantillas', 'mancuerna.jpg'),
            os.path.join(package_share_dir, 'plantillas', 'cuadriceps.png'),
        ]

        self.templates = []
        for path in template_paths:
            if os.path.exists(path):
                template = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
                if template is not None:
                    self.templates.append(template)
                    self.get_logger().info(f"✅ Plantilla cargada: {path}")
                else:
                    self.get_logger().warn(f"⚠️ No se pudo leer la imagen: {path}")
            else:
                self.get_logger().warn(f"❌ Ruta no válida: {path}")

        self.bridge = CvBridge()
        self.cv_image = None

        # Suscripción a la cámara
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # Publicador de imagen procesada
        self.image_pub = self.create_publisher(
            Image,
            '/camera/processed_image',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # Ventana para visualización (opcional)
        cv2.namedWindow("Template Matching", cv2.WINDOW_NORMAL)

        # Temporizador de procesamiento
        self.timer = self.create_timer(1.0, self.timer_callback)

    def camera_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"❌ Error al convertir imagen: {e}")

    def apply_clahe(self, gray_image):
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        return clahe.apply(gray_image)

    def timer_callback(self):
        if self.cv_image is None:
            self.get_logger().warn("⚠️ No hay imagen disponible para procesar.")
            return

        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        gray = self.apply_clahe(gray)
        img_result = self.cv_image.copy()

        for template in self.templates:
            for scale in self.scales:
                resized_template = cv2.resize(template, (0, 0), fx=scale, fy=scale)
                w, h = resized_template.shape[::-1]

                if gray.shape[0] < h or gray.shape[1] < w:
                    continue

                res = cv2.matchTemplate(gray, resized_template, cv2.TM_CCOEFF_NORMED)
                loc = np.where(res >= self.threshold)

                for pt in zip(*loc[::-1]):
                    cv2.rectangle(img_result, pt, (pt[0] + w, pt[1] + h), (0, 255, 0), 2)
                    cv2.putText(img_result, f"Match {res[pt[1], pt[0]]:.2f}", (pt[0], pt[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.imshow("Template Matching", img_result)
        cv2.waitKey(1)

        try:
            ros_image = self.bridge.cv2_to_imgmsg(img_result, encoding="bgr8")
            self.image_pub.publish(ros_image)
            self.get_logger().info("✅ Imagen procesada publicada")
        except Exception as e:
            self.get_logger().error(f"❌ Error al publicar imagen: {e}")

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TemplateMatchingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        print("Fin del nodo")
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
