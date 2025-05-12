import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.timer import Timer
import time

class Ros2OpenCVImageConverter(Node):   

    def __init__(self):
        super().__init__('Ros2OpenCVImageConverter')
        
        self.bridge_object = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/image',
            self.camera_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        # 👇 Nuevo publisher para la imagen procesada
        self.image_pub = self.create_publisher(
            Image,
            '/camera/processed_image',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        # Crear ventana una sola vez
        cv2.namedWindow("Detección de mancuerna", cv2.WINDOW_NORMAL)

        # Timer para controlar el intervalo de procesamiento
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 segundo de intervalo

        # Variable para almacenar la última imagen procesada
        self.cv_image = None

    def camera_callback(self, data):
        try:
            self.get_logger().info(f"Encoding recibido: {data.encoding}")
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            self.get_logger().info(f"Media de pixeles: {np.mean(self.cv_image)}")

        except CvBridgeError as e:
            self.get_logger().error(f'Error al convertir imagen: {e}')
            return

    def timer_callback(self):
        if self.cv_image is not None:
            gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, 50, 150)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 1000:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(w) / h

            if 2.0 < aspect_ratio < 6.0:
                cv2.rectangle(self.cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.putText(self.cv_image, "Mancuerna detectada", (x, y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        cv2.imshow("Detección de mancuerna", self.cv_image)
        cv2.waitKey(1)

        # ✅ Publicar imagen procesada
        try:
            ros_image = self.bridge_object.cv2_to_imgmsg(self.cv_image, encoding="bgr8")
            self.image_pub.publish(ros_image)
            self.get_logger().info("✅ Imagen procesada publicada en /camera/processed_image")
        except CvBridgeError as e:
            self.get_logger().error(f'❌ Error al convertir y publicar imagen: {e}')
        else:
            self.get_logger().warn("⚠️ No hay imagen disponible para procesar.")

    def destroy_node(self):
        cv2.destroyAllWindows()  # Asegura que las ventanas se cierren correctamente
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)    
    img_converter_object = Ros2OpenCVImageConverter()    
       
    try:
        rclpy.spin(img_converter_object)
    except KeyboardInterrupt:
        img_converter_object.destroy_node()
        print("Fin del programa!")
        cv2.destroyAllWindows() 
    

if __name__ == '__main__':
    main()
