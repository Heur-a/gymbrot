import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class WebcamPublisher(Node):
    """
    Nodo ROS2 para capturar video desde una webcam y publicarlo como mensajes `sensor_msgs/Image`.

    Funcionalidad:
    - Captura imágenes desde una webcam con OpenCV
    - Convierte cada frame a un mensaje de imagen ROS
    - Publica los mensajes en el tópico `webcam/image_raw`

    Attributes:
        publisher_ (Publisher): Publicador de imágenes.
        bridge (CvBridge): Utilidad para convertir imágenes OpenCV ↔ ROS.
        cap (cv2.VideoCapture): Objeto para capturar video desde la webcam.
        timer (Timer): Temporizador para ejecutar `timer_callback` a una frecuencia fija.
    """

    def __init__(self):
        """Inicializa el nodo, webcam, publicador y temporizador."""
        super().__init__('webcam_publisher')

        # Crear publicador de imágenes
        self.publisher_ = self.create_publisher(Image, 'webcam/image_raw', 10)

        # Inicializar puente CvBridge
        self.bridge = CvBridge()

        # Inicializar captura de webcam (0 = cámara por defecto)
        self.cap = cv2.VideoCapture(0)

        # Verificar si la cámara se abrió correctamente
        if not self.cap.isOpened():
            self.get_logger().error("❌ No se pudo abrir la webcam.")
        else:
            self.get_logger().info("✅ Webcam iniciada correctamente.")

        # Ejecutar callback cada 0.1s (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        """
        Captura un frame desde la webcam y lo publica en formato ROS Image.

        Returns:
            None
        """
        ret, frame = self.cap.read()  # Leer un frame

        if ret:
            # Convertir frame OpenCV a mensaje ROS
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

            # Publicar imagen
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn("⚠️ No se pudo leer un frame de la webcam.")

    def destroy_node(self):
        """
        Libera la cámara y destruye el nodo correctamente.

        Returns:
            None
        """
        self.cap.release()  # Liberar webcam
        super().destroy_node()

def main(args=None):
    """
    Función principal. Inicializa y ejecuta el nodo ROS2.

    Args:
        args (list, optional): Argumentos de línea de comandos.

    Returns:
        None
    """
    rclpy.init(args=args)
    node = WebcamPublisher()

    try:
        rclpy.spin(node)  # Ejecutar el nodo
    except KeyboardInterrupt:
        pass  # Detener con Ctrl+C

    node.destroy_node()
    rclpy.shutdown()

# Ejecutar si se llama directamente
if __name__ == '__main__':
    main()
