import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

class Ros2OpenCVImageConverter(Node):
    """
    Nodo ROS2 que convierte imágenes de ROS a OpenCV, detecta objetos tipo 'mancuerna'
    mediante contornos y publica las imágenes procesadas.

    Sus funcionalidades incluyen:
    - Suscribirse a un tópico de imagen (`/image`)
    - Procesar y detectar formas basadas en contornos y relaciones de aspecto
    - Publicar una nueva imagen con anotaciones en `/camera/processed_image`
    - Mostrar la imagen procesada con OpenCV

    Attributes:
        bridge_object (CvBridge): Convierte entre imágenes ROS y OpenCV.
        image_sub (Subscription): Suscripción al tópico de entrada.
        image_pub (Publisher): Publicador para la imagen procesada.
        cv_image (np.ndarray): Imagen OpenCV actual.
        timer (Timer): Temporizador para procesamiento periódico.
    """

    def __init__(self):
        """Inicializa el nodo, suscripciones, publicadores y la ventana de visualización."""
        super().__init__('Ros2OpenCVImageConverter')

        self.bridge_object = CvBridge()

        # Suscribirse a imágenes crudas desde la cámara
        self.image_sub = self.create_subscription(
            Image,
            '/image',
            self.camera_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # Publicador para imágenes procesadas
        self.image_pub = self.create_publisher(
            Image,
            '/camera/processed_image',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # Crear ventana para visualizar resultados
        cv2.namedWindow("Detección de mancuerna", cv2.WINDOW_NORMAL)

        # Timer para ejecutar el procesamiento cada segundo
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.cv_image = None  # Imagen actual en formato OpenCV

    def camera_callback(self, data):
        """
        Callback que se ejecuta al recibir una nueva imagen del tópico.

        Args:
            data (sensor_msgs.msg.Image): Imagen en formato ROS.

        Returns:
            None
        """
        try:
            self.get_logger().info(f"Encoding recibido: {data.encoding}")
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            self.get_logger().info(f"Media de pixeles: {np.mean(self.cv_image)}")
        except CvBridgeError as e:
            self.get_logger().error(f'Error al convertir imagen: {e}')

    def timer_callback(self):
        """
        Callback del temporizador. Procesa la imagen almacenada si está disponible:
        - Convierte a escala de grises
        - Aplica desenfoque y detección de bordes
        - Encuentra contornos y detecta formas con ciertas proporciones
        - Muestra y publica la imagen procesada

        Returns:
            None
        """
        if self.cv_image is not None:
            # Preprocesamiento de la imagen
            gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, 50, 150)

            # Detección de contornos
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 1000:
                    continue  # Ignorar objetos pequeños

                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = float(w) / h

                # Detectar objetos alargados horizontalmente (ej. mancuerna)
                if 2.0 < aspect_ratio < 6.0:
                    cv2.rectangle(self.cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                    cv2.putText(self.cv_image, "Mancuerna detectada", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            # Mostrar la imagen en ventana
            cv2.imshow("Detección de mancuerna", self.cv_image)
            cv2.waitKey(1)

            # Publicar la imagen procesada como mensaje ROS
            try:
                ros_image = self.bridge_object.cv2_to_imgmsg(self.cv_image, encoding="bgr8")
                self.image_pub.publish(ros_image)
                self.get_logger().info("✅ Imagen procesada publicada en /camera/processed_image")
            except CvBridgeError as e:
                self.get_logger().error(f'❌ Error al convertir y publicar imagen: {e}')
        else:
            self.get_logger().warn("⚠️ No hay imagen disponible para procesar.")

    def destroy_node(self):
        """
        Cierra las ventanas de OpenCV y destruye el nodo.

        Returns:
            None
        """
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    """
    Función principal que inicializa el nodo ROS2 y lo mantiene en ejecución.

    Args:
        args (list, optional): Argumentos de línea de comandos.

    Returns:
        None
    """
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
