import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

class TemplateMatchingNode(Node):
    """
    Nodo ROS2 para realizar detección de objetos mediante Template Matching con OpenCV.

    Funcionalidades:
    - Suscribirse a un tópico de imagen (`/image`)
    - Detectar una plantilla (template) en la imagen usando correlación normalizada
    - Publicar la imagen procesada en un tópico (`/camera/processed_image`)
    - Mostrar la imagen procesada en una ventana de OpenCV

    Attributes:
        bridge_object (CvBridge): Objeto para convertir imágenes ROS a OpenCV y viceversa.
        template (np.ndarray): Imagen de plantilla cargada en escala de grises.
        template_w (int): Ancho de la plantilla.
        template_h (int): Alto de la plantilla.
        cv_image (np.ndarray): Última imagen recibida convertida a OpenCV.
        timer (Timer): Temporizador para ejecutar la detección periódicamente.
    """

    def __init__(self):
        """Inicializa suscripciones, publicadores, carga la plantilla y configura la interfaz."""
        super().__init__('template_matching_node')
        
        self.bridge_object = CvBridge()

        # Suscripción al tópico de imagen
        self.image_sub = self.create_subscription(
            Image,
            '/image',
            self.camera_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # Publicador para imagen procesada
        self.image_pub = self.create_publisher(
            Image,
            '/camera/processed_image',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # Cargar plantilla en escala de grises
        self.template = cv2.imread('/ruta/a/plantilla_mancuerna.png', cv2.IMREAD_GRAYSCALE)
        if self.template is None:
            self.get_logger().error("❌ No se pudo cargar la plantilla de mancuerna")
        else:
            self.template_w, self.template_h = self.template.shape[::-1]
            self.get_logger().info("✅ Plantilla cargada correctamente")

        self.cv_image = None  # Imagen actual

        # Ventana para mostrar resultados
        cv2.namedWindow("Template Matching", cv2.WINDOW_NORMAL)

        # Temporizador para detección periódica (cada 1 segundo)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def camera_callback(self, data):
        """
        Convierte la imagen ROS a formato OpenCV y la almacena.

        Args:
            data (sensor_msgs.msg.Image): Imagen recibida del tópico.

        Returns:
            None
        """
        try:
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'❌ Error al convertir imagen: {e}')

    def timer_callback(self):
        """
        Procesa la imagen almacenada, busca coincidencias con la plantilla y publica el resultado.

        Returns:
            None
        """
        if self.cv_image is not None:
            # Convertir a escala de grises para comparación
            gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

            # Ejecutar template matching si la plantilla fue cargada
            if self.template is not None:
                res = cv2.matchTemplate(gray, self.template, cv2.TM_CCOEFF_NORMED)
                threshold = 0.6  # Umbral de coincidencia
                loc = np.where(res >= threshold)

                for pt in zip(*loc[::-1]):
                    # Dibujar rectángulo en coincidencia
                    cv2.rectangle(self.cv_image, pt, (pt[0] + self.template_w, pt[1] + self.template_h), (0, 255, 0), 2)
                    cv2.putText(self.cv_image, "Plantilla detectada", (pt[0], pt[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # Mostrar imagen con detección
            cv2.imshow("Template Matching", self.cv_image)
            cv2.waitKey(1)

            # Publicar imagen procesada
            try:
                ros_image = self.bridge_object.cv2_to_imgmsg(self.cv_image, encoding="bgr8")
                self.image_pub.publish(ros_image)
                self.get_logger().info("✅ Imagen procesada publicada")
            except CvBridgeError as e:
                self.get_logger().error(f'❌ Error al convertir y publicar imagen: {e}')
        else:
            self.get_logger().warn("⚠️ No hay imagen disponible para procesar.")

    def destroy_node(self):
        """
        Cierra la ventana de OpenCV y destruye el nodo.

        Returns:
            None
        """
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    """
    Punto de entrada del nodo ROS2.

    Args:
        args (list, optional): Argumentos de línea de comandos.

    Returns:
        None
    """
    rclpy.init(args=args)
    node = TemplateMatchingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        print("Fin del programa")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
