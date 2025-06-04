import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import mxnet as mx
import gluoncv
from gluoncv.model_zoo import get_model
from gluoncv.utils import try_import_cv2
from gluoncv.data.transforms.pose import detector_to_alpha_pose, heatmap_to_coord
from gluoncv.utils.viz import cv_plot_keypoints
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

cv2 = try_import_cv2()

def select_best_person(pred_coords, confidence):
    avg_conf = confidence.mean(axis=1).asnumpy().flatten()
    best_idx = np.argmax(avg_conf)
    return pred_coords[best_idx].asnumpy(), confidence[best_idx].asnumpy()

def process_frame(frame, detector, estimator, ctx):
    height, width = frame.shape[:2]
    frame_rgb_np = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_rgb = mx.nd.array(frame_rgb_np)
    x, frame_processed = gluoncv.data.transforms.presets.ssd.transform_test(frame_rgb, short=240)
    x = x.as_in_context(ctx)

    class_IDs, scores, bounding_boxes = detector(x)
    n_persons = int((scores > 0.5).sum().asscalar())
    if n_persons == 0:
        return None, None, None

    pose_input, upscale_bbox = detector_to_alpha_pose(frame_processed, class_IDs, scores, bounding_boxes)
    if upscale_bbox is None:
        return None, None, None

    predicted_heatmap = estimator(pose_input.as_in_context(ctx))
    pred_coords, confidence = heatmap_to_coord(predicted_heatmap, upscale_bbox)
    pred_coords, confidence = select_best_person(pred_coords, confidence)

    return pred_coords, confidence, (height, width)

def normalize_keypoints(coords, image_shape):
    h, w = image_shape
    return coords / np.array([w, h])

def calculate_pose_similarity(coords1, conf1, coords2, conf2, image_shape):
    assert coords1.shape == coords2.shape

    conf1 = conf1.flatten()
    conf2 = conf2.flatten()
    valid = (conf1 > 0.3) & (conf2 > 0.3)

    if valid.sum() == 0:
        return 0.0

    coords1_valid = normalize_keypoints(coords1[valid], image_shape)
    coords2_valid = normalize_keypoints(coords2[valid], image_shape)

    distances = np.linalg.norm(coords1_valid - coords2_valid, axis=1)
    mean_distance = distances.mean()
    similarity = max(0.0, 1.0 - mean_distance)
    return similarity

class PoseComparisonNode(Node):
    def __init__(self):
        super().__init__('pose_comparison_node')

        self.bridge = CvBridge()
        self.ctx = mx.cpu()

        # Cargar modelos una sola vez
        self.get_logger().info("Cargando modelo detector...")
        self.detector = get_model('ssd_512_mobilenet1.0_coco', pretrained=True, ctx=self.ctx)
        self.detector.reset_class(classes=['person'], reuse_weights={'person': 'person'})

        self.get_logger().info("Cargando modelo de pose...")
        self.estimator = get_model('alpha_pose_resnet101_v1b_coco', pretrained=True, ctx=self.ctx)

        # Cargar imagen de referencia y procesar pose
        package_path = get_package_share_directory('gymbrot_capture_poses')
        ref_image_path = os.path.join(package_path, 'resources', 'IMG_7666.png')
        ref_frame = cv2.imread(ref_image_path)
        if ref_frame is None:
            self.get_logger().error(f"No se pudo leer la imagen de referencia: {ref_image_path}")
            raise RuntimeError("Imagen de referencia no encontrada")

        self.ref_coords, self.ref_conf, self.ref_shape = process_frame(ref_frame, self.detector, self.estimator, self.ctx)
        if self.ref_coords is None:
            self.get_logger().error("No se detectó persona en la imagen de referencia")
            raise RuntimeError("Sin persona en imagen de referencia")

        self.get_logger().info("Imagen de referencia procesada correctamente")

        # Suscribirse al topic de imagen
        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"Error al convertir imagen ROS a OpenCV: {e}")
            return

        coords, conf, shape = process_frame(frame, self.detector, self.estimator, self.ctx)
        if coords is None:
            self.get_logger().info("No se detectó persona en la imagen recibida")
            return

        similarity = calculate_pose_similarity(self.ref_coords, self.ref_conf, coords, conf, self.ref_shape)
        self.get_logger().info(f"Índice de similitud de pose con referencia: {similarity * 100:.2f}%")

def main(args=None):
    rclpy.init(args=args)
    node = PoseComparisonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
