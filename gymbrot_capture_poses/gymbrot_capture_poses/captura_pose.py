import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from estima_pose import PoseEstimator

class PoseCaptureNode(Node):
    def __init__(self):
        super().__init__('pose_capture_node')
        self.bridge = CvBridge()
        self.estimator = PoseEstimator()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # Pose de referencia (esto deberías cargarlo desde archivo o definirlo)
        self.ref_pose = self.load_reference_pose()

    def load_reference_pose(self):
        # Ejemplo: carga desde archivo npy, o define manualmente
        try:
            return np.load('pose_referencia.npy')  # debe ser (17, 2)
        except:
            self.get_logger().warn('No se encontró la pose de referencia. Usando valores nulos.')
            return None

    def compute_similarity(self, pose1, pose2):
        if pose1 is None or pose2 is None:
            return None
        # Distancia euclidiana media entre keypoints
        dists = np.linalg.norm(pose1 - pose2, axis=1)
        return np.mean(dists)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        keypoints, confidence, vis_img = self.estimator.estimate(frame, return_vis=True)

        if keypoints is not None and self.ref_pose is not None:
            similarity = self.compute_similarity(keypoints, self.ref_pose)
            self.get_logger().info(f'Similitud con pose de referencia: {similarity:.2f}')
        else:
            self.get_logger().info('No se pudo estimar la pose o no hay referencia.')

        if vis_img is not None:
            cv2.imshow('Pose Estimation', vis_img)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = PoseCaptureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
