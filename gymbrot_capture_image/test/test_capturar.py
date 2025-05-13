import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from unittest.mock import patch, MagicMock

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from gymbrot_capture_image.capturar import Ros2OpenCVImageConverter


@pytest.fixture(scope='module')
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def ros2_open_cv_converter(rclpy_init_shutdown):
    node = Ros2OpenCVImageConverter()
    yield node
    node.destroy_node()

def test_node_initialization(ros2_open_cv_converter):
    assert isinstance(ros2_open_cv_converter, Node)
    assert ros2_open_cv_converter.image_pub.topic == '/camera/processed_image'

@patch('cv_bridge.CvBridge.imgmsg_to_cv2')
def test_camera_callback(mock_imgmsg_to_cv2, ros2_open_cv_converter):
    # Simula un frame de imagen de prueba
    dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)

    # Simula la conversión de mensaje ROS a OpenCV
    mock_imgmsg_to_cv2.return_value = dummy_frame

    # Simula un mensaje Image ROS
    mock_image_msg = MagicMock(spec=Image)
    ros2_open_cv_converter.camera_callback(mock_image_msg)

    # Verifica que se haya convertido la imagen correctamente
    mock_imgmsg_to_cv2.assert_called_once_with(mock_image_msg, desired_encoding='bgr8')

@patch('cv_bridge.CvBridge.cv2_to_imgmsg')
def test_timer_callback_publishes_image(mock_cv2_to_imgmsg, ros2_open_cv_converter):
    # Simula un frame de imagen de prueba
    dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)

    # Simula que la imagen se ha procesado
    ros2_open_cv_converter.cv_image = dummy_frame

    # Retorno falso válido: objeto real del tipo esperado
    fake_ros_image = Image()
    mock_cv2_to_imgmsg.return_value = fake_ros_image

    # Llama manualmente a la función de temporizador
    ros2_open_cv_converter.timer_callback()

    # Verifica que la conversión fue llamada
    mock_cv2_to_imgmsg.assert_called_once_with(dummy_frame, encoding='bgr8')
