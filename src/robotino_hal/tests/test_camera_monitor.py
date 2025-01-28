import pytest
from unittest.mock import MagicMock, patch
import rclpy
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import cv2
from cv_bridge import CvBridge
from robotino_hal.nodes.camera_monitor import RobotinoCameraMonitor, CameraConfig

@pytest.fixture
def rclpy_init_shutdown():
    """Fixture for initializing and shutting down rclpy"""
    rclpy.init()
    yield
    rclpy.shutdown()

def test_node_parameters(rclpy_init_shutdown):
    """Test initialization of node parameters"""
    node = RobotinoCameraMonitor()

    assert node.robotino_ip == '10.42.0.232' or 'localhost:8081'
    assert node.update_rate == 10.0
    assert node.timeout == 1.0
    assert node.camera_config.width == 640
    assert node.camera_config.height == 480
    assert node.camera_config.fps == 10

    node.destroy_node()

@patch('robotino_hal.nodes.camera_monitor.requests.get', autospec=True)
def test_get_camera_frame_success(mock_get, rclpy_init_shutdown):
    """Test successful retrieval of camera frame"""
    node = RobotinoCameraMonitor()

    # Mock successful API response
    mock_response = MagicMock()
    mock_response.status_code = 200
    mock_response.content = cv2.imencode('.jpg', np.zeros((480, 640, 3), dtype=np.uint8))[1].tobytes()
    mock_get.return_value = mock_response

    frame = node.get_camera_frame()
    assert frame is not None
    assert frame.shape == (480, 640, 3)  # Height, width, channels
    assert node.consecutive_errors == 0

    node.destroy_node()

@patch('robotino_hal.nodes.camera_monitor.requests.get', autospec=True)
def test_get_camera_frame_failure(mock_get, rclpy_init_shutdown):
    """Test handling of failed camera frame retrieval"""
    node = RobotinoCameraMonitor()

    # Mock API failure
    mock_get.side_effect = Exception('API error')

    frame = node.get_camera_frame()
    assert frame is None
    assert node.consecutive_errors == 1

    # Test multiple consecutive errors
    for _ in range(4):
        frame = node.get_camera_frame()
    assert node.consecutive_errors == 5

    node.destroy_node()

@patch('robotino_hal.nodes.camera_monitor.RobotinoCameraMonitor.get_camera_frame', autospec=True)
def test_publish_camera_data_success(mock_get_camera_frame, rclpy_init_shutdown):
    """Test successful publishing of camera data"""
    node = RobotinoCameraMonitor()

    # Mock camera frame
    mock_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    mock_get_camera_frame.return_value = mock_frame

    # Mock publishers
    mock_image_publisher = MagicMock()
    mock_camera_info_publisher = MagicMock()
    node.image_publisher = mock_image_publisher
    node.camera_info_publisher = mock_camera_info_publisher

    # Call the publish_camera_data method
    node.publish_camera_data()

    # Verify the messages were published
    mock_image_publisher.publish.assert_called_once()
    mock_camera_info_publisher.publish.assert_called_once()

    # Verify the image message
    published_img_msg = mock_image_publisher.publish.call_args[0][0]
    assert isinstance(published_img_msg, Image)
    assert published_img_msg.header.frame_id == 'camera_link'

    # Verify the camera info message
    published_info_msg = mock_camera_info_publisher.publish.call_args[0][0]
    assert isinstance(published_info_msg, CameraInfo)
    assert published_info_msg.header.frame_id == 'camera_link'
    assert published_info_msg.height == 480
    assert published_info_msg.width == 640

    node.destroy_node()

@patch('robotino_hal.nodes.camera_monitor.RobotinoCameraMonitor.get_camera_frame', autospec=True)
def test_publish_camera_data_failure(mock_get_camera_frame, rclpy_init_shutdown):
    """Test handling of failed camera frame retrieval in publish_camera_data"""
    node = RobotinoCameraMonitor()

    # Mock failed camera frame retrieval
    mock_get_camera_frame.return_value = None

    # Mock publishers
    mock_image_publisher = MagicMock()
    mock_camera_info_publisher = MagicMock()
    node.image_publisher = mock_image_publisher
    node.camera_info_publisher = mock_camera_info_publisher

    # Call the publish_camera_data method
    node.publish_camera_data()

    # Verify no messages were published
    mock_image_publisher.publish.assert_not_called()
    mock_camera_info_publisher.publish.assert_not_called()

    node.destroy_node()

if __name__ == '__main__':
    pytest.main(['-v', __file__])