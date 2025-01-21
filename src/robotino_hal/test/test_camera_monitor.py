#!/usr/bin/env python3

import pytest
from sensor_msgs.msg import Image, CameraInfo
from robotino_hal.nodes.camera_monitor import RobotinoCameraMonitor
from .test_base import RobotinoNodeTestBase
import numpy as np
import cv2
from cv_bridge import CvBridge
import time
from unittest.mock import MagicMock

class TestCameraMonitor(RobotinoNodeTestBase):
    """Test cases for camera monitor node"""
    
    def get_node_name(self):
        return 'camera_monitor'
    
    @pytest.fixture(autouse=True)
    def setup_method(self):
        """Setup before each test"""
        self.create_subscription('robotino/camera/image_raw', Image)
        self.create_subscription('robotino/camera/camera_info', CameraInfo)
        
        # Create mock image for testing
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(img, "TEST", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        _, self.mock_jpg = cv2.imencode('.jpg', img)
        
        # Setup mock response for camera endpoint
        def mock_camera_get(*args, **kwargs):
            mock_response = MagicMock()
            if args[0].endswith('/cam0'):
                mock_response.ok = True
                mock_response.content = self.mock_jpg.tobytes()
                return mock_response
            return self.mock_get(*args, **kwargs)
            
        self.mock_get.side_effect = mock_camera_get
        
        # Initialize CV bridge
        self.bridge = CvBridge()
    
    def test_camera_publishing(self):
        """Test that camera monitor publishes images and camera info"""
        # Wait for both image and camera info
        assert self.wait_for_message('robotino/camera/image_raw'), \
            "No image message received"
        assert self.wait_for_message('robotino/camera/camera_info'), \
            "No camera info message received"
        
        # Check image message
        img_msg = self.get_last_message('robotino/camera/image_raw')
        assert img_msg is not None, "No image message in queue"
        assert img_msg.header.frame_id == 'camera_link', "Wrong frame_id"
        assert img_msg.encoding == 'bgr8', "Wrong encoding"
        assert img_msg.height == 480, "Wrong image height"
        assert img_msg.width == 640, "Wrong image width"
        
        # Check camera info
        info_msg = self.get_last_message('robotino/camera/camera_info')
        assert info_msg is not None, "No camera info message in queue"
        assert info_msg.header.frame_id == 'camera_link', "Wrong frame_id"
        assert info_msg.height == 480, "Wrong height in camera info"
        assert info_msg.width == 640, "Wrong width in camera info"
    
    def test_camera_update_rate(self):
        """Test that camera monitor publishes at correct rate"""
        self.verify_publishing_rate('robotino/camera/image_raw', 10.0)
        self.verify_publishing_rate('robotino/camera/camera_info', 10.0)
    
    def test_image_content(self):
        """Test that image content is correctly transmitted"""
        assert self.wait_for_message('robotino/camera/image_raw'), \
            "No image message received"
        
        # Convert ROS message back to OpenCV image
        img_msg = self.get_last_message('robotino/camera/image_raw')
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        
        # Basic checks on image
        assert cv_image.shape == (480, 640, 3), "Wrong image dimensions"
        assert cv_image.dtype == np.uint8, "Wrong image data type"
        
        # Verify some pixels are non-zero (due to our test text)
        assert np.any(cv_image > 0), "Image should contain some non-zero pixels"
    
    def test_camera_calibration(self):
        """Test camera calibration parameters"""
        assert self.wait_for_message('robotino/camera/camera_info'), \
            "No camera info message received"
        
        info_msg = self.get_last_message('robotino/camera/camera_info')
        
        # Check camera matrix
        assert len(info_msg.k) == 9, "Camera matrix should have 9 elements"
        assert info_msg.k[0] > 0, "Focal length should be positive"
        assert info_msg.k[4] > 0, "Focal length should be positive"
        assert info_msg.k[2] > 0, "Principal point should be positive"
        assert info_msg.k[5] > 0, "Principal point should be positive"
    
    def test_api_error_handling(self):
        """Test handling of API errors"""
        def raise_error(*args, **kwargs):
            raise Exception("API Error")
        
        self.mock_get.side_effect = raise_error
        
        # Should continue running but log error
        assert self.wait_for_message('robotino/camera/image_raw', timeout=2.0) is False, \
            "Should not receive messages during API error"
        assert self.wait_for_message('robotino/camera/camera_info', timeout=2.0) is False, \
            "Should not receive messages during API error"
    
    def test_invalid_image_data(self):
        """Test handling of invalid image data"""
        # Simulate corrupted JPEG data
        def mock_corrupted_image(*args, **kwargs):
            mock_response = MagicMock()
            if args[0].endswith('/cam0'):
                mock_response.ok = True
                mock_response.content = b'corrupted data'
                return mock_response
            return self.mock_get(*args, **kwargs)
            
        self.mock_get.side_effect = mock_corrupted_image
        
        # Should handle error gracefully
        assert self.wait_for_message('robotino/camera/image_raw', timeout=2.0) is False, \
            "Should not publish corrupted image data"