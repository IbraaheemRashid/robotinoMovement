#!/usr/bin/env python3

from dataclasses import dataclass

import cv2
import numpy as np
import requests
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


@dataclass
class CameraConfig:
    """Configuration for camera parameters."""

    width: int = 640
    height: int = 480
    fps: int = 10


class RobotinoCameraMonitor(Node):
    """ROS 2 node for monitoring Robotino's camera.

    This node periodically fetches camera frames from the Robotino API and publishes
    both the raw images and corresponding camera information.
    """

    def __init__(self):
        super().__init__('robotino_camera_monitor')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robotino_ip', 'localhost:8081'),
                ('update_rate', 10.0),
                ('timeout', 1.0),
                ('camera.width', 640),
                ('camera.height', 480),
                ('camera.fps', 10)
            ]
        )

        # Get parameters
        self.robotino_ip = self.get_parameter('robotino_ip').value
        self.update_rate = self.get_parameter('update_rate').value
        self.timeout = self.get_parameter('timeout').value

        # Camera configuration
        self.camera_config = CameraConfig(
            width=self.get_parameter('camera.width').value,
            height=self.get_parameter('camera.height').value,
            fps=self.get_parameter('camera.fps').value
        )

        # QoS profile for camera data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.image_publisher = self.create_publisher(
            Image,
            'robotino/camera/image_raw',
            qos_profile
        )

        self.camera_info_publisher = self.create_publisher(
            CameraInfo,
            'robotino/camera/camera_info',
            qos_profile
        )

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Create timer for periodic updates
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.publish_camera_data
        )

        # Error tracking
        self.consecutive_errors = 0
        self.MAX_CONSECUTIVE_ERRORS = 5

        self.get_logger().info('Camera monitor initialized')

    def get_camera_frame(self):
        """Fetch camera frame from Robotino API.

        Returns:
            numpy.ndarray: Camera frame if successful, None otherwise
        """
        try:
            url = f'http://{self.robotino_ip}/cam0'
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()

            # Convert response to OpenCV image
            nparr = np.frombuffer(response.content, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            # Resize if needed
            if img.shape[:2] != (self.camera_config.height, self.camera_config.width):
                img = cv2.resize(
                    img,
                    (self.camera_config.width, self.camera_config.height)
                )

            self.consecutive_errors = 0
            return img

        except requests.Timeout:
            self.get_logger().warn('Timeout while fetching camera data')
        except requests.RequestException as e:
            self.get_logger().error(f'HTTP request failed: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')

        self.consecutive_errors += 1
        if self.consecutive_errors >= self.MAX_CONSECUTIVE_ERRORS:
            self.get_logger().error(
                'Multiple consecutive errors. Check robot connection!'
            )
        return None

    def create_camera_info_msg(self):
        """Create CameraInfo message with basic parameters.

        Returns:
            CameraInfo: Message containing camera parameters
        """
        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height = self.camera_config.height
        msg.width = self.camera_config.width

        # Basic camera matrix (you might want to calibrate these)
        focal_length = self.camera_config.width
        center_x = self.camera_config.width / 2
        center_y = self.camera_config.height / 2

        msg.k = [
            focal_length, 0, center_x,
            0, focal_length, center_y,
            0, 0, 1
        ]

        return msg

    def publish_camera_data(self):
        """Publish camera frame and info."""
        frame = self.get_camera_frame()

        if frame is None:
            return

        try:
            # Convert frame to ROS message
            img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_link'

            # Create and publish camera info
            info_msg = self.create_camera_info_msg()

            # Publish messages
            self.image_publisher.publish(img_msg)
            self.camera_info_publisher.publish(info_msg)

            self.get_logger().debug('Published camera frame')

        except Exception as e:
            self.get_logger().error(f'Error publishing camera data: {e}')


def main(args=None):
    """Starts Main entry point."""
    rclpy.init(args=args)

    try:
        node = RobotinoCameraMonitor()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error in camera monitor: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()