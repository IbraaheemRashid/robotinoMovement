#!/usr/bin/env python3

import json
from math import cos, sin
from typing import List, Optional, Tuple

import requests
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (Point, Quaternion, TransformStamped,
                             Vector3)
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class RobotinoOdometryMonitor(Node):
    """ROS 2 node for monitoring Robotino's odometry.

    This node fetches odometry data from the Robotino API and publishes it as
    ROS odometry messages and TF transforms.
    """

    def __init__(self):
        super().__init__('robotino_odometry_monitor')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robotino_ip', 'localhost'),
                ('update_rate', 20.0),
                ('timeout', 1.0),
                ('odom.frame_id', 'odom'),
                ('odom.child_frame_id', 'base_footprint'),
                ('odom.covariance_linear', 0.1),
                ('odom.covariance_angular', 0.1)
            ]
        )

        self.robotino_ip = self.get_parameter('robotino_ip').value
        self.update_rate = self.get_parameter('update_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.frame_id = self.get_parameter('odom.frame_id').value
        self.child_frame_id = self.get_parameter('odom.child_frame_id').value
        self.covariance_linear = float(self.get_parameter('odom.covariance_linear').value)
        self.covariance_angular = float(self.get_parameter('odom.covariance_angular').value)

        # Publishers
        self.odom_publisher = self.create_publisher(
            Odometry,
            'odom',
            10
        )

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.publish_odometry
        )

        # Error tracking
        self.consecutive_errors = 0
        self.MAX_CONSECUTIVE_ERRORS = 5

        self.get_logger().info(
            f'Odometry monitor initialized with update rate: {self.update_rate}Hz'
        )

    def get_odometry_data(self) -> Optional[List[float]]:
        """Fetch odometry data from Robotino API.

        Returns:
            List of odometry values [x, y, phi, vx, vy, omega, seq] if successful,
            None otherwise.
        """
        try:
            url = f'http://{self.robotino_ip}/data/odometry'
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()

            data = json.loads(response.text)
            if len(data) != 7: 
                raise ValueError('Invalid odometry data format: '
                               f'expected 7 values, got {len(data)}')

            self.consecutive_errors = 0
            return data

        except Exception as e:
            self.get_logger().error(f'Failed to get odometry data: {e}')
            self.consecutive_errors += 1
            if self.consecutive_errors >= self.MAX_CONSECUTIVE_ERRORS:
                self.get_logger().error(
                    'Multiple consecutive errors. Check robot connection!'
                )
            return None

    def euler_to_quaternion(self, yaw: float) -> Quaternion:
        """Convert euler angle to quaternion.

        Args:
            yaw: The yaw angle in radians

        Returns:
            A quaternion representing the rotation
        """
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = sin(yaw / 2)
        quat.w = cos(yaw / 2)
        return quat

    def create_odometry_message(
        self,
        data: List[float]
    ) -> Tuple[Odometry, TransformStamped]:
        """Create odometry and transform messages.

        Args:
            data: List of odometry values [x, y, phi, vx, vy, omega, seq]

        Returns:
            Tuple of (Odometry message, TransformStamped message)
        """
        current_time = self.get_clock().now().to_msg()

        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        # Set position
        odom.pose.pose.position = Point(x=float(data[0]), y=float(data[1]), z=0.0)

        # Set orientation
        quat = self.euler_to_quaternion(float(data[2]))
        odom.pose.pose.orientation = quat

        # Set velocities
        odom.twist.twist.linear = Vector3(
            x=float(data[3]),
            y=float(data[4]),
            z=0.0
        )
        odom.twist.twist.angular = Vector3(x=0.0, y=0.0, z=float(data[5]))

        # Set covariance matrices (6x6)
        pose_covariance = [
            float(self.covariance_linear), 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, float(self.covariance_linear), 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, float(self.covariance_linear), 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, float(self.covariance_angular), 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, float(self.covariance_angular), 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, float(self.covariance_angular)
        ]
        odom.pose.covariance = pose_covariance
        odom.twist.covariance = pose_covariance

        # Create transform message
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = quat

        return odom, transform

    def publish_odometry(self) -> None:
        """Publish odometry data and broadcast transform."""
        data = self.get_odometry_data()
        if not data:
            return

        try:
            odom_msg, transform_msg = self.create_odometry_message(data)

            # Publish messages
            self.odom_publisher.publish(odom_msg)
            self.tf_broadcaster.sendTransform(transform_msg)

            self.get_logger().debug(
                'Published odometry - '
                f'Position: ({odom_msg.pose.pose.position.x:.2f}, '
                f'{odom_msg.pose.pose.position.y:.2f}), '
                f'Velocity: ({odom_msg.twist.twist.linear.x:.2f}, '
                f'{odom_msg.twist.twist.linear.y:.2f})'
            )

        except Exception as e:
            self.get_logger().error(f'Error publishing odometry: {e}')


def main(args=None) -> None:
    """Main entry point."""
    rclpy.init(args=args)

    try:
        node = RobotinoOdometryMonitor()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error in odometry monitor: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()