#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import requests
import json
from typing import Optional, Dict
import math
from dataclasses import dataclass

@dataclass
class OdometryConfig:
    """Configuration for odometry parameters"""
    frame_id: str = "odom"
    child_frame_id: str = "base_link"
    covariance_linear: float = 0.1
    covariance_angular: float = 0.1

class RobotinoOdometryMonitor(Node):
    """
    ROS 2 node for monitoring Robotino's odometry.
    Publishes odometry data and broadcasts transforms.
    """

    def __init__(self):
        super().__init__('robotino_odometry_monitor')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robotino_ip', '10.42.0.232'),
                ('update_rate', 20.0),  # Hz
                ('timeout', 1.0)        # seconds
            ]
        )
        
        # Get parameters
        self.robotino_ip = self.get_parameter('robotino_ip').value
        self.update_rate = self.get_parameter('update_rate').value
        self.timeout = self.get_parameter('timeout').value
        
        # Odometry configuration
        self.odom_config = OdometryConfig()
        
        # Create publisher
        self.odom_publisher = self.create_publisher(
            Odometry,
            'robotino/odom',
            10
        )
        
        # Initialize transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create timer for periodic updates
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.publish_odometry
        )
        
        self.get_logger().info('Odometry monitor initialized')

    def get_odometry_data(self) -> Optional[Dict]:
        """
        Fetch odometry data from Robotino API.
        
        Returns:
            dict: Dictionary containing odometry data or None if failed
        """
        try:
            url = f"http://{self.robotino_ip}/data/odometry"
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            
            data_list = json.loads(response.text)
            return {
                'x': data_list[0],
                'y': data_list[1],
                'phi': data_list[2],
                'vx': data_list[3],
                'vy': data_list[4],
                'omega': data_list[5]
                # seq is data_list[6] but we don't need it
            }

        except Exception as e:
            self.get_logger().error(f'Failed to get odometry data: {e}')
            return None

    def euler_to_quaternion(self, yaw: float) -> Quaternion:
        """Convert euler angle (yaw) to quaternion."""
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2)
        quat.w = math.cos(yaw / 2)
        return quat

    def publish_odometry(self) -> None:
        """Publish odometry data and broadcast transform."""
        odom_data = self.get_odometry_data()
        
        if odom_data is None:
            return
            
        try:
            # Create odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = self.odom_config.frame_id
            odom_msg.child_frame_id = self.odom_config.child_frame_id
            
            # Set position
            odom_msg.pose.pose.position.x = float(odom_data['x'])
            odom_msg.pose.pose.position.y = float(odom_data['y'])
            odom_msg.pose.pose.position.z = 0.0
            
            # Convert orientation to quaternion
            theta = float(odom_data['phi'])  # assuming phi is orientation in radians
            quat = self.euler_to_quaternion(theta)
            odom_msg.pose.pose.orientation = quat
            
            # Set velocities
            odom_msg.twist.twist.linear.x = float(odom_data['vx'])
            odom_msg.twist.twist.linear.y = float(odom_data['vy'])
            odom_msg.twist.twist.angular.z = float(odom_data['omega'])
            
            # Set covariance
            pos_cov = self.odom_config.covariance_linear
            rot_cov = self.odom_config.covariance_angular
            odom_msg.pose.covariance = [
                pos_cov, 0, 0, 0, 0, 0,
                0, pos_cov, 0, 0, 0, 0,
                0, 0, pos_cov, 0, 0, 0,
                0, 0, 0, rot_cov, 0, 0,
                0, 0, 0, 0, rot_cov, 0,
                0, 0, 0, 0, 0, rot_cov
            ]
            
            # Publish odometry message
            self.odom_publisher.publish(odom_msg)
            
            # Broadcast transform
            transform = TransformStamped()
            transform.header = odom_msg.header
            transform.child_frame_id = odom_msg.child_frame_id
            transform.transform.translation.x = odom_msg.pose.pose.position.x
            transform.transform.translation.y = odom_msg.pose.pose.position.y
            transform.transform.translation.z = odom_msg.pose.pose.position.z
            transform.transform.rotation = quat
            
            self.tf_broadcaster.sendTransform(transform)
            
            self.get_logger().debug(
                f'Published odometry - Position: ({odom_msg.pose.pose.position.x:.2f}, '
                f'{odom_msg.pose.pose.position.y:.2f}), '
                f'Orientation: {math.degrees(theta):.1f}°'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error publishing odometry: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RobotinoOdometryMonitor()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error in odometry monitor: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()