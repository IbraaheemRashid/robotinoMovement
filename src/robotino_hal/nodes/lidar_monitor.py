#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import requests
import json
from typing import Optional, List
from dataclasses import dataclass
import math

@dataclass
class LiDARConfig:
    """Configuration for LiDAR parameters"""
    min_angle: float = -2.356194  # -135 degrees in radians
    max_angle: float = 2.356194   # 135 degrees in radians
    angle_increment: float = 0.017453  # 1 degree in radians
    time_increment: float = 0.0
    scan_time: float = 0.1
    min_range: float = 0.05  # 5cm
    max_range: float = 5.0   # 5m
    frame_id: str = "laser_link"

class RobotinoLiDARMonitor(Node):
    """
    ROS 2 node for monitoring Robotino's LiDAR.
    Publishes LaserScan messages containing LiDAR data.
    """

    def __init__(self):
        super().__init__('robotino_lidar_monitor')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robotino_ip', '10.42.0.232'),
                ('update_rate', 10.0),  # Hz
                ('timeout', 1.0)        # seconds
            ]
        )
        
        # Get parameters
        self.robotino_ip = self.get_parameter('robotino_ip').value
        self.update_rate = self.get_parameter('update_rate').value
        self.timeout = self.get_parameter('timeout').value
        
        # LiDAR configuration
        self.lidar_config = LiDARConfig()
        
        # QoS profile for LiDAR data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create publisher
        self.scan_publisher = self.create_publisher(
            LaserScan,
            'robotino/scan',
            qos_profile
        )
        
        # Create timer for periodic updates
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.publish_scan_data
        )
        
        # Error tracking
        self.consecutive_errors = 0
        self.MAX_CONSECUTIVE_ERRORS = 5
        
        self.get_logger().info('LiDAR monitor initialized')

    def get_lidar_data(self) -> Optional[List[float]]:
        """
        Fetch LiDAR data from Robotino API.
        
        Returns:
            List[float]: List of range measurements if successful, None otherwise
        """
        try:
            url = f"http://{self.robotino_ip}/data/scan0"
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            
            data = json.loads(response.text)
            
            # Update LiDAR configuration from received data
            self.lidar_config.min_angle = float(data['angle_min'])
            self.lidar_config.max_angle = float(data['angle_max'])
            self.lidar_config.angle_increment = float(data['angle_increment'])
            self.lidar_config.time_increment = float(data['time_increment'])
            self.lidar_config.scan_time = float(data['scan_time'])
            self.lidar_config.min_range = float(data['range_min'])
            self.lidar_config.max_range = float(data['range_max'])
            
            self.consecutive_errors = 0
            return data['ranges']

        except requests.Timeout:
            self.get_logger().warn('Timeout while fetching LiDAR data')
        except requests.RequestException as e:
            self.get_logger().error(f'HTTP request failed: {e}')
        except Exception as e:
            self.get_logger().error(f'Failed to get LiDAR data: {e}')
            
        self.consecutive_errors += 1
        if self.consecutive_errors >= self.MAX_CONSECUTIVE_ERRORS:
            self.get_logger().error('Multiple consecutive errors. Check robot connection!')
        return None

    def create_laser_scan_msg(self, ranges: List[float]) -> LaserScan:
        """Create a LaserScan message from range data."""
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.lidar_config.frame_id
        
        msg.angle_min = self.lidar_config.min_angle
        msg.angle_max = self.lidar_config.max_angle
        msg.angle_increment = self.lidar_config.angle_increment
        msg.time_increment = self.lidar_config.time_increment
        msg.scan_time = self.lidar_config.scan_time
        msg.range_min = self.lidar_config.min_range
        msg.range_max = self.lidar_config.max_range
        
        msg.ranges = ranges
        
        # Calculate intensities if available (set to empty if not)
        msg.intensities = []
        
        return msg

    def publish_scan_data(self) -> None:
        """Publish LiDAR scan data."""
        scan_data = self.get_lidar_data()
        
        if scan_data is None:
            return
            
        try:
            # Data is already in the correct format from the API
            ranges = [float(val) for val in scan_data]
            
            # Create and publish laser scan message
            msg = self.create_laser_scan_msg(ranges)
            self.scan_publisher.publish(msg)
            
            self.get_logger().debug(
                f'Published LiDAR scan with {len(ranges)} points. '
                f'Range: [{min(ranges):.2f}, {max(ranges):.2f}] meters'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error publishing scan data: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RobotinoLiDARMonitor()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error in LiDAR monitor: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()