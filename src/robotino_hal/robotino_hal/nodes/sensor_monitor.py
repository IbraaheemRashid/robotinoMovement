#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import requests
import json
from typing import List, Optional, Dict
from dataclasses import dataclass
from functools import partial

@dataclass
class SensorConfig:
    """Configuration parameters for distance sensors"""
    field_of_view: float = 0.26  # radians (~15 degrees)
    min_range: float = 0.04      # meters
    max_range: float = 0.40      # meters
    frame_prefix: str = "distance_sensor_"

class RobotinoSensorMonitor(Node):
    """
    ROS 2 node for monitoring Robotino's distance sensors.
    Publishes infrared sensor readings as Range messages.
    """

    def __init__(self):
        super().__init__('robotino_sensor_monitor')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robotino_ip', '10.42.0.232'),
                ('update_rate', 10.0),        # Hz
                ('num_sensors', 9),
                ('timeout', 1.0),             # seconds
                ('sensor_offset_angle', 40.0)  # degrees between sensors
            ]
        )
        
        # Get parameters
        self.robotino_ip = self.get_parameter('robotino_ip').value
        self.update_rate = self.get_parameter('update_rate').value
        self.num_sensors = self.get_parameter('num_sensors').value
        self.timeout = self.get_parameter('timeout').value
        
        # QoS profile for sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Initialize sensor configuration
        self.sensor_config = SensorConfig()
        
        # Create publishers for each sensor
        self.sensor_publishers: List[rclpy.publisher.Publisher] = []
        self.create_sensor_publishers(qos_profile)
        
        # Create timer for periodic publishing
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.publish_sensor_data
        )
        
        # Diagnostic data
        self.consecutive_errors = 0
        self.MAX_CONSECUTIVE_ERRORS = 5
        
        self.get_logger().info(
            f'Initialized sensor monitor with {self.num_sensors} sensors '
            f'at {self.update_rate}Hz'
        )

    def create_sensor_publishers(self, qos_profile: QoSProfile) -> None:
        """Create publishers for all sensors with specified QoS."""
        for i in range(self.num_sensors):
            publisher = self.create_publisher(
                Range,
                f'robotino/distance_sensor_{i}',
                qos_profile
            )
            self.sensor_publishers.append(publisher)

    def get_sensor_data(self) -> Optional[List[float]]:
        """
        Fetch sensor data from Robotino API.
        
        Returns:
            List[float]: List of sensor readings or None if failed
        """
        try:
            url = f"http://{self.robotino_ip}/data/distance_sensors"
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            
            data = json.loads(response.text)
            self.consecutive_errors = 0  # Reset error counter on success
            return data

        except requests.Timeout:
            self.get_logger().warn('Timeout while fetching sensor data')
        except requests.RequestException as e:
            self.get_logger().error(f'HTTP request failed: {e}')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse sensor data: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')
        
        self.consecutive_errors += 1
        if self.consecutive_errors >= self.MAX_CONSECUTIVE_ERRORS:
            self.get_logger().error(
                'Multiple consecutive errors. Check robot connection!'
            )
        return None

    def create_range_message(self, sensor_id: int, raw_value: float) -> Range:
        """Create a Range message for a sensor reading."""
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.sensor_config.frame_prefix}{sensor_id}'
        
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = self.sensor_config.field_of_view
        msg.min_range = self.sensor_config.min_range
        msg.max_range = self.sensor_config.max_range
        
        # Convert raw value to meters
        msg.range = float(raw_value) / 1000.0
        
        # Validate range
        if not self.sensor_config.min_range <= msg.range <= self.sensor_config.max_range:
            self.get_logger().warn(
                f'Sensor {sensor_id} reading {msg.range}m outside valid range'
            )
            
        return msg

    def publish_sensor_data(self) -> None:
        """Publish data from all sensors."""
        sensor_data = self.get_sensor_data()
        
        if sensor_data is None:
            return
            
        if len(sensor_data) != self.num_sensors:
            self.get_logger().error(
                f'Received {len(sensor_data)} readings, expected {self.num_sensors}'
            )
            return

        # Publish each sensor reading
        for i, raw_value in enumerate(sensor_data):
            try:
                msg = self.create_range_message(i, raw_value)
                self.sensor_publishers[i].publish(msg)
                self.get_logger().debug(f'Sensor {i}: {msg.range:.3f}m')
            except Exception as e:
                self.get_logger().error(f'Error publishing sensor {i}: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RobotinoSensorMonitor()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error in sensor monitor: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()