#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import requests
import json
from typing import List, Optional

class RobotinoSensorMonitor(Node):
    def __init__(self):
        super().__init__('robotino_sensor_monitor')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robotino_ip', 'localhost'),
                ('robotino_port', 8080),
                ('update_rate', 10.0),
                ('num_sensors', 9),
                ('timeout', 1.0)
            ]
        )
        
        self.robotino_ip = self.get_parameter('robotino_ip').value
        self.robotino_port = self.get_parameter('robotino_port').value
        self.update_rate = self.get_parameter('update_rate').value
        self.num_sensors = self.get_parameter('num_sensors').value
        self.timeout = self.get_parameter('timeout').value
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.sensor_publisher = self.create_publisher(
            Float32MultiArray,
            'robotino/distancesensorarray',
            qos_profile
        )
        
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.publish_sensor_data
        )
        
        self.consecutive_errors = 0
        self.MAX_CONSECUTIVE_ERRORS = 5

    def get_sensor_data(self) -> Optional[List[float]]:
        try:
            url = f"http://{self.robotino_ip}:{self.robotino_port}/data/distancesensorarray"
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            
            data = json.loads(response.text)
            self.consecutive_errors = 0
            return data

        except Exception as e:
            self.get_logger().error(f'Failed to get sensor data: {e}')
            self.consecutive_errors += 1
            if self.consecutive_errors >= self.MAX_CONSECUTIVE_ERRORS:
                self.get_logger().error('Multiple consecutive errors. Check robot connection!')
            return None

    def publish_sensor_data(self) -> None:
        sensor_data = self.get_sensor_data()
        
        if sensor_data is None or len(sensor_data) != self.num_sensors:
            return

        msg = Float32MultiArray()
        msg.data = [float(val) / 1000.0 for val in sensor_data]  # Convert mm to meters
        self.sensor_publisher.publish(msg)

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