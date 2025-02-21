#!/usr/bin/env python3

import json
from typing import List, Optional

import requests
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray


class RobotinoSensorMonitor(Node):
    """ROS 2 node for monitoring Robotino's distance sensors.

    This node fetches data from the Robotino's distance sensor array and
    publishes it as ROS messages. The sensor values are converted from
    millimeters to meters before publishing.
    """

    def __init__(self):
        super().__init__('robotino_sensor_monitor')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robotino_ip', 'localhost'),
                ('update_rate', 10.0),
                ('num_sensors', 9),
                ('timeout', 1.0)
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

        # Create publisher
        self.sensor_publisher = self.create_publisher(
            Float32MultiArray,
            'robotino/distancesensorarray',
            qos_profile
        )

        # Create timer for periodic updates
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.publish_sensor_data
        )

        # Error tracking
        self.consecutive_errors = 0
        self.MAX_CONSECUTIVE_ERRORS = 5

        self.get_logger().info(
            f'Sensor monitor initialized with update rate: {self.update_rate}Hz'
        )

    def get_sensor_data(self) -> Optional[List[float]]:
        """Fetch sensor data from Robotino API.

        Returns:
            List of sensor values in millimeters if successful, None otherwise
        """
        try:
            url = f'http://{self.robotino_ip}/data/distancesensorarray'
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()

            data = json.loads(response.text)
            self.consecutive_errors = 0
            return data

        except Exception as e:
            self.get_logger().error(f'Failed to get sensor data: {e}')
            self.consecutive_errors += 1
            if self.consecutive_errors >= self.MAX_CONSECUTIVE_ERRORS:
                self.get_logger().error(
                    'Multiple consecutive errors. Check robot connection!'
                )
            return None

    def publish_sensor_data(self) -> None:
        """Publish sensor data as Float32MultiArray message.

        The sensor values are converted from millimeters to meters before
        publishing. If the data is invalid or missing, no message is published.
        """
        sensor_data = self.get_sensor_data()
        if sensor_data is None or len(sensor_data) != self.num_sensors:
            return

        # Convert mm to meters and create message
        msg = Float32MultiArray()
        msg.data = [float(val) / 1000.0 for val in sensor_data]
        self.sensor_publisher.publish(msg)

        self.get_logger().debug(
            f'Published {len(msg.data)} sensor values'
        )


def main(args=None) -> None:
    """Main entry point."""
    rclpy.init(args=args)

    try:
        node = RobotinoSensorMonitor()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error in sensor monitor: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()