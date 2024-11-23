import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import requests
import json

class RobotinoSensorMonitor(Node):
    def __init__(self):
        super().__init__('robotino_sensor_monitor')
        self.sensor_publisher = self.create_publisher(
            Range,
            'robotino/sensor_state',
            10
        )
        self.robotino_ip = "10.42.0.232"

    def get_sensor_data(self):
        try:
            # TODO Confirm the url when i'm in the lab
            url = f"http://{self.robotino_ip}/data/distance_sensors"
            response = requests.get(url)
            response.raise_for_status()

            data = json.loads(response.text)
        except Exception as e:
            self.get_logger().error(f'Failed to get sensor data: {e}')
            return None
    