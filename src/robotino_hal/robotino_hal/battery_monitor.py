import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import requests
import json

class RobotinoBatteryMonitor(Node):
    def __init__(self):
        super().__init__('robotino_battery_monitor')
        self.battery_publisher = self.create_publisher(
            BatteryState,
            'robotino/battery_state',
            10
        )
        self.timer = self.create_timer(1.0, self.publish_battery_state)
        self.robotino_ip = "10.42.0.232"

    def get_battery_data(self):
        try:
            url = f"http://{self.robotino_ip}/data/powermanagement"
            response = requests.get(url)
            response.raise_for_status()

            data = json.loads(response.text)
            raw_voltage = float(data['voltage'])
            actual_voltage = raw_voltage * 350
            self.get_logger().info(f'Battery Voltage: {actual_voltage}')
            return actual_voltage
        except Exception as e:
            self.get_logger().error(f'Failed to get battery data: {e}')
            return None

    def publish_battery_state(self):
        voltage = self.get_battery_data()
        if voltage is not None:
            msg = BatteryState()
            msg.voltage = voltage
            msg.present = True
            voltage_range = (22.0, 24.5)
            percentage = (voltage - voltage_range[0]) / (voltage_range[1] - voltage_range[0])
            msg.percentage = max(0.0, min(1.0, percentage))
            
            self.battery_publisher.publish(msg)
            self.get_logger().info(f'Battery voltage: {voltage}V, Percentage: {percentage*100:.1f}%')

def main(args=None):
    rclpy.init(args=args)
    node = RobotinoBatteryMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()