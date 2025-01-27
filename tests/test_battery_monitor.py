import pytest
from robotino_hal.nodes.battery_monitor import RobotinoBatteryMonitor, BatteryConfig
from sensor_msgs.msg import BatteryState
import rclpy

def test_battery_config():
    """Test battery configuration"""
    config = BatteryConfig()
    assert config.min_voltage == 22.0
    assert config.max_voltage == 24.5
    assert config.voltage_scale == 350.0
    assert config.critical_percentage == 0.15
    assert config.warning_percentage == 0.25

def test_calculate_battery_percentage():
    """Test battery percentage calculation"""
    rclpy.init()
    node = RobotinoBatteryMonitor()
    
    # Test normal range
    assert abs(node.calculate_battery_percentage(23.25) - 0.5) < 0.01  # Middle voltage
    assert abs(node.calculate_battery_percentage(24.5) - 1.0) < 0.01   # Max voltage
    assert abs(node.calculate_battery_percentage(22.0) - 0.0) < 0.01   # Min voltage
    
    # Test out of bounds
    assert abs(node.calculate_battery_percentage(25.0) - 1.0) < 0.01   # Above max
    assert abs(node.calculate_battery_percentage(21.0) - 0.0) < 0.01   # Below min
    
    node.destroy_node()
    rclpy.shutdown()

def test_create_battery_message():
    """Test battery message creation"""
    rclpy.init()
    node = RobotinoBatteryMonitor()
    
    # Test with a normal voltage
    msg = node.create_battery_message(23.25)
    assert isinstance(msg, BatteryState)
    assert abs(msg.voltage - 23.25) < 0.01
    assert abs(msg.percentage - 0.5) < 0.01
    assert msg.present == True
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    pytest.main(['-v', __file__])