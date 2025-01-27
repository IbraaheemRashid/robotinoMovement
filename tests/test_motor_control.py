import pytest
from unittest.mock import MagicMock, patch
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from robotino_hal.nodes.motor_control import RobotinoMotorController

@pytest.fixture
def rclpy_init_shutdown():
    """Fixture for initializing and shutting down rclpy"""
    rclpy.init()
    yield
    rclpy.shutdown()

def test_node_parameters(rclpy_init_shutdown):
    """Test initialization of node parameters"""
    node = RobotinoMotorController()

    assert node.robotino_ip == '10.42.0.232'
    assert node.limits.max_linear_speed == 0.8
    assert node.limits.max_angular_speed == 1.5
    assert node.limits.max_acceleration == 0.5

    node.destroy_node()

def test_velocity_clamping(rclpy_init_shutdown):
    """Test velocity clamping"""
    node = RobotinoMotorController()

    # Velocity within limits
    cmd = Twist()
    cmd.linear.x = 0.5
    cmd.linear.y = 0.5
    cmd.angular.z = 1.0
    clamped = node.clamp_velocity(cmd)
    assert clamped.linear.x == 0.5
    assert clamped.linear.y == 0.5
    assert clamped.angular.z == 1.0

    # Exceeding linear limits
    cmd.linear.x = 2.0
    cmd.linear.y = 2.0
    clamped = node.clamp_velocity(cmd)
    assert (clamped.linear.x**2 + clamped.linear.y**2)**0.5 <= node.limits.max_linear_speed

    # Exceeding angular limit
    cmd.angular.z = 2.0
    clamped = node.clamp_velocity(cmd)
    assert clamped.angular.z == node.limits.max_angular_speed

    node.destroy_node()

def test_emergency_stop_behavior(rclpy_init_shutdown):
    """Test emergency stop behavior"""
    node = RobotinoMotorController()

    # Mock the stop_motors method
    node.stop_motors = MagicMock()

    # Activate emergency stop
    stop_msg = Bool(data=True)
    node.emergency_stop_callback(stop_msg)
    assert node.emergency_stop is True
    node.stop_motors.assert_called_once()

    # Release emergency stop
    stop_msg.data = False
    node.emergency_stop_callback(stop_msg)
    assert node.emergency_stop is False

    node.destroy_node()

@patch('robotino_hal.nodes.motor_control.requests.post', autospec=True)
def test_set_motor_velocity(mock_post, rclpy_init_shutdown):
    """Test setting motor velocity via API"""
    node = RobotinoMotorController()
    
    # Mock successful API response
    mock_post.return_value.status_code = 200
    mock_post.return_value.raise_for_status.return_value = None

    velocity = Twist()
    velocity.linear.x = 0.5
    velocity.linear.y = 0.3
    velocity.angular.z = 0.1

    # Test successful call
    success = node.set_motor_velocity(velocity)
    assert success is True
    mock_post.assert_called_once_with(
        f"http://{node.robotino_ip}/data/omnidrive",
        json=[0.5, 0.3, 0.1]  # No timeout
    )

    # Test API failure
    mock_post.reset_mock()
    mock_post.side_effect = Exception('API error')
    success = node.set_motor_velocity(velocity)
    assert success is False
    mock_post.assert_called_once()

    node.destroy_node()


@patch('robotino_hal.nodes.motor_control.RobotinoMotorController.set_motor_velocity', autospec=True)
def test_control_loop(mock_set_motor_velocity, rclpy_init_shutdown):
    """Test control loop functionality"""
    node = RobotinoMotorController()
    mock_set_motor_velocity.return_value = True  # Mock successful velocity setting

    # Set a target velocity
    target_velocity = Twist()
    target_velocity.linear.x = 0.4
    target_velocity.linear.y = 0.3
    target_velocity.angular.z = 0.2
    node.target_velocity = target_velocity

    # Manually call control_loop instead of relying on the timer
    node.control_loop()

    # Check if current_velocity was updated towards target_velocity
    assert node.current_velocity.linear.x > 0
    assert node.current_velocity.linear.x <= node.target_velocity.linear.x
    assert node.current_velocity.linear.y > 0
    assert node.current_velocity.linear.y <= node.target_velocity.linear.y
    assert node.current_velocity.angular.z > 0
    assert node.current_velocity.angular.z <= node.target_velocity.angular.z

    # Verify the mock was called
    mock_set_motor_velocity.assert_called_once()

    node.destroy_node()


if __name__ == '__main__':
    pytest.main(['-v', __file__])
