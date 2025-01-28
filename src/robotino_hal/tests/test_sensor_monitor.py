import pytest
from unittest.mock import MagicMock, patch
import rclpy
from std_msgs.msg import Float32MultiArray
from robotino_hal.nodes.sensor_monitor import RobotinoSensorMonitor
@pytest.fixture
def rclpy_init_shutdown():
    """Fixture for initializing and shutting down rclpy"""
    rclpy.init()
    yield
    rclpy.shutdown()

def test_node_parameters(rclpy_init_shutdown):
    """Test initialization of node parameters"""
    node = RobotinoSensorMonitor()

    assert node.robotino_ip == '10.42.0.232' or 'localhost:8081'
    assert node.update_rate == 10.0
    assert node.num_sensors == 9
    assert node.timeout == 1.0

    node.destroy_node()

@patch('robotino_hal.nodes.sensor_monitor.requests.get', autospec=True)
def test_get_sensor_data_success(mock_get, rclpy_init_shutdown):
    """Test successful retrieval of sensor data"""
    node = RobotinoSensorMonitor()

    # Mock successful API response
    mock_response = MagicMock()
    mock_response.status_code = 200
    mock_response.text = '[1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000]'
    mock_get.return_value = mock_response

    sensor_data = node.get_sensor_data()
    assert sensor_data == [1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000]
    assert node.consecutive_errors == 0

    node.destroy_node()

@patch('robotino_hal.nodes.sensor_monitor.requests.get', autospec=True)
def test_get_sensor_data_failure(mock_get, rclpy_init_shutdown):
    """Test handling of failed sensor data retrieval"""
    node = RobotinoSensorMonitor()

    # Mock API failure
    mock_get.side_effect = Exception('API error')

    sensor_data = node.get_sensor_data()
    assert sensor_data is None
    assert node.consecutive_errors == 1

    # Test multiple consecutive errors
    for _ in range(4):
        sensor_data = node.get_sensor_data()
    assert node.consecutive_errors == 5

    node.destroy_node()

@patch('robotino_hal.nodes.sensor_monitor.RobotinoSensorMonitor.get_sensor_data', autospec=True)
def test_publish_sensor_data_success(mock_get_sensor_data, rclpy_init_shutdown):
    """Test successful publishing of sensor data"""
    node = RobotinoSensorMonitor()

    # Mock sensor data
    mock_get_sensor_data.return_value = [1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000]

    # Mock the publisher
    mock_publisher = MagicMock()
    node.sensor_publisher = mock_publisher

    # Call the publish_sensor_data method
    node.publish_sensor_data()

    # Verify the message was published
    mock_publisher.publish.assert_called_once()
    published_msg = mock_publisher.publish.call_args[0][0]
    assert isinstance(published_msg, Float32MultiArray)
    assert list(published_msg.data) == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]
    node.destroy_node()

@patch('robotino_hal.nodes.sensor_monitor.RobotinoSensorMonitor.get_sensor_data', autospec=True)
def test_publish_sensor_data_failure(mock_get_sensor_data, rclpy_init_shutdown):
    """Test handling of failed sensor data retrieval in publish_sensor_data"""
    node = RobotinoSensorMonitor()

    # Mock failed sensor data retrieval
    mock_get_sensor_data.return_value = None

    # Mock the publisher
    mock_publisher = MagicMock()
    node.sensor_publisher = mock_publisher

    # Call the publish_sensor_data method
    node.publish_sensor_data()

    # Verify no message was published
    mock_publisher.publish.assert_not_called()

    node.destroy_node()

@patch('robotino_hal.nodes.sensor_monitor.RobotinoSensorMonitor.get_sensor_data', autospec=True)
def test_publish_sensor_data_invalid_length(mock_get_sensor_data, rclpy_init_shutdown):
    """Test handling of invalid sensor data length in publish_sensor_data"""
    node = RobotinoSensorMonitor()

    # Mock sensor data with incorrect length
    mock_get_sensor_data.return_value = [1000, 2000, 3000]  # Only 3 values instead of 9

    # Mock the publisher
    mock_publisher = MagicMock()
    node.sensor_publisher = mock_publisher

    # Call the publish_sensor_data method
    node.publish_sensor_data()

    # Verify no message was published
    mock_publisher.publish.assert_not_called()

    node.destroy_node()

if __name__ == '__main__':
    pytest.main(['-v', __file__])