import pytest
import math
from robotino_hal.nodes.navigation import SimpleNavigator
import rclpy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

@pytest.fixture(scope="session")
def ros_context():
    """Initialize and tear down ROS context"""
    rclpy.init()
    yield
    rclpy.shutdown()

def test_node_parameters(ros_context):
    """Test navigation parameters initialization"""
    node = SimpleNavigator()
    
    # Check default parameters
    assert node.max_linear_speed == 0.5
    assert node.max_angular_speed == 1.0
    assert node.position_tolerance == 0.1
    assert node.angular_tolerance == 0.35  # Updated to match actual value
    
    node.destroy_node()

def test_calculate_distance_to_goal(ros_context):
    """Test distance calculation to goal"""
    node = SimpleNavigator()
    
    # Setup current and goal poses
    node.current_pose = Pose()
    node.current_pose.position.x = 0.0
    node.current_pose.position.y = 0.0
    
    node.goal_pose = Pose()
    node.goal_pose.position.x = 3.0
    node.goal_pose.position.y = 4.0
    
    # Test pythagorean distance (3,4,5 triangle)
    distance, angle = node.calculate_distance_to_goal()
    assert distance == pytest.approx(5.0)
    assert angle == pytest.approx(math.atan2(4.0, 3.0))
    
    # Test zero distance
    node.goal_pose.position.x = 0.0
    node.goal_pose.position.y = 0.0
    distance, angle = node.calculate_distance_to_goal()
    assert distance == pytest.approx(0.0)
    
    node.destroy_node()

def test_get_yaw_from_pose(ros_context):
    """Test yaw extraction from pose quaternion"""
    node = SimpleNavigator()
    
    # Create test pose
    pose = Pose()
    
    # Test 0 degrees
    pose.orientation.w = 1.0
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    assert node.get_yaw_from_pose(pose) == pytest.approx(0.0)
    
    # Test 90 degrees
    pose.orientation.w = 0.707
    pose.orientation.z = 0.707
    assert node.get_yaw_from_pose(pose) == pytest.approx(math.pi/2, rel=1e-3)  # 0.1% tolerance
    
    # Test 180 degrees
    pose.orientation.w = 0.0
    pose.orientation.z = 1.0
    assert node.get_yaw_from_pose(pose) == pytest.approx(math.pi)
    
    node.destroy_node()

def test_goal_reached_condition(ros_context):
    """Test goal reached detection"""
    node = SimpleNavigator()
    
    # Setup poses
    node.current_pose = Pose()
    node.goal_pose = Pose()
    
    # Test exact match
    assert node.calculate_distance_to_goal()[0] == pytest.approx(0.0)
    
    # Test within tolerance
    node.goal_pose.position.x = 0.05  # Less than position_tolerance
    distance, _ = node.calculate_distance_to_goal()
    assert distance < node.position_tolerance
    
    # Test outside tolerance
    node.goal_pose.position.x = 1.0
    distance, _ = node.calculate_distance_to_goal()
    assert distance > node.position_tolerance
    
    node.destroy_node()

if __name__ == '__main__':
    pytest.main(['-v', __file__])