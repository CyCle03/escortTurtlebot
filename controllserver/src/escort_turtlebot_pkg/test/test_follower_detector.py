from escort_turtlebot_pkg.follower_detector_node import FollowerDetectorNode
import pytest
import rclpy


@pytest.fixture
def follower_detector_node():
    rclpy.init()
    # Use default parameters for leader/follower names ('TB3_1', 'TB3_2')
    node = FollowerDetectorNode()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_node_creation(follower_detector_node):
    """Test that the FollowerDetectorNode can be created."""
    assert follower_detector_node is not None


def test_subscriptions(follower_detector_node):
    """Test that the node subscribes to the correct scan topics."""
    # The node is initialized with default parameters
    leader_name = 'TB3_1'
    follower_name = 'TB3_2'

    # Subscriptions are created with f-strings, so we can check the names
    leader_scan_sub = follower_detector_node.subscription1
    follower_scan_sub = follower_detector_node.subscription2

    assert leader_scan_sub is not None, 'Leader scan subscription is missing'
    assert follower_scan_sub is not None, 'Follower scan subscription is missing'

    expected_leader_topic = f'/{leader_name}/scan'
    expected_follower_topic = f'/{follower_name}/scan'

    assert leader_scan_sub.topic_name == expected_leader_topic
    assert follower_scan_sub.topic_name == expected_follower_topic
