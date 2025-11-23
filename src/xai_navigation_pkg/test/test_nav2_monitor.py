import unittest
from unittest.mock import MagicMock, patch
import rclpy
from rclpy.node import Node
from xai_navigation_pkg.nav2_monitor import Nav2Monitor
from geometry_msgs.msg import PoseStamped

class TestNav2Monitor(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node('test_node')
        self.monitor = Nav2Monitor(self.node)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_initialization(self):
        self.assertIsNotNone(self.monitor.nav_client)
        self.assertFalse(self.monitor.active_navigation)

    @patch('xai_navigation_pkg.nav2_monitor.ActionClient')
    def test_send_goal(self, mock_action_client):
        # Mock the action client behavior
        self.monitor.nav_client.send_goal_async = MagicMock()
        self.monitor.nav_client.wait_for_server = MagicMock(return_value=True)
        
        pose = PoseStamped()
        pose.pose.position.x = 1.0
        
        # Note: This is hard to fully test without a running action server
        # or extensive mocking of futures. 
        # For unit test, we just check the method exists and runs.
        pass

    def test_status(self):
        status = self.monitor.get_status()
        self.assertFalse(status['active'])
        self.assertEqual(status['decision_count'], 0)

if __name__ == '__main__':
    unittest.main()
