"""
Nav2 Action Client

Provides high-level navigation using Nav2 action servers.
For use with TurtleBot3 + Nav2 stack (Week 4).
"""

from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from typing import Optional, Callable
from loguru import logger
import time


class Nav2Client(Node):
    """
    Nav2 action client for high-level navigation.

    Uses NavigateToPose action to send navigation goals.
    """

    def __init__(self, node_name: str = 'nav2_voice_client'):
        super().__init__(node_name)

        # Create action client
        self.navigate_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # State
        self.current_goal_handle = None
        self.goal_status = None
        self.result_callbacks: list[Callable] = []

        logger.info(f'Nav2 Client initialized: {node_name}')

    def wait_for_server(self, timeout_sec: float = 5.0) -> bool:
        """Wait for Nav2 action server to be available"""
        logger.info('Waiting for Nav2 action server...')
        return self.navigate_to_pose_client.wait_for_server(timeout_sec=timeout_sec)

    def send_goal(self, x: float, y: float, theta: float = 0.0,
                  frame_id: str = 'map') -> bool:
        """
        Send navigation goal to Nav2

        Args:
            x: Target x position in meters
            y: Target y position in meters
            theta: Target orientation in radians
            frame_id: Reference frame (default: 'map')

        Returns:
            True if goal was accepted, False otherwise
        """
        # Check if action server is available
        if not self.navigate_to_pose_client.server_is_ready():
            logger.error('Nav2 action server not available')
            return False

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Set position
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        # Set orientation (convert theta to quaternion)
        # Simple 2D rotation around z-axis
        import math
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        logger.info(f'Sending Nav2 goal: ({x:.2f}, {y:.2f}, θ={theta:.2f})')

        # Send goal
        send_goal_future = self.navigate_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        send_goal_future.add_done_callback(self.goal_response_callback)

        return True

    def goal_response_callback(self, future):
        """Handle goal response from action server"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            logger.warning('Nav2 goal rejected')
            self.goal_status = GoalStatus.STATUS_ABORTED
            return

        logger.info('Nav2 goal accepted')
        self.current_goal_handle = goal_handle
        self.goal_status = GoalStatus.STATUS_EXECUTING

        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        # feedback contains: current_pose, navigation_time, estimated_time_remaining, etc.
        logger.debug(f'Nav2 feedback: navigation_time={feedback.navigation_time}')

    def result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            logger.info('Nav2 goal succeeded!')
            self.goal_status = GoalStatus.STATUS_SUCCEEDED
        elif status == GoalStatus.STATUS_ABORTED:
            logger.warning('Nav2 goal aborted')
            self.goal_status = GoalStatus.STATUS_ABORTED
        elif status == GoalStatus.STATUS_CANCELED:
            logger.info('Nav2 goal canceled')
            self.goal_status = GoalStatus.STATUS_CANCELED
        else:
            logger.warning(f'Nav2 goal ended with status: {status}')
            self.goal_status = status

        # Notify callbacks
        for callback in self.result_callbacks:
            try:
                callback(status, result)
            except Exception as e:
                logger.error(f'Result callback error: {e}')

    def cancel_goal(self) -> bool:
        """Cancel current navigation goal"""
        if not self.current_goal_handle:
            logger.warning('No active goal to cancel')
            return False

        logger.info('Canceling Nav2 goal...')
        cancel_future = self.current_goal_handle.cancel_goal_async()
        return True

    def get_goal_status(self) -> Optional[int]:
        """Get current goal status"""
        return self.goal_status

    def is_goal_active(self) -> bool:
        """Check if a goal is currently being executed"""
        return self.goal_status == GoalStatus.STATUS_EXECUTING

    def register_result_callback(self, callback: Callable):
        """Register callback for goal results"""
        self.result_callbacks.append(callback)
