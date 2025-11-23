#!/usr/bin/env python3
"""
Nav2 Monitor - Interfaces with Nav2 action server to capture navigation events.

Week 3: Core component for capturing navigation decisions from Nav2 stack.

Features:
- Action client for NavigateToPose
- Callback hooks for feedback, result
- Decision event generation for logging
"""

import time
from typing import Optional, Callable, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus


class Nav2Monitor:
    """
    Monitors Nav2 navigation actions and captures decision points.

    Designed to be used as a component within XAINavigatorNode,
    not as a standalone node.
    """

    def __init__(
        self,
        node: Node,
        decision_callback: Optional[Callable[[Dict[str, Any]], None]] = None
    ):
        """
        Initialize Nav2 monitor.

        Args:
            node: Parent ROS2 node instance
            decision_callback: Function called when navigation events occur
        """
        self.node = node
        self.decision_callback = decision_callback

        # Action client for NavigateToPose
        self.callback_group = ReentrantCallbackGroup()
        self.nav_client = ActionClient(
            self.node,
            NavigateToPose,
            '/navigate_to_pose',
            callback_group=self.callback_group
        )

        # State tracking
        self.current_goal: Optional[PoseStamped] = None
        self.goal_handle = None
        self.goal_start_time: Optional[float] = None
        self.last_feedback_time: Optional[float] = None

        # Decision tracking
        self.decision_counter = 0
        self.active_navigation = False

        self.node.get_logger().info('Nav2Monitor initialized')

    def wait_for_nav2(self, timeout_sec: float = 10.0) -> bool:
        """
        Wait for Nav2 action server to become available.

        Args:
            timeout_sec: Maximum wait time

        Returns:
            True if server available, False otherwise
        """
        self.node.get_logger().info('Waiting for Nav2 action server...')

        available = self.nav_client.wait_for_server(timeout_sec=timeout_sec)

        if available:
            self.node.get_logger().info('Nav2 action server ready')
        else:
            self.node.get_logger().warn(
                f'Nav2 action server not available after {timeout_sec}s'
            )

        return available

    def send_goal(self, pose: PoseStamped) -> bool:
        """
        Send navigation goal to Nav2.

        Args:
            pose: Target pose

        Returns:
            True if goal accepted, False otherwise
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = ''  # Use default BT

        self.node.get_logger().info(
            f'Sending goal: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
        )

        # Send goal asynchronously with feedback callback
        send_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )

        # Wait for acceptance
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=5.0)

        if not send_future.done():
            self.node.get_logger().error('Goal send timeout')
            return False

        self.goal_handle = send_future.result()

        if not self.goal_handle.accepted:
            self.node.get_logger().error('Goal rejected by Nav2')
            return False

        # Track goal
        self.current_goal = pose
        self.goal_start_time = time.time()
        self.active_navigation = True

        # Log decision: goal_sent
        self._emit_decision('goal_sent', {
            'goal_x': pose.pose.position.x,
            'goal_y': pose.pose.position.y,
            'goal_z': pose.pose.position.z,
            'timestamp': self.goal_start_time
        })

        # Get result asynchronously
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

        return True

    def cancel_goal(self):
        """Cancel current navigation goal."""
        if self.goal_handle and self.active_navigation:
            self.node.get_logger().info('Canceling navigation')

            cancel_future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, cancel_future)

            self.active_navigation = False
            self._emit_decision('goal_canceled', {
                'timestamp': time.time(),
                'duration_sec': time.time() - self.goal_start_time if self.goal_start_time else 0
            })

    def _feedback_callback(self, feedback_msg):
        """Process navigation feedback from Nav2."""
        feedback = feedback_msg.feedback
        current_time = time.time()

        # Throttle to max 5Hz (every 200ms)
        if self.last_feedback_time and (current_time - self.last_feedback_time) < 0.2:
            return

        self.last_feedback_time = current_time

        # Extract feedback data
        current_pose = feedback.current_pose.pose
        distance_remaining = feedback.distance_remaining
        nav_time = feedback.navigation_time
        
        # Extract orientation (theta) from quaternion
        import math
        q = current_pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        feedback_data = {
            'current_x': current_pose.position.x,
            'current_y': current_pose.position.y,
            'current_theta': theta, # NEW
            'distance_remaining': distance_remaining,
            'navigation_time_sec': nav_time.sec + nav_time.nanosec * 1e-9,
            'active_behavior': 'follow_path', # Default, would need /controller_server/transition_event for real
            'timestamp': current_time
        }

        self._emit_decision('feedback', feedback_data)

        # Detect approaching goal
        if 0.1 < distance_remaining < 0.5:
            self._emit_decision('approaching_goal', feedback_data)

    def _result_callback(self, future):
        """Process navigation result."""
        result = future.result()
        status = result.status

        self.active_navigation = False

        # Map status to decision type
        status_map = {
            GoalStatus.STATUS_SUCCEEDED: 'goal_reached',
            GoalStatus.STATUS_CANCELED: 'goal_canceled',
            GoalStatus.STATUS_ABORTED: 'goal_aborted'
        }

        decision_type = status_map.get(status, 'goal_unknown')
        duration = time.time() - self.goal_start_time if self.goal_start_time else 0

        result_data = {
            'status': status,
            'status_name': decision_type,
            'duration_sec': duration,
            'timestamp': time.time()
        }

        self.node.get_logger().info(
            f'Navigation {decision_type}: {duration:.2f}s'
        )

        self._emit_decision(decision_type, result_data)

    def _emit_decision(self, decision_type: str, data: Dict[str, Any]):
        """
        Emit a navigation decision event.

        Args:
            decision_type: Type of decision (goal_sent, feedback, goal_reached, etc.)
            data: Decision data dictionary
        """
        self.decision_counter += 1

        decision = {
            'decision_id': self.decision_counter,
            'decision_type': decision_type,
            'data': data,
            'goal': {
                'x': self.current_goal.pose.position.x,
                'y': self.current_goal.pose.position.y
            } if self.current_goal else None,
            'timestamp': data.get('timestamp', time.time())
        }

        # Call registered callback
        if self.decision_callback:
            self.decision_callback(decision)

        self.node.get_logger().debug(
            f'Decision #{self.decision_counter}: {decision_type}'
        )

    def get_status(self) -> Dict[str, Any]:
        """Get current navigation status."""
        return {
            'active': self.active_navigation,
            'has_goal': self.current_goal is not None,
            'decision_count': self.decision_counter,
            'elapsed_sec': time.time() - self.goal_start_time if self.goal_start_time else 0
        }

    @property
    def is_navigating(self) -> bool:
        """Check if navigation is currently active."""
        return self.active_navigation
