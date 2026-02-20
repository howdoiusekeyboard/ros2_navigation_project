"""
Robot Controller using ROS2 (rclpy)

Provides low-level robot control via ROS2 topics.
Publishes Twist messages for velocity control.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import threading
from loguru import logger
from typing import Optional, Callable
import time
from app.ros2_client.nav2_client import Nav2Client


class RobotController(Node):
    """
    ROS2 node for robot velocity control and state monitoring.

    Publishes to:
    - /cmd_vel (geometry_msgs/Twist) - Velocity commands

    Subscribes to:
    - /odom (nav_msgs/Odometry) - Robot odometry
    - /scan (sensor_msgs/LaserScan) - Laser scan data
    """

    def __init__(self, node_name: str = 'voice_control_backend'):
        super().__init__(node_name)

        # QoS profiles
        self.qos_profile = QoSProfile(depth=10)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',  # For TurtleBot3
            self.qos_profile
        )

        # Alternative publisher for turtlesim
        self.turtle_cmd_vel_publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',  # For turtlesim demo
            self.qos_profile
        )

        # Subscribers
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            self.qos_profile
        )

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            self.qos_profile
        )

        # State
        self.current_odom: Optional[Odometry] = None
        self.current_scan: Optional[LaserScan] = None
        self.last_command_time: float = 0.0

        # Callbacks
        self.odom_callbacks: list[Callable] = []
        self.scan_callbacks: list[Callable] = []

        logger.info(f'ROS2 Robot Controller initialized: {node_name}')

    def odom_callback(self, msg: Odometry):
        """Handle odometry updates"""
        self.current_odom = msg
        for callback in self.odom_callbacks:
            try:
                callback(msg)
            except Exception as e:
                logger.error(f'Odometry callback error: {e}')

    def scan_callback(self, msg: LaserScan):
        """Handle laser scan updates"""
        self.current_scan = msg
        for callback in self.scan_callbacks:
            try:
                callback(msg)
            except Exception as e:
                logger.error(f'Scan callback error: {e}')

    def publish_twist(self, linear_x: float = 0.0, linear_y: float = 0.0,
                     linear_z: float = 0.0, angular_x: float = 0.0,
                     angular_y: float = 0.0, angular_z: float = 0.0,
                     use_turtlesim: bool = False):
        """
        Publish velocity command to robot

        Args:
            linear_x: Linear velocity in x (forward/backward) m/s
            linear_y: Linear velocity in y (left/right) m/s
            linear_z: Linear velocity in z (up/down) m/s
            angular_x: Angular velocity around x (roll) rad/s
            angular_y: Angular velocity around y (pitch) rad/s
            angular_z: Angular velocity around z (yaw/rotation) rad/s
            use_turtlesim: If True, publish to /turtle1/cmd_vel instead
        """
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.linear.y = float(linear_y)
        twist.linear.z = float(linear_z)
        twist.angular.x = float(angular_x)
        twist.angular.y = float(angular_y)
        twist.angular.z = float(angular_z)

        # Publish to appropriate topic
        if use_turtlesim:
            self.turtle_cmd_vel_publisher.publish(twist)
            topic = '/turtle1/cmd_vel'
        else:
            self.cmd_vel_publisher.publish(twist)
            topic = '/cmd_vel'

        self.last_command_time = time.time()

        logger.debug(
            f'Published Twist to {topic}: '
            f'linear=({linear_x:.2f}, {linear_y:.2f}, {linear_z:.2f}), '
            f'angular=({angular_x:.2f}, {angular_y:.2f}, {angular_z:.2f})'
        )

    def stop(self, use_turtlesim: bool = False):
        """Emergency stop - all velocities to zero"""
        self.publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, use_turtlesim)
        logger.info('Emergency stop executed')

    def move_forward(self, speed: float = 0.5, use_turtlesim: bool = False):
        """Move forward at specified speed"""
        self.publish_twist(linear_x=speed, use_turtlesim=use_turtlesim)

    def move_backward(self, speed: float = 0.5, use_turtlesim: bool = False):
        """Move backward at specified speed"""
        self.publish_twist(linear_x=-speed, use_turtlesim=use_turtlesim)

    def rotate(self, angular_speed: float = 0.5, use_turtlesim: bool = False):
        """Rotate at specified angular speed (positive = counter-clockwise)"""
        self.publish_twist(angular_z=angular_speed, use_turtlesim=use_turtlesim)

    def move_circular(self, linear_speed: float = 0.5, angular_speed: float = 0.3,
                     use_turtlesim: bool = False):
        """Move in a circle with specified linear and angular speeds"""
        self.publish_twist(
            linear_x=linear_speed,
            angular_z=angular_speed,
            use_turtlesim=use_turtlesim
        )

    def get_current_pose(self) -> Optional[dict]:
        """Get current robot pose from odometry"""
        if not self.current_odom:
            return None

        pose = self.current_odom.pose.pose
        return {
            'x': pose.position.x,
            'y': pose.position.y,
            'z': pose.position.z,
            'orientation': {
                'x': pose.orientation.x,
                'y': pose.orientation.y,
                'z': pose.orientation.z,
                'w': pose.orientation.w
            }
        }

    def get_current_velocity(self) -> Optional[dict]:
        """Get current robot velocity from odometry"""
        if not self.current_odom:
            return None

        twist = self.current_odom.twist.twist
        return {
            'linear': {
                'x': twist.linear.x,
                'y': twist.linear.y,
                'z': twist.linear.z
            },
            'angular': {
                'x': twist.angular.x,
                'y': twist.angular.y,
                'z': twist.angular.z
            }
        }

    def register_odom_callback(self, callback: Callable):
        """Register callback for odometry updates"""
        self.odom_callbacks.append(callback)

    def register_scan_callback(self, callback: Callable):
        """Register callback for laser scan updates"""
        self.scan_callbacks.append(callback)


class ROS2Manager:
    """
    Manager for ROS2 node lifecycle.
    Runs ROS2 spin in a separate thread to not block FastAPI.
    """

    def __init__(self):
        self.robot_controller: Optional[RobotController] = None
        self.nav2_client: Optional[Nav2Client] = None
        self._executor: Optional[MultiThreadedExecutor] = None
        self.spin_thread: Optional[threading.Thread] = None
        self.running = False

    def initialize(self):
        """Initialize ROS2 and create robot controller node"""
        try:
            # Initialize rclpy if not already done
            if not rclpy.ok():
                rclpy.init()

            # Create robot controller node
            self.robot_controller = RobotController()
            
            # Create Nav2 action client node (for NavigateToPose)
            # Safe even if Nav2 is not running; server availability is checked at call time.
            self.nav2_client = Nav2Client()
            
            # Multi-node executor so action futures progress correctly
            self._executor = MultiThreadedExecutor(num_threads=2)
            self._executor.add_node(self.robot_controller)
            self._executor.add_node(self.nav2_client)

            # Start spinning in background thread
            self.running = True
            self.spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
            self.spin_thread.start()

            logger.info('ROS2 Manager initialized successfully')

        except Exception as e:
            logger.error(f'Failed to initialize ROS2 Manager: {e}')
            raise

    def _spin_loop(self):
        """Spin ROS2 node in background thread"""
        logger.info('ROS2 spin loop started')
        while self.running and rclpy.ok():
            try:
                if self._executor is not None:
                    self._executor.spin_once(timeout_sec=0.1)
                else:
                    # Fallback (should not happen): spin robot controller only
                    rclpy.spin_once(self.robot_controller, timeout_sec=0.1)
            except Exception as e:
                logger.error(f'ROS2 spin error: {e}')
                if not self.running:
                    break
        logger.info('ROS2 spin loop stopped')

    def shutdown(self):
        """Shutdown ROS2 gracefully"""
        logger.info('Shutting down ROS2 Manager...')
        self.running = False

        if self.spin_thread:
            self.spin_thread.join(timeout=2.0)

        if self._executor:
            try:
                if self.nav2_client:
                    self._executor.remove_node(self.nav2_client)
                if self.robot_controller:
                    self._executor.remove_node(self.robot_controller)
            except Exception:
                pass

        if self.nav2_client:
            self.nav2_client.destroy_node()

        if self.robot_controller:
            self.robot_controller.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        logger.info('ROS2 Manager shut down complete')

    def get_controller(self) -> RobotController:
        """Get robot controller instance"""
        if not self.robot_controller:
            raise RuntimeError('ROS2 Manager not initialized')
        return self.robot_controller
    
    def get_nav2_client(self) -> Nav2Client:
        """Get Nav2 client instance (NavigateToPose)."""
        if not self.nav2_client:
            raise RuntimeError('ROS2 Manager not initialized')
        return self.nav2_client


# Global singleton instance
ros2_manager = ROS2Manager()
