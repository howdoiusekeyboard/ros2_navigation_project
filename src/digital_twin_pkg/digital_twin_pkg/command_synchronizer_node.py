#!/usr/bin/env python3
"""
Command Synchronizer Node

Forwards velocity commands to both real and digital twin robots
to ensure synchronized movement for comparison.

Week 1 Day 4: Basic command forwarding
Week 4-5: Will add latency simulation and noise injection for testing
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
import time


class CommandSynchronizerNode(Node):
    """
    Synchronizes commands between real robot and digital twin.

    Subscriptions:
        /cmd_vel (Twist): Primary command input

    Publications:
        /real/cmd_vel (Twist): Command to real robot
        /twin/cmd_vel (Twist): Command to digital twin
        /sync/status (String): Synchronization status info
    """

    def __init__(self):
        super().__init__('command_synchronizer_node')

        # Parameters
        self.declare_parameter('sync_delay', 0.0)  # Delay in seconds
        self.declare_parameter('enable_noise', False)  # Add noise to twin
        self.declare_parameter('noise_factor', 0.05)  # 5% noise
        self.declare_parameter('log_commands', True)

        self.sync_delay = self.get_parameter('sync_delay').value
        self.enable_noise = self.get_parameter('enable_noise').value
        self.noise_factor = self.get_parameter('noise_factor').value
        self.log_commands = self.get_parameter('log_commands').value

        # Statistics
        self.command_count = 0
        self.last_command_time = None

        # Publishers
        self.real_cmd_pub = self.create_publisher(
            Twist, '/real/cmd_vel', 10
        )
        self.twin_cmd_pub = self.create_publisher(
            Twist, '/twin/cmd_vel', 10
        )
        self.status_pub = self.create_publisher(
            String, '/sync/status', 10
        )

        # Subscriber
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10
        )

        # Status timer (publish sync stats every 5 seconds)
        self.status_timer = self.create_timer(5.0, self.publish_status)

        self.get_logger().info('Command Synchronizer Node initialized')
        self.get_logger().info(f'Sync delay: {self.sync_delay}s')
        self.get_logger().info(f'Noise enabled: {self.enable_noise}')

    def cmd_callback(self, msg: Twist):
        """
        Handle incoming velocity command.
        Forward to both real and twin robots.
        """
        self.command_count += 1
        current_time = time.time()

        # Forward to REAL robot (no modification)
        self.real_cmd_pub.publish(msg)

        # Forward to TWIN robot (potentially with delay/noise for testing)
        twin_cmd = self._process_twin_command(msg)

        if self.sync_delay > 0:
            # Simple blocking delay (Week 4-5: use timer for non-blocking)
            time.sleep(self.sync_delay)

        self.twin_cmd_pub.publish(twin_cmd)

        # Logging
        if self.log_commands:
            self.get_logger().debug(
                f'Cmd #{self.command_count}: linear={msg.linear.x:.3f}, '
                f'angular={msg.angular.z:.3f}'
            )

        self.last_command_time = current_time

    def _process_twin_command(self, original_cmd: Twist) -> Twist:
        """
        Process command for digital twin.
        Can add noise or modifications for testing anomaly detection.
        """
        twin_cmd = Twist()

        if self.enable_noise:
            # Add small noise to simulate imperfect synchronization
            import random
            noise_linear = random.gauss(0, self.noise_factor * abs(original_cmd.linear.x))
            noise_angular = random.gauss(0, self.noise_factor * abs(original_cmd.angular.z))

            twin_cmd.linear.x = original_cmd.linear.x + noise_linear
            twin_cmd.linear.y = original_cmd.linear.y
            twin_cmd.linear.z = original_cmd.linear.z

            twin_cmd.angular.x = original_cmd.angular.x
            twin_cmd.angular.y = original_cmd.angular.y
            twin_cmd.angular.z = original_cmd.angular.z + noise_angular
        else:
            # Perfect copy
            twin_cmd.linear.x = original_cmd.linear.x
            twin_cmd.linear.y = original_cmd.linear.y
            twin_cmd.linear.z = original_cmd.linear.z

            twin_cmd.angular.x = original_cmd.angular.x
            twin_cmd.angular.y = original_cmd.angular.y
            twin_cmd.angular.z = original_cmd.angular.z

        return twin_cmd

    def publish_status(self):
        """Publish synchronization status information."""
        status = {
            'total_commands': self.command_count,
            'sync_delay_ms': self.sync_delay * 1000,
            'noise_enabled': self.enable_noise,
            'noise_factor': self.noise_factor if self.enable_noise else 0,
            'last_command_age': (
                time.time() - self.last_command_time
                if self.last_command_time else None
            ),
        }

        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

        if self.command_count > 0 and self.command_count % 100 == 0:
            self.get_logger().info(
                f'Synchronized {self.command_count} commands'
            )


def main(args=None):
    rclpy.init(args=args)
    node = CommandSynchronizerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
