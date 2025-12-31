#!/usr/bin/env python3
"""
Real Robot Bridge Node

Purpose:
- Subscribe to "physical robot" topics (e.g. /odom, /scan, /cmd_vel)
- Republish them into a dedicated namespace (default: /real/*)

Why:
- The digital twin monitor expects /real/* and /twin/* topics.
- On a physical TurtleBot3, topics are usually un-namespaced (/odom, /scan, /cmd_vel).
- This node avoids changing anything on the robot; it only mirrors data locally.
"""

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class RealRobotBridgeNode(Node):
    def __init__(self):
        super().__init__("real_robot_bridge_node")

        # Inputs (physical robot)
        self.declare_parameter("odom_in", "/odom")
        self.declare_parameter("scan_in", "/scan")
        self.declare_parameter("cmd_vel_in", "/cmd_vel")

        # Outputs (namespaced)
        self.declare_parameter("namespace_out", "/real")
        self.declare_parameter("republish_cmd_vel", True)

        odom_in = str(self.get_parameter("odom_in").value)
        scan_in = str(self.get_parameter("scan_in").value)
        cmd_vel_in = str(self.get_parameter("cmd_vel_in").value)
        ns_out = str(self.get_parameter("namespace_out").value).rstrip("/")
        republish_cmd_vel = bool(self.get_parameter("republish_cmd_vel").value)

        if not ns_out.startswith("/"):
            ns_out = "/" + ns_out

        self.odom_out = f"{ns_out}/odom"
        self.scan_out = f"{ns_out}/scan"
        self.cmd_vel_out = f"{ns_out}/cmd_vel"

        # QoS profile for sensor data (BEST_EFFORT to match TurtleBot3 lidar)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, self.odom_out, 10)
        self.scan_pub = self.create_publisher(LaserScan, self.scan_out, sensor_qos)
        self.cmd_vel_pub: Optional[rclpy.publisher.Publisher] = None
        if republish_cmd_vel:
            self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_out, 10)

        # Subscribers - use sensor_qos for /scan to match TurtleBot3's BEST_EFFORT
        self.odom_sub = self.create_subscription(Odometry, odom_in, self._odom_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_in, self._scan_cb, sensor_qos)
        self.cmd_vel_sub = None
        if republish_cmd_vel:
            self.cmd_vel_sub = self.create_subscription(Twist, cmd_vel_in, self._cmd_vel_cb, 10)

        self.get_logger().info("RealRobotBridgeNode started")
        self.get_logger().info(f"ODOM: {odom_in} -> {self.odom_out}")
        self.get_logger().info(f"SCAN: {scan_in} -> {self.scan_out}")
        if republish_cmd_vel:
            self.get_logger().info(f"CMD_VEL: {cmd_vel_in} -> {self.cmd_vel_out}")
        else:
            self.get_logger().info("CMD_VEL republish disabled")

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom_pub.publish(msg)

    def _scan_cb(self, msg: LaserScan) -> None:
        self.scan_pub.publish(msg)

    def _cmd_vel_cb(self, msg: Twist) -> None:
        if self.cmd_vel_pub is not None:
            self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RealRobotBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
