#!/usr/bin/env python3
"""
Behavior Monitor Node

Monitors robot behavior for anomalies without requiring a digital twin.
Detects sudden changes in velocity, unexpected stops, sensor anomalies, etc.

Publishes:
    /twin/sensor_diff - Behavior metrics (for dashboard compatibility)
    /anomaly/score - Anomaly score
    /anomaly/alert - Alert when anomaly detected
"""

import json
import math
import time
from collections import deque
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class BehaviorMonitorNode(Node):
    """Monitors robot behavior for anomalies."""

    def __init__(self):
        super().__init__('behavior_monitor_node')

        # Parameters
        self.declare_parameter('window_size', 50)  # samples for moving average
        self.declare_parameter('velocity_threshold', 0.1)  # m/s change threshold
        self.declare_parameter('scan_threshold', 0.5)  # m change in min distance
        
        self.window_size = self.get_parameter('window_size').value
        self.velocity_threshold = self.get_parameter('velocity_threshold').value
        self.scan_threshold = self.get_parameter('scan_threshold').value

        # State - rolling windows
        self.linear_vel_history = deque(maxlen=self.window_size)
        self.angular_vel_history = deque(maxlen=self.window_size)
        self.min_scan_history = deque(maxlen=self.window_size)
        self.position_history = deque(maxlen=self.window_size)
        
        # Current values
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.current_min_scan = float('inf')
        self.current_x = 0.0
        self.current_y = 0.0
        
        # Anomaly tracking
        self.anomaly_score = 0.0
        self.last_anomaly_time = 0.0

        # QoS
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.sensor_diff_pub = self.create_publisher(String, '/twin/sensor_diff', 10)
        self.anomaly_score_pub = self.create_publisher(Float32, '/anomaly/score', 10)
        self.anomaly_alert_pub = self.create_publisher(String, '/anomaly/alert', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/real/odom', self._odom_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/real/scan', self._scan_callback, sensor_qos
        )

        # Timer for publishing metrics
        self.publish_timer = self.create_timer(0.1, self._publish_metrics)  # 10 Hz

        self.get_logger().info("Behavior Monitor Node initialized")
        self.get_logger().info(f"Window size: {self.window_size}, Vel threshold: {self.velocity_threshold}")

    def _odom_callback(self, msg: Odometry):
        """Process odometry data."""
        self.current_linear_vel = msg.twist.twist.linear.x
        self.current_angular_vel = msg.twist.twist.angular.z
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        self.linear_vel_history.append(self.current_linear_vel)
        self.angular_vel_history.append(self.current_angular_vel)
        self.position_history.append((self.current_x, self.current_y))

    def _scan_callback(self, msg: LaserScan):
        """Process laser scan data."""
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid_ranges:
            self.current_min_scan = min(valid_ranges)
            self.min_scan_history.append(self.current_min_scan)

    def _compute_metrics(self) -> dict:
        """Compute behavior metrics."""
        metrics = {
            'position_diff_x': self.current_x,
            'position_diff_y': self.current_y,
            'position_diff_total': math.sqrt(self.current_x**2 + self.current_y**2),
            'orientation_diff': 0.0,
            'linear_vel_diff': self.current_linear_vel,
            'angular_vel_diff': self.current_angular_vel,
            'scan_diff_mean': self.current_min_scan if self.current_min_scan != float('inf') else 0.0,
            'scan_diff_max': 0.0,
            'scan_diff_variance': 0.0,
        }
        
        # Compute velocity variance (as indicator of erratic behavior)
        if len(self.linear_vel_history) > 5:
            vel_list = list(self.linear_vel_history)
            mean_vel = sum(vel_list) / len(vel_list)
            variance = sum((v - mean_vel)**2 for v in vel_list) / len(vel_list)
            metrics['scan_diff_variance'] = variance  # Reusing field for velocity variance
            
        # Compute distance traveled
        if len(self.position_history) > 1:
            positions = list(self.position_history)
            total_dist = 0.0
            for i in range(1, len(positions)):
                dx = positions[i][0] - positions[i-1][0]
                dy = positions[i][1] - positions[i-1][1]
                total_dist += math.sqrt(dx**2 + dy**2)
            metrics['scan_diff_max'] = total_dist  # Reusing field for distance traveled
        
        return metrics

    def _compute_anomaly_score(self, metrics: dict) -> float:
        """Compute anomaly score based on behavior patterns."""
        score = 0.0
        
        # Check for sudden velocity changes
        if len(self.linear_vel_history) >= 10:
            recent = list(self.linear_vel_history)[-5:]
            older = list(self.linear_vel_history)[-10:-5]
            
            recent_avg = sum(recent) / len(recent)
            older_avg = sum(older) / len(older)
            
            vel_change = abs(recent_avg - older_avg)
            if vel_change > self.velocity_threshold:
                score -= 0.3 * (vel_change / self.velocity_threshold)
        
        # Check for obstacle proximity
        if self.current_min_scan < 0.3:  # Very close to obstacle
            score -= 0.5
        elif self.current_min_scan < 0.5:
            score -= 0.2
        
        # Check for spinning (high angular velocity)
        if abs(self.current_angular_vel) > 1.5:
            score -= 0.2
        
        # Normalize score
        return max(-2.0, min(0.5, score))

    def _publish_metrics(self):
        """Publish metrics to dashboard."""
        metrics = self._compute_metrics()
        self.anomaly_score = self._compute_anomaly_score(metrics)
        
        # Publish sensor diff (for dashboard)
        diff_msg = String()
        diff_msg.data = json.dumps(metrics)
        self.sensor_diff_pub.publish(diff_msg)
        
        # Publish anomaly score
        score_msg = Float32()
        score_msg.data = self.anomaly_score
        self.anomaly_score_pub.publish(score_msg)
        
        # Publish alert if anomaly detected
        if self.anomaly_score < -0.5 and (time.time() - self.last_anomaly_time) > 2.0:
            self.last_anomaly_time = time.time()
            alert = {
                'is_anomaly': True,
                'score': self.anomaly_score,
                'threshold': -0.5,
                'severity': 2 if self.anomaly_score < -1.0 else 1,
                'explanation': self._generate_explanation(metrics),
                'recommended_action': 'Check robot surroundings' if self.current_min_scan < 0.5 else 'Monitor behavior',
                'features': metrics
            }
            alert_msg = String()
            alert_msg.data = json.dumps(alert)
            self.anomaly_alert_pub.publish(alert_msg)
            self.get_logger().warn(f"ANOMALY: {alert['explanation']}")

    def _generate_explanation(self, metrics: dict) -> str:
        """Generate human-readable explanation."""
        parts = []
        
        if self.current_min_scan < 0.3:
            parts.append(f"Very close to obstacle ({self.current_min_scan:.2f}m)")
        elif self.current_min_scan < 0.5:
            parts.append(f"Approaching obstacle ({self.current_min_scan:.2f}m)")
        
        if abs(self.current_angular_vel) > 1.5:
            parts.append(f"High rotation speed ({self.current_angular_vel:.2f} rad/s)")
        
        if len(self.linear_vel_history) >= 10:
            recent = list(self.linear_vel_history)[-5:]
            older = list(self.linear_vel_history)[-10:-5]
            vel_change = abs(sum(recent)/len(recent) - sum(older)/len(older))
            if vel_change > self.velocity_threshold:
                parts.append(f"Sudden velocity change ({vel_change:.2f} m/s)")
        
        return "; ".join(parts) if parts else "Behavior anomaly detected"


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
