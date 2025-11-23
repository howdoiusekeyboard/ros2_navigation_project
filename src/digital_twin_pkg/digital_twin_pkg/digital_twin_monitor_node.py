#!/usr/bin/env python3
"""
Digital Twin Monitor Node

Compares real robot to digital twin simulation for anomaly detection.
Uses Isolation Forest for unsupervised anomaly scoring.

Week 5-6 will add ML model training and SHAP explanations.
Week 1: Skeleton with sensor comparison and feature extraction.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import json
import math


class DigitalTwinMonitorNode(Node):
    """
    Monitors real robot vs digital twin for anomaly detection.

    Subscriptions:
        /real/odom (Odometry): Real robot odometry
        /twin/odom (Odometry): Digital twin odometry
        /real/scan (LaserScan): Real robot laser scan
        /twin/scan (LaserScan): Digital twin laser scan

    Publications:
        /anomaly/score (Float32): Current anomaly score
        /anomaly/alert (String): JSON alert with explanation
        /twin/sensor_diff (String): JSON sensor comparison data
    """

    def __init__(self):
        super().__init__('digital_twin_monitor_node')

        # Parameters
        self.declare_parameter('anomaly_threshold', -0.5)
        self.declare_parameter('update_rate', 10.0)  # Hz

        self.anomaly_threshold = self.get_parameter('anomaly_threshold').value
        self.update_rate = self.get_parameter('update_rate').value

        # State
        self.real_odom = None
        self.twin_odom = None
        self.real_scan = None
        self.twin_scan = None
        self.anomaly_model = None  # Will be loaded in Week 6

        # Publishers
        self.anomaly_score_pub = self.create_publisher(
            Float32, '/anomaly/score', 10
        )
        self.anomaly_alert_pub = self.create_publisher(
            String, '/anomaly/alert', 10
        )
        self.sensor_diff_pub = self.create_publisher(
            String, '/twin/sensor_diff', 10
        )

        # Subscribers
        self.real_odom_sub = self.create_subscription(
            Odometry, '/real/odom', self.real_odom_callback, 10
        )
        self.twin_odom_sub = self.create_subscription(
            Odometry, '/twin/odom', self.twin_odom_callback, 10
        )
        self.real_scan_sub = self.create_subscription(
            LaserScan, '/real/scan', self.real_scan_callback, 10
        )
        self.twin_scan_sub = self.create_subscription(
            LaserScan, '/twin/scan', self.twin_scan_callback, 10
        )

        # Timer for periodic comparison
        self.comparison_timer = self.create_timer(
            1.0 / self.update_rate, self.compare_sensors
        )

        self.get_logger().info('Digital Twin Monitor Node initialized')
        self.get_logger().info(f'Anomaly threshold: {self.anomaly_threshold}')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz')

    def real_odom_callback(self, msg: Odometry):
        """Store real robot odometry."""
        self.real_odom = msg

    def twin_odom_callback(self, msg: Odometry):
        """Store digital twin odometry."""
        self.twin_odom = msg

    def real_scan_callback(self, msg: LaserScan):
        """Store real robot laser scan."""
        self.real_scan = msg

    def twin_scan_callback(self, msg: LaserScan):
        """Store digital twin laser scan."""
        self.twin_scan = msg

    def compare_sensors(self):
        """
        Compare real vs twin sensor data and compute anomaly score.
        Called periodically by timer.
        """
        if not self.real_odom or not self.twin_odom:
            return

        # Extract features
        features = self._extract_features()

        # Publish sensor differences
        self._publish_sensor_diff(features)

        # Compute anomaly score (Week 6: use ML model)
        anomaly_score = self._compute_anomaly_score(features)

        # Publish score
        score_msg = Float32()
        score_msg.data = anomaly_score
        self.anomaly_score_pub.publish(score_msg)

        # Check if anomaly
        if anomaly_score < self.anomaly_threshold:
            self._publish_anomaly_alert(anomaly_score, features)

    def _extract_features(self) -> dict:
        """Extract comparison features for anomaly detection."""
        features = {}

        # Position differences
        if self.real_odom and self.twin_odom:
            real_pos = self.real_odom.pose.pose.position
            twin_pos = self.twin_odom.pose.pose.position

            features['position_diff_x'] = real_pos.x - twin_pos.x
            features['position_diff_y'] = real_pos.y - twin_pos.y
            features['position_diff_total'] = math.sqrt(
                features['position_diff_x']**2 + features['position_diff_y']**2
            )

            # Orientation difference (simplified - just z component)
            real_orient = self.real_odom.pose.pose.orientation
            twin_orient = self.twin_odom.pose.pose.orientation
            # Approximate theta from quaternion (assumes 2D)
            real_theta = 2 * math.atan2(real_orient.z, real_orient.w)
            twin_theta = 2 * math.atan2(twin_orient.z, twin_orient.w)
            features['orientation_diff'] = real_theta - twin_theta

            # Velocity differences
            real_vel = self.real_odom.twist.twist
            twin_vel = self.twin_odom.twist.twist
            features['linear_vel_diff'] = real_vel.linear.x - twin_vel.linear.x
            features['angular_vel_diff'] = real_vel.angular.z - twin_vel.angular.z

        # Laser scan differences
        if self.real_scan and self.twin_scan:
            scan_diffs = []
            min_len = min(len(self.real_scan.ranges), len(self.twin_scan.ranges))
            for i in range(min_len):
                real_r = self.real_scan.ranges[i]
                twin_r = self.twin_scan.ranges[i]
                # Handle inf values
                if math.isinf(real_r) or math.isinf(twin_r):
                    scan_diffs.append(0.0)
                else:
                    scan_diffs.append(abs(real_r - twin_r))

            if scan_diffs:
                features['scan_diff_mean'] = sum(scan_diffs) / len(scan_diffs)
                features['scan_diff_max'] = max(scan_diffs)
                mean = features['scan_diff_mean']
                variance = sum((x - mean)**2 for x in scan_diffs) / len(scan_diffs)
                features['scan_diff_variance'] = variance

        return features

    def _compute_anomaly_score(self, features: dict) -> float:
        """
        Compute anomaly score from features.

        Week 6: Use trained Isolation Forest model.
        Week 1: Simple threshold-based scoring.
        """
        # Simple scoring based on position deviation
        # Will be replaced with ML model in Week 6

        if 'position_diff_total' not in features:
            return 0.0

        pos_diff = features.get('position_diff_total', 0.0)
        vel_diff = abs(features.get('linear_vel_diff', 0.0))
        scan_diff = features.get('scan_diff_mean', 0.0)

        # Simple weighted score (negative = more anomalous)
        # Threshold-based scoring
        score = 0.0

        # Position deviation penalty
        if pos_diff > 0.5:  # More than 50cm drift
            score -= 1.0
        elif pos_diff > 0.2:
            score -= 0.5
        else:
            score += 0.5

        # Velocity mismatch penalty
        if vel_diff > 0.1:
            score -= 0.3

        # Scan difference penalty
        if scan_diff > 0.3:
            score -= 0.2

        return score

    def _publish_sensor_diff(self, features: dict):
        """Publish sensor comparison data as JSON."""
        msg = String()
        msg.data = json.dumps(features, default=str)
        self.sensor_diff_pub.publish(msg)

    def _publish_anomaly_alert(self, score: float, features: dict):
        """Publish anomaly alert with explanation."""
        alert = {
            'is_anomaly': True,
            'score': score,
            'threshold': self.anomaly_threshold,
            'features': features,
            'explanation': self._generate_anomaly_explanation(features),
            'severity': self._compute_severity(score),
            'recommended_action': self._recommend_action(score)
        }

        msg = String()
        msg.data = json.dumps(alert, default=str)
        self.anomaly_alert_pub.publish(msg)

        self.get_logger().warn(f'ANOMALY DETECTED: score={score:.3f}')

    def _generate_anomaly_explanation(self, features: dict) -> str:
        """
        Generate human-readable explanation for anomaly.

        Week 6: Use SHAP for feature importance.
        Week 1: Simple feature-based explanation.
        """
        explanation_parts = []

        pos_diff = features.get('position_diff_total', 0.0)
        if pos_diff > 0.2:
            explanation_parts.append(f"Position drift: {pos_diff:.3f}m")

        vel_diff = abs(features.get('linear_vel_diff', 0.0))
        if vel_diff > 0.05:
            explanation_parts.append(f"Velocity mismatch: {vel_diff:.3f}m/s")

        scan_diff = features.get('scan_diff_mean', 0.0)
        if scan_diff > 0.2:
            explanation_parts.append(f"Sensor difference: {scan_diff:.3f}m")

        if explanation_parts:
            return "Anomaly caused by: " + ", ".join(explanation_parts)
        else:
            return "Unknown anomaly source"

    def _compute_severity(self, score: float) -> int:
        """Compute severity level (0=info, 1=warning, 2=critical)."""
        if score < -1.0:
            return 2  # Critical
        elif score < -0.5:
            return 1  # Warning
        else:
            return 0  # Info

    def _recommend_action(self, score: float) -> str:
        """Suggest action based on severity."""
        if score < -1.0:
            return "STOP robot and investigate immediately"
        elif score < -0.5:
            return "Monitor closely, consider recalibration"
        else:
            return "Continue monitoring"


def main(args=None):
    rclpy.init(args=args)
    node = DigitalTwinMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
