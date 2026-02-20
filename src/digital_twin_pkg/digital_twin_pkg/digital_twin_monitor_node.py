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
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import json
import math
import os
import pickle
import time
from typing import Any, Dict, List, Optional

try:
    import numpy as np
except Exception:  # pragma: no cover
    np = None

try:
    from sklearn.ensemble import IsolationForest
except Exception:  # pragma: no cover
    IsolationForest = None

try:
    import shap
except Exception:  # pragma: no cover
    shap = None


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
        self.declare_parameter('model_path', '~/.ros/digital_twin/anomaly_model.pkl')
        self.declare_parameter('training_mode', False)
        self.declare_parameter('baseline_samples', 600)  # 60s @ 10Hz
        self.declare_parameter('shap_enabled', True)
        self.declare_parameter('shap_min_interval_sec', 2.0)  # rate-limit SHAP

        self.anomaly_threshold = self.get_parameter('anomaly_threshold').value
        self.update_rate = self.get_parameter('update_rate').value
        self.model_path = os.path.expanduser(self.get_parameter('model_path').value)
        self.training_mode = bool(self.get_parameter('training_mode').value)
        self.baseline_samples = int(self.get_parameter('baseline_samples').value)
        self.shap_enabled = bool(self.get_parameter('shap_enabled').value)
        self.shap_min_interval_sec = float(self.get_parameter('shap_min_interval_sec').value)

        # State
        self.real_odom = None
        self.twin_odom = None
        self.real_scan = None
        self.twin_scan = None
        self.anomaly_model: Optional[Any] = None
        self._feature_names = [
            "position_diff_x",
            "position_diff_y",
            "position_diff_total",
            "orientation_diff",
            "linear_vel_diff",
            "angular_vel_diff",
            "scan_diff_mean",
            "scan_diff_max",
            "scan_diff_variance",
        ]
        self._training_buffer: List[List[float]] = []
        self._shap_explainer = None
        self._last_shap_time = 0.0

        # Try load trained model (optional; node still works without it)
        self._try_load_model()

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

        # QoS profile for sensor data (BEST_EFFORT to match TurtleBot3 lidar)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.real_odom_sub = self.create_subscription(
            Odometry, '/real/odom', self.real_odom_callback, 10
        )
        self.twin_odom_sub = self.create_subscription(
            Odometry, '/twin/odom', self.twin_odom_callback, 10
        )
        # Use BEST_EFFORT QoS for scan topics to match TurtleBot3 lidar
        self.real_scan_sub = self.create_subscription(
            LaserScan, '/real/scan', self.real_scan_callback, sensor_qos
        )
        self.twin_scan_sub = self.create_subscription(
            LaserScan, '/twin/scan', self.twin_scan_callback, sensor_qos
        )

        # Timer for periodic comparison
        self.comparison_timer = self.create_timer(
            1.0 / self.update_rate, self.compare_sensors
        )

        self.get_logger().info('Digital Twin Monitor Node initialized')
        self.get_logger().info(f'Anomaly threshold: {self.anomaly_threshold}')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz')
        self.get_logger().info(f'Training mode: {self.training_mode} (baseline_samples={self.baseline_samples})')
        self.get_logger().info(f'Model path: {self.model_path}')
        if self.anomaly_model is None:
            self.get_logger().warn('No anomaly model loaded. Using heuristic scoring until model is trained/available.')
        if self.shap_enabled and shap is None:
            self.get_logger().warn('SHAP not installed; anomaly explanations will be heuristic-only.')
        if IsolationForest is None:
            self.get_logger().warn('scikit-learn not installed; ML training/loading disabled.')

    def _try_load_model(self) -> None:
        """Load persisted IsolationForest model from disk if available."""
        try:
            # Prevent path traversal and arbitrary file load
            safe_base = os.path.abspath(os.path.expanduser('~/.ros/digital_twin'))
            resolved_path = os.path.abspath(self.model_path)
            if not resolved_path.startswith(safe_base):
                self.get_logger().error("Invalid model path: Path traversal detected")
                return

            if not os.path.exists(resolved_path):
                return

            with open(resolved_path, "rb") as f:
                payload = pickle.load(f)

            # Accept either raw model or payload dict
            if isinstance(payload, dict) and "model" in payload:
                self.anomaly_model = payload["model"]
                feature_names = payload.get("feature_names")
                if isinstance(feature_names, list) and feature_names:
                    self._feature_names = feature_names
            else:
                self.anomaly_model = payload

            self.get_logger().info("Loaded anomaly model from disk")
        except Exception as e:
            self.get_logger().error(f"Failed to load anomaly model: {e}")
            self.anomaly_model = None

    def _persist_model(self) -> None:
        """Persist trained model to disk."""
        try:
            os.makedirs(os.path.dirname(self.model_path), exist_ok=True)
            with open(self.model_path, "wb") as f:
                pickle.dump(
                    {"model": self.anomaly_model, "feature_names": self._feature_names, "created_at": time.time()},
                    f,
                )
            self.get_logger().info(f"Saved anomaly model to {self.model_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save anomaly model: {e}")

    def _features_to_vector(self, features: Dict[str, float]) -> List[float]:
        """Convert feature dict to consistent ordered vector."""
        return [float(features.get(name, 0.0) or 0.0) for name in self._feature_names]

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

        # Optional training mode: collect baseline and train IsolationForest
        if self.training_mode and self.anomaly_model is None and IsolationForest is not None and np is not None:
            self._training_buffer.append(self._features_to_vector(features))
            if len(self._training_buffer) >= self.baseline_samples:
                self.get_logger().info(f"Training IsolationForest on {len(self._training_buffer)} samples...")
                X = np.asarray(self._training_buffer, dtype=float)
                self.anomaly_model = IsolationForest(
                    n_estimators=200,
                    contamination="auto",
                    random_state=42,
                )
                self.anomaly_model.fit(X)
                self._persist_model()
                self._training_buffer = []
                self.get_logger().info("Training complete. Switching to ML-based anomaly scoring.")

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

        If model is available: IsolationForest decision_function (higher = more normal).
        Otherwise: heuristic scoring fallback (keeps node usable without ML deps).
        """
        # ML scoring
        if self.anomaly_model is not None and np is not None:
            try:
                vec = np.asarray([self._features_to_vector(features)], dtype=float)
                return float(self.anomaly_model.decision_function(vec)[0])
            except Exception as e:
                self.get_logger().error(f"ML scoring failed; falling back to heuristic. Error: {e}")

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
            'explanation': self._generate_anomaly_explanation(features, score=score),
            'severity': self._compute_severity(score),
            'recommended_action': self._recommend_action(score)
        }

        msg = String()
        msg.data = json.dumps(alert, default=str)
        self.anomaly_alert_pub.publish(msg)

        self.get_logger().warn(f'ANOMALY DETECTED: score={score:.3f}')

    def _generate_anomaly_explanation(self, features: dict, score: float) -> str:
        """
        Generate human-readable explanation for anomaly.

        If SHAP + model are available, provide feature-attribution explanation (rate-limited).
        Otherwise, provide heuristic explanation.
        """
        # SHAP-based explanation (only on anomaly, and rate-limited)
        if (
            self.shap_enabled
            and shap is not None
            and self.anomaly_model is not None
            and np is not None
            and (time.time() - self._last_shap_time) >= self.shap_min_interval_sec
        ):
            try:
                self._last_shap_time = time.time()
                if self._shap_explainer is None:
                    self._shap_explainer = shap.TreeExplainer(self.anomaly_model)

                x = np.asarray([self._features_to_vector(features)], dtype=float)
                shap_values = self._shap_explainer.shap_values(x)

                # Normalize to 1D vector
                if isinstance(shap_values, list):
                    shap_arr = np.asarray(shap_values[0], dtype=float).reshape(-1)
                else:
                    shap_arr = np.asarray(shap_values, dtype=float).reshape(-1)

                idxs = np.argsort(np.abs(shap_arr))[::-1][:3]
                parts = []
                for idx in idxs:
                    name = self._feature_names[int(idx)]
                    val = float(features.get(name, 0.0) or 0.0)
                    impact = float(shap_arr[int(idx)])
                    parts.append(f"{name}={val:.3f} (impact {impact:+.3f})")

                return (
                    f"Anomaly explanation (SHAP): score={score:.3f}. "
                    f"Top contributing factors: " + ", ".join(parts)
                )
            except Exception as e:
                self.get_logger().warn(f"SHAP explanation failed; using heuristic explanation. Error: {e}")

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
