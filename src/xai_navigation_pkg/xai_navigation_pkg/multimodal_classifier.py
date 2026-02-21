#!/usr/bin/env python3
"""
Multi-Modal Classifier - Sensor Fusion for Tesla-Style Obstacle Classification

Fuses visual detections from YOLO-World with LiDAR-based costmap processing
to create robust, multi-modal obstacle classifications.

Architecture:
    /yolo/detections (camera) ─┐
                               ├──> MultiModalClassifier ──> /multimodal/classifications
    /local_costmap (lidar) ────┘

Fusion Strategy:
1. Camera provides semantic understanding (person, chair, cart)
2. LiDAR provides spatial precision (distance, size, velocity)
3. When both agree: High confidence classification
4. When only LiDAR: Fall back to existing heuristics
5. When only camera: Use visual classification with lower confidence

Based on Tesla FSD approach: "Eyes (camera) for identification, Radar (LiDAR) for ranging"
"""

import math
import time
from typing import Dict, Any, List, Optional, Tuple
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

# YOLO messages
try:
    from yolo_msgs.msg import DetectionArray, Detection
    YOLO_MSGS_AVAILABLE = True
except ImportError:
    YOLO_MSGS_AVAILABLE = False
    DetectionArray = None
    Detection = None

# Import existing classifier
try:
    from .obstacle_classifier import ObstacleClassifier, ObstacleClassification
    CLASSIFIER_AVAILABLE = True
except ImportError:
    CLASSIFIER_AVAILABLE = False
    ObstacleClassifier = None
    ObstacleClassification = None


@dataclass
class FusedObstacle:
    """Result of multi-modal obstacle fusion."""

    # Spatial data (from LiDAR)
    x: float
    y: float
    distance: float
    size: Optional[float] = None
    velocity: Optional[float] = None
    velocity_x: Optional[float] = None
    velocity_y: Optional[float] = None

    # Classification data
    obstacle_type: str = 'unknown'
    priority_weight: float = 1.5
    confidence: float = 0.5

    # Source information
    detection_source: str = 'lidar_heuristic'  # 'camera', 'lidar_heuristic', 'fusion'
    camera_class: Optional[str] = None
    camera_confidence: Optional[float] = None

    # Intent prediction
    intent: str = 'stationary'  # 'approaching', 'crossing', 'stationary', 'moving_away'
    collision_risk: float = 0.0
    time_to_collision: Optional[float] = None

    # Reasoning for XAI
    reasoning: str = ''
    contributing_factors: List[str] = field(default_factory=list)


# Tesla-style class to priority type mapping
CLASS_TO_PRIORITY_TYPE = {
    # Human tier (Priority 10.0)
    'person': 'human',
    'pedestrian': 'human',
    'child': 'human',
    'wheelchair user': 'human',
    'man': 'human',
    'woman': 'human',

    # Vehicle tier (Priority 5.0)
    'wheelchair': 'vehicle',
    'cart': 'vehicle',
    'shopping cart': 'vehicle',
    'bicycle': 'vehicle',
    'scooter': 'vehicle',
    'stroller': 'vehicle',
    'robot': 'vehicle',

    # Furniture tier (Priority 2.0)
    'chair': 'furniture',
    'office chair': 'furniture',
    'table': 'furniture',
    'desk': 'furniture',
    'couch': 'furniture',
    'sofa': 'furniture',
    'box': 'furniture',
    'cabinet': 'furniture',
    'shelf': 'furniture',
    'bench': 'furniture',

    # Wall tier (Priority 1.0)
    'wall': 'wall',
    'pillar': 'wall',
    'door': 'wall',
    'column': 'wall',
}

PRIORITY_WEIGHTS = {
    'human': 10.0,
    'vehicle': 5.0,
    'dynamic': 3.0,
    'furniture': 2.0,
    'wall': 1.0,
    'unknown': 1.5,
}


class MultiModalClassifier:
    """
    Fuses camera (YOLO-World) and LiDAR (costmap) for robust classification.

    Key Features:
    - Sensor fusion with confidence weighting
    - Tesla-style priority hierarchy
    - Intent prediction based on velocity vectors
    - Fallback to heuristics when camera unavailable
    """

    def __init__(
        self,
        camera_confidence_threshold: float = 0.4,
        fusion_distance_threshold: float = 0.5,
        enable_intent_prediction: bool = True,
        heuristic_classifier: Optional['ObstacleClassifier'] = None
    ):
        """
        Initialize multi-modal classifier.

        Args:
            camera_confidence_threshold: Minimum camera detection confidence
            fusion_distance_threshold: Max distance (m) to match camera-LiDAR detections
            enable_intent_prediction: Enable velocity-based intent prediction
            heuristic_classifier: Existing ObstacleClassifier for fallback
        """
        self.camera_confidence_threshold = camera_confidence_threshold
        self.fusion_distance_threshold = fusion_distance_threshold
        self.enable_intent_prediction = enable_intent_prediction

        # Use existing classifier for heuristic fallback
        self.heuristic_classifier = heuristic_classifier
        if not self.heuristic_classifier and CLASSIFIER_AVAILABLE:
            self.heuristic_classifier = ObstacleClassifier()

        # Track previous obstacle positions for velocity estimation
        self._previous_obstacles: Dict[str, Tuple[float, float, float, float]] = {}

        # Camera intrinsic parameters (TurtleBot3 Waffle Intel RealSense R200)
        # These are approximate values - should be calibrated for accuracy
        self.camera_fx = 554.25  # Focal length x
        self.camera_fy = 554.25  # Focal length y
        self.camera_cx = 320.5   # Principal point x
        self.camera_cy = 240.5   # Principal point y
        self.camera_width = 640
        self.camera_height = 480

        # Camera pose relative to robot base
        self.camera_height_m = 0.287  # TurtleBot3 Waffle camera height
        self.camera_pitch = 0.0       # Camera tilt angle

    def fuse_detections(
        self,
        lidar_obstacles: List[Dict[str, Any]],
        vision_detections: Optional[List[Any]] = None,
        robot_pose: Optional[Tuple[float, float, float]] = None,
        robot_velocity: Optional[Tuple[float, float]] = None,
        dt: float = 0.1
    ) -> List[FusedObstacle]:
        """
        Fuse LiDAR obstacles with camera detections.

        Args:
            lidar_obstacles: List of obstacles from costmap processing
            vision_detections: List of YOLO detections (yolo_msgs/Detection)
            robot_pose: Robot (x, y, theta) in world frame
            robot_velocity: Robot (vx, vy) velocity
            dt: Time delta since last call

        Returns:
            List of FusedObstacle with combined classification
        """
        fused_obstacles = []
        current_time = time.time()

        # Process each LiDAR obstacle
        for lidar_obs in lidar_obstacles:
            obs_x = lidar_obs.get('x', 0.0)
            obs_y = lidar_obs.get('y', 0.0)
            obs_id = self._get_obstacle_id(obs_x, obs_y)

            # Calculate distance to robot
            if robot_pose:
                distance = math.sqrt(
                    (obs_x - robot_pose[0])**2 +
                    (obs_y - robot_pose[1])**2
                )
            else:
                distance = lidar_obs.get('distance', float('inf'))

            # Estimate velocity from position history
            velocity, vx, vy = self._estimate_velocity(obs_id, obs_x, obs_y, current_time, dt)

            # Try to match with camera detection
            matched_vision = None
            if vision_detections and robot_pose:
                matched_vision = self._find_matching_detection(
                    obs_x, obs_y, vision_detections, robot_pose
                )

            # Create fused obstacle
            if matched_vision:
                # High confidence fusion - camera identified the obstacle
                fused = self._create_camera_fused_obstacle(
                    lidar_obs, matched_vision, distance, velocity, vx, vy
                )
            else:
                # Fall back to heuristic classification
                fused = self._create_heuristic_obstacle(
                    lidar_obs, distance, velocity, vx, vy
                )

            # Add intent prediction if enabled
            if self.enable_intent_prediction and robot_pose and robot_velocity:
                self._predict_intent(fused, robot_pose, robot_velocity)

            # Update position history
            self._previous_obstacles[obs_id] = (current_time, obs_x, obs_y, velocity or 0.0)

            fused_obstacles.append(fused)

        # Sort by priority (highest first)
        fused_obstacles.sort(key=lambda o: o.priority_weight, reverse=True)

        # Cleanup old position history
        self._cleanup_history(current_time)

        return fused_obstacles

    def _find_matching_detection(
        self,
        obs_x: float,
        obs_y: float,
        detections: List[Any],
        robot_pose: Tuple[float, float, float]
    ) -> Optional[Any]:
        """
        Find camera detection that matches LiDAR obstacle position.

        Uses simple projection: converts world position to image coordinates
        and checks if it falls within any detection bounding box.
        """
        if not detections:
            return None

        # Project world position to image coordinates
        pixel_x, pixel_y = self._world_to_image(obs_x, obs_y, robot_pose)

        if pixel_x is None or pixel_y is None:
            return None

        # Find matching detection
        for detection in detections:
            if not hasattr(detection, 'bbox'):
                continue

            bbox = detection.bbox

            # Check if projected point is within bounding box
            # Account for some margin due to projection errors
            margin = 30  # pixels

            x_min = bbox.center.position.x - bbox.size.x / 2 - margin
            x_max = bbox.center.position.x + bbox.size.x / 2 + margin
            y_min = bbox.center.position.y - bbox.size.y / 2 - margin
            y_max = bbox.center.position.y + bbox.size.y / 2 + margin

            if x_min <= pixel_x <= x_max and y_min <= pixel_y <= y_max:
                # Check confidence threshold
                if detection.score >= self.camera_confidence_threshold:
                    return detection

        return None

    def _world_to_image(
        self,
        world_x: float,
        world_y: float,
        robot_pose: Tuple[float, float, float]
    ) -> Tuple[Optional[float], Optional[float]]:
        """
        Project world coordinates to image pixel coordinates.

        This is a simplified projection that assumes:
        - Camera is mounted on the robot looking forward
        - Robot is on a flat plane (z=0)
        """
        robot_x, robot_y, robot_theta = robot_pose

        # Transform to robot frame
        dx = world_x - robot_x
        dy = world_y - robot_y

        # Rotate to robot frame (camera looks along x-axis in robot frame)
        cos_theta = math.cos(-robot_theta)
        sin_theta = math.sin(-robot_theta)

        cam_x = dx * cos_theta - dy * sin_theta  # Forward distance
        cam_y = dx * sin_theta + dy * cos_theta  # Left-right offset
        cam_z = -self.camera_height_m  # Height difference (negative = below camera)

        # Check if point is in front of camera
        if cam_x <= 0.1:  # Behind or too close to camera
            return None, None

        # Project to image plane (pinhole camera model)
        pixel_x = self.camera_cx + (self.camera_fx * cam_y / cam_x)
        pixel_y = self.camera_cy + (self.camera_fy * cam_z / cam_x)

        # Check if within image bounds
        if 0 <= pixel_x < self.camera_width and 0 <= pixel_y < self.camera_height:
            return pixel_x, pixel_y

        return None, None

    def _create_camera_fused_obstacle(
        self,
        lidar_obs: Dict[str, Any],
        vision_detection: Any,
        distance: float,
        velocity: Optional[float],
        vx: Optional[float],
        vy: Optional[float]
    ) -> FusedObstacle:
        """Create obstacle with camera-based classification."""

        # Get class name and map to priority type
        class_name = vision_detection.class_name if hasattr(vision_detection, 'class_name') else 'unknown'
        camera_confidence = vision_detection.score if hasattr(vision_detection, 'score') else 0.5

        priority_type = CLASS_TO_PRIORITY_TYPE.get(class_name.lower(), 'unknown')
        priority_weight = PRIORITY_WEIGHTS.get(priority_type, 1.5)

        # Boost confidence since we have camera + LiDAR agreement
        fused_confidence = min(camera_confidence * 0.95, 0.98)

        # Build reasoning
        factors = [
            f"Camera detected '{class_name}' with {camera_confidence:.0%} confidence",
            f"LiDAR confirmed obstacle at {distance:.2f}m",
            f"Priority type: {priority_type} (weight: {priority_weight})"
        ]

        if velocity and velocity > 0.1:
            factors.append(f"Moving at {velocity:.2f} m/s")

        reasoning = f"Detected {class_name} using camera (Tesla-style priority: {priority_type})"

        return FusedObstacle(
            x=lidar_obs.get('x', 0.0),
            y=lidar_obs.get('y', 0.0),
            distance=distance,
            size=lidar_obs.get('size'),
            velocity=velocity,
            velocity_x=vx,
            velocity_y=vy,
            obstacle_type=priority_type,
            priority_weight=priority_weight,
            confidence=fused_confidence,
            detection_source='fusion',
            camera_class=class_name,
            camera_confidence=camera_confidence,
            reasoning=reasoning,
            contributing_factors=factors
        )

    def _create_heuristic_obstacle(
        self,
        lidar_obs: Dict[str, Any],
        distance: float,
        velocity: Optional[float],
        vx: Optional[float],
        vy: Optional[float]
    ) -> FusedObstacle:
        """Create obstacle with LiDAR-only heuristic classification."""

        obstacle_type = 'unknown'
        priority_weight = 1.5
        confidence = 0.5
        factors = ["No camera detection available - using LiDAR heuristics"]

        # Use existing heuristic classifier if available
        if self.heuristic_classifier:
            classification = self.heuristic_classifier.classify(
                lidar_obs,
                previous_position=None,
                dt=0.5
            )

            obstacle_type = classification.obstacle_type
            priority_weight = classification.priority_weight
            confidence = classification.confidence
            factors = classification.contributing_factors
        else:
            # Simple velocity-based heuristic
            if velocity:
                if velocity > 1.5:
                    obstacle_type = 'vehicle'
                    priority_weight = PRIORITY_WEIGHTS['vehicle']
                    confidence = 0.6
                    factors.append(f"Fast movement ({velocity:.2f} m/s) suggests vehicle")
                elif 0.3 < velocity <= 1.5:
                    obstacle_type = 'human'
                    priority_weight = PRIORITY_WEIGHTS['human']
                    confidence = 0.6
                    factors.append(f"Walking speed ({velocity:.2f} m/s) suggests human")
                elif velocity > 0.1:
                    obstacle_type = 'dynamic'
                    priority_weight = PRIORITY_WEIGHTS['dynamic']
                    confidence = 0.5
                    factors.append(f"Moving slowly ({velocity:.2f} m/s)")
                else:
                    obstacle_type = 'furniture'
                    priority_weight = PRIORITY_WEIGHTS['furniture']
                    confidence = 0.4
                    factors.append("Stationary object")

        reasoning = f"Classified as {obstacle_type} using LiDAR heuristics"

        return FusedObstacle(
            x=lidar_obs.get('x', 0.0),
            y=lidar_obs.get('y', 0.0),
            distance=distance,
            size=lidar_obs.get('size'),
            velocity=velocity,
            velocity_x=vx,
            velocity_y=vy,
            obstacle_type=obstacle_type,
            priority_weight=priority_weight,
            confidence=confidence,
            detection_source='lidar_heuristic',
            reasoning=reasoning,
            contributing_factors=factors
        )

    def _predict_intent(
        self,
        obstacle: FusedObstacle,
        robot_pose: Tuple[float, float, float],
        robot_velocity: Tuple[float, float]
    ):
        """
        Predict obstacle intent based on velocity vectors.

        Intent categories:
        - approaching: Moving toward robot
        - crossing: Moving perpendicular to robot
        - stationary: Not moving
        - moving_away: Moving away from robot
        """
        if obstacle.velocity is None or obstacle.velocity < 0.1:
            obstacle.intent = 'stationary'
            obstacle.collision_risk = 0.0
            return

        robot_x, robot_y, _ = robot_pose
        robot_vx, robot_vy = robot_velocity

        # Vector from obstacle to robot
        dx = robot_x - obstacle.x
        dy = robot_y - obstacle.y
        dist = math.sqrt(dx**2 + dy**2)

        if dist < 0.1:
            dist = 0.1

        # Normalize direction to robot
        dir_x = dx / dist
        dir_y = dy / dist

        # Calculate relative velocity (obstacle velocity toward robot)
        obs_vx = obstacle.velocity_x or 0.0
        obs_vy = obstacle.velocity_y or 0.0

        # Relative velocity component toward robot
        approach_velocity = obs_vx * dir_x + obs_vy * dir_y

        # Cross velocity (perpendicular to approach direction)
        cross_velocity = abs(obs_vx * dir_y - obs_vy * dir_x)

        # Determine intent
        if approach_velocity > 0.3:
            obstacle.intent = 'approaching'

            # Calculate time to collision
            if approach_velocity > 0:
                obstacle.time_to_collision = dist / approach_velocity

                # Collision risk based on TTC
                if obstacle.time_to_collision < 2.0:
                    obstacle.collision_risk = min(2.0 / obstacle.time_to_collision, 1.0)
                else:
                    obstacle.collision_risk = 0.3

            # Boost priority for approaching obstacles
            obstacle.priority_weight *= 1.5
            obstacle.contributing_factors.append(
                f"Approaching at {approach_velocity:.2f} m/s (TTC: {obstacle.time_to_collision:.1f}s)"
            )

        elif approach_velocity < -0.3:
            obstacle.intent = 'moving_away'
            obstacle.collision_risk = 0.0

            # Reduce priority for moving-away obstacles
            obstacle.priority_weight *= 0.5
            obstacle.contributing_factors.append("Moving away from robot")

        elif cross_velocity > 0.3:
            obstacle.intent = 'crossing'
            obstacle.collision_risk = 0.3
            obstacle.contributing_factors.append(
                f"Crossing path at {cross_velocity:.2f} m/s"
            )
        else:
            obstacle.intent = 'stationary'
            obstacle.collision_risk = 0.0

    def _estimate_velocity(
        self,
        obs_id: str,
        x: float,
        y: float,
        current_time: float,
        dt: float
    ) -> Tuple[Optional[float], Optional[float], Optional[float]]:
        """Estimate velocity from position history."""

        if obs_id not in self._previous_obstacles:
            return None, None, None

        prev_time, prev_x, prev_y, _ = self._previous_obstacles[obs_id]
        actual_dt = current_time - prev_time

        if actual_dt < 0.05:  # Too short time delta
            return None, None, None

        vx = (x - prev_x) / actual_dt
        vy = (y - prev_y) / actual_dt
        velocity = math.sqrt(vx**2 + vy**2)

        return velocity, vx, vy

    def _get_obstacle_id(self, x: float, y: float) -> str:
        """Generate unique ID for obstacle based on position matching costmap resolution (0.05m)."""
        return f"obs_{int(x*20)}_{int(y*20)}"

    def _cleanup_history(self, current_time: float, max_age: float = 3.0):
        """Remove old position history entries."""
        cutoff = current_time - max_age
        to_remove = [
            obs_id for obs_id, (t, _, _, _) in self._previous_obstacles.items()
            if t < cutoff
        ]
        for obs_id in to_remove:
            del self._previous_obstacles[obs_id]


class MultiModalClassifierNode(Node):
    """
    ROS2 Node wrapper for MultiModalClassifier.

    Subscribes to:
    - /yolo/detections (yolo_msgs/DetectionArray)
    - /local_costmap/costmap (nav_msgs/OccupancyGrid)
    - /odom or /robot_pose (for velocity)

    Publishes:
    - Not currently publishing (Publishing loop and custom message TBD)
    """

    def __init__(self):
        super().__init__('multimodal_classifier_node')

        # Parameters
        self.declare_parameter('camera_confidence_threshold', 0.4)
        self.declare_parameter('fusion_distance_threshold', 0.5)
        self.declare_parameter('enable_intent_prediction', True)
        self.declare_parameter('update_rate', 10.0)

        # Get parameters
        camera_conf = self.get_parameter('camera_confidence_threshold').value
        fusion_dist = self.get_parameter('fusion_distance_threshold').value
        enable_intent = self.get_parameter('enable_intent_prediction').value

        # Initialize classifier
        self.classifier = MultiModalClassifier(
            camera_confidence_threshold=camera_conf,
            fusion_distance_threshold=fusion_dist,
            enable_intent_prediction=enable_intent
        )

        self.latest_detections: Optional[List] = None
        self.latest_costmap: Optional[OccupancyGrid] = None

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        if YOLO_MSGS_AVAILABLE:
            self.detection_sub = self.create_subscription(
                DetectionArray,
                '/yolo/detections',
                self.detection_callback,
                sensor_qos
            )

        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.costmap_callback,
            sensor_qos
        )

        self.get_logger().info('MultiModalClassifier node initialized')

    def detection_callback(self, msg):
        """Store latest camera detections."""
        self.latest_detections = msg.detections if hasattr(msg, 'detections') else []

    def costmap_callback(self, msg):
        """Store latest costmap."""
        self.latest_costmap = msg


def main(args=None):
    rclpy.init(args=args)

    node = MultiModalClassifierNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
