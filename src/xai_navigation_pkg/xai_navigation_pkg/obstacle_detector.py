#!/usr/bin/env python3
"""
Obstacle Detector - Identifies obstacles affecting navigation and their impact.

Week 3: Core analysis component for understanding navigation constraints.
Week 4+: Extended with weighted obstacle classification (Tesla-style).

Provides:
- Obstacle detection along planned paths
- Impact severity classification
- Avoidance suggestion generation
- Weighted obstacle classification (human > vehicle > furniture)
- Priority-based obstacle ranking
"""

import math
import time
from typing import List, Tuple, Optional, Dict, Any
from nav_msgs.msg import Path

# Import classifier (handle import error gracefully for testing)
try:
    from .obstacle_classifier import ObstacleClassifier, ObstacleClassification
    CLASSIFIER_AVAILABLE = True
except ImportError:
    CLASSIFIER_AVAILABLE = False
    ObstacleClassifier = None
    ObstacleClassification = None


class ObstacleDetector:
    """
    Detects obstacles and determines their impact on navigation.
    """

    # Severity levels
    SEVERITY_CRITICAL = 'critical'    # <0.3m, immediate action needed
    SEVERITY_WARNING = 'warning'      # 0.3-1.0m, caution required
    SEVERITY_INFO = 'info'            # >1.0m, for awareness only

    def __init__(
        self,
        critical_distance: float = 0.3,
        warning_distance: float = 1.0,
        enable_classification: bool = True,
        zones_config_path: Optional[str] = None,
        custom_weights: Optional[Dict[str, float]] = None
    ):
        """
        Initialize obstacle detector.

        Args:
            critical_distance: Distance (m) considered critical
            warning_distance: Distance (m) considered warning
            enable_classification: Enable weighted obstacle classification
            zones_config_path: Path to semantic zones YAML config
            custom_weights: Custom priority weights for obstacle types
        """
        self.critical_distance = critical_distance
        self.warning_distance = warning_distance

        # Track detected obstacles
        self.current_obstacles: List[Dict[str, Any]] = []

        # Track previous obstacle positions for velocity estimation
        self._previous_obstacles: Dict[str, Tuple[float, float, float]] = {}
        self._last_detection_time: float = 0.0

        # Initialize classifier if available and enabled
        self.enable_classification = enable_classification and CLASSIFIER_AVAILABLE
        self.classifier: Optional[ObstacleClassifier] = None

        if self.enable_classification:
            self.classifier = ObstacleClassifier(
                weights=custom_weights,
                zones_config_path=zones_config_path
            )

    def detect_on_path(
        self,
        path: Path,
        costmap_processor,
        current_position: Optional[Tuple[float, float]] = None,
        check_radius: float = 0.5
    ) -> Dict[str, Any]:
        """
        Detect obstacles along a planned path.

        Args:
            path: Planned navigation path
            costmap_processor: CostmapProcessor instance
            current_position: Robot's current (x, y) position
            check_radius: Radius to check around path waypoints

        Returns:
            Detection results dictionary
        """
        if not path or not path.poses:
            return {'detected': False, 'obstacles': []}

        if not costmap_processor.has_local_costmap:
            return {'detected': False, 'reason': 'no_costmap'}

        clearance = costmap_processor.analyze_path_clearance(
            path.poses, check_radius, 'local'
        )

        if not clearance.get('obstacles'):
            return {
                'detected': False,
                'path_clear': True,
                'min_clearance': clearance.get('min_clearance')
            }

        # Classify obstacles by severity
        obstacles = clearance['obstacles']
        critical = []
        warning = []
        info = []

        for obs in obstacles:
            dist = obs.get('distance', float('inf'))

            if dist < self.critical_distance:
                obs['severity'] = self.SEVERITY_CRITICAL
                critical.append(obs)
            elif dist < self.warning_distance:
                obs['severity'] = self.SEVERITY_WARNING
                warning.append(obs)
            else:
                obs['severity'] = self.SEVERITY_INFO
                info.append(obs)

        # Find closest obstacle to robot
        closest = None
        if current_position and obstacles:
            min_dist = float('inf')
            for obs in obstacles:
                dist = math.sqrt(
                    (current_position[0] - obs['x'])**2 +
                    (current_position[1] - obs['y'])**2
                )
                if dist < min_dist:
                    min_dist = dist
                    closest = obs
                    closest['distance_from_robot'] = dist

        self.current_obstacles = obstacles

        return {
            'detected': True,
            'total_count': len(obstacles),
            'critical_count': len(critical),
            'warning_count': len(warning),
            'info_count': len(info),
            'closest': closest,
            'min_clearance': clearance.get('min_clearance'),
            'obstacles': obstacles[:5]  # Limit for performance
        }

    def check_direct_path_blocked(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        costmap_processor,
        sample_count: int = 30
    ) -> Dict[str, Any]:
        """
        Check if direct path from start to goal is blocked.

        Args:
            start: Start position (x, y)
            goal: Goal position (x, y)
            costmap_processor: CostmapProcessor instance
            sample_count: Number of points to sample along line

        Returns:
            Blocking information dictionary
        """
        if not costmap_processor.has_local_costmap:
            return {'blocked': False, 'reason': 'no_costmap'}

        obstacles = []

        for i in range(sample_count):
            t = i / (sample_count - 1)
            x = start[0] + t * (goal[0] - start[0])
            y = start[1] + t * (goal[1] - start[1])

            cost = costmap_processor.get_cost_at_position(x, y, 'local')

            if cost is not None and cost >= costmap_processor.INSCRIBED_OBSTACLE:
                dist_from_start = math.sqrt(
                    (x - start[0])**2 + (y - start[1])**2
                )
                obstacles.append({
                    'x': x,
                    'y': y,
                    'cost': cost,
                    'is_lethal': cost == costmap_processor.LETHAL_OBSTACLE,
                    'distance_from_start': dist_from_start
                })

        if not obstacles:
            return {
                'blocked': False,
                'direct_path_clear': True
            }

        # Sort by distance from start
        obstacles.sort(key=lambda o: o['distance_from_start'])

        return {
            'blocked': True,
            'obstacle_count': len(obstacles),
            'first_obstacle': obstacles[0],
            'first_obstacle_distance': obstacles[0]['distance_from_start']
        }

    def analyze_impact(
        self,
        obstacle: Dict[str, Any],
        path: Path,
        current_position: Tuple[float, float],
        goal: Tuple[float, float]
    ) -> Dict[str, Any]:
        """
        Analyze how an obstacle impacts navigation.

        Args:
            obstacle: Obstacle information dictionary
            path: Current planned path
            current_position: Robot's current position
            goal: Navigation goal

        Returns:
            Impact analysis dictionary
        """
        obs_x = obstacle['x']
        obs_y = obstacle['y']

        # Distance from robot
        dist_from_robot = math.sqrt(
            (current_position[0] - obs_x)**2 +
            (current_position[1] - obs_y)**2
        )

        # Severity
        if dist_from_robot < self.critical_distance:
            severity = self.SEVERITY_CRITICAL
            action = 'stop_or_replan'
        elif dist_from_robot < self.warning_distance:
            severity = self.SEVERITY_WARNING
            action = 'slow_down'
        else:
            severity = self.SEVERITY_INFO
            action = 'monitor'

        # Check if on planned path (within 0.5m of any waypoint)
        on_path = False
        if path and path.poses:
            for i, pose in enumerate(path.poses[:15]):  # Check first 15 waypoints
                dist = math.sqrt(
                    (pose.pose.position.x - obs_x)**2 +
                    (pose.pose.position.y - obs_y)**2
                )
                if dist < 0.5:
                    on_path = True
                    break

        return {
            'obstacle_position': (obs_x, obs_y),
            'distance_from_robot': dist_from_robot,
            'severity': severity,
            'on_planned_path': on_path,
            'recommended_action': action,
            'is_lethal': obstacle.get('is_lethal', False)
        }

    def suggest_avoidance(
        self,
        obstacle: Dict[str, Any],
        current_position: Tuple[float, float],
        goal: Tuple[float, float]
    ) -> str:
        """
        Suggest avoidance direction for an obstacle.

        Args:
            obstacle: Obstacle information
            current_position: Robot position
            goal: Navigation goal

        Returns:
            Avoidance suggestion string
        """
        obs_x = obstacle['x']
        obs_y = obstacle['y']

        # Vectors
        to_obs = (obs_x - current_position[0], obs_y - current_position[1])
        to_goal = (goal[0] - current_position[0], goal[1] - current_position[1])

        # Cross product to determine left/right
        cross = to_goal[0] * to_obs[1] - to_goal[1] * to_obs[0]

        if abs(cross) < 0.1:
            return 'obstacle_directly_ahead'
        elif cross > 0:
            return 'pass_on_right'
        else:
            return 'pass_on_left'

    def clear_obstacles(self):
        """Clear tracked obstacles."""
        self.current_obstacles = []
        self._previous_obstacles = {}

    @property
    def has_obstacles(self) -> bool:
        """Check if any obstacles are currently tracked."""
        return len(self.current_obstacles) > 0

    # === NEW: Weighted Classification Methods (Week 4+) ===

    def classify_obstacle(
        self,
        obstacle: Dict[str, Any],
        costmap_processor=None
    ) -> Optional[Dict[str, Any]]:
        """
        Classify an obstacle with weighted priority (Tesla-style).

        Args:
            obstacle: Obstacle dict with 'x', 'y' coordinates
            costmap_processor: Optional CostmapProcessor for size estimation

        Returns:
            Classification dict with type, weight, confidence, reasoning
            or None if classification is disabled
        """
        if not self.enable_classification or self.classifier is None:
            return None

        # Get obstacle ID for velocity tracking
        obs_id = self._get_obstacle_id(obstacle)

        # Get previous position for velocity estimation
        prev_pos = None
        dt = 0.5  # default
        current_time = time.time()

        if obs_id in self._previous_obstacles:
            prev_time, prev_x, prev_y = self._previous_obstacles[obs_id]
            prev_pos = (prev_x, prev_y)
            dt = max(current_time - prev_time, 0.1)

        # Classify the obstacle
        classification = self.classifier.classify(
            obstacle,
            costmap_processor=costmap_processor,
            previous_position=prev_pos,
            dt=dt
        )

        # Update position history for next velocity calculation
        self._previous_obstacles[obs_id] = (
            current_time,
            obstacle.get('x', 0.0),
            obstacle.get('y', 0.0)
        )

        # Clean up old entries
        self._cleanup_position_history(current_time)

        # Convert to dict for serialization
        return {
            'obstacle_type': classification.obstacle_type,
            'priority_weight': classification.priority_weight,
            'confidence': classification.confidence,
            'reasoning': classification.reasoning,
            'contributing_factors': classification.contributing_factors,
            'zone_name': classification.zone_name,
            'estimated_velocity': classification.estimated_velocity,
            'estimated_size': classification.estimated_size
        }

    def detect_and_classify(
        self,
        path: Path,
        costmap_processor,
        current_position: Optional[Tuple[float, float]] = None,
        check_radius: float = 0.5
    ) -> Dict[str, Any]:
        """
        Detect obstacles and classify them with weighted priorities.

        Combines detect_on_path with classification for complete analysis.

        Args:
            path: Planned navigation path
            costmap_processor: CostmapProcessor instance
            current_position: Robot's current (x, y) position
            check_radius: Radius to check around path waypoints

        Returns:
            Enhanced detection results with classification data
        """
        # First do standard detection
        detection = self.detect_on_path(
            path, costmap_processor, current_position, check_radius
        )

        if not detection.get('detected'):
            return detection

        # Add classification to each obstacle
        classified_obstacles = []
        for obs in detection.get('obstacles', []):
            classification = self.classify_obstacle(obs, costmap_processor)
            if classification:
                obs['classification'] = classification
                obs['priority_weight'] = classification['priority_weight']
                obs['obstacle_type'] = classification['obstacle_type']
            classified_obstacles.append(obs)

        # Sort by priority (highest first)
        classified_obstacles.sort(
            key=lambda o: o.get('priority_weight', 0),
            reverse=True
        )

        # Update detection results
        detection['obstacles'] = classified_obstacles
        detection['classification_enabled'] = self.enable_classification

        # Update closest to be highest priority obstacle
        if classified_obstacles:
            detection['highest_priority'] = classified_obstacles[0]

        return detection

    def get_priority_explanation(
        self,
        obstacle: Dict[str, Any]
    ) -> str:
        """
        Get human-friendly explanation of obstacle priority.

        Args:
            obstacle: Obstacle with classification data

        Returns:
            Explanation string suitable for XAI display
        """
        if not self.classifier:
            return "Obstacle detected"

        classification = obstacle.get('classification')
        if not classification:
            return "Unclassified obstacle detected"

        # Create mock classification for priority explanation
        class MockClassification:
            pass

        mock = MockClassification()
        mock.obstacle_type = classification.get('obstacle_type', 'unknown')

        return self.classifier.get_priority_explanation(mock)

    def _get_obstacle_id(self, obstacle: Dict[str, Any]) -> str:
        """Generate a unique ID for obstacle based on position."""
        x = obstacle.get('x', 0.0)
        y = obstacle.get('y', 0.0)
        # Round to 0.1m grid for consistent IDs
        return f"obs_{int(x*10)}_{int(y*10)}"

    def _cleanup_position_history(self, current_time: float, max_age: float = 5.0):
        """Remove old position entries."""
        cutoff = current_time - max_age
        to_remove = [
            obs_id for obs_id, (t, _, _) in self._previous_obstacles.items()
            if t < cutoff
        ]
        for obs_id in to_remove:
            del self._previous_obstacles[obs_id]

    def load_zones(self, config_path: str) -> bool:
        """
        Load semantic zones configuration.

        Args:
            config_path: Path to zones YAML file

        Returns:
            True if loaded successfully
        """
        if self.classifier:
            return self.classifier.load_zones(config_path)
        return False

    @property
    def classification_enabled(self) -> bool:
        """Check if classification is enabled and available."""
        return self.enable_classification and self.classifier is not None

