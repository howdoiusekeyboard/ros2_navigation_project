#!/usr/bin/env python3
"""
Path Analyzer - Compares navigation paths and detects significant changes.

Week 3: Core analysis component for understanding path planning decisions.

Key concepts:
- Path length calculation
- Deviation detection between old and new paths
- Path smoothness analysis
"""

import math
from typing import List, Tuple, Optional, Dict, Any
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class PathAnalyzer:
    """
    Analyzes and compares navigation paths to detect changes.
    """

    def __init__(self, deviation_threshold: float = 0.3):
        """
        Initialize path analyzer.

        Args:
            deviation_threshold: Minimum deviation (meters) to consider significant
        """
        self.deviation_threshold = deviation_threshold
        self.previous_path: Optional[Path] = None

    def update_path(self, path: Path):
        """Store current path for future comparison."""
        self.previous_path = path

    def calculate_path_length(self, path: Path) -> float:
        """
        Calculate total path length in meters.

        Args:
            path: Nav2 Path message

        Returns:
            Total length in meters
        """
        if not path or len(path.poses) < 2:
            return 0.0

        total = 0.0
        for i in range(len(path.poses) - 1):
            p1 = path.poses[i].pose.position
            p2 = path.poses[i + 1].pose.position
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            total += math.sqrt(dx*dx + dy*dy)

        return total

    def compare_paths(
        self,
        old_path: Path,
        new_path: Path
    ) -> Dict[str, Any]:
        """
        Compare two paths and identify differences.

        Args:
            old_path: Previous path
            new_path: New path

        Returns:
            Comparison results dictionary
        """
        if not old_path or not new_path:
            return {'comparable': False, 'reason': 'missing_path'}

        if not old_path.poses or not new_path.poses:
            return {'comparable': False, 'reason': 'empty_path'}

        old_length = self.calculate_path_length(old_path)
        new_length = self.calculate_path_length(new_path)

        # Find maximum deviation
        max_deviation, deviation_point = self._find_max_deviation(
            old_path.poses, new_path.poses
        )

        # Calculate average deviation
        avg_deviation = self._calculate_avg_deviation(
            old_path.poses, new_path.poses
        )

        # Determine if change is significant
        is_significant = (
            max_deviation > self.deviation_threshold or
            abs(new_length - old_length) > 1.0  # >1m length change
        )

        return {
            'comparable': True,
            'old_length': old_length,
            'new_length': new_length,
            'length_change': new_length - old_length,
            'max_deviation': max_deviation,
            'deviation_point': deviation_point,
            'avg_deviation': avg_deviation,
            'is_significant': is_significant,
            'waypoint_change': len(new_path.poses) - len(old_path.poses)
        }

    def detect_path_change_reason(
        self,
        old_path: Path,
        new_path: Path,
        costmap_processor=None
    ) -> str:
        """
        Attempt to determine why path changed.

        Args:
            old_path: Previous path
            new_path: New path
            costmap_processor: Optional CostmapProcessor for obstacle check

        Returns:
            Reason string
        """
        comparison = self.compare_paths(old_path, new_path)

        if not comparison.get('comparable'):
            return 'paths_incomparable'

        if not comparison.get('is_significant'):
            return 'minor_adjustment'

        # Check if old path now has obstacles
        if costmap_processor and costmap_processor.has_local_costmap:
            # Check middle of old path
            if len(old_path.poses) > 2:
                mid_idx = len(old_path.poses) // 2
                mid_pose = old_path.poses[mid_idx]
                cost = costmap_processor.get_cost_at_position(
                    mid_pose.pose.position.x,
                    mid_pose.pose.position.y,
                    'local'
                )

                if cost and cost >= costmap_processor.INSCRIBED_OBSTACLE:
                    return 'obstacle_detected'

        # Check length change
        if comparison['length_change'] > 1.0:
            return 'detour_required'
        elif comparison['length_change'] < -0.5:
            return 'shorter_path_found'

        # Check if goal changed
        if old_path.poses and new_path.poses:
            old_goal = old_path.poses[-1].pose.position
            new_goal = new_path.poses[-1].pose.position
            goal_dist = math.sqrt(
                (old_goal.x - new_goal.x)**2 + (old_goal.y - new_goal.y)**2
            )

            if goal_dist > 0.5:
                return 'goal_changed'

        return 'path_optimization'

    def get_path_statistics(self, path: Path) -> Dict[str, Any]:
        """
        Get comprehensive statistics about a path.

        Args:
            path: Path to analyze

        Returns:
            Statistics dictionary
        """
        if not path or not path.poses:
            return {}

        length = self.calculate_path_length(path)
        smoothness = self._calculate_smoothness(path)

        # Extract coordinates
        coords = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        x_coords = [c[0] for c in coords]
        y_coords = [c[1] for c in coords]

        return {
            'waypoint_count': len(path.poses),
            'length': length,
            'smoothness': smoothness,
            'bounds': {
                'min_x': min(x_coords),
                'max_x': max(x_coords),
                'min_y': min(y_coords),
                'max_y': max(y_coords)
            },
            'start': coords[0] if coords else None,
            'end': coords[-1] if coords else None
        }

    def _find_max_deviation(
        self,
        poses1: List[PoseStamped],
        poses2: List[PoseStamped]
    ) -> Tuple[float, Optional[Tuple[float, float]]]:
        """Find maximum deviation between two paths."""
        if not poses1 or not poses2:
            return 0.0, None

        max_dev = 0.0
        max_point = None

        # Sample at regular intervals
        sample_count = min(len(poses1), len(poses2), 20)

        for i in range(sample_count):
            idx1 = int(i * len(poses1) / sample_count)
            idx2 = int(i * len(poses2) / sample_count)

            p1 = poses1[idx1].pose.position
            p2 = poses2[idx2].pose.position

            dev = math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

            if dev > max_dev:
                max_dev = dev
                max_point = (p1.x, p1.y)

        return max_dev, max_point

    def _calculate_avg_deviation(
        self,
        poses1: List[PoseStamped],
        poses2: List[PoseStamped]
    ) -> float:
        """Calculate average deviation between paths."""
        if not poses1 or not poses2:
            return 0.0

        sample_count = min(len(poses1), len(poses2), 20)
        deviations = []

        for i in range(sample_count):
            idx1 = int(i * len(poses1) / sample_count)
            idx2 = int(i * len(poses2) / sample_count)

            p1 = poses1[idx1].pose.position
            p2 = poses2[idx2].pose.position

            dev = math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)
            deviations.append(dev)

        return sum(deviations) / len(deviations) if deviations else 0.0

    def _calculate_smoothness(self, path: Path) -> float:
        """
        Calculate path smoothness (lower = smoother).

        Returns average angle change between consecutive segments.
        """
        if len(path.poses) < 3:
            return 0.0

        angle_changes = []

        for i in range(1, len(path.poses) - 1):
            p0 = path.poses[i - 1].pose.position
            p1 = path.poses[i].pose.position
            p2 = path.poses[i + 1].pose.position

            # Vectors
            v1 = (p1.x - p0.x, p1.y - p0.y)
            v2 = (p2.x - p1.x, p2.y - p1.y)

            # Angle between vectors
            dot = v1[0] * v2[0] + v1[1] * v2[1]
            mag1 = math.sqrt(v1[0]**2 + v1[1]**2)
            mag2 = math.sqrt(v2[0]**2 + v2[1]**2)

            if mag1 > 0 and mag2 > 0:
                cos_angle = max(-1.0, min(1.0, dot / (mag1 * mag2)))
                angle = math.acos(cos_angle)
                angle_changes.append(abs(angle))

        return sum(angle_changes) / len(angle_changes) if angle_changes else 0.0
