#!/usr/bin/env python3
"""
Obstacle Classifier - Tesla-style weighted obstacle classification.

Implements full heuristic classification:
1. Zone-based: Predefined map regions (hallway = human zones)
2. Velocity-based: Moving obstacles likely human/vehicle
3. Size-based: Narrow cylinders (~0.5m) = human, wide = furniture

Based on Dr. Sujala's guidance: prioritize humans > vehicles > furniture > walls,
similar to Tesla's approach where human safety is weighted highest.
"""

import math
import time
import yaml
import os
from typing import Dict, Any, List, Optional, Tuple
from dataclasses import dataclass, field


@dataclass
class ObstacleClassification:
    """Result of obstacle classification."""
    obstacle_type: str  # human, vehicle, dynamic, furniture, wall, unknown
    priority_weight: float
    confidence: float  # 0.0 to 1.0
    reasoning: str
    contributing_factors: List[str] = field(default_factory=list)
    zone_name: Optional[str] = None
    estimated_velocity: Optional[float] = None
    estimated_size: Optional[float] = None


@dataclass
class SemanticZone:
    """Defines a semantic region on the map."""
    name: str
    zone_type: str  # human_zone, furniture_zone, restricted_zone
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    weight_multiplier: float = 1.0
    description: str = ""


class ObstacleClassifier:
    """
    Classifies obstacles by type and calculates priority weights.

    Uses FULL HEURISTICS (Tesla-style):
    1. Zone-based: Predefined map regions (hallway = human zones)
    2. Velocity-based: Moving obstacles likely human/vehicle
    3. Size-based: Narrow cylinders (~0.5m) = human, wide = furniture

    Priority weights (higher = more urgent to avoid):
    - Human: 10.0 (highest priority - always protect)
    - Vehicle: 5.0 (high priority - potential danger)
    - Dynamic: 3.0 (moving but unknown type)
    - Furniture: 2.0 (static, can navigate around)
    - Wall: 1.0 (static structure, lowest priority)
    - Unknown: 1.5 (default cautious weight)
    """

    # Default priority weights (Tesla-style: humans >> objects)
    DEFAULT_WEIGHTS = {
        'human': 10.0,
        'vehicle': 5.0,
        'dynamic': 3.0,
        'furniture': 2.0,
        'wall': 1.0,
        'unknown': 1.5
    }

    # Velocity thresholds (m/s)
    VEHICLE_VELOCITY_THRESHOLD = 1.5  # > 1.5 m/s likely vehicle
    HUMAN_VELOCITY_MIN = 0.3  # Slow walking
    HUMAN_VELOCITY_MAX = 1.5  # Fast walking

    # Size thresholds (meters)
    HUMAN_SIZE_MAX = 0.6  # Typical human width
    FURNITURE_SIZE_MIN = 0.8  # Larger than human

    def __init__(
        self,
        weights: Optional[Dict[str, float]] = None,
        zones_config_path: Optional[str] = None
    ):
        """
        Initialize obstacle classifier.

        Args:
            weights: Custom priority weights (optional)
            zones_config_path: Path to semantic zones YAML file
        """
        self.weights = weights or self.DEFAULT_WEIGHTS.copy()
        self.zones: List[SemanticZone] = []

        # Position history for velocity tracking
        # Key: obstacle_id (approximate position hash), Value: list of (timestamp, x, y)
        self._position_history: Dict[str, List[Tuple[float, float, float]]] = {}
        self._history_max_age = 2.0  # seconds
        self._history_max_entries = 10

        # Load zones if config provided
        if zones_config_path:
            self.load_zones(zones_config_path)

    def load_zones(self, config_path: str) -> bool:
        """
        Load semantic zones from YAML configuration.

        Args:
            config_path: Path to zones YAML file

        Returns:
            True if loaded successfully
        """
        try:
            expanded_path = os.path.expanduser(config_path)
            with open(expanded_path, 'r') as f:
                config = yaml.safe_load(f)

            self.zones = []
            for zone_data in config.get('zones', []):
                bounds = zone_data.get('bounds', {})
                zone = SemanticZone(
                    name=zone_data.get('name', 'unnamed'),
                    zone_type=zone_data.get('type', 'unknown'),
                    x_min=bounds.get('x_min', -999),
                    x_max=bounds.get('x_max', 999),
                    y_min=bounds.get('y_min', -999),
                    y_max=bounds.get('y_max', 999),
                    weight_multiplier=zone_data.get('weight_multiplier', 1.0),
                    description=zone_data.get('description', '')
                )
                self.zones.append(zone)

            return True
        except Exception as e:
            print(f"Failed to load zones config: {e}")
            return False

    def classify(
        self,
        obstacle: Dict[str, Any],
        costmap_processor=None,
        previous_position: Optional[Tuple[float, float]] = None,
        dt: float = 0.5
    ) -> ObstacleClassification:
        """
        Classify an obstacle using full heuristics.

        Args:
            obstacle: Obstacle dict with at least 'x', 'y' keys
            costmap_processor: Optional CostmapProcessor for size estimation
            previous_position: Previous (x, y) of this obstacle for velocity calc
            dt: Time delta since previous position

        Returns:
            ObstacleClassification with type, weight, confidence, and reasoning
        """
        obs_x = obstacle.get('x', 0.0)
        obs_y = obstacle.get('y', 0.0)

        factors = []
        scores = {
            'human': 0.0,
            'vehicle': 0.0,
            'dynamic': 0.0,
            'furniture': 0.0,
            'wall': 0.0
        }

        # === 1. Zone-based classification ===
        zone_result = self._check_zone(obs_x, obs_y)
        if zone_result:
            zone, zone_type = zone_result
            if zone_type == 'human_zone':
                scores['human'] += 0.4
                factors.append(f"Located in '{zone.name}' (human zone)")
            elif zone_type == 'furniture_zone':
                scores['furniture'] += 0.3
                factors.append(f"Located in '{zone.name}' (furniture zone)")
            elif zone_type == 'restricted_zone':
                scores['wall'] += 0.3
                factors.append(f"Located in '{zone.name}' (restricted zone)")

        # === 2. Velocity-based classification ===
        velocity = self._estimate_velocity(obstacle, previous_position, dt)
        if velocity is not None:
            if velocity > self.VEHICLE_VELOCITY_THRESHOLD:
                scores['vehicle'] += 0.5
                scores['dynamic'] += 0.2
                factors.append(f"Moving fast ({velocity:.2f} m/s) - likely vehicle")
            elif self.HUMAN_VELOCITY_MIN < velocity <= self.HUMAN_VELOCITY_MAX:
                scores['human'] += 0.5
                scores['dynamic'] += 0.2
                factors.append(f"Walking speed ({velocity:.2f} m/s) - likely human")
            elif velocity > 0.1:
                scores['dynamic'] += 0.4
                factors.append(f"Moving slowly ({velocity:.2f} m/s) - dynamic object")
            else:
                scores['furniture'] += 0.2
                scores['wall'] += 0.1
                factors.append("Stationary object")

        # === 3. Size-based classification ===
        size = None
        if costmap_processor:
            size = self.estimate_size_from_costmap(obstacle, costmap_processor)
            if size is not None:
                if size < self.HUMAN_SIZE_MAX:
                    scores['human'] += 0.3
                    factors.append(f"Narrow profile ({size:.2f}m) - human-sized")
                elif size >= self.FURNITURE_SIZE_MIN:
                    scores['furniture'] += 0.3
                    factors.append(f"Wide profile ({size:.2f}m) - furniture-sized")
                else:
                    scores['dynamic'] += 0.1

        # === 4. Costmap lethality ===
        if obstacle.get('is_lethal', False):
            scores['wall'] += 0.2
            factors.append("Lethal cost in costmap - likely permanent obstacle")

        # === Determine final classification ===
        if not factors:
            factors.append("No distinguishing features detected")

        # Find highest scoring type
        best_type = max(scores, key=scores.get)
        best_score = scores[best_type]

        # If no strong signal, default to 'unknown'
        if best_score < 0.2:
            best_type = 'unknown'
            confidence = 0.3
            reasoning = "Insufficient data for confident classification"
        else:
            confidence = min(best_score, 1.0)
            reasoning = self._generate_reasoning(best_type, factors)

        # Calculate priority weight
        base_weight = self.weights.get(best_type, self.weights['unknown'])

        # Apply zone multiplier if in a zone
        zone_multiplier = 1.0
        zone_name = None
        if zone_result:
            zone, _ = zone_result
            zone_multiplier = zone.weight_multiplier
            zone_name = zone.name

        priority_weight = base_weight * zone_multiplier

        return ObstacleClassification(
            obstacle_type=best_type,
            priority_weight=priority_weight,
            confidence=confidence,
            reasoning=reasoning,
            contributing_factors=factors,
            zone_name=zone_name,
            estimated_velocity=velocity,
            estimated_size=size
        )

    def _check_zone(
        self,
        x: float,
        y: float
    ) -> Optional[Tuple[SemanticZone, str]]:
        """Check if position is within any semantic zone."""
        for zone in self.zones:
            if (zone.x_min <= x <= zone.x_max and
                zone.y_min <= y <= zone.y_max):
                return (zone, zone.zone_type)
        return None

    def _estimate_velocity(
        self,
        obstacle: Dict[str, Any],
        previous_position: Optional[Tuple[float, float]],
        dt: float
    ) -> Optional[float]:
        """
        Estimate obstacle velocity from position history.

        Args:
            obstacle: Current obstacle data
            previous_position: Previous (x, y) if known
            dt: Time delta

        Returns:
            Velocity in m/s, or None if cannot estimate
        """
        if previous_position is None:
            return None

        if dt <= 0:
            return None

        current_x = obstacle.get('x', 0.0)
        current_y = obstacle.get('y', 0.0)
        prev_x, prev_y = previous_position

        distance = math.sqrt((current_x - prev_x)**2 + (current_y - prev_y)**2)
        velocity = distance / dt

        return velocity

    def track_obstacle_position(
        self,
        obstacle_id: str,
        x: float,
        y: float,
        timestamp: Optional[float] = None
    ):
        """
        Track obstacle position for velocity estimation.

        Args:
            obstacle_id: Unique identifier for obstacle
            x, y: Current position
            timestamp: Current time (defaults to time.time())
        """
        if timestamp is None:
            timestamp = time.time()

        if obstacle_id not in self._position_history:
            self._position_history[obstacle_id] = []

        history = self._position_history[obstacle_id]
        history.append((timestamp, x, y))

        # Prune old entries
        cutoff = timestamp - self._history_max_age
        self._position_history[obstacle_id] = [
            entry for entry in history
            if entry[0] > cutoff
        ][-self._history_max_entries:]

    def get_velocity_from_history(
        self,
        obstacle_id: str
    ) -> Optional[float]:
        """
        Calculate velocity from tracked position history.

        Args:
            obstacle_id: Obstacle identifier

        Returns:
            Velocity in m/s, or None
        """
        if obstacle_id not in self._position_history:
            return None

        history = self._position_history[obstacle_id]
        if len(history) < 2:
            return None

        # Use first and last entries for velocity calculation
        t1, x1, y1 = history[0]
        t2, x2, y2 = history[-1]

        dt = t2 - t1
        if dt <= 0:
            return None

        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        return distance / dt

    def estimate_size_from_costmap(
        self,
        obstacle: Dict[str, Any],
        costmap_processor
    ) -> Optional[float]:
        """
        Estimate obstacle width from costmap cluster analysis.

        Args:
            obstacle: Obstacle data with x, y coordinates
            costmap_processor: CostmapProcessor instance

        Returns:
            Estimated width in meters, or None
        """
        if not costmap_processor.has_local_costmap:
            return None

        obs_x = obstacle.get('x', 0.0)
        obs_y = obstacle.get('y', 0.0)

        # Sample points around obstacle to find extent
        search_radius = 1.0  # meters
        samples = 16

        occupied_points = []

        for i in range(samples):
            angle = (2 * math.pi * i) / samples
            for r in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]:
                sample_x = obs_x + r * math.cos(angle)
                sample_y = obs_y + r * math.sin(angle)

                cost = costmap_processor.get_cost_at_position(
                    sample_x, sample_y, 'local'
                )

                if cost is not None and cost >= costmap_processor.INSCRIBED_OBSTACLE:
                    occupied_points.append((sample_x, sample_y))
                    break  # Found edge in this direction

        if len(occupied_points) < 3:
            return None

        # Estimate size as max distance between occupied points
        max_dist = 0.0
        for i, (x1, y1) in enumerate(occupied_points):
            for x2, y2 in occupied_points[i+1:]:
                dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                max_dist = max(max_dist, dist)

        return max_dist if max_dist > 0 else None

    def calculate_priority(
        self,
        classification: ObstacleClassification,
        distance_to_robot: float
    ) -> float:
        """
        Calculate final priority score considering distance.

        Higher score = more urgent to avoid.

        Args:
            classification: Obstacle classification result
            distance_to_robot: Distance from robot to obstacle in meters

        Returns:
            Priority score (higher = more urgent)
        """
        base_priority = classification.priority_weight

        # Distance factor: closer = higher priority
        # Uses inverse relationship with minimum distance clamp
        distance_factor = 1.0 / max(distance_to_robot, 0.1)

        # Velocity factor: moving obstacles more urgent
        velocity_factor = 1.0
        if classification.estimated_velocity is not None:
            velocity_factor = 1.0 + classification.estimated_velocity

        # Confidence factor: higher confidence = more reliable priority
        confidence_factor = 0.5 + (0.5 * classification.confidence)

        return base_priority * distance_factor * velocity_factor * confidence_factor

    def _generate_reasoning(
        self,
        obstacle_type: str,
        factors: List[str]
    ) -> str:
        """Generate human-readable reasoning for classification."""
        type_descriptions = {
            'human': "I believe this is a person",
            'vehicle': "This appears to be a moving vehicle",
            'dynamic': "This is a moving object",
            'furniture': "This appears to be furniture or a static object",
            'wall': "This is a wall or permanent structure",
            'unknown': "I'm not certain what this obstacle is"
        }

        base = type_descriptions.get(obstacle_type, "Unclassified obstacle")

        if factors:
            details = ". ".join(factors[:2])  # Limit to 2 factors for brevity
            return f"{base}. {details}."

        return base + "."

    def get_priority_explanation(
        self,
        classification: ObstacleClassification
    ) -> str:
        """
        Generate user-friendly priority explanation for XAI.

        Args:
            classification: Obstacle classification result

        Returns:
            Explanation suitable for display/speech
        """
        explanations = {
            'human': "As a potential person, I treat this with highest priority for safety.",
            'vehicle': "This moving vehicle requires extra caution.",
            'dynamic': "Since this is moving, I'm being extra careful.",
            'furniture': "This appears to be furniture, which I can safely navigate around.",
            'wall': "This is a permanent structure that I'll simply avoid.",
            'unknown': "I'm treating this with caution since I can't identify it."
        }

        return explanations.get(
            classification.obstacle_type,
            "I'm treating this obstacle with appropriate caution."
        )
