#!/usr/bin/env python3
"""
Specialized Explanation Types

Implements domain-specific explanation logic for different
navigation scenarios. Each explainer adds specialized context
and post-processing for its decision type.
"""

from typing import Dict, Any, List, Optional
from dataclasses import dataclass
import math


@dataclass
class PathMetrics:
    """Metrics for path comparison."""
    original_length: float
    new_length: float
    length_difference: float
    max_deviation: float
    deviation_point: Optional[tuple] = None


class PathSelectionExplainer:
    """
    Specialized explainer for path selection decisions.
    
    Adds context about:
    - Path comparison metrics
    - Reason for change (obstacle, optimization, recovery)
    - Impact on navigation (time, distance, safety)
    """
    
    def __init__(self):
        """Initialize path explainer."""
        self.explanation_history: List[Dict[str, Any]] = []
    
    def add_context(
        self,
        decision_data: Dict[str, Any],
        base_context: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        Add path-specific context to explanation request.
        
        Args:
            decision_data: Raw decision data from Nav2
            base_context: Base context from ExplanationEngine
            
        Returns:
            Enhanced context dictionary
        """
        # Calculate path metrics
        metrics = self._calculate_path_metrics(decision_data)
        
        # Determine change reason
        reason = self._determine_change_reason(decision_data, metrics)
        
        # Add to context
        enhanced_context = {
            **base_context,
            'path_metrics': {
                'length_change': metrics.length_difference,
                'is_longer': metrics.length_difference > 0,
                'deviation_severity': self._categorize_deviation(metrics.max_deviation)
            },
            'change_reason': reason,
            'impact': self._assess_impact(metrics, reason)
        }
        
        return enhanced_context
    
    def _calculate_path_metrics(self, decision_data: Dict[str, Any]) -> PathMetrics:
        """Calculate metrics comparing old and new paths."""
        original_length = decision_data.get('original_length', 0.0)
        new_length = decision_data.get('new_length', 0.0)
        
        return PathMetrics(
            original_length=original_length,
            new_length=new_length,
            length_difference=new_length - original_length,
            max_deviation=decision_data.get('max_deviation', 0.0),
            deviation_point=decision_data.get('deviation_point')
        )
    
    def _determine_change_reason(
        self,
        decision_data: Dict[str, Any],
        metrics: PathMetrics
    ) -> str:
        """Determine why the path changed."""
        # Check explicit reason first
        if 'reason' in decision_data:
            return decision_data['reason']
        
        # Infer from metrics
        if metrics.length_difference < -0.2:
            return 'optimization'
        elif 'obstacle' in str(decision_data).lower():
            return 'obstacle_avoidance'
        elif metrics.max_deviation > 0.5:
            return 'significant_reroute'
        else:
            return 'minor_adjustment'
    
    def _categorize_deviation(self, deviation: float) -> str:
        """Categorize deviation severity."""
        if deviation < 0.2:
            return 'minimal'
        elif deviation < 0.5:
            return 'moderate'
        else:
            return 'significant'
    
    def _assess_impact(
        self,
        metrics: PathMetrics,
        reason: str
    ) -> Dict[str, Any]:
        """Assess the impact of the path change."""
        return {
            'time_impact': abs(metrics.length_difference) * 2,  # Rough seconds
            'distance_impact': abs(metrics.length_difference),
            'safety_benefit': reason == 'obstacle_avoidance',
            'efficiency_benefit': metrics.length_difference < 0
        }
    
    def post_process(
        self,
        explanation: str,
        decision_data: Dict[str, Any]
    ) -> str:
        """Post-process explanation to add path-specific details."""
        # Add numerical details if not present
        if not any(char.isdigit() for char in explanation):
            length_diff = decision_data.get('length_change', 0.0)
            if abs(length_diff) > 0.1:
                explanation += f" This adds about {abs(length_diff):.1f} meters to my path."
        
        return explanation


class ObstacleAvoidanceExplainer:
    """
    Specialized explainer for obstacle avoidance decisions.
    
    Adds context about:
    - Obstacle characteristics (size, distance, type)
    - Avoidance strategy (stop, go around, back up)
    - Safety considerations
    """
    
    def __init__(self):
        """Initialize obstacle explainer."""
        pass
    
    def add_context(
        self,
        decision_data: Dict[str, Any],
        base_context: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Add obstacle-specific context."""
        obstacle_info = self._analyze_obstacle(decision_data)
        avoidance_strategy = self._determine_strategy(decision_data, obstacle_info)
        
        enhanced_context = {
            **base_context,
            'obstacle': obstacle_info,
            'strategy': avoidance_strategy,
            'safety_priority': True
        }
        
        return enhanced_context
    
    def _analyze_obstacle(self, decision_data: Dict[str, Any]) -> Dict[str, Any]:
        """Analyze obstacle characteristics."""
        distance = decision_data.get('distance_to_robot', 999.0)
        severity = decision_data.get('severity', 'unknown')
        
        return {
            'distance': distance,
            'severity': severity,
            'is_critical': distance < 0.3,
            'location': self._describe_location(decision_data),
            'is_moving': decision_data.get('is_moving', False)
        }
    
    def _describe_location(self, decision_data: Dict[str, Any]) -> str:
        """Describe obstacle location in user-friendly terms."""
        # Get obstacle position relative to robot
        obs_x = decision_data.get('obstacle_x', 0.0)
        obs_y = decision_data.get('obstacle_y', 0.0)
        
        # Simple quadrant-based description
        if obs_y > 0.2:
            return "to my left"
        elif obs_y < -0.2:
            return "to my right"
        elif obs_x > 0:
            return "ahead of me"
        else:
            return "behind me"
    
    def _determine_strategy(
        self,
        decision_data: Dict[str, Any],
        obstacle_info: Dict[str, Any]
    ) -> str:
        """Determine avoidance strategy."""
        action = decision_data.get('action', 'unknown')
        
        if action == 'stop':
            return 'stopping and waiting'
        elif action == 'replan':
            return 'going around'
        elif action == 'backup':
            return 'backing up'
        else:
            return 'adjusting course'
    
    def post_process(
        self,
        explanation: str,
        decision_data: Dict[str, Any]
    ) -> str:
        """Post-process obstacle explanation."""
        # Ensure safety reassurance if stopped
        if 'stop' in explanation.lower() and 'safe' not in explanation.lower():
            explanation += " I'll continue when it's safe."
        
        return explanation


class GoalModificationExplainer:
    """
    Specialized explainer for goal modification/abortion.
    
    Adds context about:
    - Why goal was unreachable
    - Alternatives attempted
    - Suggested user actions
    """
    
    def __init__(self):
        """Initialize goal explainer."""
        pass
    
    def add_context(
        self,
        decision_data: Dict[str, Any],
        base_context: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Add goal-specific context."""
        failure_analysis = self._analyze_failure(decision_data)
        alternatives = self._suggest_alternatives(decision_data, failure_analysis)
        
        enhanced_context = {
            **base_context,
            'failure_reason': failure_analysis['reason'],
            'attempts_made': decision_data.get('attempts', 0),
            'closest_distance': decision_data.get('distance_from_goal', 999.0),
            'alternatives': alternatives
        }
        
        return enhanced_context
    
    def _analyze_failure(self, decision_data: Dict[str, Any]) -> Dict[str, Any]:
        """Analyze why the goal failed."""
        reason = decision_data.get('reason', 'unknown')
        
        # Categorize failure type
        if 'obstacle' in reason.lower() or 'blocked' in reason.lower():
            category = 'blocked'
        elif 'unreachable' in reason.lower() or 'invalid' in reason.lower():
            category = 'unreachable'
        elif 'timeout' in reason.lower():
            category = 'timeout'
        else:
            category = 'other'
        
        return {
            'reason': reason,
            'category': category,
            'is_temporary': category in ['blocked', 'timeout']
        }
    
    def _suggest_alternatives(
        self,
        decision_data: Dict[str, Any],
        failure_analysis: Dict[str, Any]
    ) -> List[str]:
        """Suggest alternative actions."""
        alternatives = []
        
        if failure_analysis['is_temporary']:
            alternatives.append("wait and try again")
        
        if failure_analysis['category'] == 'blocked':
            alternatives.append("choose a nearby location")
        
        alternatives.append("select a different goal")
        
        return alternatives
    
    def post_process(
        self,
        explanation: str,
        decision_data: Dict[str, Any]
    ) -> str:
        """Post-process goal explanation."""
        # Ensure question about next action if not present
        if '?' not in explanation:
            explanation += " What would you like me to do?"
        
        return explanation


class ExplanationTypeFactory:
    """
    Factory for creating specialized explainers.
    """
    
    @staticmethod
    def create_explainer(decision_type: str):
        """Create appropriate explainer for decision type."""
        if decision_type == 'path_changed':
            return PathSelectionExplainer()
        elif decision_type == 'obstacle_detected':
            return ObstacleAvoidanceExplainer()
        elif decision_type in ['goal_aborted', 'planning_failed']:
            return GoalModificationExplainer()
        else:
            return None  # Use default explanation
