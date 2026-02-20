#!/usr/bin/env python3
"""
XAI Navigator Node - Main orchestrator for explainable navigation.

Week 3: Full decision logging implementation with all components integrated.
Week 4+: Extended with weighted obstacle classification (Tesla-style).

Integrates:
- Nav2Monitor: Action client and navigation event capture
- DecisionDatabase: Local SQLite storage
- CostmapProcessor: Costmap analysis
- PathAnalyzer: Path comparison
- ObstacleDetector: Obstacle detection with weighted classification
- BackendSync: Backend API synchronization

Topics:
    Subscriptions:
        /plan (nav_msgs/Path)
        /local_plan (nav_msgs/Path)
        /local_costmap/costmap (nav_msgs/OccupancyGrid)
        /global_costmap/costmap (nav_msgs/OccupancyGrid)
        /goal_pose (geometry_msgs/PoseStamped)
        /conversation/processed_command (std_msgs/String)

    Publications:
        /navigation/decision (std_msgs/String)
        /navigation/explanation (std_msgs/String)
        /navigation/explanation_detailed (std_msgs/String)
        /navigation/obstacle_classification (std_msgs/String)  # NEW
"""

import json
import time
import threading
from typing import Optional, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped

# Local modules
from .nav2_monitor import Nav2Monitor
from .decision_database import DecisionDatabase
from .costmap_processor import CostmapProcessor
from .path_analyzer import PathAnalyzer
from .obstacle_detector import ObstacleDetector
from .backend_sync import BackendSync
from .explanation_engine import ExplanationEngine, ExplanationContext
import asyncio
import os
import random # For mocking telemetry

class TelemetryMonitor:
    """
    Monitors system telemetry for Anomaly Detection (Week 3 Apple Standard).
    
    Captures:
    - Battery Voltage
    - CPU Usage
    - WiFi Signal Strength
    - Motor Currents
    """
    
    def __init__(self):
        self.battery_voltage = 12.6
        self.cpu_usage = 15.0
        self.wifi_signal = -45.0
        
    def get_snapshot(self) -> Dict[str, Any]:
        """Get current system telemetry snapshot."""
        # In a real robot, read from /battery_state, /diagnostics, etc.
        # Here we mock realistic values with slight noise
        
        self.battery_voltage = max(10.5, self.battery_voltage - 0.001) # Slow drain
        self.cpu_usage = max(5.0, min(100.0, self.cpu_usage + random.uniform(-2.0, 2.0)))
        self.wifi_signal = max(-90.0, min(-30.0, self.wifi_signal + random.uniform(-1.0, 1.0)))
        
        # Mock motor currents (4 wheels)
        currents = [
            abs(random.gauss(0.5, 0.1)),
            abs(random.gauss(0.5, 0.1)),
            abs(random.gauss(0.5, 0.1)),
            abs(random.gauss(0.5, 0.1))
        ]
        
        return {
            'battery': self.battery_voltage,
            'cpu': self.cpu_usage,
            'wifi': self.wifi_signal,
            'currents': currents
        }


class XAINavigatorNode(Node):
    """
    Main node for XAI-enabled navigation decision logging.

    Captures all navigation events from Nav2 and stores them
    for explanation generation (Week 4 will add Gemini integration).
    """

    def __init__(self):
        super().__init__('xai_navigator_node')

        # Declare parameters
        self.declare_parameter('enable_logging', True)
        self.declare_parameter('backend_url', 'http://localhost:8000')
        self.declare_parameter('sync_interval', 5.0)
        self.declare_parameter('explanation_level', 'detailed')
        # Check if use_sim_time is already declared (common in launch files)
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        # Load parameters
        self.enable_logging = self.get_parameter('enable_logging').value
        self.backend_url = self.get_parameter('backend_url').value
        self.sync_interval = self.get_parameter('sync_interval').value
        self.explanation_level = self.get_parameter('explanation_level').value
        
        # NEW: Explanation settings
        self.declare_parameter('enable_explanations', True)
        self.declare_parameter('gemini_api_key', '')
        self.declare_parameter('explanation_cache_enabled', True)
        self.declare_parameter('explanation_max_latency', 2.0)
        
        self.enable_explanations = self.get_parameter('enable_explanations').value
        self.gemini_api_key = self.get_parameter('gemini_api_key').value
        self.explanation_cache_enabled = self.get_parameter('explanation_cache_enabled').value

        # Callback group for parallel execution
        self.callback_group = ReentrantCallbackGroup()

        # Initialize components
        self._init_components()
        self._init_subscribers()
        self._init_publishers()
        self._init_timers()

        # State
        self.current_pose: Optional[PoseStamped] = None
        self.current_goal: Optional[PoseStamped] = None
        self.session_id: Optional[str] = None
        self._nav2_checked = False

        # Explanation templates
        self.templates = {
            'path_computed': "Planning path to ({goal_x:.2f}, {goal_y:.2f}). Distance: {distance:.2f}m",
            'obstacle_detected': "Detected obstacle at ({obs_x:.2f}, {obs_y:.2f}). {action}",
            'goal_reached': "Successfully arrived at ({goal_x:.2f}, {goal_y:.2f})",
            'goal_aborted': "Navigation failed: could not reach goal",
            'path_changed': "Path updated: {reason}. Length change: {change:+.2f}m",
            'recovery_triggered': "Recovery behavior activated: {behavior}",
        }

        self.get_logger().info('XAI Navigator Node initialized')
        self.get_logger().info(f'Logging: {self.enable_logging}, Backend: {self.backend_url}')

    def _init_components(self):
        """Initialize processing components."""
        # Nav2 monitor
        self.nav2_monitor = Nav2Monitor(
            self,
            decision_callback=self._handle_nav_decision
        )

        # Local database
        self.decision_db = DecisionDatabase()
        self.get_logger().info('Local decision database initialized')

        # Processors
        self.costmap_processor = CostmapProcessor()
        self.path_analyzer = PathAnalyzer(deviation_threshold=0.3)

        # NEW: Configure ObstacleDetector with weighted classification
        self.declare_parameter('enable_obstacle_weighting', True)
        self.declare_parameter('semantic_zones_config', 'semantic_zones.yaml')

        enable_weighting = self.get_parameter('enable_obstacle_weighting').value
        zones_config = self.get_parameter('semantic_zones_config').value

        # Build full path to zones config
        zones_path = None
        if zones_config:
            import ament_index_python
            try:
                pkg_share = ament_index_python.get_package_share_directory('xai_navigation_pkg')
                zones_path = os.path.join(pkg_share, 'config', zones_config)
                if not os.path.exists(zones_path):
                    # Try local path
                    zones_path = os.path.join(
                        os.path.dirname(__file__), '..', 'config', zones_config
                    )
            except Exception:
                zones_path = None

        self.obstacle_detector = ObstacleDetector(
            enable_classification=enable_weighting,
            zones_config_path=zones_path
        )

        if self.obstacle_detector.classification_enabled:
            self.get_logger().info('Weighted obstacle classification ENABLED')
        else:
            self.get_logger().info('Obstacle classification disabled or not available')

        self.telemetry_monitor = TelemetryMonitor() # NEW: Week 3 Apple Standard

        # Backend sync
        self.backend_sync = BackendSync(
            backend_url=self.backend_url,
            sync_interval=self.sync_interval
        )
        self.backend_sync.set_database(self.decision_db)
        self.backend_sync.start()
        self.get_logger().info('Backend sync service started')
        
        # Explanation Engine
        if not self.gemini_api_key:
            self.gemini_api_key = os.getenv('GEMINI_API_KEY', '')
            
        if self.gemini_api_key and self.enable_explanations:
            try:
                self.explanation_engine = ExplanationEngine(
                    gemini_api_key=self.gemini_api_key,
                    decision_db=self.decision_db,
                    cache_enabled=self.explanation_cache_enabled
                )
                self.get_logger().info('Explanation Engine initialized')
            except Exception as e:
                self.get_logger().error(f'Failed to init Explanation Engine: {e}')
                self.explanation_engine = None
        else:
            self.get_logger().warn('Explanation Engine disabled (no API key or disabled)')
            self.explanation_engine = None
            
        # Async loop for explanations
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(target=self._start_async_loop, daemon=True)
        self._loop_thread.start()

    def _start_async_loop(self):
        """Run asyncio loop in background thread."""
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _init_subscribers(self):
        """Initialize topic subscribers."""
        # Path plans
        self.plan_sub = self.create_subscription(
            Path, '/plan',
            self._plan_callback, 10,
            callback_group=self.callback_group
        )

        self.local_plan_sub = self.create_subscription(
            Path, '/local_plan',
            self._local_plan_callback, 10,
            callback_group=self.callback_group
        )

        # Costmaps
        self.local_costmap_sub = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap',
            self._local_costmap_callback, 10,
            callback_group=self.callback_group
        )

        self.global_costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap',
            self._global_costmap_callback, 10,
            callback_group=self.callback_group
        )

        # Goal pose
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose',
            self._goal_callback, 10,
            callback_group=self.callback_group
        )

        # Commands from conversation memory (Week 2 integration)
        self.command_sub = self.create_subscription(
            String, '/conversation/processed_command',
            self._command_callback, 10,
            callback_group=self.callback_group
        )

    def _init_publishers(self):
        """Initialize topic publishers."""
        # Decision events (for dashboard)
        self.decision_pub = self.create_publisher(
            String, '/navigation/decision', 10
        )

        # Explanations
        self.explanation_pub = self.create_publisher(
            String, '/navigation/explanation', 10
        )

        self.explanation_detailed_pub = self.create_publisher(
            String, '/navigation/explanation_detailed', 10
        )

        # NEW: Obstacle classification events (Week 4+)
        self.classification_pub = self.create_publisher(
            String, '/navigation/obstacle_classification', 10
        )

    def _init_timers(self):
        """Initialize timers."""
        # Check Nav2 connection after startup
        self.create_timer(
            2.0,
            self._check_nav2_connection,
            callback_group=self.callback_group
        )

        # Periodic obstacle check during navigation
        self.obstacle_timer = self.create_timer(
            0.5,  # 2Hz
            self._periodic_obstacle_check,
            callback_group=self.callback_group
        )

        # NEW: Telemetry logging (1Hz)
        self.telemetry_timer = self.create_timer(
            1.0,
            self._log_telemetry_snapshot,
            callback_group=self.callback_group
        )

    def _log_telemetry_snapshot(self):
        """Log system telemetry for anomaly detection."""
        if not self.enable_logging:
            return
            
        snapshot = self.telemetry_monitor.get_snapshot()
        
        try:
            self.decision_db.log_telemetry(
                self.session_id,
                snapshot['battery'],
                snapshot['cpu'],
                snapshot['wifi'],
                snapshot['currents']
            )
        except Exception as e:
            self.get_logger().debug(f'Telemetry log error: {e}')

    def _check_nav2_connection(self):
        """Check Nav2 connection once at startup."""
        if not self._nav2_checked:
            self._nav2_checked = True
            if self.nav2_monitor.wait_for_nav2(timeout_sec=5.0):
                self.get_logger().info('Nav2 connection established')
            else:
                self.get_logger().warn('Nav2 not available - will retry on goal send')

    # === Topic Callbacks ===

    def _plan_callback(self, msg: Path):
        """Handle global path plan updates."""
        # Check for significant path change
        if self.path_analyzer.previous_path:
            comparison = self.path_analyzer.compare_paths(
                self.path_analyzer.previous_path, msg
            )

            if comparison.get('is_significant'):
                reason = self.path_analyzer.detect_path_change_reason(
                    self.path_analyzer.previous_path, msg,
                    self.costmap_processor
                )

                self.get_logger().info(
                    f'Path changed: {reason} '
                    f'(deviation: {comparison["max_deviation"]:.2f}m)'
                )

                # Log path change decision
                self._log_decision('path_changed', {
                    'reason': reason,
                    'length_change': comparison['length_change'],
                    'max_deviation': comparison['max_deviation'],
                    'new_length': comparison['new_length']
                })

                # Store in database
                if self.enable_logging:
                    self.decision_db.log_path_change(
                        0,
                        comparison['old_length'],
                        comparison['new_length'],
                        comparison['max_deviation'],
                        reason
                    )

                # Publish explanation
                explanation = self.templates['path_changed'].format(
                    reason=reason,
                    change=comparison['length_change']
                )
                self._publish_explanation(explanation)

        # Update stored path
        self.path_analyzer.update_path(msg)

        # Publish path statistics
        stats = self.path_analyzer.get_path_statistics(msg)
        if stats and self.current_goal:
            explanation = self.templates['path_computed'].format(
                goal_x=self.current_goal.pose.position.x,
                goal_y=self.current_goal.pose.position.y,
                distance=stats['length']
            )
            self._publish_explanation(explanation)

    def _local_plan_callback(self, msg: Path):
        """Handle local trajectory updates."""
        pass

    def _local_costmap_callback(self, msg: OccupancyGrid):
        """Handle local costmap updates."""
        self.costmap_processor.update_costmap(msg, 'local')

    def _global_costmap_callback(self, msg: OccupancyGrid):
        """Handle global costmap updates."""
        self.costmap_processor.update_costmap(msg, 'global')

    def _goal_callback(self, msg: PoseStamped):
        """Handle new navigation goal from RViz or other sources."""
        self.current_goal = msg

        goal_str = f"({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
        self.get_logger().info(f'Goal received: {goal_str}')

        self._publish_explanation(f"Navigation goal set: {goal_str}")

        self._log_decision('goal_received', {
            'goal_x': msg.pose.position.x,
            'goal_y': msg.pose.position.y,
            'frame_id': msg.header.frame_id
        })

    def _command_callback(self, msg: String):
        """Handle navigation commands from conversation memory (Week 2)."""
        try:
            command = json.loads(msg.data)

            if command.get('action') == 'navigate':
                params = command.get('parameters', {})
                goal = params.get('goal')

                if goal:
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.pose.position.x = float(goal['x'])
                    pose.pose.position.y = float(goal['y'])
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.w = 1.0

                    self.session_id = command.get('session_id')
                    self.nav2_monitor.send_goal(pose)

                    self.get_logger().info(
                        f'Navigation started via command to ({goal["x"]:.2f}, {goal["y"]:.2f})'
                    )

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid command JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Command error: {e}')

    def _periodic_obstacle_check(self):
        """Periodic check for obstacles on current path with weighted classification."""
        if not self.nav2_monitor.is_navigating:
            return

        if not self.path_analyzer.previous_path:
            return

        if not self.costmap_processor.has_local_costmap:
            return

        # Use detect_and_classify for weighted detection
        detection = self.obstacle_detector.detect_and_classify(
            self.path_analyzer.previous_path,
            self.costmap_processor
        )

        if detection.get('detected') and detection.get('critical_count', 0) > 0:
            # Get highest priority obstacle (could be different from closest)
            highest_priority = detection.get('highest_priority') or detection.get('closest')

            if highest_priority:
                # Extract classification info
                classification = highest_priority.get('classification', {})
                obstacle_type = classification.get('obstacle_type', 'unknown')
                priority_weight = classification.get('priority_weight', 1.0)
                reasoning = classification.get('reasoning', '')

                self.get_logger().warn(
                    f'Obstacle detected: {obstacle_type} at '
                    f'({highest_priority["x"]:.2f}, {highest_priority["y"]:.2f}) '
                    f'[Priority: {priority_weight:.1f}]'
                )

                # Log decision with classification info
                self._log_decision('obstacle_detected', {
                    'obstacle_x': highest_priority['x'],
                    'obstacle_y': highest_priority['y'],
                    'severity': highest_priority.get('severity', 'critical'),
                    'count': detection['total_count'],
                    'obstacle_type': obstacle_type,
                    'priority_weight': priority_weight,
                    'classification_reasoning': reasoning
                })

                # Log to database with classification
                if self.enable_logging:
                    self.decision_db.log_obstacle_event(
                        0,
                        highest_priority['x'],
                        highest_priority['y'],
                        highest_priority.get('distance', 0),
                        highest_priority.get('severity', 'critical'),
                        'detected',
                        # Classification fields
                        obstacle_type=obstacle_type,
                        priority_weight=priority_weight,
                        classification_confidence=classification.get('confidence'),
                        classification_reasoning=reasoning,
                        zone_name=classification.get('zone_name'),
                        estimated_velocity=classification.get('estimated_velocity'),
                        estimated_size=classification.get('estimated_size')
                    )

                # Generate weighted explanation
                priority_explanation = self.obstacle_detector.get_priority_explanation(
                    highest_priority
                )

                # Format explanation with obstacle type
                if obstacle_type != 'unknown':
                    explanation = (
                        f"I detected a {obstacle_type} at "
                        f"({highest_priority['x']:.2f}, {highest_priority['y']:.2f}). "
                        f"{priority_explanation}"
                    )
                else:
                    explanation = self.templates['obstacle_detected'].format(
                        obs_x=highest_priority['x'],
                        obs_y=highest_priority['y'],
                        action="Monitoring for path changes."
                    )

                self._publish_explanation(explanation)

                # Publish classification event
                self._publish_classification(highest_priority, detection)

    def _publish_classification(self, obstacle: Dict[str, Any], detection: Dict[str, Any]):
        """Publish obstacle classification event for dashboard."""
        classification = obstacle.get('classification', {})

        msg = String()
        msg.data = json.dumps({
            'timestamp': time.time(),
            'obstacle': {
                'x': obstacle.get('x'),
                'y': obstacle.get('y'),
                'distance': obstacle.get('distance', 0),
                'severity': obstacle.get('severity', 'unknown')
            },
            'classification': {
                'type': classification.get('obstacle_type', 'unknown'),
                'priority_weight': classification.get('priority_weight', 1.0),
                'confidence': classification.get('confidence', 0.0),
                'reasoning': classification.get('reasoning', ''),
                'zone_name': classification.get('zone_name'),
                'velocity': classification.get('estimated_velocity'),
                'size': classification.get('estimated_size')
            },
            'detection_summary': {
                'total_count': detection.get('total_count', 0),
                'critical_count': detection.get('critical_count', 0),
                'classification_enabled': detection.get('classification_enabled', False)
            }
        })
        self.classification_pub.publish(msg)

    # === Decision Handling ===

    def _handle_nav_decision(self, decision: Dict[str, Any]):
        """Handle navigation decision event from Nav2Monitor."""
        if not self.enable_logging:
            return

        decision_type = decision.get('decision_type', 'unknown')
        decision['session_id'] = self.session_id

        if self.costmap_processor.has_local_costmap:
            current = decision.get('data', {})
            if current.get('current_x') and current.get('current_y'):
                region = self.costmap_processor.get_region_statistics(
                    current['current_x'],
                    current['current_y'],
                    radius=1.5
                )
                decision['local_region'] = region

        db_id = None
        try:
            db_id = self.decision_db.log_decision(decision, self.session_id)
            decision['db_id'] = db_id
        except Exception as e:
            self.get_logger().error(f'DB write error: {e}')

        self._publish_decision(decision)

        if decision_type == 'goal_reached':
            if self.current_goal:
                explanation = self.templates['goal_reached'].format(
                    goal_x=self.current_goal.pose.position.x,
                    goal_y=self.current_goal.pose.position.y
                )
            else:
                explanation = "Successfully arrived at destination."
            self._publish_explanation(explanation)

        elif decision_type == 'goal_aborted':
            self._publish_explanation(self.templates['goal_aborted'])

        self.get_logger().debug(f'Decision logged: {decision_type}')

        # Trigger explanation generation
        if self.explanation_engine and self.enable_explanations and db_id is not None:
            asyncio.run_coroutine_threadsafe(
                self._generate_explanation_async(db_id, decision),
                self._loop
            )



    async def _generate_explanation_async(self, decision_id: int, decision: Dict[str, Any]):
        """Generate explanation asynchronously."""
        try:
            explanation = await self.explanation_engine.explain_decision(decision_id)
            
            # Publish detailed explanation
            self._publish_explanation_detailed(explanation, decision)
            
            # Also publish simple text to /navigation/explanation for backward compatibility
            msg = String()
            msg.data = explanation.text
            self.explanation_pub.publish(msg)
            
            self.get_logger().info(
                f"Generated explanation ({explanation.generation_time:.2f}s): {explanation.text}"
            )
            
        except Exception as e:
            self.get_logger().error(f"Failed to generate explanation: {e}")

    def _publish_explanation_detailed(
        self,
        explanation: 'ExplanationResponse',
        decision: Dict[str, Any]
    ):
        """Publish detailed explanation with metadata."""
        msg = String()
        msg.data = json.dumps({
            'text': explanation.text,
            'confidence': explanation.confidence,
            'generation_time': explanation.generation_time,
            'cached': explanation.cached,
            'decision_type': decision.get('decision_type'),
            'timestamp': decision.get('timestamp')
        })
        self.explanation_detailed_pub.publish(msg)

    def _log_decision(self, decision_type: str, data: Dict[str, Any]):
        """Log a decision manually."""
        decision = {
            'decision_id': 0,
            'decision_type': decision_type,
            'data': data,
            'timestamp': time.time(),
            'goal': {
                'x': self.current_goal.pose.position.x,
                'y': self.current_goal.pose.position.y
            } if self.current_goal else None
        }
        self._handle_nav_decision(decision)

    # === Publishing ===

    def _publish_decision(self, decision: Dict[str, Any]):
        """Publish decision event to topic."""
        msg = String()
        msg.data = json.dumps({
            'decision_type': decision.get('decision_type'),
            'timestamp': decision.get('timestamp'),
            'db_id': decision.get('db_id'),
            'goal': decision.get('goal')
        })
        self.decision_pub.publish(msg)

    def _publish_explanation(self, text: str):
        """Publish explanation to topics."""
        msg = String()
        msg.data = text
        self.explanation_pub.publish(msg)

        detailed_msg = String()
        detailed_msg.data = json.dumps({
            'simple': text,
            'timestamp': time.time(),
            'level': self.explanation_level
        })
        self.explanation_detailed_pub.publish(detailed_msg)

    # === Utilities ===

    def get_status(self) -> Dict[str, Any]:
        """Get comprehensive node status."""
        nav_status = self.nav2_monitor.get_status()
        db_stats = self.decision_db.get_statistics()
        sync_stats = self.backend_sync.get_statistics()

        return {
            'navigation': nav_status,
            'database': db_stats,
            'sync': sync_stats,
            'has_local_costmap': self.costmap_processor.has_local_costmap,
            'has_global_costmap': self.costmap_processor.has_global_costmap,
            'has_path': self.path_analyzer.previous_path is not None,
            'current_goal': {
                'x': self.current_goal.pose.position.x,
                'y': self.current_goal.pose.position.y
            } if self.current_goal else None
        }

    def destroy_node(self):
        """Clean up resources."""
        self.get_logger().info('Shutting down XAI Navigator Node...')
        self.backend_sync.stop()
        self.decision_db.close()
        if hasattr(self, '_loop') and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
            if hasattr(self, '_loop_thread'):
                self._loop_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = XAINavigatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
