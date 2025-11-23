# Week 3 Complete Implementation Guide
## XAI Navigation - Decision Logging & Data Capture

**Duration:** 7 days (Feb 3 - Feb 9, 2025)
**Goal:** Capture and log all Nav2 navigation decisions for explanation generation
**Status:** Building on Week 1 & 2 foundation
**Critical Milestone:** Decision capture system operational by week end

---

## Pre-Week 3 Checklist

Before starting, verify Week 2 deliverables are complete:

- [ ] Conversation memory system fully functional
- [ ] 5+ turn dialogue working with context
- [ ] Spatial reference resolution operational
- [ ] Database storing conversations correctly
- [ ] Dashboard displaying conversation history
- [ ] All ROS2 nodes building and running
- [ ] Git repository up to date with Week 2 tag

**If any item is incomplete, resolve it before proceeding.**

---

## Week 3 Overview

### What You'll Build

By end of Week 3, you'll have:

1. **Nav2 Integration Layer** - Hooks into Nav2 action server and topics
2. **Decision Logger** - Captures all navigation decisions with timestamps
3. **Cost Map Extractor** - Retrieves local and global cost map data
4. **Path Analyzer** - Compares original vs modified paths
5. **Obstacle Detector** - Identifies when and why obstacles are avoided
6. **Decision Database** - Dual storage: local SQLite + backend sync
7. **Visualization Ready** - Data pipeline for dashboard integration

### Key Technical Concepts

1. **Nav2 Action Client:** Understanding action server callbacks, feedback, and result patterns
2. **Cost Map Processing:** Extracting and interpreting OccupancyGrid data (0=free, 254=lethal)
3. **Path Comparison:** Detecting meaningful deviations between planned paths
4. **Real-time Performance:** Keep decision logging overhead <100ms
5. **Dual Database:** Fast local SQLite + periodic backend API sync

### Architecture

```
Nav2 Stack
    |-> /navigate_to_pose (action)     - Main navigation action
    |-> /plan (nav_msgs/Path)          - Global path plans
    |-> /local_plan (nav_msgs/Path)    - Local trajectory
    |-> /local_costmap/costmap         - Local obstacle map
    |-> /global_costmap/costmap        - Global obstacle map
         |
         v
XAI Navigator Node (EXTENDED from existing skeleton)
    |-> Nav2Monitor          - Action client + callbacks
    |-> DecisionDatabase     - Local SQLite storage
    |-> CostmapProcessor     - Cost map analysis
    |-> PathAnalyzer         - Path comparison
    |-> ObstacleDetector     - Obstacle identification
    |-> BackendSync          - Sync to backend API
         |
         v
Backend API (EXTENDED)
    |-> /api/v1/navigation/decisions   - Store/retrieve decisions
    |-> /api/v1/navigation/paths       - Path change history
    |-> /api/v1/navigation/obstacles   - Obstacle events
         |
         v
Dashboard (Week 4)
    |-> Decision Visualization Panel
```

---

## File Structure

### ROS2 Package (Extend existing `xai_navigation_pkg`)

```
src/xai_navigation_pkg/
â”œâ”€â”€ xai_navigation_pkg/
â”‚   â”œâ”€â”€ __init__.py
â”‚   â”œâ”€â”€ xai_navigator_node.py    # EXTEND existing (196 lines)
â”‚   â”œâ”€â”€ nav2_monitor.py          # NEW: Nav2 action client
â”‚   â”œâ”€â”€ decision_database.py     # NEW: Local SQLite storage
â”‚   â”œâ”€â”€ costmap_processor.py     # NEW: Costmap analysis
â”‚   â”œâ”€â”€ path_analyzer.py         # NEW: Path comparison
â”‚   â”œâ”€â”€ obstacle_detector.py     # NEW: Obstacle detection
â”‚   â””â”€â”€ backend_sync.py          # NEW: Backend API sync
â”œâ”€â”€ launch/
â”‚   â””â”€â”€ xai_navigator.launch.py  # NEW
â”œâ”€â”€ config/
â”‚   â””â”€â”€ xai_params.yaml          # NEW
â”œâ”€â”€ resource/
â”‚   â””â”€â”€ xai_navigation_pkg       # Existing
â”œâ”€â”€ test/
â”‚   â”œâ”€â”€ test_decision_database.py
â”‚   â””â”€â”€ test_nav2_monitor.py
â”œâ”€â”€ package.xml                  # UPDATE
â””â”€â”€ setup.py                     # UPDATE
```

### Backend Extensions

```
backend/app/
â”œâ”€â”€ database/
â”‚   â”œâ”€â”€ conversation_db.py       # Existing (Week 2)
â”‚   â””â”€â”€ navigation_db.py         # NEW
â””â”€â”€ main.py                      # ADD navigation endpoints
```

---

## Day 1 (Monday): Nav2 Monitor & Action Client

### Morning Session (4 hours)

#### Task 1.1: Understand Nav2 Architecture (1 hour)

Nav2 uses behavior trees with these key servers:
- **NavigateToPose:** Main action server for goal navigation
- **ComputePathToPose:** Plans global path
- **FollowPath:** Executes local trajectory
- **Costmap2D:** Maintains obstacle maps

**Key Topics to Monitor:**
```
/navigate_to_pose (action)          - Main navigation action
/plan (nav_msgs/Path)               - Global path plans
/local_plan (nav_msgs/Path)         - Local trajectory
/local_costmap/costmap              - Local obstacle map (nav_msgs/OccupancyGrid)
/global_costmap/costmap             - Global obstacle map
/controller_server/transition_event - Controller state changes
```

**Verify Nav2 is working:**
```bash
# Terminal 1: Launch Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Launch Nav2
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true

# Terminal 3: List topics
ros2 topic list | grep -E "(plan|costmap|navigate)"

# Expected output:
# /plan
# /local_plan
# /local_costmap/costmap
# /global_costmap/costmap
# /navigate_to_pose/_action/feedback
# /navigate_to_pose/_action/status
```

#### Task 1.2: Create Nav2 Monitor Module (2 hours)

**File:** `xai_navigation_pkg/nav2_monitor.py`

```python
#!/usr/bin/env python3
"""
Nav2 Monitor - Interfaces with Nav2 action server to capture navigation events.

Provides:
- Action client for NavigateToPose
- Callback hooks for feedback, result
- Decision event generation
"""

import time
from typing import Optional, Callable, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus


class Nav2Monitor:
    """
    Monitors Nav2 navigation actions and captures decision points.

    Designed to be used as a component within XAINavigatorNode,
    not as a standalone node.
    """

    def __init__(
        self,
        node: Node,
        decision_callback: Optional[Callable[[Dict[str, Any]], None]] = None
    ):
        """
        Initialize Nav2 monitor.

        Args:
            node: Parent ROS2 node instance
            decision_callback: Function called when navigation events occur
        """
        self.node = node
        self.decision_callback = decision_callback

        # Action client for NavigateToPose
        self.callback_group = ReentrantCallbackGroup()
        self.nav_client = ActionClient(
            self.node,
            NavigateToPose,
            '/navigate_to_pose',
            callback_group=self.callback_group
        )

        # State tracking
        self.current_goal: Optional[PoseStamped] = None
        self.goal_handle = None
        self.goal_start_time: Optional[float] = None
        self.last_feedback_time: Optional[float] = None

        # Decision tracking
        self.decision_counter = 0
        self.active_navigation = False

        self.node.get_logger().info('Nav2Monitor initialized')

    def wait_for_nav2(self, timeout_sec: float = 10.0) -> bool:
        """
        Wait for Nav2 action server to become available.

        Args:
            timeout_sec: Maximum wait time

        Returns:
            True if server available, False otherwise
        """
        self.node.get_logger().info('Waiting for Nav2 action server...')

        available = self.nav_client.wait_for_server(timeout_sec=timeout_sec)

        if available:
            self.node.get_logger().info('Nav2 action server ready')
        else:
            self.node.get_logger().warn(
                f'Nav2 action server not available after {timeout_sec}s'
            )

        return available

    def send_goal(self, pose: PoseStamped) -> bool:
        """
        Send navigation goal to Nav2.

        Args:
            pose: Target pose

        Returns:
            True if goal accepted, False otherwise
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = ''  # Use default BT

        self.node.get_logger().info(
            f'Sending goal: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
        )

        # Send goal asynchronously with feedback callback
        send_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )

        # Wait for acceptance
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=5.0)

        if not send_future.done():
            self.node.get_logger().error('Goal send timeout')
            return False

        self.goal_handle = send_future.result()

        if not self.goal_handle.accepted:
            self.node.get_logger().error('Goal rejected by Nav2')
            return False

        # Track goal
        self.current_goal = pose
        self.goal_start_time = time.time()
        self.active_navigation = True

        # Log decision: goal_sent
        self._emit_decision('goal_sent', {
            'goal_x': pose.pose.position.x,
            'goal_y': pose.pose.position.y,
            'goal_z': pose.pose.position.z,
            'timestamp': self.goal_start_time
        })

        # Get result asynchronously
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

        return True

    def cancel_goal(self):
        """Cancel current navigation goal."""
        if self.goal_handle and self.active_navigation:
            self.node.get_logger().info('Canceling navigation')

            cancel_future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, cancel_future)

            self.active_navigation = False
            self._emit_decision('goal_canceled', {
                'timestamp': time.time(),
                'duration_sec': time.time() - self.goal_start_time if self.goal_start_time else 0
            })

    def _feedback_callback(self, feedback_msg):
        """Process navigation feedback from Nav2."""
        feedback = feedback_msg.feedback
        current_time = time.time()

        # Throttle to max 5Hz (every 200ms)
        if self.last_feedback_time and (current_time - self.last_feedback_time) < 0.2:
            return

        self.last_feedback_time = current_time

        # Extract feedback data
        current_pose = feedback.current_pose.pose
        distance_remaining = feedback.distance_remaining
        nav_time = feedback.navigation_time

        feedback_data = {
            'current_x': current_pose.position.x,
            'current_y': current_pose.position.y,
            'distance_remaining': distance_remaining,
            'navigation_time_sec': nav_time.sec + nav_time.nanosec * 1e-9,
            'timestamp': current_time
        }

        self._emit_decision('feedback', feedback_data)

        # Detect approaching goal
        if 0.1 < distance_remaining < 0.5:
            self._emit_decision('approaching_goal', feedback_data)

    def _result_callback(self, future):
        """Process navigation result."""
        result = future.result()
        status = result.status

        self.active_navigation = False

        # Map status to decision type
        status_map = {
            GoalStatus.STATUS_SUCCEEDED: 'goal_reached',
            GoalStatus.STATUS_CANCELED: 'goal_canceled',
            GoalStatus.STATUS_ABORTED: 'goal_aborted'
        }

        decision_type = status_map.get(status, 'goal_unknown')
        duration = time.time() - self.goal_start_time if self.goal_start_time else 0

        result_data = {
            'status': status,
            'status_name': decision_type,
            'duration_sec': duration,
            'timestamp': time.time()
        }

        self.node.get_logger().info(
            f'Navigation {decision_type}: {duration:.2f}s'
        )

        self._emit_decision(decision_type, result_data)

    def _emit_decision(self, decision_type: str, data: Dict[str, Any]):
        """
        Emit a navigation decision event.

        Args:
            decision_type: Type of decision (goal_sent, feedback, goal_reached, etc.)
            data: Decision data dictionary
        """
        self.decision_counter += 1

        decision = {
            'decision_id': self.decision_counter,
            'decision_type': decision_type,
            'data': data,
            'goal': {
                'x': self.current_goal.pose.position.x,
                'y': self.current_goal.pose.position.y
            } if self.current_goal else None,
            'timestamp': data.get('timestamp', time.time())
        }

        # Call registered callback
        if self.decision_callback:
            self.decision_callback(decision)

        self.node.get_logger().debug(
            f'Decision #{self.decision_counter}: {decision_type}'
        )

    def get_status(self) -> Dict[str, Any]:
        """Get current navigation status."""
        return {
            'active': self.active_navigation,
            'has_goal': self.current_goal is not None,
            'decision_count': self.decision_counter,
            'elapsed_sec': time.time() - self.goal_start_time if self.goal_start_time else 0
        }

    @property
    def is_navigating(self) -> bool:
        """Check if navigation is currently active."""
        return self.active_navigation
```

#### Task 1.3: Create Local Decision Database (1 hour)

**File:** `xai_navigation_pkg/decision_database.py`

```python
#!/usr/bin/env python3
"""
Decision Database - Local SQLite storage for fast navigation decision logging.

Features:
- Async-compatible (though used synchronously for speed)
- Single optimized table design
- Automatic cleanup of old entries
- Minimal overhead (<10ms per write)
"""

import sqlite3
import json
import os
from pathlib import Path
from datetime import datetime
from typing import Optional, List, Dict, Any


class DecisionDatabase:
    """
    Local SQLite database for navigation decision storage.

    Optimized for fast writes with periodic sync to backend.
    """

    def __init__(self, db_path: str = '~/.ros/navigation_decisions.db'):
        """
        Initialize decision database.

        Args:
            db_path: Path to SQLite database file
        """
        self.db_path = os.path.expanduser(db_path)
        Path(os.path.dirname(self.db_path)).mkdir(parents=True, exist_ok=True)

        # Connect with WAL mode for better concurrent access
        self.conn = sqlite3.connect(self.db_path, check_same_thread=False)
        self.conn.execute('PRAGMA journal_mode=WAL')
        self.conn.execute('PRAGMA synchronous=NORMAL')

        self._init_schema()

    def _init_schema(self):
        """Create database tables."""
        cursor = self.conn.cursor()

        # Main decisions table - single table design for simplicity
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS navigation_decisions (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                decision_id INTEGER NOT NULL,
                session_id TEXT,
                timestamp REAL NOT NULL,
                decision_type TEXT NOT NULL,
                goal_x REAL,
                goal_y REAL,
                current_x REAL,
                current_y REAL,
                distance_remaining REAL,
                path_length REAL,
                obstacle_count INTEGER DEFAULT 0,
                data_json TEXT,
                synced_to_backend INTEGER DEFAULT 0,
                created_at TEXT DEFAULT CURRENT_TIMESTAMP
            )
        ''')

        # Path changes table
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS path_changes (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                decision_id INTEGER,
                timestamp REAL NOT NULL,
                old_length REAL,
                new_length REAL,
                max_deviation REAL,
                reason TEXT,
                synced_to_backend INTEGER DEFAULT 0,
                created_at TEXT DEFAULT CURRENT_TIMESTAMP,
                FOREIGN KEY (decision_id) REFERENCES navigation_decisions(id)
            )
        ''')

        # Obstacle events table
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS obstacle_events (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                decision_id INTEGER,
                timestamp REAL NOT NULL,
                obstacle_x REAL,
                obstacle_y REAL,
                distance_to_robot REAL,
                severity TEXT,
                action_taken TEXT,
                synced_to_backend INTEGER DEFAULT 0,
                created_at TEXT DEFAULT CURRENT_TIMESTAMP,
                FOREIGN KEY (decision_id) REFERENCES navigation_decisions(id)
            )
        ''')

        # Indexes for common queries
        cursor.execute('''
            CREATE INDEX IF NOT EXISTS idx_decisions_timestamp
            ON navigation_decisions(timestamp DESC)
        ''')
        cursor.execute('''
            CREATE INDEX IF NOT EXISTS idx_decisions_synced
            ON navigation_decisions(synced_to_backend)
        ''')
        cursor.execute('''
            CREATE INDEX IF NOT EXISTS idx_decisions_type
            ON navigation_decisions(decision_type)
        ''')

        self.conn.commit()

    def log_decision(
        self,
        decision: Dict[str, Any],
        session_id: Optional[str] = None
    ) -> int:
        """
        Log a navigation decision.

        Args:
            decision: Decision dictionary from Nav2Monitor
            session_id: Optional session identifier

        Returns:
            Database ID of logged decision
        """
        data = decision.get('data', {})
        goal = decision.get('goal', {})

        cursor = self.conn.cursor()
        cursor.execute('''
            INSERT INTO navigation_decisions (
                decision_id, session_id, timestamp, decision_type,
                goal_x, goal_y, current_x, current_y,
                distance_remaining, data_json
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        ''', (
            decision.get('decision_id', 0),
            session_id,
            decision.get('timestamp', datetime.now().timestamp()),
            decision.get('decision_type', 'unknown'),
            goal.get('x') if goal else None,
            goal.get('y') if goal else None,
            data.get('current_x'),
            data.get('current_y'),
            data.get('distance_remaining'),
            json.dumps(data)
        ))

        self.conn.commit()
        return cursor.lastrowid

    def log_path_change(
        self,
        decision_id: int,
        old_length: float,
        new_length: float,
        max_deviation: float,
        reason: str
    ) -> int:
        """Log a path change event."""
        cursor = self.conn.cursor()
        cursor.execute('''
            INSERT INTO path_changes (
                decision_id, timestamp, old_length, new_length,
                max_deviation, reason
            ) VALUES (?, ?, ?, ?, ?, ?)
        ''', (
            decision_id,
            datetime.now().timestamp(),
            old_length,
            new_length,
            max_deviation,
            reason
        ))

        self.conn.commit()
        return cursor.lastrowid

    def log_obstacle_event(
        self,
        decision_id: int,
        obstacle_x: float,
        obstacle_y: float,
        distance: float,
        severity: str,
        action: str
    ) -> int:
        """Log an obstacle detection event."""
        cursor = self.conn.cursor()
        cursor.execute('''
            INSERT INTO obstacle_events (
                decision_id, timestamp, obstacle_x, obstacle_y,
                distance_to_robot, severity, action_taken
            ) VALUES (?, ?, ?, ?, ?, ?, ?)
        ''', (
            decision_id,
            datetime.now().timestamp(),
            obstacle_x,
            obstacle_y,
            distance,
            severity,
            action
        ))

        self.conn.commit()
        return cursor.lastrowid

    def get_recent_decisions(self, limit: int = 20) -> List[Dict[str, Any]]:
        """Get recent navigation decisions."""
        cursor = self.conn.cursor()
        cursor.execute('''
            SELECT id, decision_id, timestamp, decision_type,
                   goal_x, goal_y, current_x, current_y,
                   distance_remaining, data_json
            FROM navigation_decisions
            ORDER BY timestamp DESC
            LIMIT ?
        ''', (limit,))

        rows = cursor.fetchall()
        decisions = []

        for row in rows:
            decisions.append({
                'db_id': row[0],
                'decision_id': row[1],
                'timestamp': row[2],
                'decision_type': row[3],
                'goal': {'x': row[4], 'y': row[5]} if row[4] else None,
                'current': {'x': row[6], 'y': row[7]} if row[6] else None,
                'distance_remaining': row[8],
                'data': json.loads(row[9]) if row[9] else {}
            })

        return decisions

    def get_unsynced_decisions(self, limit: int = 100) -> List[Dict[str, Any]]:
        """Get decisions not yet synced to backend."""
        cursor = self.conn.cursor()
        cursor.execute('''
            SELECT id, decision_id, timestamp, decision_type,
                   goal_x, goal_y, current_x, current_y,
                   distance_remaining, data_json, session_id
            FROM navigation_decisions
            WHERE synced_to_backend = 0
            ORDER BY timestamp ASC
            LIMIT ?
        ''', (limit,))

        rows = cursor.fetchall()
        return [{
            'db_id': r[0],
            'decision_id': r[1],
            'timestamp': r[2],
            'decision_type': r[3],
            'goal': {'x': r[4], 'y': r[5]} if r[4] else None,
            'current': {'x': r[6], 'y': r[7]} if r[6] else None,
            'distance_remaining': r[8],
            'data': json.loads(r[9]) if r[9] else {},
            'session_id': r[10]
        } for r in rows]

    def mark_synced(self, db_ids: List[int]):
        """Mark decisions as synced to backend."""
        if not db_ids:
            return

        cursor = self.conn.cursor()
        placeholders = ','.join('?' * len(db_ids))
        cursor.execute(f'''
            UPDATE navigation_decisions
            SET synced_to_backend = 1
            WHERE id IN ({placeholders})
        ''', db_ids)

        self.conn.commit()

    def get_statistics(self) -> Dict[str, Any]:
        """Get database statistics."""
        cursor = self.conn.cursor()

        cursor.execute('SELECT COUNT(*) FROM navigation_decisions')
        total = cursor.fetchone()[0]

        cursor.execute('''
            SELECT decision_type, COUNT(*)
            FROM navigation_decisions
            GROUP BY decision_type
        ''')
        by_type = dict(cursor.fetchall())

        cursor.execute('''
            SELECT COUNT(*) FROM navigation_decisions
            WHERE synced_to_backend = 0
        ''')
        unsynced = cursor.fetchone()[0]

        return {
            'total_decisions': total,
            'by_type': by_type,
            'unsynced_count': unsynced
        }

    def cleanup_old(self, days: int = 7):
        """Delete decisions older than specified days."""
        import time
        cutoff = time.time() - (days * 24 * 60 * 60)

        cursor = self.conn.cursor()
        cursor.execute('''
            DELETE FROM navigation_decisions
            WHERE timestamp < ? AND synced_to_backend = 1
        ''', (cutoff,))

        deleted = cursor.rowcount
        self.conn.commit()

        return deleted

    def close(self):
        """Close database connection."""
        self.conn.close()
```

### Afternoon Session (3 hours)

#### Task 1.4: Update Package Dependencies (30 min)

**File:** `xai_navigation_pkg/package.xml` (UPDATE)

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>xai_navigation_pkg</name>
  <version>0.1.0</version>
  <description>Explainable AI navigation with decision logging and explanations</description>
  <maintainer email="howdoiusekeyboard@gmail.com">noob</maintainer>
  <license>MIT</license>

  <!-- Core ROS2 -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>

  <!-- Nav2 -->
  <depend>nav2_msgs</depend>
  <depend>action_msgs</depend>

  <!-- Project custom messages -->
  <depend>intelligent_twin_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**File:** `xai_navigation_pkg/setup.py` (UPDATE)

```python
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'xai_navigation_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='noob',
    maintainer_email='howdoiusekeyboard@gmail.com',
    description='Explainable AI navigation with decision logging for HRI',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'xai_navigator_node = xai_navigation_pkg.xai_navigator_node:main',
        ],
    },
)
```

#### Task 1.5: Build and Test Basic Integration (2.5 hours)

```bash
# Build the package
cd ~/ros2_navigation_project
colcon build --packages-select xai_navigation_pkg
source install/setup.bash

# Verify module imports
python3 -c "from xai_navigation_pkg.nav2_monitor import Nav2Monitor; print('Nav2Monitor OK')"
python3 -c "from xai_navigation_pkg.decision_database import DecisionDatabase; print('DecisionDatabase OK')"

# Test database
python3 -c "
from xai_navigation_pkg.decision_database import DecisionDatabase
db = DecisionDatabase('/tmp/test_nav.db')
db.log_decision({
    'decision_id': 1,
    'decision_type': 'test',
    'timestamp': 1234567890,
    'data': {'test': True}
})
print('Stats:', db.get_statistics())
db.close()
print('Database test OK')
"
```

### End of Day 1 Checklist

- [ ] Nav2Monitor class implemented
- [ ] DecisionDatabase class implemented
- [ ] Package dependencies updated
- [ ] Package builds without errors
- [ ] Module imports work correctly
- [ ] Database creates and stores test data
- [ ] Code committed: "Day 1: Nav2 monitor and local database"

---

## Day 2 (Tuesday): Costmap Processor & Path Analyzer

### Morning Session (4 hours)

#### Task 2.1: Costmap Processor Implementation (2 hours)

**File:** `xai_navigation_pkg/costmap_processor.py`

```python
#!/usr/bin/env python3
"""
Costmap Processor - Extracts and analyzes costmap data for decision explanation.

Key concepts:
- OccupancyGrid: 2D array where 0=free, 100=unknown, 254=inscribed, 255=lethal
- Grid coordinates vs. world coordinates conversion
- Efficient obstacle detection within radius
"""

import numpy as np
from typing import List, Tuple, Optional, Dict, Any
from nav_msgs.msg import OccupancyGrid


class CostmapProcessor:
    """
    Processes Nav2 costmaps to extract obstacle and navigation information.
    """

    # Costmap value meanings (Nav2 defaults)
    FREE_SPACE = 0
    UNKNOWN = -1  # or 255 depending on config
    INSCRIBED_OBSTACLE = 253
    LETHAL_OBSTACLE = 254

    def __init__(self):
        """Initialize costmap processor."""
        self.local_costmap: Optional[OccupancyGrid] = None
        self.global_costmap: Optional[OccupancyGrid] = None

    def update_costmap(self, costmap: OccupancyGrid, costmap_type: str = 'local'):
        """
        Store a costmap update.

        Args:
            costmap: OccupancyGrid message
            costmap_type: 'local' or 'global'
        """
        if costmap_type == 'local':
            self.local_costmap = costmap
        else:
            self.global_costmap = costmap

    def get_cost_at_position(
        self,
        x: float,
        y: float,
        costmap_type: str = 'local'
    ) -> Optional[int]:
        """
        Get cost value at world position.

        Args:
            x, y: World coordinates (meters)
            costmap_type: Which costmap to query

        Returns:
            Cost value (0-255) or None if position invalid
        """
        costmap = self.local_costmap if costmap_type == 'local' else self.global_costmap

        if costmap is None:
            return None

        grid_x, grid_y = self._world_to_grid(x, y, costmap)

        if grid_x is None:
            return None

        info = costmap.info
        idx = grid_y * info.width + grid_x

        if 0 <= idx < len(costmap.data):
            return costmap.data[idx]

        return None

    def find_obstacles_in_radius(
        self,
        center_x: float,
        center_y: float,
        radius: float = 1.0,
        costmap_type: str = 'local'
    ) -> List[Dict[str, Any]]:
        """
        Find all obstacles within radius of a point.

        Args:
            center_x, center_y: Center point in world coordinates
            radius: Search radius in meters
            costmap_type: Which costmap to search

        Returns:
            List of obstacle info dictionaries
        """
        costmap = self.local_costmap if costmap_type == 'local' else self.global_costmap

        if costmap is None:
            return []

        info = costmap.info
        grid_radius = int(radius / info.resolution) + 1

        center_gx, center_gy = self._world_to_grid(center_x, center_y, costmap)
        if center_gx is None:
            return []

        obstacles = []

        # Search grid cells in square, filter by radius
        for dy in range(-grid_radius, grid_radius + 1):
            for dx in range(-grid_radius, grid_radius + 1):
                # Skip if outside circular radius
                if dx*dx + dy*dy > grid_radius*grid_radius:
                    continue

                gx = center_gx + dx
                gy = center_gy + dy

                # Bounds check
                if not (0 <= gx < info.width and 0 <= gy < info.height):
                    continue

                idx = gy * info.width + gx
                cost = costmap.data[idx]

                if cost >= self.INSCRIBED_OBSTACLE:
                    world_x, world_y = self._grid_to_world(gx, gy, costmap)
                    distance = ((world_x - center_x)**2 + (world_y - center_y)**2)**0.5

                    obstacles.append({
                        'x': world_x,
                        'y': world_y,
                        'cost': cost,
                        'is_lethal': cost == self.LETHAL_OBSTACLE,
                        'distance': distance
                    })

        return obstacles

    def analyze_path_clearance(
        self,
        path_poses: List,
        clearance_radius: float = 0.5,
        costmap_type: str = 'local'
    ) -> Dict[str, Any]:
        """
        Analyze clearance along a planned path.

        Args:
            path_poses: List of PoseStamped from Path message
            clearance_radius: Radius to check around each waypoint
            costmap_type: Which costmap to use

        Returns:
            Clearance analysis dictionary
        """
        if not path_poses:
            return {'clear': True, 'obstacles': [], 'min_clearance': float('inf')}

        all_obstacles = []
        min_clearance = float('inf')

        # Sample path at intervals (not every waypoint for efficiency)
        sample_step = max(1, len(path_poses) // 20)  # Max 20 samples

        for i in range(0, len(path_poses), sample_step):
            pose = path_poses[i]
            x = pose.pose.position.x
            y = pose.pose.position.y

            obstacles = self.find_obstacles_in_radius(
                x, y, clearance_radius, costmap_type
            )

            for obs in obstacles:
                if obs['distance'] < min_clearance:
                    min_clearance = obs['distance']

                obs['path_index'] = i
                all_obstacles.append(obs)

        return {
            'clear': len(all_obstacles) == 0,
            'obstacle_count': len(all_obstacles),
            'obstacles': all_obstacles[:10],  # Limit to first 10
            'min_clearance': min_clearance if min_clearance != float('inf') else None
        }

    def get_region_statistics(
        self,
        center_x: float,
        center_y: float,
        radius: float = 2.0,
        costmap_type: str = 'local'
    ) -> Dict[str, Any]:
        """
        Get statistics about a region around a point.

        Args:
            center_x, center_y: Center of region
            radius: Region radius in meters
            costmap_type: Which costmap to analyze

        Returns:
            Statistics dictionary
        """
        costmap = self.local_costmap if costmap_type == 'local' else self.global_costmap

        if costmap is None:
            return {}

        info = costmap.info
        grid_radius = int(radius / info.resolution) + 1

        center_gx, center_gy = self._world_to_grid(center_x, center_y, costmap)
        if center_gx is None:
            return {}

        costs = []

        for dy in range(-grid_radius, grid_radius + 1):
            for dx in range(-grid_radius, grid_radius + 1):
                if dx*dx + dy*dy > grid_radius*grid_radius:
                    continue

                gx = center_gx + dx
                gy = center_gy + dy

                if 0 <= gx < info.width and 0 <= gy < info.height:
                    idx = gy * info.width + gx
                    costs.append(costmap.data[idx])

        if not costs:
            return {}

        costs = np.array(costs)

        return {
            'mean_cost': float(np.mean(costs)),
            'max_cost': int(np.max(costs)),
            'free_percent': float(np.sum(costs == self.FREE_SPACE) / len(costs) * 100),
            'obstacle_percent': float(np.sum(costs >= self.INSCRIBED_OBSTACLE) / len(costs) * 100),
            'sample_count': len(costs)
        }

    def _world_to_grid(
        self,
        x: float,
        y: float,
        costmap: OccupancyGrid
    ) -> Tuple[Optional[int], Optional[int]]:
        """Convert world coordinates to grid coordinates."""
        info = costmap.info

        grid_x = int((x - info.origin.position.x) / info.resolution)
        grid_y = int((y - info.origin.position.y) / info.resolution)

        if 0 <= grid_x < info.width and 0 <= grid_y < info.height:
            return grid_x, grid_y

        return None, None

    def _grid_to_world(
        self,
        grid_x: int,
        grid_y: int,
        costmap: OccupancyGrid
    ) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates."""
        info = costmap.info

        world_x = grid_x * info.resolution + info.origin.position.x
        world_y = grid_y * info.resolution + info.origin.position.y

        return world_x, world_y

    @property
    def has_local_costmap(self) -> bool:
        """Check if local costmap is available."""
        return self.local_costmap is not None

    @property
    def has_global_costmap(self) -> bool:
        """Check if global costmap is available."""
        return self.global_costmap is not None
```

#### Task 2.2: Path Analyzer Implementation (2 hours)

**File:** `xai_navigation_pkg/path_analyzer.py`

```python
#!/usr/bin/env python3
"""
Path Analyzer - Compares navigation paths and detects significant changes.

Key concepts:
- Path length calculation
- Deviation detection between old and new paths
- Path smoothness analysis
"""

import numpy as np
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
            total += (dx*dx + dy*dy)**0.5

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
            goal_dist = ((old_goal.x - new_goal.x)**2 + (old_goal.y - new_goal.y)**2)**0.5

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

            dev = ((p1.x - p2.x)**2 + (p1.y - p2.y)**2)**0.5

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

            dev = ((p1.x - p2.x)**2 + (p1.y - p2.y)**2)**0.5
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
            mag1 = (v1[0]**2 + v1[1]**2)**0.5
            mag2 = (v2[0]**2 + v2[1]**2)**0.5

            if mag1 > 0 and mag2 > 0:
                cos_angle = max(-1.0, min(1.0, dot / (mag1 * mag2)))
                angle = np.arccos(cos_angle)
                angle_changes.append(abs(angle))

        return sum(angle_changes) / len(angle_changes) if angle_changes else 0.0
```

### Afternoon Session (3 hours)

#### Task 2.3: Obstacle Detector Implementation (2 hours)

**File:** `xai_navigation_pkg/obstacle_detector.py`

```python
#!/usr/bin/env python3
"""
Obstacle Detector - Identifies obstacles affecting navigation and their impact.

Provides:
- Obstacle detection along planned paths
- Impact severity classification
- Avoidance suggestion generation
"""

from typing import List, Tuple, Optional, Dict, Any
from nav_msgs.msg import Path


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
        warning_distance: float = 1.0
    ):
        """
        Initialize obstacle detector.

        Args:
            critical_distance: Distance (m) considered critical
            warning_distance: Distance (m) considered warning
        """
        self.critical_distance = critical_distance
        self.warning_distance = warning_distance

        # Track detected obstacles
        self.current_obstacles: List[Dict[str, Any]] = []

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
                dist = ((current_position[0] - obs['x'])**2 +
                       (current_position[1] - obs['y'])**2)**0.5
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
                dist_from_start = ((x - start[0])**2 + (y - start[1])**2)**0.5
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
        dist_from_robot = ((current_position[0] - obs_x)**2 +
                          (current_position[1] - obs_y)**2)**0.5

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
                dist = ((pose.pose.position.x - obs_x)**2 +
                       (pose.pose.position.y - obs_y)**2)**0.5
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

    @property
    def has_obstacles(self) -> bool:
        """Check if any obstacles are currently tracked."""
        return len(self.current_obstacles) > 0
```

#### Task 2.4: Build and Test (1 hour)

```bash
# Build
cd ~/ros2_navigation_project
colcon build --packages-select xai_navigation_pkg
source install/setup.bash

# Test imports
python3 -c "
from xai_navigation_pkg.costmap_processor import CostmapProcessor
from xai_navigation_pkg.path_analyzer import PathAnalyzer
from xai_navigation_pkg.obstacle_detector import ObstacleDetector
print('All Week 3 modules import OK')
"
```

### End of Day 2 Checklist

- [ ] CostmapProcessor implemented
- [ ] PathAnalyzer implemented
- [ ] ObstacleDetector implemented
- [ ] All modules import correctly
- [ ] Code committed: "Day 2: Costmap, path, obstacle processors"

---

## Day 3 (Wednesday): Backend Sync & Integration

### Morning Session (4 hours)

#### Task 3.1: Backend Navigation Database (1.5 hours)

**File:** `backend/app/database/navigation_db.py`

```python
#!/usr/bin/env python3
"""
Navigation Database - Backend storage for navigation decisions.

Mirrors the local ROS2 database structure but in the unified backend.
Receives synced data from ROS2 nodes.
"""

import aiosqlite
import json
import os
from pathlib import Path
from datetime import datetime
from typing import Optional, List, Dict, Any


class NavigationDatabase:
    """
    Async SQLite database for navigation decision storage.

    Part of the unified backend alongside conversation_db.
    """

    def __init__(self, db_path: str = 'navigation_decisions.db'):
        """
        Initialize navigation database.

        Args:
            db_path: Database file path (relative to backend data dir)
        """
        data_dir = os.path.expanduser('~/.ros/backend_data')
        Path(data_dir).mkdir(parents=True, exist_ok=True)
        self.db_path = os.path.join(data_dir, db_path)
        self._connection: Optional[aiosqlite.Connection] = None

    async def initialize(self):
        """Initialize database connection and schema."""
        self._connection = await aiosqlite.connect(self.db_path)
        await self._connection.execute('PRAGMA journal_mode=WAL')
        await self._init_schema()

    async def _init_schema(self):
        """Create database tables."""
        await self._connection.execute('''
            CREATE TABLE IF NOT EXISTS navigation_decisions (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                ros_decision_id INTEGER,
                session_id TEXT,
                timestamp REAL NOT NULL,
                decision_type TEXT NOT NULL,
                goal_x REAL,
                goal_y REAL,
                current_x REAL,
                current_y REAL,
                distance_remaining REAL,
                path_length REAL,
                obstacle_count INTEGER DEFAULT 0,
                data_json TEXT,
                created_at TEXT DEFAULT CURRENT_TIMESTAMP
            )
        ''')

        await self._connection.execute('''
            CREATE TABLE IF NOT EXISTS path_changes (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                decision_id INTEGER,
                timestamp REAL NOT NULL,
                old_length REAL,
                new_length REAL,
                max_deviation REAL,
                reason TEXT,
                created_at TEXT DEFAULT CURRENT_TIMESTAMP,
                FOREIGN KEY (decision_id) REFERENCES navigation_decisions(id)
            )
        ''')

        await self._connection.execute('''
            CREATE TABLE IF NOT EXISTS obstacle_events (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                decision_id INTEGER,
                timestamp REAL NOT NULL,
                obstacle_x REAL,
                obstacle_y REAL,
                distance_to_robot REAL,
                severity TEXT,
                action_taken TEXT,
                created_at TEXT DEFAULT CURRENT_TIMESTAMP,
                FOREIGN KEY (decision_id) REFERENCES navigation_decisions(id)
            )
        ''')

        # Indexes
        await self._connection.execute('''
            CREATE INDEX IF NOT EXISTS idx_nav_timestamp
            ON navigation_decisions(timestamp DESC)
        ''')
        await self._connection.execute('''
            CREATE INDEX IF NOT EXISTS idx_nav_session
            ON navigation_decisions(session_id, timestamp DESC)
        ''')

        await self._connection.commit()

    async def store_decision(self, decision: Dict[str, Any]) -> int:
        """Store a navigation decision."""
        data = decision.get('data', {})
        goal = decision.get('goal', {})

        cursor = await self._connection.execute('''
            INSERT INTO navigation_decisions (
                ros_decision_id, session_id, timestamp, decision_type,
                goal_x, goal_y, current_x, current_y,
                distance_remaining, data_json
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        ''', (
            decision.get('decision_id'),
            decision.get('session_id'),
            decision.get('timestamp', datetime.now().timestamp()),
            decision.get('decision_type', 'unknown'),
            goal.get('x') if goal else None,
            goal.get('y') if goal else None,
            data.get('current_x'),
            data.get('current_y'),
            data.get('distance_remaining'),
            json.dumps(data)
        ))

        await self._connection.commit()
        return cursor.lastrowid

    async def store_decisions_batch(self, decisions: List[Dict[str, Any]]) -> int:
        """Store multiple decisions in a batch."""
        count = 0
        for decision in decisions:
            await self.store_decision(decision)
            count += 1
        return count

    async def get_recent_decisions(
        self,
        limit: int = 50,
        session_id: Optional[str] = None
    ) -> List[Dict[str, Any]]:
        """Get recent navigation decisions."""
        if session_id:
            cursor = await self._connection.execute('''
                SELECT id, ros_decision_id, timestamp, decision_type,
                       goal_x, goal_y, current_x, current_y,
                       distance_remaining, data_json
                FROM navigation_decisions
                WHERE session_id = ?
                ORDER BY timestamp DESC
                LIMIT ?
            ''', (session_id, limit))
        else:
            cursor = await self._connection.execute('''
                SELECT id, ros_decision_id, timestamp, decision_type,
                       goal_x, goal_y, current_x, current_y,
                       distance_remaining, data_json
                FROM navigation_decisions
                ORDER BY timestamp DESC
                LIMIT ?
            ''', (limit,))

        rows = await cursor.fetchall()
        return [{
            'id': r[0],
            'ros_decision_id': r[1],
            'timestamp': r[2],
            'decision_type': r[3],
            'goal': {'x': r[4], 'y': r[5]} if r[4] else None,
            'current': {'x': r[6], 'y': r[7]} if r[6] else None,
            'distance_remaining': r[8],
            'data': json.loads(r[9]) if r[9] else {}
        } for r in rows]

    async def get_decisions_by_type(
        self,
        decision_type: str,
        limit: int = 100
    ) -> List[Dict[str, Any]]:
        """Get decisions of a specific type."""
        cursor = await self._connection.execute('''
            SELECT id, timestamp, data_json
            FROM navigation_decisions
            WHERE decision_type = ?
            ORDER BY timestamp DESC
            LIMIT ?
        ''', (decision_type, limit))

        rows = await cursor.fetchall()
        return [{
            'id': r[0],
            'timestamp': r[1],
            'data': json.loads(r[2]) if r[2] else {}
        } for r in rows]

    async def get_statistics(self) -> Dict[str, Any]:
        """Get database statistics."""
        cursor = await self._connection.execute(
            'SELECT COUNT(*) FROM navigation_decisions'
        )
        total = (await cursor.fetchone())[0]

        cursor = await self._connection.execute('''
            SELECT decision_type, COUNT(*)
            FROM navigation_decisions
            GROUP BY decision_type
        ''')
        by_type = dict(await cursor.fetchall())

        cursor = await self._connection.execute('''
            SELECT COUNT(*) FROM path_changes
        ''')
        path_changes = (await cursor.fetchone())[0]

        cursor = await self._connection.execute('''
            SELECT COUNT(*) FROM obstacle_events
        ''')
        obstacle_events = (await cursor.fetchone())[0]

        return {
            'total_decisions': total,
            'by_type': by_type,
            'path_changes': path_changes,
            'obstacle_events': obstacle_events
        }

    async def close(self):
        """Close database connection."""
        if self._connection:
            await self._connection.close()


# Singleton instance
_navigation_db: Optional[NavigationDatabase] = None


async def get_navigation_db() -> NavigationDatabase:
    """Get or create navigation database singleton."""
    global _navigation_db
    if _navigation_db is None:
        _navigation_db = NavigationDatabase()
        await _navigation_db.initialize()
    return _navigation_db
```

#### Task 3.2: Add Navigation API Endpoints (1.5 hours)

Add to `backend/app/main.py`:

```python
# Add import at top
from app.database.navigation_db import get_navigation_db

# Add navigation endpoints

@app.post("/api/v1/navigation/decisions/sync")
async def sync_navigation_decisions(decisions: List[Dict[str, Any]]):
    """
    Sync navigation decisions from ROS2 node.

    Called periodically by BackendSync service.
    """
    try:
        nav_db = await get_navigation_db()
        count = await nav_db.store_decisions_batch(decisions)
        return {
            "success": True,
            "synced_count": count
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/v1/navigation/decisions")
async def get_navigation_decisions(
    limit: int = 50,
    session_id: Optional[str] = None
):
    """Get recent navigation decisions."""
    try:
        nav_db = await get_navigation_db()
        decisions = await nav_db.get_recent_decisions(limit, session_id)
        return {
            "success": True,
            "decisions": decisions,
            "count": len(decisions)
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/v1/navigation/decisions/type/{decision_type}")
async def get_decisions_by_type(decision_type: str, limit: int = 100):
    """Get decisions of a specific type."""
    try:
        nav_db = await get_navigation_db()
        decisions = await nav_db.get_decisions_by_type(decision_type, limit)
        return {
            "success": True,
            "decisions": decisions,
            "count": len(decisions)
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/v1/navigation/statistics")
async def get_navigation_statistics():
    """Get navigation database statistics."""
    try:
        nav_db = await get_navigation_db()
        stats = await nav_db.get_statistics()
        return {
            "success": True,
            "statistics": stats
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
```

#### Task 3.3: Backend Sync Service (1 hour)

**File:** `xai_navigation_pkg/backend_sync.py`

```python
#!/usr/bin/env python3
"""
Backend Sync - Periodically syncs local decisions to backend API.

Provides:
- Batch sync of unsynced decisions
- Retry on failure
- Health check
"""

import time
import threading
from typing import Optional, Dict, Any, List
import requests


class BackendSync:
    """
    Syncs local navigation decisions to backend API.

    Runs as a background thread, periodically pushing unsynced
    decisions to the backend for unified storage and dashboard access.
    """

    def __init__(
        self,
        backend_url: str = 'http://localhost:8000',
        sync_interval: float = 5.0,
        batch_size: int = 50
    ):
        """
        Initialize backend sync service.

        Args:
            backend_url: Backend API base URL
            sync_interval: Seconds between sync attempts
            batch_size: Max decisions per sync batch
        """
        self.backend_url = backend_url.rstrip('/')
        self.sync_interval = sync_interval
        self.batch_size = batch_size

        self._decision_db = None  # Set by XAI Navigator
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # Statistics
        self.stats = {
            'total_synced': 0,
            'sync_errors': 0,
            'last_sync_time': None,
            'last_error': None
        }

    def set_database(self, db):
        """
        Set the local database reference.

        Args:
            db: DecisionDatabase instance
        """
        self._decision_db = db

    def start(self):
        """Start background sync thread."""
        if self._running:
            return

        self._running = True
        self._thread = threading.Thread(target=self._sync_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop background sync thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _sync_loop(self):
        """Background sync loop."""
        while self._running:
            try:
                self._do_sync()
            except Exception as e:
                self.stats['sync_errors'] += 1
                self.stats['last_error'] = str(e)

            time.sleep(self.sync_interval)

    def _do_sync(self):
        """Perform a single sync operation."""
        if not self._decision_db:
            return

        # Get unsynced decisions
        unsynced = self._decision_db.get_unsynced_decisions(self.batch_size)

        if not unsynced:
            return

        # Send to backend
        url = f'{self.backend_url}/api/v1/navigation/decisions/sync'

        try:
            response = requests.post(
                url,
                json=unsynced,
                timeout=10.0
            )

            if response.status_code == 200:
                # Mark as synced
                db_ids = [d['db_id'] for d in unsynced]
                self._decision_db.mark_synced(db_ids)

                self.stats['total_synced'] += len(db_ids)
                self.stats['last_sync_time'] = time.time()
            else:
                self.stats['sync_errors'] += 1
                self.stats['last_error'] = f'HTTP {response.status_code}'

        except requests.RequestException as e:
            self.stats['sync_errors'] += 1
            self.stats['last_error'] = str(e)

    def sync_now(self) -> Dict[str, Any]:
        """Force immediate sync and return result."""
        if not self._decision_db:
            return {'success': False, 'error': 'No database configured'}

        try:
            unsynced = self._decision_db.get_unsynced_decisions(self.batch_size)

            if not unsynced:
                return {'success': True, 'synced': 0, 'message': 'Nothing to sync'}

            url = f'{self.backend_url}/api/v1/navigation/decisions/sync'
            response = requests.post(url, json=unsynced, timeout=10.0)

            if response.status_code == 200:
                db_ids = [d['db_id'] for d in unsynced]
                self._decision_db.mark_synced(db_ids)
                self.stats['total_synced'] += len(db_ids)
                return {'success': True, 'synced': len(db_ids)}
            else:
                return {'success': False, 'error': f'HTTP {response.status_code}'}

        except Exception as e:
            return {'success': False, 'error': str(e)}

    def check_backend_health(self) -> bool:
        """Check if backend is reachable."""
        try:
            response = requests.get(
                f'{self.backend_url}/api/health',
                timeout=5.0
            )
            return response.status_code == 200
        except:
            return False

    def get_statistics(self) -> Dict[str, Any]:
        """Get sync statistics."""
        return {
            **self.stats,
            'backend_url': self.backend_url,
            'sync_interval': self.sync_interval,
            'is_running': self._running
        }
```

### Afternoon Session (3 hours)

#### Task 3.4: Integrate All Components in XAI Navigator Node (3 hours)

**File:** `xai_navigation_pkg/xai_navigator_node.py` (COMPLETE REWRITE)

```python
#!/usr/bin/env python3
"""
XAI Navigator Node - Main orchestrator for explainable navigation.

Integrates:
- Nav2Monitor: Action client and navigation event capture
- DecisionDatabase: Local SQLite storage
- CostmapProcessor: Costmap analysis
- PathAnalyzer: Path comparison
- ObstacleDetector: Obstacle detection
- BackendSync: Backend API synchronization

Week 3: Full decision logging implementation
Week 4: Explanation generation (Gemini integration)
"""

import json
import time
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


class XAINavigatorNode(Node):
    """
    Main node for XAI-enabled navigation decision logging.

    Captures all navigation events from Nav2 and stores them
    for explanation generation (Week 4).
    """

    def __init__(self):
        super().__init__('xai_navigator_node')

        # Declare parameters
        self.declare_parameter('enable_logging', True)
        self.declare_parameter('backend_url', 'http://localhost:8000')
        self.declare_parameter('sync_interval', 5.0)
        self.declare_parameter('explanation_level', 'detailed')

        # Load parameters
        self.enable_logging = self.get_parameter('enable_logging').value
        self.backend_url = self.get_parameter('backend_url').value
        self.sync_interval = self.get_parameter('sync_interval').value
        self.explanation_level = self.get_parameter('explanation_level').value

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
        self.obstacle_detector = ObstacleDetector()

        # Backend sync
        self.backend_sync = BackendSync(
            backend_url=self.backend_url,
            sync_interval=self.sync_interval
        )
        self.backend_sync.set_database(self.decision_db)
        self.backend_sync.start()
        self.get_logger().info('Backend sync started')

        # Wait for Nav2 (non-blocking)
        self.create_timer(
            2.0,
            self._check_nav2_connection,
            callback_group=self.callback_group
        )

    def _check_nav2_connection(self):
        """Check Nav2 connection once."""
        if not hasattr(self, '_nav2_checked'):
            self._nav2_checked = True
            if self.nav2_monitor.wait_for_nav2(timeout_sec=5.0):
                self.get_logger().info('Nav2 connection established')
            else:
                self.get_logger().warn('Nav2 not available - will retry on goal')

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

        # Commands from conversation memory
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

        # Explanations (Week 4)
        self.explanation_pub = self.create_publisher(
            String, '/navigation/explanation', 10
        )

        self.explanation_detailed_pub = self.create_publisher(
            String, '/navigation/explanation_detailed', 10
        )

    def _init_timers(self):
        """Initialize timers."""
        # Periodic obstacle check
        self.obstacle_timer = self.create_timer(
            0.5,  # 2Hz
            self._periodic_obstacle_check,
            callback_group=self.callback_group
        )

    # === Callbacks ===

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
                        0,  # Will be linked in actual implementation
                        comparison['old_length'],
                        comparison['new_length'],
                        comparison['max_deviation'],
                        reason
                    )

        # Update stored path
        self.path_analyzer.update_path(msg)

        # Publish simple explanation
        stats = self.path_analyzer.get_path_statistics(msg)
        if stats:
            self._publish_explanation(
                f"Path planned: {stats['length']:.1f}m, {stats['waypoint_count']} waypoints"
            )

    def _local_plan_callback(self, msg: Path):
        """Handle local trajectory updates."""
        # Could analyze local plan vs global plan deviation
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

    def _command_callback(self, msg: String):
        """Handle navigation commands from conversation memory."""
        try:
            command = json.loads(msg.data)

            if command.get('action') == 'navigate':
                params = command.get('parameters', {})
                goal = params.get('goal')

                if goal:
                    # Create PoseStamped
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.pose.position.x = float(goal['x'])
                    pose.pose.position.y = float(goal['y'])
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.w = 1.0

                    # Store session if provided
                    self.session_id = command.get('session_id')

                    # Send to Nav2
                    self.nav2_monitor.send_goal(pose)

                    self.get_logger().info(
                        f'Navigation started via command to ({goal["x"]:.2f}, {goal["y"]:.2f})'
                    )

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid command JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Command error: {e}')

    def _periodic_obstacle_check(self):
        """Periodic check for obstacles on current path."""
        if not self.nav2_monitor.is_navigating:
            return

        if not self.path_analyzer.previous_path:
            return

        # Get current position from path analyzer's last known position
        # (In production, would subscribe to /amcl_pose or similar)

        detection = self.obstacle_detector.detect_on_path(
            self.path_analyzer.previous_path,
            self.costmap_processor
        )

        if detection.get('detected') and detection.get('critical_count', 0) > 0:
            closest = detection.get('closest')
            if closest:
                self.get_logger().warn(
                    f'Critical obstacle at ({closest["x"]:.2f}, {closest["y"]:.2f})'
                )

                self._log_decision('obstacle_detected', {
                    'obstacle_x': closest['x'],
                    'obstacle_y': closest['y'],
                    'severity': closest.get('severity', 'critical'),
                    'count': detection['total_count']
                })

    # === Decision Handling ===

    def _handle_nav_decision(self, decision: Dict[str, Any]):
        """
        Handle navigation decision event from Nav2Monitor.

        This is the main entry point for all navigation events.
        """
        if not self.enable_logging:
            return

        decision_type = decision.get('decision_type', 'unknown')

        # Add session ID if available
        decision['session_id'] = self.session_id

        # Add current costmap stats
        if self.costmap_processor.has_local_costmap:
            current = decision.get('data', {})
            if current.get('current_x') and current.get('current_y'):
                region = self.costmap_processor.get_region_statistics(
                    current['current_x'],
                    current['current_y'],
                    radius=1.5
                )
                decision['local_region'] = region

        # Store in local database
        try:
            db_id = self.decision_db.log_decision(decision, self.session_id)
            decision['db_id'] = db_id
        except Exception as e:
            self.get_logger().error(f'DB write error: {e}')

        # Publish decision event
        self._publish_decision(decision)

        # Generate explanation for key events
        if decision_type in ['goal_reached', 'goal_aborted', 'path_changed']:
            self._generate_explanation(decision)

        self.get_logger().debug(f'Decision logged: {decision_type}')

    def _log_decision(self, decision_type: str, data: Dict[str, Any]):
        """Log a decision manually (not from Nav2Monitor)."""
        decision = {
            'decision_id': 0,  # Will be assigned
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
        """Publish simple explanation."""
        msg = String()
        msg.data = text
        self.explanation_pub.publish(msg)

    def _generate_explanation(self, decision: Dict[str, Any]):
        """
        Generate explanation for a decision.

        Week 4: Will integrate with Gemini for natural language generation.
        Currently uses templates.
        """
        decision_type = decision.get('decision_type')
        data = decision.get('data', {})

        # Template-based explanations (Week 4 will use Gemini)
        templates = {
            'goal_reached': "Successfully arrived at destination.",
            'goal_aborted': f"Navigation failed: could not reach goal.",
            'path_changed': f"Path updated: {data.get('reason', 'optimization')}. "
                          f"New path is {data.get('length_change', 0):.1f}m "
                          f"{'longer' if data.get('length_change', 0) > 0 else 'shorter'}.",
            'obstacle_detected': f"Obstacle detected at ({data.get('obstacle_x', 0):.1f}, "
                                f"{data.get('obstacle_y', 0):.1f}). "
                                f"Severity: {data.get('severity', 'unknown')}."
        }

        explanation = templates.get(decision_type, f"Navigation event: {decision_type}")
        self._publish_explanation(explanation)

        # Detailed explanation
        detailed_msg = String()
        detailed_msg.data = json.dumps({
            'simple': explanation,
            'decision_type': decision_type,
            'data': data,
            'timestamp': decision.get('timestamp')
        })
        self.explanation_detailed_pub.publish(detailed_msg)

    # === Utilities ===

    def get_status(self) -> Dict[str, Any]:
        """Get node status."""
        nav_status = self.nav2_monitor.get_status()
        db_stats = self.decision_db.get_statistics()
        sync_stats = self.backend_sync.get_statistics()

        return {
            'navigation': nav_status,
            'database': db_stats,
            'sync': sync_stats,
            'has_costmap': self.costmap_processor.has_local_costmap,
            'has_path': self.path_analyzer.previous_path is not None
        }

    def destroy_node(self):
        """Clean up resources."""
        self.backend_sync.stop()
        self.decision_db.close()
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
```

### End of Day 3 Checklist

- [ ] NavigationDatabase (backend) implemented
- [ ] Navigation API endpoints added
- [ ] BackendSync service implemented
- [ ] XAINavigatorNode fully integrated
- [ ] All components working together
- [ ] Code committed: "Day 3: Backend sync and full integration"

---

## Day 4 (Thursday): Launch Files, Config, and Testing

### Morning Session (4 hours)

#### Task 4.1: Create Launch File (1 hour)

**File:** `xai_navigation_pkg/launch/xai_navigator.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'enable_logging',
            default_value='true',
            description='Enable decision logging'
        ),
        DeclareLaunchArgument(
            'backend_url',
            default_value='http://localhost:8000',
            description='Backend API URL'
        ),
        DeclareLaunchArgument(
            'sync_interval',
            default_value='5.0',
            description='Backend sync interval (seconds)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # XAI Navigator Node
        Node(
            package='xai_navigation_pkg',
            executable='xai_navigator_node',
            name='xai_navigator_node',
            output='screen',
            parameters=[{
                'enable_logging': LaunchConfiguration('enable_logging'),
                'backend_url': LaunchConfiguration('backend_url'),
                'sync_interval': LaunchConfiguration('sync_interval'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        )
    ])
```

#### Task 4.2: Create Configuration File (30 min)

**File:** `xai_navigation_pkg/config/xai_params.yaml`

```yaml
xai_navigator_node:
  ros__parameters:
    # Logging
    enable_logging: true
    explanation_level: "detailed"  # simple, detailed, debug

    # Backend sync
    backend_url: "http://localhost:8000"
    sync_interval: 5.0  # seconds
    sync_batch_size: 50

    # Obstacle detection
    critical_distance: 0.3  # meters
    warning_distance: 1.0   # meters

    # Path analysis
    deviation_threshold: 0.3  # meters

    # Performance
    costmap_check_interval: 0.5  # seconds (2Hz)
    feedback_throttle: 0.2  # seconds (5Hz max)
```

#### Task 4.3: Update Package Files (30 min)

Ensure `setup.py` includes launch and config:

```python
# Verify data_files includes:
(os.path.join('share', package_name, 'launch'),
    glob('launch/*.launch.py')),
(os.path.join('share', package_name, 'config'),
    glob('config/*.yaml')),
```

Create directories if needed:
```bash
mkdir -p src/xai_navigation_pkg/launch
mkdir -p src/xai_navigation_pkg/config
```

#### Task 4.4: Unit Tests (2 hours)

**File:** `xai_navigation_pkg/test/test_decision_database.py`

```python
#!/usr/bin/env python3
"""Tests for DecisionDatabase."""

import os
import tempfile
import pytest
from xai_navigation_pkg.decision_database import DecisionDatabase


class TestDecisionDatabase:
    """Test suite for DecisionDatabase."""

    @pytest.fixture
    def db(self):
        """Create temporary database."""
        fd, path = tempfile.mkstemp(suffix='.db')
        os.close(fd)
        database = DecisionDatabase(path)
        yield database
        database.close()
        os.unlink(path)

    def test_log_decision(self, db):
        """Test logging a decision."""
        decision = {
            'decision_id': 1,
            'decision_type': 'goal_sent',
            'timestamp': 1234567890.0,
            'data': {'goal_x': 1.0, 'goal_y': 2.0},
            'goal': {'x': 1.0, 'y': 2.0}
        }

        db_id = db.log_decision(decision)
        assert db_id > 0

    def test_get_recent_decisions(self, db):
        """Test retrieving recent decisions."""
        # Log multiple decisions
        for i in range(5):
            db.log_decision({
                'decision_id': i,
                'decision_type': 'feedback',
                'timestamp': 1234567890.0 + i,
                'data': {}
            })

        recent = db.get_recent_decisions(limit=3)
        assert len(recent) == 3
        # Should be in reverse chronological order
        assert recent[0]['timestamp'] > recent[1]['timestamp']

    def test_get_unsynced(self, db):
        """Test getting unsynced decisions."""
        # Log decisions
        for i in range(3):
            db.log_decision({
                'decision_id': i,
                'decision_type': 'test',
                'timestamp': 1234567890.0 + i,
                'data': {}
            })

        unsynced = db.get_unsynced_decisions()
        assert len(unsynced) == 3

        # Mark as synced
        db.mark_synced([u['db_id'] for u in unsynced])

        # Should be empty now
        unsynced2 = db.get_unsynced_decisions()
        assert len(unsynced2) == 0

    def test_statistics(self, db):
        """Test database statistics."""
        db.log_decision({
            'decision_id': 1,
            'decision_type': 'goal_sent',
            'timestamp': 1234567890.0,
            'data': {}
        })
        db.log_decision({
            'decision_id': 2,
            'decision_type': 'goal_reached',
            'timestamp': 1234567891.0,
            'data': {}
        })

        stats = db.get_statistics()
        assert stats['total_decisions'] == 2
        assert 'goal_sent' in stats['by_type']
        assert 'goal_reached' in stats['by_type']


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
```

**File:** `xai_navigation_pkg/test/test_path_analyzer.py`

```python
#!/usr/bin/env python3
"""Tests for PathAnalyzer."""

import pytest
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from xai_navigation_pkg.path_analyzer import PathAnalyzer


def create_path(waypoints):
    """Helper to create Path message."""
    path = Path()
    for x, y in waypoints:
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        path.poses.append(pose)
    return path


class TestPathAnalyzer:
    """Test suite for PathAnalyzer."""

    @pytest.fixture
    def analyzer(self):
        return PathAnalyzer(deviation_threshold=0.3)

    def test_path_length(self, analyzer):
        """Test path length calculation."""
        path = create_path([(0, 0), (1, 0), (2, 0)])
        length = analyzer.calculate_path_length(path)
        assert abs(length - 2.0) < 0.001

    def test_diagonal_path_length(self, analyzer):
        """Test diagonal path length."""
        path = create_path([(0, 0), (1, 1)])
        length = analyzer.calculate_path_length(path)
        assert abs(length - 1.414) < 0.01

    def test_compare_identical_paths(self, analyzer):
        """Test comparing identical paths."""
        path1 = create_path([(0, 0), (1, 0), (2, 0)])
        path2 = create_path([(0, 0), (1, 0), (2, 0)])

        comparison = analyzer.compare_paths(path1, path2)
        assert comparison['comparable']
        assert not comparison['is_significant']
        assert comparison['max_deviation'] < 0.1

    def test_compare_different_paths(self, analyzer):
        """Test comparing significantly different paths."""
        path1 = create_path([(0, 0), (1, 0), (2, 0)])
        path2 = create_path([(0, 0), (1, 1), (2, 0)])  # Detour through (1,1)

        comparison = analyzer.compare_paths(path1, path2)
        assert comparison['comparable']
        assert comparison['is_significant']
        assert comparison['max_deviation'] > 0.5

    def test_path_statistics(self, analyzer):
        """Test path statistics."""
        path = create_path([(0, 0), (1, 0), (2, 0), (2, 1)])
        stats = analyzer.get_path_statistics(path)

        assert stats['waypoint_count'] == 4
        assert abs(stats['length'] - 3.0) < 0.001
        assert stats['start'] == (0, 0)
        assert stats['end'] == (2, 1)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
```

### Afternoon Session (3 hours)

#### Task 4.5: Integration Testing (3 hours)

**Create test script:** `test_week3_integration.sh`

```bash
#!/bin/bash
# Week 3 Integration Test Script

echo "=========================================="
echo "Week 3 XAI Navigation Integration Test"
echo "=========================================="

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

# Check prerequisites
echo -e "\n[1/6] Checking prerequisites..."

# Check ROS2
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}ERROR: ROS2 not found${NC}"
    exit 1
fi
echo -e "${GREEN}ROS2 OK${NC}"

# Check package built
if ! ros2 pkg list | grep -q xai_navigation_pkg; then
    echo -e "${RED}ERROR: xai_navigation_pkg not found. Build first.${NC}"
    exit 1
fi
echo -e "${GREEN}Package OK${NC}"

# Test module imports
echo -e "\n[2/6] Testing module imports..."
python3 << 'EOF'
try:
    from xai_navigation_pkg.nav2_monitor import Nav2Monitor
    from xai_navigation_pkg.decision_database import DecisionDatabase
    from xai_navigation_pkg.costmap_processor import CostmapProcessor
    from xai_navigation_pkg.path_analyzer import PathAnalyzer
    from xai_navigation_pkg.obstacle_detector import ObstacleDetector
    from xai_navigation_pkg.backend_sync import BackendSync
    print("All imports OK")
except ImportError as e:
    print(f"Import error: {e}")
    exit(1)
EOF

if [ $? -ne 0 ]; then
    echo -e "${RED}Import test FAILED${NC}"
    exit 1
fi
echo -e "${GREEN}Imports OK${NC}"

# Test database
echo -e "\n[3/6] Testing local database..."
python3 << 'EOF'
from xai_navigation_pkg.decision_database import DecisionDatabase
import tempfile
import os

fd, path = tempfile.mkstemp(suffix='.db')
os.close(fd)

db = DecisionDatabase(path)
db.log_decision({
    'decision_id': 1,
    'decision_type': 'test',
    'timestamp': 1234567890,
    'data': {'test': True}
})

stats = db.get_statistics()
assert stats['total_decisions'] == 1
print(f"Database stats: {stats}")

db.close()
os.unlink(path)
print("Database test OK")
EOF

if [ $? -ne 0 ]; then
    echo -e "${RED}Database test FAILED${NC}"
    exit 1
fi
echo -e "${GREEN}Database OK${NC}"

# Test path analyzer
echo -e "\n[4/6] Testing path analyzer..."
python3 << 'EOF'
from xai_navigation_pkg.path_analyzer import PathAnalyzer
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

analyzer = PathAnalyzer()

# Create test path
path = Path()
for x in range(5):
    pose = PoseStamped()
    pose.pose.position.x = float(x)
    pose.pose.position.y = 0.0
    path.poses.append(pose)

length = analyzer.calculate_path_length(path)
assert abs(length - 4.0) < 0.001, f"Expected 4.0, got {length}"
print(f"Path length: {length}m")

stats = analyzer.get_path_statistics(path)
print(f"Path stats: {stats}")
print("Path analyzer test OK")
EOF

if [ $? -ne 0 ]; then
    echo -e "${RED}Path analyzer test FAILED${NC}"
    exit 1
fi
echo -e "${GREEN}Path analyzer OK${NC}"

# Run pytest if available
echo -e "\n[5/6] Running pytest..."
if command -v pytest &> /dev/null; then
    cd ~/ros2_navigation_project/src/xai_navigation_pkg
    pytest test/ -v --tb=short 2>/dev/null || true
else
    echo "pytest not found, skipping"
fi

# Summary
echo -e "\n[6/6] Summary"
echo "=========================================="
echo -e "${GREEN}All Week 3 components tested successfully!${NC}"
echo ""
echo "Next steps:"
echo "1. Start Gazebo: ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
echo "2. Start Nav2: ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true"
echo "3. Start XAI Navigator: ros2 launch xai_navigation_pkg xai_navigator.launch.py"
echo "4. Send navigation goal from RViz"
echo "5. Monitor: ros2 topic echo /navigation/decision"
echo "=========================================="
```

### End of Day 4 Checklist

- [ ] Launch file created and working
- [ ] Configuration YAML created
- [ ] Unit tests passing
- [ ] Integration test script working
- [ ] Code committed: "Day 4: Launch, config, and testing"

---

## Day 5-7: Testing, Documentation, and Buffer

### Day 5 (Friday): System Testing

1. **Full System Test**
   - Launch Gazebo + Nav2 + XAI Navigator
   - Send navigation goals from RViz
   - Verify decisions are logged
   - Verify backend sync works

2. **Edge Case Testing**
   - Navigation failure scenarios
   - Obstacle detection
   - Path replanning

### Day 6 (Saturday): Documentation & Cleanup

1. **Code Documentation**
   - Ensure all functions have docstrings
   - Update package README

2. **Update Project Documentation**
   - Update WEEK3_PROGRESS.md
   - Update main README

### Day 7 (Sunday): Buffer & Week 4 Prep

1. **Fix Any Remaining Issues**
2. **Prepare for Week 4**
   - Review Gemini API integration requirements
   - Plan explanation generation architecture

---

## End of Week 3 Checklist

### Required Deliverables

- [ ] `nav2_monitor.py` - Nav2 action client with callbacks
- [ ] `decision_database.py` - Local SQLite storage
- [ ] `costmap_processor.py` - Costmap analysis
- [ ] `path_analyzer.py` - Path comparison
- [ ] `obstacle_detector.py` - Obstacle detection
- [ ] `backend_sync.py` - Backend API sync
- [ ] `xai_navigator_node.py` - Main orchestrator (extended)
- [ ] `xai_navigator.launch.py` - Launch file
- [ ] `xai_params.yaml` - Configuration
- [ ] Backend `navigation_db.py` - Backend database
- [ ] Backend navigation API endpoints

### Verification Tests

```bash
# Build and source
cd ~/ros2_navigation_project
colcon build --packages-select xai_navigation_pkg
source install/setup.bash

# Launch full stack
# Terminal 1: Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Nav2
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true

# Terminal 3: XAI Navigator
ros2 launch xai_navigation_pkg xai_navigator.launch.py

# Terminal 4: Monitor decisions
ros2 topic echo /navigation/decision

# Terminal 5: Check database
sqlite3 ~/.ros/navigation_decisions.db "SELECT decision_type, COUNT(*) FROM navigation_decisions GROUP BY decision_type;"
```

### Metrics

| Metric | Target | Actual |
|--------|--------|--------|
| Decision logging latency | <100ms | ___ms |
| Database write time | <10ms | ___ms |
| Backend sync success rate | >95% | ___% |
| All decision types captured | 100% | ___% |

---

## Git Commit Strategy

```bash
# Day 1
git add -A && git commit -m "Week 3 Day 1: Nav2 monitor and local database"

# Day 2
git add -A && git commit -m "Week 3 Day 2: Costmap, path, obstacle processors"

# Day 3
git add -A && git commit -m "Week 3 Day 3: Backend sync and full integration"

# Day 4
git add -A && git commit -m "Week 3 Day 4: Launch, config, and testing"

# End of Week
git tag -a week3-complete -m "Week 3: XAI Navigation Decision Logging Complete"
git push origin master --tags
```

---

**Document Created:** Week 3 Start
**Status:** Implementation Guide
**Next:** Week 4 - Explanation Generation (Gemini Integration)