#!/usr/bin/env python3
"""
Decision Database - Local SQLite storage for fast navigation decision logging.

Week 3: Fast local storage before sync to backend.

Features:
- SQLite with WAL mode for concurrent access
- Single optimized table design
- Automatic tracking of synced vs unsynced decisions
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
        self.conn.execute('PRAGMA foreign_keys=ON')

        self._init_schema()

    def _init_schema(self):
        """Create database tables."""
        cursor = self.conn.cursor()

        # Main decisions table - Enhanced for Week 3 Apple Standard
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS navigation_decisions (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                decision_id INTEGER NOT NULL,
                session_id TEXT,
                timestamp REAL NOT NULL,
                decision_type TEXT NOT NULL,
                
                -- Spatial Context
                pose_x REAL,
                pose_y REAL,
                pose_theta REAL, -- NEW
                
                -- Navigation Context
                goal_x REAL,
                goal_y REAL,
                
                -- XAI Data
                nearest_obstacle_dist REAL, -- NEW
                active_behavior TEXT, -- NEW: 'follow_path', 'recovery', 'spin'
                
                -- Legacy/Detailed Data
                current_x REAL, -- Kept for backward compatibility (maps to pose_x)
                current_y REAL, -- Kept for backward compatibility (maps to pose_y)
                distance_remaining REAL,
                path_length REAL,
                obstacle_count INTEGER DEFAULT 0,
                data_json TEXT,
                
                -- Sync Status
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

        # Obstacle events table - Extended for weighted classification
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
                -- NEW: Weighted Classification Fields (Week 4+)
                obstacle_type TEXT,           -- human, vehicle, furniture, wall, unknown
                priority_weight REAL,         -- weighted priority score
                classification_confidence REAL, -- 0.0 to 1.0
                classification_reasoning TEXT, -- human-readable explanation
                zone_name TEXT,               -- semantic zone if applicable
                estimated_velocity REAL,      -- m/s if moving
                estimated_size REAL,          -- estimated width in meters
                synced_to_backend INTEGER DEFAULT 0,
                created_at TEXT DEFAULT CURRENT_TIMESTAMP,
                FOREIGN KEY (decision_id) REFERENCES navigation_decisions(id)
            )
        ''')

        # NEW: Telemetry Snapshots for Anomaly Detection
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS telemetry_snapshots (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp REAL NOT NULL,
                session_id TEXT,
                battery_voltage REAL,
                cpu_usage REAL,
                wifi_signal REAL,
                motor_currents_json TEXT,
                synced_to_backend INTEGER DEFAULT 0,
                created_at TEXT DEFAULT CURRENT_TIMESTAMP
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
            CREATE INDEX IF NOT EXISTS idx_telemetry_synced
            ON telemetry_snapshots(synced_to_backend)
        ''')

        self.conn.commit()
        
        # Run schema migrations in case db predates new columns
        self._migrate_schema()

    def _migrate_schema(self):
        """Migrate existing tables to include new columns."""
        cursor = self.conn.cursor()
        
        # Determine existing columns in obstacle_events
        cursor.execute('PRAGMA table_info(obstacle_events)')
        columns = [row[1] for row in cursor.fetchall()]
        
        # Define expected new columns and their types
        new_columns = {
            'obstacle_type': 'TEXT',
            'priority_weight': 'REAL',
            'classification_confidence': 'REAL',
            'classification_reasoning': 'TEXT',
            'zone_name': 'TEXT',
            'estimated_velocity': 'REAL',
            'estimated_size': 'REAL'
        }
        
        # Add any missing columns
        for col, col_type in new_columns.items():
            if col not in columns:
                try:
                    cursor.execute(f'ALTER TABLE obstacle_events ADD COLUMN {col} {col_type}')
                except sqlite3.OperationalError as e:
                    # Ignore duplicate column errors if they randomly occur
                    pass
                    
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
        
        # Map legacy fields to new schema
        decision_type = decision.get('decision_type', 'unknown')
        pose_x = data.get('current_x')
        pose_y = data.get('current_y')
        
        # New fields (with defaults if missing)
        pose_theta = data.get('current_theta', 0.0)
        active_behavior = data.get('active_behavior', 'unknown')
        nearest_obstacle = data.get('nearest_obstacle_dist', -1.0)

        cursor = self.conn.cursor()
        cursor.execute('''
            INSERT INTO navigation_decisions (
                decision_id, session_id, timestamp, decision_type,
                pose_x, pose_y, pose_theta,
                goal_x, goal_y,
                nearest_obstacle_dist, active_behavior,
                current_x, current_y,
                distance_remaining, data_json
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        ''', (
            decision.get('decision_id', 0),
            session_id,
            decision.get('timestamp', datetime.now().timestamp()),
            decision_type,
            pose_x, pose_y, pose_theta,
            goal.get('x') if goal else None,
            goal.get('y') if goal else None,
            nearest_obstacle, active_behavior,
            pose_x, pose_y, # Legacy columns
            data.get('distance_remaining'),
            json.dumps(data)
        ))

        self.conn.commit()
        return cursor.lastrowid

    def log_telemetry(
        self,
        session_id: str,
        battery: float,
        cpu: float,
        wifi: float,
        currents: List[float]
    ) -> int:
        """Log a telemetry snapshot for anomaly detection."""
        cursor = self.conn.cursor()
        cursor.execute('''
            INSERT INTO telemetry_snapshots (
                timestamp, session_id, battery_voltage,
                cpu_usage, wifi_signal, motor_currents_json
            ) VALUES (?, ?, ?, ?, ?, ?)
        ''', (
            datetime.now().timestamp(),
            session_id,
            battery,
            cpu,
            wifi,
            json.dumps(currents)
        ))
        self.conn.commit()
        return cursor.lastrowid

    def log_path_change(
        self,
        decision_id: Optional[int],
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
        decision_id: Optional[int],
        obstacle_x: float,
        obstacle_y: float,
        distance: float,
        severity: str,
        action: str,
        # NEW: Classification fields (Week 4+)
        obstacle_type: Optional[str] = None,
        priority_weight: Optional[float] = None,
        classification_confidence: Optional[float] = None,
        classification_reasoning: Optional[str] = None,
        zone_name: Optional[str] = None,
        estimated_velocity: Optional[float] = None,
        estimated_size: Optional[float] = None
    ) -> int:
        """
        Log an obstacle detection event with optional classification.

        Args:
            decision_id: Related navigation decision ID
            obstacle_x, obstacle_y: Obstacle position
            distance: Distance from robot
            severity: Severity level (critical, warning, info)
            action: Action taken (detected, stop, replan)
            obstacle_type: Classification type (human, vehicle, furniture, wall, unknown)
            priority_weight: Weighted priority score
            classification_confidence: Confidence in classification (0.0-1.0)
            classification_reasoning: Human-readable explanation
            zone_name: Semantic zone name if applicable
            estimated_velocity: Estimated velocity in m/s
            estimated_size: Estimated size in meters

        Returns:
            Database ID of logged event
        """
        cursor = self.conn.cursor()
        cursor.execute('''
            INSERT INTO obstacle_events (
                decision_id, timestamp, obstacle_x, obstacle_y,
                distance_to_robot, severity, action_taken,
                obstacle_type, priority_weight, classification_confidence,
                classification_reasoning, zone_name,
                estimated_velocity, estimated_size
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        ''', (
            decision_id,
            datetime.now().timestamp(),
            obstacle_x,
            obstacle_y,
            distance,
            severity,
            action,
            obstacle_type,
            priority_weight,
            classification_confidence,
            classification_reasoning,
            zone_name,
            estimated_velocity,
            estimated_size
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

    def close(self):
        """Close database connection."""
        self.conn.close()
