#!/usr/bin/env python3
"""
Navigation Database - Backend storage for navigation decisions.

Week 3: Stores navigation decisions synced from ROS2 XAI Navigator node.
Provides unified data layer for dashboard visualization.

Tables:
- navigation_decisions: Main decision events (goal_sent, feedback, goal_reached, etc.)
- path_changes: Path modification events
- obstacle_events: Obstacle detection events
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
    Receives synced data from ROS2 XAI Navigator node.
    """

    def __init__(self, db_path: str = 'navigation_decisions.db'):
        """
        Initialize navigation database.

        Args:
            db_path: Database file name (stored in ~/.ros/backend_data/)
        """
        data_dir = os.path.expanduser('~/.ros/backend_data')
        Path(data_dir).mkdir(parents=True, exist_ok=True)
        self.db_path = os.path.join(data_dir, db_path)
        self._connection: Optional[aiosqlite.Connection] = None

    async def initialize(self):
        """Initialize database connection and schema."""
        self._connection = await aiosqlite.connect(self.db_path)
        await self._connection.execute('PRAGMA journal_mode=WAL')
        await self._connection.execute('PRAGMA synchronous=NORMAL')
        await self._init_schema()

    async def _init_schema(self):
        """Create database tables."""
        # Main decisions table
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

        # Path changes table
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

        # Obstacle events table
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

        # Indexes for common queries
        await self._connection.execute('''
            CREATE INDEX IF NOT EXISTS idx_nav_timestamp
            ON navigation_decisions(timestamp DESC)
        ''')
        await self._connection.execute('''
            CREATE INDEX IF NOT EXISTS idx_nav_session
            ON navigation_decisions(session_id, timestamp DESC)
        ''')
        await self._connection.execute('''
            CREATE INDEX IF NOT EXISTS idx_nav_type
            ON navigation_decisions(decision_type)
        ''')

        await self._connection.commit()

    async def store_decision(self, decision: Dict[str, Any]) -> int:
        """
        Store a navigation decision.

        Args:
            decision: Decision dictionary from ROS2 sync

        Returns:
            Database ID of stored decision
        """
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
        """
        Store multiple decisions in a batch.

        Args:
            decisions: List of decision dictionaries

        Returns:
            Number of decisions stored
        """
        count = 0
        for decision in decisions:
            await self.store_decision(decision)
            count += 1
        return count

    async def store_path_change(
        self,
        decision_id: int,
        old_length: float,
        new_length: float,
        max_deviation: float,
        reason: str
    ) -> int:
        """Store a path change event."""
        cursor = await self._connection.execute('''
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

        await self._connection.commit()
        return cursor.lastrowid

    async def store_obstacle_event(
        self,
        decision_id: int,
        obstacle_x: float,
        obstacle_y: float,
        distance: float,
        severity: str,
        action: str
    ) -> int:
        """Store an obstacle detection event."""
        cursor = await self._connection.execute('''
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

        await self._connection.commit()
        return cursor.lastrowid

    async def get_recent_decisions(
        self,
        limit: int = 50,
        session_id: Optional[str] = None
    ) -> List[Dict[str, Any]]:
        """
        Get recent navigation decisions.

        Args:
            limit: Maximum number of decisions
            session_id: Optional filter by session

        Returns:
            List of decision dictionaries
        """
        if session_id:
            cursor = await self._connection.execute('''
                SELECT id, ros_decision_id, timestamp, decision_type,
                       goal_x, goal_y, current_x, current_y,
                       distance_remaining, data_json, session_id
                FROM navigation_decisions
                WHERE session_id = ?
                ORDER BY timestamp DESC
                LIMIT ?
            ''', (session_id, limit))
        else:
            cursor = await self._connection.execute('''
                SELECT id, ros_decision_id, timestamp, decision_type,
                       goal_x, goal_y, current_x, current_y,
                       distance_remaining, data_json, session_id
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
            'goal': {'x': r[4], 'y': r[5]} if r[4] is not None else None,
            'current': {'x': r[6], 'y': r[7]} if r[6] is not None else None,
            'distance_remaining': r[8],
            'data': json.loads(r[9]) if r[9] else {},
            'session_id': r[10]
        } for r in rows]

    async def get_decisions_by_type(
        self,
        decision_type: str,
        limit: int = 100
    ) -> List[Dict[str, Any]]:
        """Get decisions of a specific type."""
        cursor = await self._connection.execute('''
            SELECT id, ros_decision_id, timestamp, goal_x, goal_y, data_json
            FROM navigation_decisions
            WHERE decision_type = ?
            ORDER BY timestamp DESC
            LIMIT ?
        ''', (decision_type, limit))

        rows = await cursor.fetchall()
        return [{
            'id': r[0],
            'ros_decision_id': r[1],
            'timestamp': r[2],
            'goal': {'x': r[3], 'y': r[4]} if r[3] is not None else None,
            'data': json.loads(r[5]) if r[5] else {}
        } for r in rows]

    async def get_path_changes(self, limit: int = 50) -> List[Dict[str, Any]]:
        """Get recent path change events."""
        cursor = await self._connection.execute('''
            SELECT id, decision_id, timestamp, old_length, new_length,
                   max_deviation, reason
            FROM path_changes
            ORDER BY timestamp DESC
            LIMIT ?
        ''', (limit,))

        rows = await cursor.fetchall()
        return [{
            'id': r[0],
            'decision_id': r[1],
            'timestamp': r[2],
            'old_length': r[3],
            'new_length': r[4],
            'max_deviation': r[5],
            'reason': r[6]
        } for r in rows]

    async def get_obstacle_events(self, limit: int = 50) -> List[Dict[str, Any]]:
        """Get recent obstacle events."""
        cursor = await self._connection.execute('''
            SELECT id, decision_id, timestamp, obstacle_x, obstacle_y,
                   distance_to_robot, severity, action_taken
            FROM obstacle_events
            ORDER BY timestamp DESC
            LIMIT ?
        ''', (limit,))

        rows = await cursor.fetchall()
        return [{
            'id': r[0],
            'decision_id': r[1],
            'timestamp': r[2],
            'position': {'x': r[3], 'y': r[4]},
            'distance': r[5],
            'severity': r[6],
            'action': r[7]
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

        cursor = await self._connection.execute(
            'SELECT COUNT(*) FROM path_changes'
        )
        path_changes = (await cursor.fetchone())[0]

        cursor = await self._connection.execute(
            'SELECT COUNT(*) FROM obstacle_events'
        )
        obstacle_events = (await cursor.fetchone())[0]

        # Get latest decision timestamp
        cursor = await self._connection.execute('''
            SELECT MAX(timestamp) FROM navigation_decisions
        ''')
        latest = (await cursor.fetchone())[0]

        return {
            'total_decisions': total,
            'by_type': by_type,
            'path_changes': path_changes,
            'obstacle_events': obstacle_events,
            'latest_timestamp': latest
        }

    async def cleanup_old(self, days: int = 7) -> int:
        """
        Delete decisions older than specified days.

        Args:
            days: Number of days to keep

        Returns:
            Number of deleted records
        """
        import time
        cutoff = time.time() - (days * 24 * 60 * 60)

        cursor = await self._connection.execute('''
            DELETE FROM navigation_decisions
            WHERE timestamp < ?
        ''', (cutoff,))

        deleted = cursor.rowcount
        await self._connection.commit()
        return deleted

    async def close(self):
        """Close database connection."""
        if self._connection:
            await self._connection.close()
            self._connection = None


# Singleton instance
_navigation_db: Optional[NavigationDatabase] = None


async def get_navigation_db() -> NavigationDatabase:
    """
    Get or create navigation database singleton.

    Returns:
        Initialized NavigationDatabase instance
    """
    global _navigation_db
    if _navigation_db is None:
        _navigation_db = NavigationDatabase()
        await _navigation_db.initialize()
    return _navigation_db
