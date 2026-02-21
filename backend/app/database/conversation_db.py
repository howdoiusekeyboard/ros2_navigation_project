"""
Conversation database for storing multi-turn dialogue history with spatial context.

Design Philosophy:
- Single optimized table (not 3) - simpler is better
- Async SQLite for non-blocking I/O
- Indexed for fast retrieval (<1ms for 100 turns)
- Session-based organization
- In-memory caching for spatial references
"""

import asyncio
import json
import logging
import os
import sqlite3
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from uuid import uuid4

import aiosqlite

logger = logging.getLogger(__name__)


class ConversationDatabase:
    """
    Async SQLite database for conversation memory with spatial awareness.

    Performance targets:
    - Single turn insertion: < 1ms
    - History retrieval (10 turns): < 1ms
    - Database size: ~200 bytes per turn
    """

    def __init__(self, db_path: str = "data/conversations.db"):
        """
        Initialize conversation database.

        Args:
            db_path: Path to SQLite database file (relative to backend root)
        """
        # Resolve path relative to backend directory
        base_path = Path(__file__).parent.parent.parent.resolve()
        data_dir_str = os.path.abspath(str(base_path / 'data'))
        
        # Defuse path injection by aggressively sanitizing user input
        sanitized_filename = os.path.basename(str(db_path))
        if not sanitized_filename or sanitized_filename in ('.', '..'):
            raise ValueError("Database path is missing or resolves to current/parent directory")
            
        resolved_path_str = os.path.abspath(os.path.join(data_dir_str, sanitized_filename))
        
        # Robust path traversal immunity using backward-compatible checks
        if os.path.commonpath([data_dir_str, resolved_path_str]) != data_dir_str:
            raise ValueError("Invalid database path for conversation history: Path traversal detected")
             
        self.db_path = Path(resolved_path_str)
        self.db_path.parent.mkdir(parents=True, exist_ok=True)

        self._connection: Optional[aiosqlite.Connection] = None
        self._lock = asyncio.Lock()

        # In-memory cache for spatial references (per session)
        self._spatial_cache: Dict[str, Dict[str, Tuple[float, float, str]]] = {}

        safe_path = repr(str(self.db_path))
        logger.info("ConversationDatabase initialized at: %s", safe_path)

    async def connect(self):
        """Establish database connection and initialize schema."""
        if self._connection is not None:
            return

        async with self._lock:
            if self._connection is not None:
                return

            self._connection = await aiosqlite.connect(
                str(self.db_path),
                timeout=10.0
            )

            # Enable Write-Ahead Logging for better concurrency
            await self._connection.execute("PRAGMA journal_mode=WAL")
            await self._connection.execute("PRAGMA synchronous=NORMAL")

            await self._init_schema()

            logger.info("Database connection established")

    async def close(self):
        """Close database connection."""
        if self._connection:
            await self._connection.close()
            self._connection = None
            logger.info("Database connection closed")

    async def _init_schema(self):
        """
        Initialize database schema with single optimized table.

        Design: One table with all necessary fields, indexed for performance.
        """
        await self._connection.execute("""
            CREATE TABLE IF NOT EXISTS conversations (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                session_id TEXT NOT NULL,
                timestamp TEXT NOT NULL,
                turn_number INTEGER NOT NULL,

                -- Conversation content
                user_input TEXT NOT NULL,
                robot_response TEXT,
                action_type TEXT,

                -- Spatial context
                location_x REAL,
                location_y REAL,
                location_label TEXT,

                -- Metadata
                confidence REAL,
                latency_ms INTEGER,
                metadata_json TEXT,

                -- Soft delete support
                is_deleted BOOLEAN DEFAULT 0
            )
        """)

        # Create indexes for fast queries
        await self._connection.execute("""
            CREATE INDEX IF NOT EXISTS idx_session_timestamp
            ON conversations(session_id, timestamp DESC)
            WHERE is_deleted = 0
        """)

        await self._connection.execute("""
            CREATE INDEX IF NOT EXISTS idx_session_location
            ON conversations(session_id, location_x, location_y)
            WHERE is_deleted = 0 AND location_x IS NOT NULL
        """)

        await self._connection.commit()
        logger.info("Database schema initialized with optimized indexes")

    def generate_session_id(self) -> str:
        """Generate a new session ID."""
        return str(uuid4())[:8]  # Short UUID for readability

    async def add_turn(
        self,
        session_id: str,
        user_input: str,
        robot_response: str = None,
        action_type: str = "unknown",
        location_x: float = None,
        location_y: float = None,
        location_label: str = None,
        confidence: float = None,
        latency_ms: int = None,
        metadata: Dict = None
    ) -> int:
        """
        Add a conversation turn to the database.

        Args:
            session_id: Session identifier
            user_input: User's command/query
            robot_response: Robot's response
            action_type: Command type (navigate, query, stop, etc.)
            location_x, location_y: Spatial coordinates (if applicable)
            location_label: Semantic location label (e.g., "kitchen")
            confidence: LLM confidence score (0-1)
            latency_ms: Processing latency in milliseconds
            metadata: Additional metadata as dict

        Returns:
            Turn ID (database row ID)
        """
        await self.connect()

        # Get turn number for this session
        turn_number = await self._get_next_turn_number(session_id)

        # Serialize metadata
        metadata_json = json.dumps(metadata) if metadata else None

        cursor = await self._connection.execute("""
            INSERT INTO conversations (
                session_id, timestamp, turn_number,
                user_input, robot_response, action_type,
                location_x, location_y, location_label,
                confidence, latency_ms, metadata_json
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (
            session_id,
            datetime.now().isoformat(),
            turn_number,
            user_input,
            robot_response,
            action_type,
            location_x,
            location_y,
            location_label,
            confidence,
            latency_ms,
            metadata_json
        ))

        await self._connection.commit()
        turn_id = cursor.lastrowid

        # Update spatial cache if location provided
        if location_x is not None and location_label:
            self._update_spatial_cache(
                session_id, location_label, location_x, location_y, location_label
            )

        safe_session = repr(session_id)
        safe_input = repr(user_input[:50])
        logger.debug(
            "Stored turn %s for session %s: %s... (ID: %s)",
            turn_number, safe_session, safe_input, turn_id
        )

        return turn_id

    async def _get_next_turn_number(self, session_id: str) -> int:
        """Get the next turn number for a session."""
        cursor = await self._connection.execute("""
            SELECT MAX(turn_number) FROM conversations
            WHERE session_id = ? AND is_deleted = 0
        """, (session_id,))

        result = await cursor.fetchone()
        max_turn = result[0] if result[0] is not None else 0
        return max_turn + 1

    async def get_history(
        self,
        session_id: str,
        limit: int = 10,
        include_metadata: bool = False
    ) -> List[Dict]:
        """
        Retrieve conversation history for a session.

        Args:
            session_id: Session identifier
            limit: Maximum number of turns to retrieve
            include_metadata: Whether to include metadata_json field

        Returns:
            List of conversation turns (newest first)
        """
        await self.connect()

        fields = """
            id, timestamp, turn_number, user_input, robot_response,
            action_type, location_x, location_y, location_label,
            confidence, latency_ms
        """

        if include_metadata:
            fields += ", metadata_json"

        cursor = await self._connection.execute(f"""
            SELECT {fields}
            FROM conversations
            WHERE session_id = ? AND is_deleted = 0
            ORDER BY timestamp DESC
            LIMIT ?
        """, (session_id, limit))

        rows = await cursor.fetchall()

        # Build result dictionaries
        history = []
        for row in rows:
            turn = {
                "id": row[0],
                "timestamp": row[1],
                "turn_number": row[2],
                "user_input": row[3],
                "robot_response": row[4],
                "action_type": row[5],
                "location_x": row[6],
                "location_y": row[7],
                "location_label": row[8],
                "confidence": row[9],
                "latency_ms": row[10]
            }

            if include_metadata and len(row) > 11:
                turn["metadata"] = json.loads(row[11]) if row[11] else None

            history.append(turn)

        return history

    async def get_session_summary(self, session_id: str) -> Optional[Dict]:
        """
        Get summary statistics for a session.

        Returns:
            Dict with turn count, duration, avg confidence, etc.
        """
        await self.connect()

        cursor = await self._connection.execute("""
            SELECT
                COUNT(*) as turn_count,
                MIN(timestamp) as start_time,
                MAX(timestamp) as end_time,
                AVG(confidence) as avg_confidence,
                AVG(latency_ms) as avg_latency,
                SUM(CASE WHEN action_type = 'navigate' THEN 1 ELSE 0 END) as nav_count
            FROM conversations
            WHERE session_id = ? AND is_deleted = 0
        """, (session_id,))

        row = await cursor.fetchone()

        if row[0] == 0:  # No turns found
            return None

        return {
            "session_id": session_id,
            "turn_count": row[0],
            "start_time": row[1],
            "end_time": row[2],
            "avg_confidence": round(row[3], 3) if row[3] else None,
            "avg_latency_ms": round(row[4], 1) if row[4] else None,
            "navigation_count": row[5]
        }

    async def get_recent_sessions(self, limit: int = 10) -> List[Dict]:
        """
        Get list of recent sessions.

        Returns:
            List of session summaries
        """
        await self.connect()

        cursor = await self._connection.execute("""
            SELECT DISTINCT session_id, MAX(timestamp) as last_activity
            FROM conversations
            WHERE is_deleted = 0
            GROUP BY session_id
            ORDER BY last_activity DESC
            LIMIT ?
        """, (limit,))

        rows = await cursor.fetchall()

        sessions = []
        for row in rows:
            summary = await self.get_session_summary(row[0])
            if summary:
                sessions.append(summary)

        return sessions

    async def get_spatial_references(
        self,
        session_id: str
    ) -> Dict[str, Tuple[float, float, str]]:
        """
        Get all spatial references (labeled locations) for a session.

        Returns:
            Dict mapping location_label -> (x, y, label)
        """
        # Check cache first
        if session_id in self._spatial_cache:
            return self._spatial_cache[session_id].copy()

        await self.connect()

        cursor = await self._connection.execute("""
            SELECT location_label, location_x, location_y, MAX(timestamp)
            FROM conversations
            WHERE session_id = ?
              AND is_deleted = 0
              AND location_label IS NOT NULL
              AND location_x IS NOT NULL
            GROUP BY location_label
            ORDER BY timestamp DESC
        """, (session_id,))

        rows = await cursor.fetchall()

        references = {}
        for row in rows:
            label, x, y, _ = row
            references[label.lower()] = (x, y, label)

        # Update cache
        self._spatial_cache[session_id] = references

        return references.copy()

    def _update_spatial_cache(
        self,
        session_id: str,
        key: str,
        x: float,
        y: float,
        label: str
    ):
        """Update in-memory spatial reference cache."""
        if session_id not in self._spatial_cache:
            self._spatial_cache[session_id] = {}

        self._spatial_cache[session_id][key.lower()] = (x, y, label)

    async def delete_session(self, session_id: str) -> bool:
        """
        Soft delete a session (marks as deleted, doesn't remove data).

        Returns:
            True if session existed and was deleted
        """
        await self.connect()

        cursor = await self._connection.execute("""
            UPDATE conversations
            SET is_deleted = 1
            WHERE session_id = ?
        """, (session_id,))

        await self._connection.commit()

        # Clear from cache
        if session_id in self._spatial_cache:
            del self._spatial_cache[session_id]

        deleted_count = cursor.rowcount
        safe_session = repr(session_id)
        logger.info("Soft deleted %s turns from session %s", deleted_count, safe_session)

        return deleted_count > 0

    async def cleanup_old_sessions(self, days_old: int = 30) -> int:
        """
        Permanently delete sessions older than specified days.

        Returns:
            Number of turns deleted
        """
        await self.connect()

        cursor = await self._connection.execute("""
            DELETE FROM conversations
            WHERE datetime(timestamp) < datetime('now', ? || ' days')
        """, (f"-{days_old}",))

        await self._connection.commit()

        deleted_count = cursor.rowcount
        logger.info("Cleaned up %s old conversation turns", deleted_count)

        return deleted_count


# Singleton instance
_db_instance: Optional[ConversationDatabase] = None


def get_conversation_db() -> ConversationDatabase:
    """
    Get singleton conversation database instance.

    Usage:
        db = get_conversation_db()
        await db.add_turn(...)
    """
    global _db_instance

    if _db_instance is None:
        db_path = os.getenv("CONVERSATION_DB_PATH", "data/conversations.db")
        _db_instance = ConversationDatabase(db_path)

    return _db_instance
