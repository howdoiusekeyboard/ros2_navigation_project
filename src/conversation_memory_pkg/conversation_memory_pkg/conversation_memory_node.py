#!/usr/bin/env python3
"""
Conversation Memory Node

Stores dialogue history in SQLite and provides context for LLM command parsing.
Implements ROSGPT pattern with context injection.

Week 2 will add full business logic.
Week 1: Skeleton with SQLite setup and ROS2 interfaces.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import sqlite3
import json
import os
from datetime import datetime


class ConversationMemoryNode(Node):
    """
    Manages conversation history and context for voice-controlled robot.

    Subscriptions:
        /voice/transcription (String): Whisper transcription output
        /robot/pose (PoseStamped): Current robot position

    Publications:
        /conversation/context (String): JSON context for downstream processing
        /conversation/history (String): Serialized conversation history
    """

    def __init__(self):
        super().__init__('conversation_memory_node')

        # Parameters
        self.declare_parameter('history_length', 5)
        self.declare_parameter('db_path', '~/.ros/conversation_history.db')

        self.history_length = self.get_parameter('history_length').value
        self.db_path = os.path.expanduser(self.get_parameter('db_path').value)

        # Initialize SQLite database
        self._init_database()

        # Current state
        self.current_pose = None
        self.current_location = "unknown"

        # Publishers
        self.context_publisher = self.create_publisher(
            String, '/conversation/context', 10
        )
        self.history_publisher = self.create_publisher(
            String, '/conversation/history', 10
        )

        # Subscribers
        self.transcription_sub = self.create_subscription(
            String, '/voice/transcription', self.transcription_callback, 10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, '/robot/pose', self.pose_callback, 10
        )

        # Timer for periodic history publication
        self.history_timer = self.create_timer(5.0, self.publish_history)

        self.get_logger().info(f'Conversation Memory Node initialized')
        self.get_logger().info(f'Database path: {self.db_path}')
        self.get_logger().info(f'History length: {self.history_length}')

    def _init_database(self):
        """Initialize SQLite database with conversation history table."""
        try:
            # Create directory if it doesn't exist
            db_dir = os.path.dirname(self.db_path)
            if db_dir and not os.path.exists(db_dir):
                os.makedirs(db_dir)

            # Connect to database
            self.conn = sqlite3.connect(self.db_path, check_same_thread=False)
            self.cursor = self.conn.cursor()

            # Create table if not exists
            self.cursor.execute('''
                CREATE TABLE IF NOT EXISTS conversation_history (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TEXT NOT NULL,
                    user_input TEXT NOT NULL,
                    robot_response TEXT,
                    location TEXT,
                    pose_x REAL,
                    pose_y REAL,
                    pose_theta REAL
                )
            ''')
            self.conn.commit()

            self.get_logger().info('SQLite database initialized successfully')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize database: {e}')
            raise

    def transcription_callback(self, msg: String):
        """
        Handle incoming voice transcription.

        This is where we'll add Gemini parsing in Week 2.
        For now, just log and store.
        """
        user_input = msg.data
        self.get_logger().info(f'Received transcription: "{user_input}"')

        # Store in database
        self._save_conversation_turn(user_input)

        # Build and publish context
        context = self._build_context(user_input)
        self._publish_context(context)

    def pose_callback(self, msg: PoseStamped):
        """Update current robot pose."""
        self.current_pose = msg.pose
        # TODO Week 2: Convert pose to semantic location (kitchen, hallway, etc.)
        self.current_location = self._pose_to_location(msg.pose)

    def _pose_to_location(self, pose):
        """
        Convert pose to semantic location.
        Week 2: Implement location mapping.
        """
        # Placeholder - will be implemented in Week 2
        # For now, just return coordinates
        x = pose.position.x
        y = pose.position.y
        return f"({x:.2f}, {y:.2f})"

    def _save_conversation_turn(self, user_input: str, robot_response: str = None):
        """Save a conversation turn to SQLite database."""
        try:
            timestamp = datetime.now().isoformat()
            pose_x = self.current_pose.position.x if self.current_pose else 0.0
            pose_y = self.current_pose.position.y if self.current_pose else 0.0
            pose_theta = 0.0  # TODO: Extract from quaternion

            self.cursor.execute('''
                INSERT INTO conversation_history
                (timestamp, user_input, robot_response, location, pose_x, pose_y, pose_theta)
                VALUES (?, ?, ?, ?, ?, ?, ?)
            ''', (timestamp, user_input, robot_response, self.current_location,
                  pose_x, pose_y, pose_theta))
            self.conn.commit()

            self.get_logger().debug(f'Saved conversation turn to database')

        except Exception as e:
            self.get_logger().error(f'Failed to save conversation turn: {e}')

    def _get_history(self, n_turns: int = None):
        """Retrieve last N conversation turns from database."""
        if n_turns is None:
            n_turns = self.history_length

        try:
            self.cursor.execute('''
                SELECT user_input, robot_response, location, timestamp
                FROM conversation_history
                ORDER BY id DESC
                LIMIT ?
            ''', (n_turns,))

            rows = self.cursor.fetchall()
            # Reverse to get chronological order
            return list(reversed(rows))

        except Exception as e:
            self.get_logger().error(f'Failed to retrieve history: {e}')
            return []

    def _build_context(self, current_input: str):
        """
        Build context dictionary for LLM prompt injection.

        Week 2: This will be sent to Gemini for context-aware parsing.
        """
        history = self._get_history()

        context = {
            'current_input': current_input,
            'conversation_history': [
                {
                    'user': row[0],
                    'robot': row[1] or '',
                    'location': row[2] or '',
                    'time': row[3]
                }
                for row in history
            ],
            'current_location': self.current_location,
            'current_pose': {
                'x': self.current_pose.position.x if self.current_pose else 0.0,
                'y': self.current_pose.position.y if self.current_pose else 0.0,
                'z': self.current_pose.position.z if self.current_pose else 0.0
            } if self.current_pose else None,
            'timestamp': datetime.now().isoformat()
        }

        return context

    def _publish_context(self, context: dict):
        """Publish context as JSON string."""
        msg = String()
        msg.data = json.dumps(context)
        self.context_publisher.publish(msg)
        self.get_logger().debug(f'Published context: {len(msg.data)} bytes')

    def publish_history(self):
        """Periodically publish conversation history for dashboard."""
        history = self._get_history(self.history_length)

        history_data = {
            'turns': [
                {
                    'user': row[0],
                    'robot': row[1] or '',
                    'location': row[2] or '',
                    'time': row[3]
                }
                for row in history
            ],
            'total_turns': len(history),
            'timestamp': datetime.now().isoformat()
        }

        msg = String()
        msg.data = json.dumps(history_data)
        self.history_publisher.publish(msg)

    def destroy_node(self):
        """Clean up resources."""
        if hasattr(self, 'conn'):
            self.conn.close()
            self.get_logger().info('Database connection closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ConversationMemoryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
