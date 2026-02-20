#!/usr/bin/env python3
"""
Integration Tests for Conversation Memory System

Tests the conversation memory node, spatial reference resolution,
and command parsing functionality.
"""

import pytest
import sqlite3
import tempfile
import os
import json
from datetime import datetime
from unittest.mock import MagicMock, patch

# Test imports
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class TestSemanticZoneResolver:
    """Tests for semantic zone resolution."""

    @pytest.fixture
    def zone_resolver(self):
        """Create a SemanticZoneResolver with test zones."""
        # Import the resolver class
        from conversation_memory_pkg.conversation_memory_node import SemanticZoneResolver

        # Create temp config file
        config = {
            'zones': [
                {
                    'name': 'central_hallway',
                    'type': 'human_zone',
                    'bounds': {'x_min': -1.0, 'x_max': 1.0, 'y_min': -1.5, 'y_max': 1.5}
                },
                {
                    'name': 'north_entrance',
                    'type': 'human_zone',
                    'bounds': {'x_min': -0.5, 'x_max': 0.5, 'y_min': 1.5, 'y_max': 2.5}
                },
                {
                    'name': 'kitchen',
                    'type': 'furniture_zone',
                    'bounds': {'x_min': 2.0, 'x_max': 4.0, 'y_min': 2.0, 'y_max': 4.0}
                }
            ]
        }

        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            import yaml
            yaml.dump(config, f)
            config_path = f.name

        try:
            resolver = SemanticZoneResolver(config_path)
            yield resolver
        finally:
            os.unlink(config_path)

    def test_pose_to_zone_in_zone(self, zone_resolver):
        """Test pose within a defined zone."""
        # Position in central_hallway
        zone = zone_resolver.pose_to_zone(0.0, 0.0)
        assert zone == 'central_hallway'

    def test_pose_to_zone_outside_zone(self, zone_resolver):
        """Test pose outside all defined zones."""
        zone = zone_resolver.pose_to_zone(10.0, 10.0)
        assert zone is None

    def test_zone_to_coords_predefined(self, zone_resolver):
        """Test getting coordinates from predefined zone."""
        coords = zone_resolver.zone_to_coords('kitchen')
        assert coords is not None
        # Center of kitchen bounds
        assert coords[0] == pytest.approx(3.0, abs=0.1)  # (2+4)/2
        assert coords[1] == pytest.approx(3.0, abs=0.1)  # (2+4)/2

    def test_zone_to_coords_unknown(self, zone_resolver):
        """Test getting coordinates from unknown zone."""
        coords = zone_resolver.zone_to_coords('nonexistent')
        assert coords is None

    def test_learn_location(self, zone_resolver):
        """Test learning a new location."""
        zone_resolver.learn_location('lab', 5.0, 5.0)

        # Should be able to retrieve it
        coords = zone_resolver.zone_to_coords('lab')
        assert coords == (5.0, 5.0)

    def test_learned_location_overrides_zone(self, zone_resolver):
        """Test that learned locations take priority."""
        # Learn a location with same name as zone
        zone_resolver.learn_location('kitchen', 1.0, 1.0)

        coords = zone_resolver.zone_to_coords('kitchen')
        assert coords == (1.0, 1.0)  # Not the zone center

    def test_pose_to_zone_learned_location(self, zone_resolver):
        """Test pose near learned location returns location name."""
        zone_resolver.learn_location('lab', 5.0, 5.0)

        # Position near lab (within 1m)
        zone = zone_resolver.pose_to_zone(5.2, 5.3)
        assert zone == 'lab'


class TestConversationDatabase:
    """Tests for conversation history database."""

    @pytest.fixture
    def db_connection(self):
        """Create a test database."""
        with tempfile.NamedTemporaryFile(suffix='.db', delete=False) as f:
            db_path = f.name

        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()

        # Create tables
        cursor.execute('''
            CREATE TABLE conversation_history (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                session_id TEXT NOT NULL,
                timestamp TEXT NOT NULL,
                user_input TEXT NOT NULL,
                robot_response TEXT,
                parsed_action TEXT,
                location TEXT,
                pose_x REAL,
                pose_y REAL,
                pose_theta REAL
            )
        ''')

        cursor.execute('''
            CREATE TABLE spatial_references (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                session_id TEXT NOT NULL,
                name TEXT NOT NULL,
                x REAL NOT NULL,
                y REAL NOT NULL,
                created_at TEXT NOT NULL,
                UNIQUE(session_id, name)
            )
        ''')

        conn.commit()

        yield conn, db_path

        conn.close()
        os.unlink(db_path)

    def test_save_conversation_turn(self, db_connection):
        """Test saving a conversation turn."""
        conn, db_path = db_connection
        cursor = conn.cursor()

        session_id = 'test_session_1'
        timestamp = datetime.now().isoformat()

        cursor.execute('''
            INSERT INTO conversation_history
            (session_id, timestamp, user_input, robot_response, parsed_action, location, pose_x, pose_y, pose_theta)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
        ''', (session_id, timestamp, 'go forward', 'Moving forward', 'move_forward', 'hallway', 1.0, 2.0, 0.5))
        conn.commit()

        # Verify insertion
        cursor.execute('SELECT * FROM conversation_history WHERE session_id = ?', (session_id,))
        row = cursor.fetchone()

        assert row is not None
        assert row[3] == 'go forward'
        assert row[4] == 'Moving forward'
        assert row[5] == 'move_forward'

    def test_retrieve_history_order(self, db_connection):
        """Test history is retrieved in correct order."""
        conn, db_path = db_connection
        cursor = conn.cursor()

        session_id = 'test_session_2'

        # Insert multiple turns
        for i in range(5):
            cursor.execute('''
                INSERT INTO conversation_history
                (session_id, timestamp, user_input, robot_response, parsed_action, location, pose_x, pose_y, pose_theta)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            ''', (session_id, f'2024-01-{15+i}T12:00:00', f'command_{i}', f'response_{i}', 'action', 'loc', 0, 0, 0))
        conn.commit()

        # Retrieve last 3
        cursor.execute('''
            SELECT user_input FROM conversation_history
            WHERE session_id = ?
            ORDER BY id DESC
            LIMIT 3
        ''', (session_id,))
        rows = cursor.fetchall()

        # Should be in reverse order (newest first)
        assert rows[0][0] == 'command_4'
        assert rows[1][0] == 'command_3'
        assert rows[2][0] == 'command_2'

    def test_spatial_reference_storage(self, db_connection):
        """Test storing and retrieving spatial references."""
        conn, db_path = db_connection
        cursor = conn.cursor()

        session_id = 'test_session_3'

        # Store a location
        cursor.execute('''
            INSERT INTO spatial_references
            (session_id, name, x, y, created_at)
            VALUES (?, ?, ?, ?, ?)
        ''', (session_id, 'meeting_room', 3.5, 4.5, datetime.now().isoformat()))
        conn.commit()

        # Retrieve it
        cursor.execute('''
            SELECT name, x, y FROM spatial_references
            WHERE session_id = ?
        ''', (session_id,))
        row = cursor.fetchone()

        assert row[0] == 'meeting_room'
        assert row[1] == pytest.approx(3.5)
        assert row[2] == pytest.approx(4.5)

    def test_spatial_reference_uniqueness(self, db_connection):
        """Test that duplicate location names are handled."""
        conn, db_path = db_connection
        cursor = conn.cursor()

        session_id = 'test_session_4'

        # Store initial location
        cursor.execute('''
            INSERT INTO spatial_references
            (session_id, name, x, y, created_at)
            VALUES (?, ?, ?, ?, ?)
        ''', (session_id, 'lab', 1.0, 1.0, datetime.now().isoformat()))
        conn.commit()

        # Update with INSERT OR REPLACE
        cursor.execute('''
            INSERT OR REPLACE INTO spatial_references
            (session_id, name, x, y, created_at)
            VALUES (?, ?, ?, ?, ?)
        ''', (session_id, 'lab', 2.0, 2.0, datetime.now().isoformat()))
        conn.commit()

        # Should only have one entry with updated coords
        cursor.execute('''
            SELECT x, y FROM spatial_references
            WHERE session_id = ? AND name = ?
        ''', (session_id, 'lab'))
        row = cursor.fetchone()

        assert row[0] == pytest.approx(2.0)
        assert row[1] == pytest.approx(2.0)


class TestCommandParsing:
    """Tests for command parsing (fallback regex parser)."""

    @pytest.fixture
    def parser(self):
        """Create parser helper function."""
        import re
        import math

        def parse_command_fallback(command):
            cmd_lower = command.lower().strip()

            # Stop command
            if re.search(r'\b(stop|halt|freeze)\b', cmd_lower):
                return {
                    'action': 'stop',
                    'parameters': {},
                    'confidence': 0.95,
                    'reasoning': 'Stop command detected'
                }

            # Forward movement
            match = re.search(r'(?:move |go )?forward(?: (\d+(?:\.\d+)?))?(?: meters?)?', cmd_lower)
            if match:
                distance = float(match.group(1)) if match.group(1) else 1.0
                return {
                    'action': 'move_forward',
                    'parameters': {'distance': min(distance, 5.0)},
                    'confidence': 0.85,
                    'reasoning': f'Forward movement: {distance}m'
                }

            # Turn/rotate
            match = re.search(r'(?:turn |rotate )(left|right)(?: (\d+))?(?: degrees?)?', cmd_lower)
            if match:
                direction = match.group(1)
                degrees = float(match.group(2)) if match.group(2) else 90.0
                radians = math.radians(degrees) * (1 if direction == 'left' else -1)
                return {
                    'action': 'rotate',
                    'parameters': {'angle': radians},
                    'confidence': 0.85,
                    'reasoning': f'Rotate {direction} {degrees} degrees'
                }

            return {
                'action': 'unknown',
                'parameters': {},
                'confidence': 0.0,
                'reasoning': 'Could not parse command'
            }

        return parse_command_fallback

    def test_stop_command(self, parser):
        """Test stop command parsing."""
        result = parser("stop")
        assert result['action'] == 'stop'
        assert result['confidence'] >= 0.9

        result = parser("halt now")
        assert result['action'] == 'stop'

        result = parser("freeze!")
        assert result['action'] == 'stop'

    def test_forward_command(self, parser):
        """Test forward movement parsing."""
        result = parser("go forward")
        assert result['action'] == 'move_forward'
        assert result['parameters']['distance'] == 1.0

        result = parser("move forward 2 meters")
        assert result['action'] == 'move_forward'
        assert result['parameters']['distance'] == 2.0

        result = parser("forward 3.5")
        assert result['action'] == 'move_forward'
        assert result['parameters']['distance'] == 3.5

    def test_turn_command(self, parser):
        """Test rotation command parsing."""
        import math

        result = parser("turn left")
        assert result['action'] == 'rotate'
        assert result['parameters']['angle'] == pytest.approx(math.radians(90))

        result = parser("rotate right 45 degrees")
        assert result['action'] == 'rotate'
        assert result['parameters']['angle'] == pytest.approx(-math.radians(45))

    def test_unknown_command(self, parser):
        """Test unknown command handling."""
        result = parser("do something random")
        assert result['action'] == 'unknown'
        assert result['confidence'] == 0.0


class TestLocationLearning:
    """Tests for location learning patterns."""

    @pytest.fixture
    def location_patterns(self):
        """Get location learning regex patterns."""
        import re

        patterns = [
            r'call this (?:place |location |spot )?(?:the )?(.+)',
            r'this is (?:the )?(.+)',
            r'remember this (?:as |place as )?(?:the )?(.+)',
            r'name this (?:place |location )?(?:the )?(.+)'
        ]

        def extract_location_name(user_input):
            for pattern in patterns:
                match = re.search(pattern, user_input.lower())
                if match:
                    return match.group(1).strip()
            return None

        return extract_location_name

    def test_call_this_pattern(self, location_patterns):
        """Test 'call this place X' pattern."""
        name = location_patterns("call this place the kitchen")
        assert name == "kitchen"

        name = location_patterns("call this the lab")
        assert name == "lab"

    def test_this_is_pattern(self, location_patterns):
        """Test 'this is X' pattern."""
        name = location_patterns("this is the meeting room")
        assert name == "meeting room"

    def test_remember_this_pattern(self, location_patterns):
        """Test 'remember this as X' pattern."""
        name = location_patterns("remember this as the office")
        assert name == "office"

    def test_name_this_pattern(self, location_patterns):
        """Test 'name this place X' pattern."""
        name = location_patterns("name this location charging station")
        assert name == "charging station"

    def test_no_match(self, location_patterns):
        """Test no match returns None."""
        name = location_patterns("go forward 2 meters")
        assert name is None


class TestContextBuilding:
    """Tests for LLM context building."""

    def test_context_structure(self):
        """Test context dictionary structure."""
        context = {
            'current_input': 'go to the kitchen',
            'session_id': 'session_123',
            'conversation_history': [
                {'user': 'go forward', 'robot': 'Moving forward', 'location': 'hallway', 'time': '12:00'},
                {'user': 'turn left', 'robot': 'Turning left', 'location': 'hallway', 'time': '12:01'}
            ],
            'current_location': 'entrance',
            'current_zone': 'north_entrance',
            'current_pose': {'x': 0.5, 'y': 2.0, 'z': 0.0},
            'known_locations': {
                'kitchen': {'x': 3.0, 'y': 3.0},
                'lab': {'x': 5.0, 'y': 5.0}
            }
        }

        # Verify required fields
        assert 'current_input' in context
        assert 'session_id' in context
        assert 'conversation_history' in context
        assert isinstance(context['conversation_history'], list)
        assert 'known_locations' in context

    def test_context_to_summary_format(self):
        """Test context summary formatting for LLM."""
        context = {
            'conversation_history': [
                {'user': 'go forward', 'robot': 'Moving', 'location': 'hall', 'time': '12:00', 'action': 'move'},
            ],
            'current_location': 'entrance',
            'current_pose': {'x': 0.5, 'y': 2.0},
            'known_locations': {'kitchen': {'x': 3.0, 'y': 3.0}}
        }

        # Build summary
        lines = []
        if context.get('conversation_history'):
            lines.append("Recent conversation:")
            for turn in context['conversation_history'][-5:]:
                lines.append(f"  User: {turn['user']}")
                if turn.get('robot'):
                    lines.append(f"  Robot: {turn['robot']}")

        if context.get('current_location'):
            lines.append(f"\nCurrent location: {context['current_location']}")

        if context.get('current_pose'):
            pose = context['current_pose']
            lines.append(f"Current position: ({pose['x']:.2f}, {pose['y']:.2f})")

        summary = "\n".join(lines)

        assert "Recent conversation:" in summary
        assert "User: go forward" in summary
        assert "Robot: Moving" in summary
        assert "Current location: entrance" in summary
        assert "Current position: (0.50, 2.00)" in summary


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
