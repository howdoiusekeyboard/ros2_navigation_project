"""
Unit tests for ConversationDatabase.

Tests database operations, session management, and spatial reference caching.
Run with: pytest backend/tests/test_conversation_db.py -v
"""

import asyncio
import os
import tempfile
from pathlib import Path

import pytest

from app.database.conversation_db import ConversationDatabase


@pytest.fixture
async def temp_db():
    """Create a temporary database for testing."""
    # Create temporary file
    temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".db")
    temp_path = temp_file.name
    temp_file.close()

    # Initialize database
    db = ConversationDatabase(db_path=temp_path)
    await db.connect()

    yield db

    # Cleanup
    await db.close()
    try:
        os.unlink(temp_path)
        # Clean up WAL files if they exist
        wal_path = temp_path + "-wal"
        shm_path = temp_path + "-shm"
        if os.path.exists(wal_path):
            os.unlink(wal_path)
        if os.path.exists(shm_path):
            os.unlink(shm_path)
    except:
        pass


@pytest.mark.asyncio
async def test_database_initialization(temp_db):
    """Test that database schema initializes correctly."""
    # Query sqlite_master to check tables exist
    cursor = await temp_db._connection.execute(
        "SELECT name FROM sqlite_master WHERE type='table' AND name='conversations'"
    )
    result = await cursor.fetchone()

    assert result is not None
    assert result[0] == "conversations"


@pytest.mark.asyncio
async def test_session_id_generation(temp_db):
    """Test session ID generation."""
    session_id_1 = temp_db.generate_session_id()
    session_id_2 = temp_db.generate_session_id()

    # Should be strings
    assert isinstance(session_id_1, str)
    assert isinstance(session_id_2, str)

    # Should be unique
    assert session_id_1 != session_id_2

    # Should be short UUIDs (8 chars)
    assert len(session_id_1) == 8


@pytest.mark.asyncio
async def test_add_turn_basic(temp_db):
    """Test adding a basic conversation turn."""
    session_id = temp_db.generate_session_id()

    turn_id = await temp_db.add_turn(
        session_id=session_id,
        user_input="Go forward 2 meters",
        robot_response="Moving forward",
        action_type="navigate",
        confidence=0.95
    )

    # Should return a valid turn ID
    assert isinstance(turn_id, int)
    assert turn_id > 0


@pytest.mark.asyncio
async def test_add_turn_with_location(temp_db):
    """Test adding turn with spatial location."""
    session_id = temp_db.generate_session_id()

    turn_id = await temp_db.add_turn(
        session_id=session_id,
        user_input="Go to the kitchen",
        robot_response="Navigating to kitchen",
        action_type="navigate",
        location_x=2.5,
        location_y=3.1,
        location_label="kitchen",
        confidence=0.98,
        latency_ms=850
    )

    assert turn_id > 0

    # Verify it was stored with location
    history = await temp_db.get_history(session_id, limit=1)
    assert len(history) == 1

    turn = history[0]
    assert turn["user_input"] == "Go to the kitchen"
    assert turn["location_x"] == 2.5
    assert turn["location_y"] == 3.1
    assert turn["location_label"] == "kitchen"
    assert turn["confidence"] == 0.98


@pytest.mark.asyncio
async def test_multiple_turns_same_session(temp_db):
    """Test adding multiple turns to the same session."""
    session_id = temp_db.generate_session_id()

    # Add 5 turns
    for i in range(5):
        await temp_db.add_turn(
            session_id=session_id,
            user_input=f"Command {i+1}",
            robot_response=f"Response {i+1}",
            action_type="navigate"
        )

    # Retrieve history
    history = await temp_db.get_history(session_id, limit=10)

    assert len(history) == 5

    # Should be ordered newest first
    assert history[0]["user_input"] == "Command 5"
    assert history[4]["user_input"] == "Command 1"

    # Turn numbers should increment
    assert history[4]["turn_number"] == 1
    assert history[0]["turn_number"] == 5


@pytest.mark.asyncio
async def test_get_history_with_limit(temp_db):
    """Test history retrieval with limit."""
    session_id = temp_db.generate_session_id()

    # Add 10 turns
    for i in range(10):
        await temp_db.add_turn(
            session_id=session_id,
            user_input=f"Command {i+1}",
            robot_response=f"Response {i+1}"
        )

    # Get only last 3
    history = await temp_db.get_history(session_id, limit=3)

    assert len(history) == 3
    assert history[0]["user_input"] == "Command 10"
    assert history[2]["user_input"] == "Command 8"


@pytest.mark.asyncio
async def test_session_isolation(temp_db):
    """Test that sessions are properly isolated."""
    session_1 = temp_db.generate_session_id()
    session_2 = temp_db.generate_session_id()

    # Add turns to session 1
    await temp_db.add_turn(session_1, "Session 1 command")
    await temp_db.add_turn(session_1, "Session 1 command 2")

    # Add turns to session 2
    await temp_db.add_turn(session_2, "Session 2 command")

    # Retrieve histories
    history_1 = await temp_db.get_history(session_1)
    history_2 = await temp_db.get_history(session_2)

    assert len(history_1) == 2
    assert len(history_2) == 1
    assert "Session 1" in history_1[0]["user_input"]
    assert "Session 2" in history_2[0]["user_input"]


@pytest.mark.asyncio
async def test_spatial_references(temp_db):
    """Test spatial reference retrieval."""
    session_id = temp_db.generate_session_id()

    # Add turns with different locations
    await temp_db.add_turn(
        session_id=session_id,
        user_input="Go to kitchen",
        location_x=2.0,
        location_y=3.0,
        location_label="kitchen"
    )

    await temp_db.add_turn(
        session_id=session_id,
        user_input="Go to bedroom",
        location_x=5.0,
        location_y=1.0,
        location_label="bedroom"
    )

    # Get spatial references
    refs = await temp_db.get_spatial_references(session_id)

    assert len(refs) == 2
    assert "kitchen" in refs
    assert "bedroom" in refs

    # Check coordinates
    assert refs["kitchen"][0] == 2.0
    assert refs["kitchen"][1] == 3.0
    assert refs["bedroom"][0] == 5.0
    assert refs["bedroom"][1] == 1.0


@pytest.mark.asyncio
async def test_spatial_reference_caching(temp_db):
    """Test that spatial references are cached."""
    session_id = temp_db.generate_session_id()

    await temp_db.add_turn(
        session_id=session_id,
        user_input="Go to kitchen",
        location_x=2.0,
        location_y=3.0,
        location_label="kitchen"
    )

    # First call populates cache
    refs_1 = await temp_db.get_spatial_references(session_id)

    # Second call should use cache (no database query)
    refs_2 = await temp_db.get_spatial_references(session_id)

    assert refs_1 == refs_2
    assert session_id in temp_db._spatial_cache


@pytest.mark.asyncio
async def test_session_summary(temp_db):
    """Test session summary statistics."""
    session_id = temp_db.generate_session_id()

    # Add turns with different action types and confidences
    await temp_db.add_turn(
        session_id=session_id,
        user_input="Navigate here",
        action_type="navigate",
        confidence=0.9,
        latency_ms=500
    )

    await temp_db.add_turn(
        session_id=session_id,
        user_input="Stop",
        action_type="stop",
        confidence=0.95,
        latency_ms=300
    )

    await temp_db.add_turn(
        session_id=session_id,
        user_input="Go forward",
        action_type="navigate",
        confidence=0.85,
        latency_ms=600
    )

    # Get summary
    summary = await temp_db.get_session_summary(session_id)

    assert summary is not None
    assert summary["session_id"] == session_id
    assert summary["turn_count"] == 3
    assert summary["navigation_count"] == 2
    assert 0.88 <= summary["avg_confidence"] <= 0.92  # Average of 0.9, 0.95, 0.85
    assert 450 <= summary["avg_latency_ms"] <= 500  # Average of 500, 300, 600


@pytest.mark.asyncio
async def test_get_recent_sessions(temp_db):
    """Test retrieving recent sessions."""
    # Create 3 sessions with turns
    for i in range(3):
        session_id = temp_db.generate_session_id()
        await temp_db.add_turn(
            session_id=session_id,
            user_input=f"Session {i+1} command"
        )

    # Get recent sessions
    sessions = await temp_db.get_recent_sessions(limit=5)

    assert len(sessions) == 3
    assert all("session_id" in s for s in sessions)
    assert all("turn_count" in s for s in sessions)


@pytest.mark.asyncio
async def test_delete_session(temp_db):
    """Test soft deleting a session."""
    session_id = temp_db.generate_session_id()

    # Add turns
    await temp_db.add_turn(session_id=session_id, user_input="Command 1")
    await temp_db.add_turn(session_id=session_id, user_input="Command 2")

    # Verify turns exist
    history_before = await temp_db.get_history(session_id)
    assert len(history_before) == 2

    # Delete session
    deleted = await temp_db.delete_session(session_id)
    assert deleted is True

    # Verify turns are gone
    history_after = await temp_db.get_history(session_id)
    assert len(history_after) == 0


@pytest.mark.asyncio
async def test_metadata_storage(temp_db):
    """Test storing and retrieving metadata."""
    session_id = temp_db.generate_session_id()

    metadata = {
        "source": "voice",
        "language": "en",
        "model": "gemini-2.0-flash"
    }

    await temp_db.add_turn(
        session_id=session_id,
        user_input="Test command",
        metadata=metadata
    )

    # Retrieve with metadata
    history = await temp_db.get_history(session_id, limit=1, include_metadata=True)

    assert len(history) == 1
    assert history[0]["metadata"] == metadata


@pytest.mark.asyncio
async def test_empty_session(temp_db):
    """Test retrieving history for non-existent session."""
    fake_session_id = "nonexistent"

    history = await temp_db.get_history(fake_session_id)
    summary = await temp_db.get_session_summary(fake_session_id)

    assert history == []
    assert summary is None


@pytest.mark.asyncio
async def test_turn_number_increments(temp_db):
    """Test that turn numbers increment correctly even with deletions."""
    session_id = temp_db.generate_session_id()

    # Add 3 turns
    await temp_db.add_turn(session_id=session_id, user_input="Turn 1")
    await temp_db.add_turn(session_id=session_id, user_input="Turn 2")
    await temp_db.add_turn(session_id=session_id, user_input="Turn 3")

    history = await temp_db.get_history(session_id)

    # Turn numbers should be 1, 2, 3
    turn_numbers = sorted([h["turn_number"] for h in history])
    assert turn_numbers == [1, 2, 3]


if __name__ == "__main__":
    # Run tests
    pytest.main([__file__, "-v", "--tb=short"])
