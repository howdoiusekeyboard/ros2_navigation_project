# Week 2 Complete Implementation Guide
## Conversational Memory with Context-Aware Dialogue

**Duration:** 7 days (Jan 27 - Feb 2, 2025)  
**Goal:** Enable 5+ turn conversations with spatial context awareness  
**Status:** Building on Week 1 foundation ✓  
**Critical Milestone:** Working conversational memory system by Friday

---

## Pre-Week 2 Checklist

Before starting, verify Week 1 deliverables are complete:

- [ ] All ROS2 packages build successfully (`colcon build`)
- [ ] Custom messages defined in `intelligent_twin_msgs`
- [ ] `conversation_memory_node` skeleton exists
- [ ] Database initialization code works
- [ ] Digital twin simulation launches
- [ ] Dashboard renders with new panels
- [ ] Git repository up to date

**If any item is incomplete, resolve it before proceeding.**

---

## Week 2 Overview

### What You'll Build

By end of Week 2, you'll have:

1. **SQLite database** storing conversation history with locations
2. **Context retrieval system** that fetches relevant past interactions
3. **LLM prompt engineering** that injects context into Gemini API calls
4. **Spatial reference resolver** that understands "there", "before", "back"
5. **Session management** that persists across robot restarts
6. **Integration** with your existing voice control pipeline

### Key Technical Challenges

1. **Context Window Management:** Keeping prompts under Gemini token limits
2. **Reference Resolution:** Mapping vague terms to specific coordinates
3. **Conversation Flow:** Maintaining natural dialogue rhythm
4. **Memory Relevance:** Selecting which past interactions to include
5. **Performance:** Keeping response times under 2 seconds

---

## Day 1 (Monday): Database Schema & Core Memory Functions

### Morning Session (4 hours)

#### Task 1.1: Enhanced Database Schema (1.5 hours)

Update your database schema to support rich conversational context.

**File:** `conversation_memory_node/conversation_memory_node/memory_manager.py`

```python
def init_database(self):
    """Initialize SQLite database with enhanced schema for conversational memory"""
    import os
    from pathlib import Path
    
    # Expand path and ensure directory exists
    db_path = os.path.expanduser(self.db_path)
    db_dir = os.path.dirname(db_path)
    Path(db_dir).mkdir(parents=True, exist_ok=True)
    
    self.conn = sqlite3.connect(db_path, check_same_thread=False)
    self.cursor = self.conn.cursor()
    
    # Main conversations table
    self.cursor.execute('''
        CREATE TABLE IF NOT EXISTS conversations (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            session_id TEXT NOT NULL,
            timestamp TEXT NOT NULL,
            turn_number INTEGER NOT NULL,
            user_input TEXT NOT NULL,
            robot_response TEXT,
            location_x REAL,
            location_y REAL,
            location_z REAL,
            location_label TEXT,
            command_type TEXT,
            success BOOLEAN,
            context_used TEXT,
            metadata TEXT
        )
    ''')
    
    # Spatial references table (for "there", "that place", etc.)
    self.cursor.execute('''
        CREATE TABLE IF NOT EXISTS spatial_references (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            reference_term TEXT NOT NULL,
            location_x REAL NOT NULL,
            location_y REAL NOT NULL,
            location_z REAL,
            location_label TEXT,
            timestamp TEXT NOT NULL,
            session_id TEXT NOT NULL,
            frequency INTEGER DEFAULT 1
        )
    ''')
    
    # Session metadata table
    self.cursor.execute('''
        CREATE TABLE IF NOT EXISTS sessions (
            session_id TEXT PRIMARY KEY,
            start_time TEXT NOT NULL,
            end_time TEXT,
            total_turns INTEGER DEFAULT 0,
            is_active BOOLEAN DEFAULT 1
        )
    ''')
    
    # Create indexes for faster queries
    self.cursor.execute('''
        CREATE INDEX IF NOT EXISTS idx_session_timestamp 
        ON conversations(session_id, timestamp DESC)
    ''')
    
    self.cursor.execute('''
        CREATE INDEX IF NOT EXISTS idx_spatial_ref 
        ON spatial_references(reference_term, session_id)
    ''')
    
    self.conn.commit()
    self.get_logger().info(f'Enhanced database schema initialized at {db_path}')
```

**Action Items:**
- [ ] Update `init_database()` method with enhanced schema
- [ ] Add necessary imports: `import os, json, uuid` at top of file
- [ ] Test database creation: `ros2 run conversation_memory_node memory_manager`
- [ ] Verify tables: `sqlite3 ~/.ros/conversation_history.db ".tables"`

#### Task 1.2: Session Management (1.5 hours)

Implement session lifecycle management to group related conversations.

```python
class ConversationMemoryNode(Node):
    def __init__(self):
        super().__init__('conversation_memory')
        
        # ... existing initialization ...
        
        # Session management
        self.current_session_id = None
        self.turn_counter = 0
        self.session_start_time = None
        
        # Initialize or resume session
        self.start_new_session()
        
        self.get_logger().info(f'Session started: {self.current_session_id}')
    
    def start_new_session(self):
        """Start a new conversation session"""
        from datetime import datetime
        import uuid
        
        self.current_session_id = str(uuid.uuid4())[:8]  # Short UUID
        self.turn_counter = 0
        self.session_start_time = datetime.now().isoformat()
        
        self.cursor.execute('''
            INSERT INTO sessions (session_id, start_time, total_turns, is_active)
            VALUES (?, ?, 0, 1)
        ''', (self.current_session_id, self.session_start_time))
        self.conn.commit()
        
        self.get_logger().info(f'New session created: {self.current_session_id}')
        return self.current_session_id
    
    def end_session(self):
        """End current conversation session"""
        from datetime import datetime
        
        if not self.current_session_id:
            return
        
        end_time = datetime.now().isoformat()
        
        self.cursor.execute('''
            UPDATE sessions 
            SET end_time = ?, total_turns = ?, is_active = 0
            WHERE session_id = ?
        ''', (end_time, self.turn_counter, self.current_session_id))
        self.conn.commit()
        
        self.get_logger().info(
            f'Session ended: {self.current_session_id} ({self.turn_counter} turns)'
        )
    
    def resume_last_session(self):
        """Resume the most recent active session"""
        self.cursor.execute('''
            SELECT session_id, start_time, total_turns 
            FROM sessions 
            WHERE is_active = 1 
            ORDER BY start_time DESC 
            LIMIT 1
        ''')
        
        result = self.cursor.fetchone()
        if result:
            self.current_session_id = result[0]
            self.session_start_time = result[1]
            self.turn_counter = result[2]
            self.get_logger().info(f'Resumed session: {self.current_session_id}')
            return True
        return False
```

**Action Items:**
- [ ] Add session management methods to `ConversationMemoryNode`
- [ ] Update `__init__()` to start sessions
- [ ] Test session creation and retrieval
- [ ] Verify session data in database

#### Task 1.3: Core Memory Operations (1 hour)

Implement CRUD operations for conversation entries.

```python
def add_conversation_entry(
    self, 
    user_input: str, 
    robot_response: str = None,
    command_type: str = 'unknown',
    success: bool = True,
    location_label: str = None,
    context_used: dict = None
):
    """
    Add a conversation turn to memory
    
    Args:
        user_input: What the user said
        robot_response: What the robot said/did
        command_type: Type of command (navigate, query, info, etc.)
        success: Whether command executed successfully
        location_label: Semantic label for location (e.g., "kitchen")
        context_used: Dictionary of context that was used for this turn
    """
    from datetime import datetime
    import json
    
    timestamp = datetime.now().isoformat()
    self.turn_counter += 1
    
    # Get current position
    pos = self.current_pose.position if self.current_pose else None
    loc_x = pos.x if pos else None
    loc_y = pos.y if pos else None
    loc_z = pos.z if pos else None
    
    # Serialize context and metadata
    context_json = json.dumps(context_used) if context_used else None
    metadata = json.dumps({
        'turn_number': self.turn_counter,
        'session_duration': self._get_session_duration()
    })
    
    # Insert into database
    self.cursor.execute('''
        INSERT INTO conversations (
            session_id, timestamp, turn_number, user_input, robot_response,
            location_x, location_y, location_z, location_label,
            command_type, success, context_used, metadata
        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
    ''', (
        self.current_session_id, timestamp, self.turn_counter,
        user_input, robot_response, loc_x, loc_y, loc_z,
        location_label, command_type, success, context_json, metadata
    ))
    
    # Update session turn count
    self.cursor.execute('''
        UPDATE sessions SET total_turns = ? WHERE session_id = ?
    ''', (self.turn_counter, self.current_session_id))
    
    self.conn.commit()
    
    self.get_logger().info(
        f'Turn {self.turn_counter} stored: "{user_input[:50]}..."'
    )
    
    return self.cursor.lastrowid

def get_recent_history(self, limit: int = None, session_id: str = None):
    """
    Retrieve recent conversation history
    
    Args:
        limit: Maximum number of turns to retrieve (default: self.history_length)
        session_id: Specific session to retrieve (default: current session)
    
    Returns:
        List of tuples: (timestamp, user_input, robot_response, location, turn_number)
    """
    if limit is None:
        limit = self.history_length
    
    if session_id is None:
        session_id = self.current_session_id
    
    self.cursor.execute('''
        SELECT timestamp, user_input, robot_response, 
               location_x, location_y, location_z, location_label,
               turn_number, command_type, success
        FROM conversations
        WHERE session_id = ?
        ORDER BY timestamp DESC
        LIMIT ?
    ''', (session_id, limit))
    
    return self.cursor.fetchall()

def get_context_for_llm(self, max_turns: int = 5):
    """
    Format recent conversation history for LLM context injection
    
    Args:
        max_turns: Maximum number of previous turns to include
    
    Returns:
        Formatted string suitable for LLM prompt
    """
    history = self.get_recent_history(limit=max_turns)
    
    if not history:
        return "No previous conversation history."
    
    context_lines = ["Recent conversation:"]
    
    # Reverse to show chronological order (oldest first)
    for entry in reversed(history):
        timestamp, user_input, robot_response, loc_x, loc_y, loc_z, \
            loc_label, turn_num, cmd_type, success = entry
        
        # Format location
        if loc_label:
            location_str = f" at {loc_label}"
        elif loc_x is not None:
            location_str = f" at ({loc_x:.2f}, {loc_y:.2f})"
        else:
            location_str = ""
        
        # Format turn
        context_lines.append(f"Turn {turn_num}:")
        context_lines.append(f"  User: {user_input}")
        if robot_response:
            context_lines.append(f"  Robot: {robot_response}{location_str}")
    
    return "\n".join(context_lines)

def _get_session_duration(self):
    """Calculate duration of current session in seconds"""
    from datetime import datetime
    
    if not self.session_start_time:
        return 0
    
    start = datetime.fromisoformat(self.session_start_time)
    now = datetime.now()
    return (now - start).total_seconds()
```

**Action Items:**
- [ ] Implement all three memory operation methods
- [ ] Add helper method `_get_session_duration()`
- [ ] Test adding conversation entries
- [ ] Test retrieving history with different limits
- [ ] Verify context formatting for LLM

### Afternoon Session (3 hours)

#### Task 1.4: Spatial Reference System (2 hours)

Implement the system that resolves vague location terms to coordinates.

```python
def add_spatial_reference(
    self,
    reference_term: str,
    location_x: float,
    location_y: float,
    location_z: float = 0.0,
    location_label: str = None
):
    """
    Store a spatial reference for future lookup
    
    Args:
        reference_term: The term to remember (e.g., "there", "that place")
        location_x, location_y, location_z: Coordinates
        location_label: Optional semantic label
    """
    from datetime import datetime
    
    timestamp = datetime.now().isoformat()
    
    # Check if reference already exists in this session
    self.cursor.execute('''
        SELECT id, frequency FROM spatial_references
        WHERE reference_term = ? AND session_id = ?
    ''', (reference_term, self.current_session_id))
    
    existing = self.cursor.fetchone()
    
    if existing:
        # Update existing reference with new location and increment frequency
        self.cursor.execute('''
            UPDATE spatial_references
            SET location_x = ?, location_y = ?, location_z = ?,
                location_label = ?, timestamp = ?, frequency = frequency + 1
            WHERE id = ?
        ''', (location_x, location_y, location_z, location_label, 
              timestamp, existing[0]))
        
        self.get_logger().debug(
            f'Updated spatial reference: "{reference_term}" -> ({location_x:.2f}, {location_y:.2f})'
        )
    else:
        # Create new reference
        self.cursor.execute('''
            INSERT INTO spatial_references (
                reference_term, location_x, location_y, location_z,
                location_label, timestamp, session_id, frequency
            ) VALUES (?, ?, ?, ?, ?, ?, ?, 1)
        ''', (reference_term, location_x, location_y, location_z,
              location_label, timestamp, self.current_session_id))
        
        self.get_logger().info(
            f'Stored spatial reference: "{reference_term}" -> ({location_x:.2f}, {location_y:.2f})'
        )
    
    self.conn.commit()

def resolve_spatial_reference(self, reference_term: str):
    """
    Resolve a spatial reference term to coordinates
    
    Args:
        reference_term: Term to resolve (e.g., "there", "before", "back")
    
    Returns:
        tuple: (x, y, z, label) or None if not found
    """
    # Normalize the term
    term = reference_term.lower().strip()
    
    # Direct lookup
    self.cursor.execute('''
        SELECT location_x, location_y, location_z, location_label, frequency
        FROM spatial_references
        WHERE reference_term = ? AND session_id = ?
        ORDER BY timestamp DESC
        LIMIT 1
    ''', (term, self.current_session_id))
    
    result = self.cursor.fetchone()
    
    if result:
        x, y, z, label, freq = result
        self.get_logger().info(
            f'Resolved "{term}" -> ({x:.2f}, {y:.2f}) [{label or "unlabeled"}] (used {freq} times)'
        )
        return (x, y, z, label)
    
    # Try pattern matching for variations
    patterns = {
        'there': ['there', 'that place', 'that location', 'that spot'],
        'before': ['before', 'previous', 'last place', 'earlier'],
        'back': ['back', 'return', 'go back']
    }
    
    for base_term, variations in patterns.items():
        if term in variations:
            return self.resolve_spatial_reference(base_term)
    
    # Try to infer from conversation history
    if term in ['before', 'previous', 'last', 'earlier']:
        return self._resolve_from_history('previous')
    
    if term in ['back', 'return', 'go back']:
        return self._resolve_from_history('return')
    
    self.get_logger().warn(f'Could not resolve spatial reference: "{term}"')
    return None

def _resolve_from_history(self, intent: str):
    """
    Resolve spatial reference by looking at conversation history
    
    Args:
        intent: Either 'previous' or 'return'
    
    Returns:
        tuple: (x, y, z, label) or None
    """
    if intent == 'previous':
        # Get the location from previous successful navigation
        self.cursor.execute('''
            SELECT location_x, location_y, location_z, location_label
            FROM conversations
            WHERE session_id = ? AND command_type = 'navigate' 
                  AND success = 1 AND location_x IS NOT NULL
            ORDER BY timestamp DESC
            LIMIT 1, 1
        ''', (self.current_session_id,))
        
    elif intent == 'return':
        # Get location from 2 turns ago (before current position)
        self.cursor.execute('''
            SELECT location_x, location_y, location_z, location_label
            FROM conversations
            WHERE session_id = ? AND location_x IS NOT NULL
            ORDER BY timestamp DESC
            LIMIT 1 OFFSET 1
        ''', (self.current_session_id,))
    
    result = self.cursor.fetchone()
    return result if result else None

def extract_and_store_references(self, user_input: str, robot_response: str):
    """
    Extract potential spatial references from conversation and store them
    
    Args:
        user_input: What the user said
        robot_response: What the robot responded
    """
    # Common reference phrases to watch for
    reference_phrases = [
        'go there', 'move there', 'navigate there',
        'that place', 'this location', 'this spot',
        'here', 'right here'
    ]
    
    user_lower = user_input.lower()
    
    # Check if user is establishing a reference
    for phrase in reference_phrases:
        if phrase in user_lower:
            # Extract the reference term
            if 'there' in phrase:
                ref_term = 'there'
            elif 'place' in phrase:
                ref_term = 'that place'
            elif 'here' in phrase:
                ref_term = 'here'
            else:
                continue
            
            # Store current location as this reference
            if self.current_pose:
                pos = self.current_pose.position
                self.add_spatial_reference(
                    ref_term, pos.x, pos.y, pos.z
                )
```

**Action Items:**
- [ ] Implement all spatial reference methods
- [ ] Test reference storage: Call `add_spatial_reference()` with test data
- [ ] Test resolution: Call `resolve_spatial_reference("there")`
- [ ] Test history-based resolution
- [ ] Verify database entries in `spatial_references` table

#### Task 1.5: Testing & Validation (1 hour)

Create comprehensive tests for Day 1 work.

**File:** `conversation_memory_node/test/test_memory_manager.py`

```python
#!/usr/bin/env python3
import unittest
import sqlite3
import tempfile
import os
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import Header

# Mock ROS2 for testing
class MockNode:
    def __init__(self):
        self.logger = self
    
    def info(self, msg):
        print(f"[INFO] {msg}")
    
    def warn(self, msg):
        print(f"[WARN] {msg}")
    
    def debug(self, msg):
        print(f"[DEBUG] {msg}")
    
    def create_subscription(self, *args, **kwargs):
        pass
    
    def create_publisher(self, *args, **kwargs):
        pass
    
    def declare_parameter(self, name, value):
        pass
    
    def get_parameter(self, name):
        class Param:
            def __init__(self, val):
                self.value = val
        return Param(10 if 'length' in name else '/tmp/test_conv.db')
    
    def get_logger(self):
        return self

class TestConversationMemory(unittest.TestCase):
    def setUp(self):
        """Set up test database"""
        self.temp_db = tempfile.NamedTemporaryFile(delete=False, suffix='.db')
        self.db_path = self.temp_db.name
        self.temp_db.close()
        
        # Create a mock memory manager
        # (You'll need to adapt your ConversationMemoryNode to be testable)
        self.conn = sqlite3.connect(self.db_path)
        self.cursor = self.conn.cursor()
    
    def tearDown(self):
        """Clean up test database"""
        self.conn.close()
        os.unlink(self.db_path)
    
    def test_database_schema_creation(self):
        """Test that all tables are created"""
        self.cursor.execute(
            "SELECT name FROM sqlite_master WHERE type='table'"
        )
        tables = [row[0] for row in self.cursor.fetchall()]
        
        self.assertIn('conversations', tables)
        self.assertIn('spatial_references', tables)
        self.assertIn('sessions', tables)
    
    def test_session_creation(self):
        """Test session management"""
        # Create sessions table
        self.cursor.execute('''
            CREATE TABLE sessions (
                session_id TEXT PRIMARY KEY,
                start_time TEXT NOT NULL,
                end_time TEXT,
                total_turns INTEGER DEFAULT 0,
                is_active BOOLEAN DEFAULT 1
            )
        ''')
        
        # Insert test session
        session_id = "test_session_123"
        self.cursor.execute('''
            INSERT INTO sessions (session_id, start_time, total_turns, is_active)
            VALUES (?, datetime('now'), 0, 1)
        ''', (session_id,))
        self.conn.commit()
        
        # Verify
        self.cursor.execute('SELECT session_id FROM sessions WHERE session_id = ?', (session_id,))
        result = self.cursor.fetchone()
        self.assertIsNotNone(result)
        self.assertEqual(result[0], session_id)
    
    def test_conversation_entry_storage(self):
        """Test storing conversation entries"""
        # Create conversations table
        self.cursor.execute('''
            CREATE TABLE conversations (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                session_id TEXT NOT NULL,
                timestamp TEXT NOT NULL,
                turn_number INTEGER NOT NULL,
                user_input TEXT NOT NULL,
                robot_response TEXT,
                location_x REAL,
                location_y REAL,
                location_z REAL
            )
        ''')
        
        # Insert test entry
        self.cursor.execute('''
            INSERT INTO conversations (
                session_id, timestamp, turn_number, user_input, 
                robot_response, location_x, location_y, location_z
            ) VALUES (?, datetime('now'), 1, ?, ?, ?, ?, ?)
        ''', ('test_session', 'Go to kitchen', 'Navigating', 2.5, 1.2, 0.0))
        self.conn.commit()
        
        # Verify
        self.cursor.execute('SELECT user_input FROM conversations WHERE turn_number = 1')
        result = self.cursor.fetchone()
        self.assertEqual(result[0], 'Go to kitchen')
    
    def test_spatial_reference_storage(self):
        """Test storing and retrieving spatial references"""
        # Create table
        self.cursor.execute('''
            CREATE TABLE spatial_references (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                reference_term TEXT NOT NULL,
                location_x REAL NOT NULL,
                location_y REAL NOT NULL,
                location_z REAL,
                session_id TEXT NOT NULL,
                timestamp TEXT NOT NULL,
                frequency INTEGER DEFAULT 1
            )
        ''')
        
        # Store reference
        self.cursor.execute('''
            INSERT INTO spatial_references (
                reference_term, location_x, location_y, location_z,
                session_id, timestamp
            ) VALUES (?, ?, ?, ?, ?, datetime('now'))
        ''', ('there', 3.0, 2.0, 0.0, 'test_session'))
        self.conn.commit()
        
        # Retrieve
        self.cursor.execute('''
            SELECT location_x, location_y FROM spatial_references
            WHERE reference_term = ? AND session_id = ?
        ''', ('there', 'test_session'))
        result = self.cursor.fetchone()
        
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result[0], 3.0)
        self.assertAlmostEqual(result[1], 2.0)

if __name__ == '__main__':
    unittest.main()
```

**Action Items:**
- [ ] Create test file
- [ ] Run tests: `python3 test_memory_manager.py`
- [ ] Fix any failing tests
- [ ] Achieve 100% test pass rate
- [ ] Add tests to your CI/CD if applicable

### End of Day 1 Checklist

- [ ] Enhanced database schema created with 3 tables
- [ ] Session management implemented (start, end, resume)
- [ ] Core memory operations working (add, retrieve, format)
- [ ] Spatial reference system implemented
- [ ] All tests passing
- [ ] Code committed to Git with message: "Day 1: Database & memory core"

**Evening Task (Optional, 30 mins):**
- Document today's progress in `docs/weekly_logs/week2.md`
- Note any challenges or deviations from plan
- Prepare questions for tomorrow's work

---

## Day 2 (Tuesday): LLM Integration & Context Injection

### Morning Session (4 hours)

#### Task 2.1: Gemini API Context-Aware Prompts (2 hours)

Extend your existing Gemini integration to inject conversation context.

**File:** `conversation_memory_node/conversation_memory_node/llm_processor.py`

Create a new file for LLM-related processing:

```python
#!/usr/bin/env python3
import json
import os
from typing import Dict, List, Optional, Tuple

class LLMContextProcessor:
    """
    Handles context injection and prompt engineering for conversation memory
    """
    
    def __init__(self, api_key: str = None):
        """
        Initialize LLM processor
        
        Args:
            api_key: Gemini API key (defaults to environment variable)
        """
        self.api_key = api_key or os.getenv('GEMINI_API_KEY')
        if not self.api_key:
            raise ValueError("GEMINI_API_KEY not found in environment or parameters")
        
        self.api_url = "https://generativelanguage.googleapis.com/v1beta/models/gemini-pro:generateContent"
        
        # Prompt templates
        self.system_prompt = """You are an intelligent robotic assistant with conversational memory.
You can remember previous interactions and understand spatial references.

Your capabilities:
- Navigate to locations using coordinates or semantic labels
- Remember previous conversation turns
- Resolve spatial references like "there", "back", "previous location"
- Explain when you cannot understand or execute commands

Always respond with valid JSON in this format:
{
  "action": "navigate" | "query" | "clarify" | "info",
  "parameters": {
    "goal": {"x": float, "y": float, "z": float} | null,
    "location_label": string | null,
    "clarification_needed": string | null
  },
  "explanation": "Brief explanation of what you understood"
}"""
    
    def build_context_aware_prompt(
        self,
        user_input: str,
        conversation_history: str,
        spatial_references: Dict[str, Tuple[float, float, float]],
        current_location: Tuple[float, float, float] = None
    ) -> str:
        """
        Build a complete prompt with context injection
        
        Args:
            user_input: Current user input
            conversation_history: Formatted history from memory_manager
            spatial_references: Dict of reference_term -> (x, y, z)
            current_location: Current robot position (x, y, z)
        
        Returns:
            Complete prompt string for LLM
        """
        context_parts = [self.system_prompt, ""]
        
        # Add conversation history if available
        if conversation_history and conversation_history != "No previous conversation history.":
            context_parts.append("=== Conversation History ===")
            context_parts.append(conversation_history)
            context_parts.append("")
        
        # Add spatial references if available
        if spatial_references:
            context_parts.append("=== Known Spatial References ===")
            for term, (x, y, z) in spatial_references.items():
                context_parts.append(f'- "{term}": position ({x:.2f}, {y:.2f}, {z:.2f})')
            context_parts.append("")
        
        # Add current location
        if current_location:
            x, y, z = current_location
            context_parts.append(f"=== Current Robot Position ===")
            context_parts.append(f"({x:.2f}, {y:.2f}, {z:.2f})")
            context_parts.append("")
        
        # Add current user input
        context_parts.append("=== Current User Input ===")
        context_parts.append(user_input)
        context_parts.append("")
        context_parts.append("Please parse this command and respond with JSON:")
        
        return "\n".join(context_parts)
    
    async def process_with_context(
        self,
        user_input: str,
        conversation_history: str,
        spatial_references: Dict[str, Tuple[float, float, float]],
        current_location: Tuple[float, float, float] = None
    ) -> Dict:
        """
        Send context-aware prompt to Gemini and parse response
        
        Args:
            user_input: Current user command
            conversation_history: Formatted conversation context
            spatial_references: Known spatial references
            current_location: Current robot position
        
        Returns:
            Parsed JSON response from LLM
        """
        import aiohttp
        
        # Build prompt
        prompt = self.build_context_aware_prompt(
            user_input, conversation_history, 
            spatial_references, current_location
        )
        
        # Prepare API request
        headers = {
            'Content-Type': 'application/json',
        }
        
        payload = {
            "contents": [{
                "parts": [{
                    "text": prompt
                }]
            }],
            "generationConfig": {
                "temperature": 0.2,  # Lower for more consistent JSON output
                "topK": 40,
                "topP": 0.95,
                "maxOutputTokens": 1024,
            }
        }
        
        # Make API call
        async with aiohttp.ClientSession() as session:
            url = f"{self.api_url}?key={self.api_key}"
            async with session.post(url, headers=headers, json=payload) as response:
                if response.status != 200:
                    error_text = await response.text()
                    raise Exception(f"Gemini API error: {response.status} - {error_text}")
                
                result = await response.json()
        
        # Extract and parse response
        try:
            text_response = result['candidates'][0]['content']['parts'][0]['text']
            
            # Try to extract JSON from response (may have markdown formatting)
            json_str = text_response
            if '```json' in text_response:
                json_str = text_response.split('```json')[1].split('```')[0].strip()
            elif '```' in text_response:
                json_str = text_response.split('```')[1].split('```')[0].strip()
            
            parsed = json.loads(json_str)
            return parsed
            
        except (KeyError, IndexError, json.JSONDecodeError) as e:
            raise Exception(f"Failed to parse LLM response: {e}\nResponse: {result}")
    
    def resolve_references_in_response(
        self,
        llm_response: Dict,
        spatial_references: Dict[str, Tuple[float, float, float]]
    ) -> Dict:
        """
        Post-process LLM response to resolve any remaining spatial references
        
        Args:
            llm_response: Parsed JSON from LLM
            spatial_references: Available spatial references
        
        Returns:
            Updated response with resolved coordinates
        """
        params = llm_response.get('parameters', {})
        
        # Check if location_label needs resolution
        label = params.get('location_label', '').lower()
        
        if label in spatial_references and not params.get('goal'):
            x, y, z = spatial_references[label]
            params['goal'] = {'x': x, 'y': y, 'z': z}
            llm_response['parameters'] = params
        
        return llm_response
```

**Action Items:**
- [ ] Create `llm_processor.py` file
- [ ] Install aiohttp: `pip install aiohttp`
- [ ] Test prompt building with sample data
- [ ] Verify API calls work with your Gemini key

#### Task 2.2: Integration with Memory Manager (1.5 hours)

Connect the LLM processor to your memory manager.

**File:** `conversation_memory_node/conversation_memory_node/memory_manager.py`

Add to your `ConversationMemoryNode` class:

```python
# Add to imports at top
from .llm_processor import LLMContextProcessor
import asyncio

class ConversationMemoryNode(Node):
    def __init__(self):
        super().__init__('conversation_memory')
        
        # ... existing initialization ...
        
        # LLM Processor
        try:
            self.llm_processor = LLMContextProcessor()
            self.get_logger().info('LLM processor initialized')
        except ValueError as e:
            self.get_logger().error(f'Failed to initialize LLM: {e}')
            self.llm_processor = None
        
        # Create event loop for async operations
        self.loop = asyncio.get_event_loop()
    
    def transcription_callback(self, msg):
        """Handle incoming voice transcriptions with context awareness"""
        user_input = msg.data
        self.get_logger().info(f'Processing: "{user_input}"')
        
        if not self.llm_processor:
            self.get_logger().error('LLM processor not available')
            return
        
        try:
            # Get conversation context
            conversation_history = self.get_context_for_llm(max_turns=5)
            
            # Get known spatial references
            spatial_refs = self.get_all_spatial_references()
            
            # Get current location
            current_loc = None
            if self.current_pose:
                pos = self.current_pose.position
                current_loc = (pos.x, pos.y, pos.z)
            
            # Process with LLM (run async in sync context)
            llm_response = self.loop.run_until_complete(
                self.llm_processor.process_with_context(
                    user_input,
                    conversation_history,
                    spatial_refs,
                    current_loc
                )
            )
            
            # Resolve any references in response
            llm_response = self.llm_processor.resolve_references_in_response(
                llm_response, spatial_refs
            )
            
            self.get_logger().info(f'LLM response: {json.dumps(llm_response, indent=2)}')
            
            # Handle the parsed command
            self.handle_llm_command(user_input, llm_response)
            
        except Exception as e:
            self.get_logger().error(f'Error processing transcription: {e}')
            import traceback
            traceback.print_exc()
    
    def handle_llm_command(self, user_input: str, llm_response: dict):
        """
        Execute the command parsed by LLM and store in memory
        
        Args:
            user_input: Original user input
            llm_response: Parsed command from LLM
        """
        action = llm_response.get('action', 'unknown')
        params = llm_response.get('parameters', {})
        explanation = llm_response.get('explanation', '')
        
        # Store the interaction
        robot_response = explanation
        
        if action == 'navigate':
            goal = params.get('goal')
            if goal:
                # Send navigation command
                self.send_navigation_goal(goal['x'], goal['y'], goal.get('z', 0.0))
                
                # Store spatial reference if mentioned
                location_label = params.get('location_label')
                if location_label:
                    self.add_spatial_reference(
                        location_label, goal['x'], goal['y'], goal.get('z', 0.0),
                        location_label
                    )
                
                robot_response = f"Navigating to ({goal['x']:.2f}, {goal['y']:.2f})"
            else:
                robot_response = "Cannot navigate: no goal specified"
                action = 'clarify'
        
        elif action == 'clarify':
            clarification = params.get('clarification_needed', 'Could not understand command')
            robot_response = f"Clarification needed: {clarification}"
        
        elif action == 'query':
            # Handle information queries
            robot_response = "Handling query: " + explanation
        
        # Store conversation turn
        self.add_conversation_entry(
            user_input=user_input,
            robot_response=robot_response,
            command_type=action,
            success=(action != 'clarify'),
            location_label=params.get('location_label'),
            context_used={'llm_response': llm_response}
        )
        
        # Extract and store any new spatial references from the conversation
        self.extract_and_store_references(user_input, robot_response)
        
        # Publish response (to be picked up by voice synthesis or dashboard)
        self.publish_robot_response(robot_response)
    
    def get_all_spatial_references(self) -> dict:
        """
        Get all spatial references for current session
        
        Returns:
            Dict mapping reference_term to (x, y, z) tuple
        """
        self.cursor.execute('''
            SELECT reference_term, location_x, location_y, location_z
            FROM spatial_references
            WHERE session_id = ?
            ORDER BY timestamp DESC
        ''', (self.current_session_id,))
        
        refs = {}
        for row in self.cursor.fetchall():
            term, x, y, z = row
            refs[term] = (x, y, z)
        
        return refs
    
    def send_navigation_goal(self, x: float, y: float, z: float = 0.0):
        """
        Send navigation goal to Nav2
        (This will be fully implemented in Week 3-4)
        """
        self.get_logger().info(f'Would navigate to: ({x:.2f}, {y:.2f}, {z:.2f})')
        # TODO: Implement Nav2 action client in Week 3
        pass
    
    def publish_robot_response(self, response: str):
        """
        Publish robot response for voice synthesis or dashboard display
        """
        from std_msgs.msg import String
        
        # TODO: Create publisher in __init__ if not exists
        if not hasattr(self, 'response_pub'):
            self.response_pub = self.create_publisher(
                String, '/robot/response', 10
            )
        
        msg = String()
        msg.data = response
        self.response_pub.publish(msg)
        
        self.get_logger().info(f'Published response: "{response}"')
```

**Action Items:**
- [ ] Add LLM processor integration to memory manager
- [ ] Test with sample transcriptions
- [ ] Verify conversation history is injected
- [ ] Verify spatial references are passed correctly

#### Task 2.3: Testing Context Injection (30 mins)

Test the complete context-aware pipeline.

**Test Script:** `conversation_memory_node/test/test_context_integration.py`

```python
#!/usr/bin/env python3
"""
Test script for context-aware conversation
Run this after starting the memory_manager node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class ConversationTester(Node):
    def __init__(self):
        super().__init__('conversation_tester')
        
        self.transcription_pub = self.create_publisher(
            String, '/voice/transcription', 10
        )
        
        self.response_sub = self.create_subscription(
            String, '/robot/response', self.response_callback, 10
        )
        
        self.responses_received = []
        
    def response_callback(self, msg):
        self.responses_received.append(msg.data)
        self.get_logger().info(f'Robot responded: "{msg.data}"')
    
    def send_command(self, text, wait_time=3.0):
        """Send a command and wait for response"""
        self.get_logger().info(f'Sending: "{text}"')
        
        msg = String()
        msg.data = text
        self.transcription_pub.publish(msg)
        
        # Wait for processing
        time.sleep(wait_time)
    
    def run_test_conversation(self):
        """Run a test conversation with multiple turns"""
        
        print("\n" + "="*60)
        print("STARTING CONTEXT-AWARE CONVERSATION TEST")
        print("="*60 + "\n")
        
        # Give system time to initialize
        time.sleep(2)
        
        # Turn 1: Simple navigation
        self.send_command("Go to the kitchen at position 5, 3")
        
        # Turn 2: Another navigation to establish context
        self.send_command("Now move to the bedroom at position 2, 8")
        
        # Turn 3: Use "there" reference
        self.send_command("Go back there")  # Should resolve to (5, 3)
        
        # Turn 4: Use "previous" reference
        self.send_command("Return to the previous location")  # Should go to (2, 8)
        
        # Turn 5: Use contextual reference
        self.send_command("Take me to where we were before that")  # Should go to (5, 3)
        
        print("\n" + "="*60)
        print(f"TEST COMPLETE - Received {len(self.responses_received)} responses")
        print("="*60 + "\n")
        
        for i, response in enumerate(self.responses_received, 1):
            print(f"Response {i}: {response}")
        
        return len(self.responses_received) >= 5

def main():
    rclpy.init()
    tester = ConversationTester()
    
    try:
        success = tester.run_test_conversation()
        
        if success:
            print("\n✓ Test PASSED - All responses received")
        else:
            print("\n✗ Test FAILED - Not all responses received")
        
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Action Items:**
- [ ] Create test script
- [ ] Terminal 1: Launch memory manager: `ros2 run conversation_memory_node memory_manager`
- [ ] Terminal 2: Run test: `python3 test_context_integration.py`
- [ ] Verify all 5 turns are processed correctly
- [ ] Check database for stored conversations: `sqlite3 ~/.ros/conversation_history.db "SELECT * FROM conversations;"`

### Afternoon Session (3 hours)

#### Task 2.4: Prompt Engineering Optimization (2 hours)

Refine prompts for better context understanding and response quality.

**File:** `conversation_memory_node/conversation_memory_node/prompt_templates.py`

Create specialized prompt templates:

```python
#!/usr/bin/env python3
"""
Prompt templates for different conversation scenarios
"""

class PromptTemplates:
    """Collection of optimized prompt templates"""
    
    SYSTEM_BASE = """You are an intelligent mobile robot assistant with memory of past conversations.

CAPABILITIES:
- Navigate to specific coordinates or labeled locations
- Remember previous locations and conversation history  
- Understand spatial references like "there", "back", "previous location"
- Ask for clarification when commands are ambiguous

RESPONSE FORMAT:
Always respond with valid JSON matching this schema:
{
  "action": "navigate" | "query" | "clarify" | "confirm",
  "parameters": {
    "goal": {"x": float, "y": float} | null,
    "location_label": string | null,
    "clarification_needed": string | null,
    "query_response": string | null
  },
  "explanation": "Brief human-readable explanation",
  "confidence": float (0.0 to 1.0)
}

RULES:
1. If user mentions "there", "that place", "back", or "previous" - check conversation history for coordinates
2. If coordinates are unclear, set action="clarify" and explain what's missing
3. If asked about past actions, set action="query" and reference conversation history
4. Always include an explanation field for transparency
5. Set confidence based on how certain you are about understanding the command
"""
    
    @staticmethod
    def build_navigation_prompt(
        user_input: str,
        history: str,
        references: dict,
        current_pos: tuple
    ) -> str:
        """Build prompt for navigation commands"""
        
        parts = [PromptTemplates.SYSTEM_BASE, ""]
        
        # Add context sections
        if history and "No previous" not in history:
            parts.append("=== CONVERSATION HISTORY ===")
            parts.append(history)
            parts.append("")
        
        if references:
            parts.append("=== KNOWN LOCATIONS ===")
            for term, coords in references.items():
                parts.append(f'"{term}": ({coords[0]:.2f}, {coords[1]:.2f})')
            parts.append("")
        
        if current_pos:
            parts.append("=== CURRENT POSITION ===")
            parts.append(f"Robot is at ({current_pos[0]:.2f}, {current_pos[1]:.2f})")
            parts.append("")
        
        parts.append("=== USER COMMAND ===")
        parts.append(f'"{user_input}"')
        parts.append("")
        parts.append("Parse this command and respond with JSON:")
        
        return "\n".join(parts)
    
    @staticmethod
    def build_clarification_prompt(
        user_input: str,
        previous_attempt: dict,
        history: str
    ) -> str:
        """Build prompt when clarification is needed"""
        
        parts = [
            "The user's previous command was unclear:",
            f'User said: "{user_input}"',
            "",
            "Previous parsing attempt:",
            str(previous_attempt),
            "",
        ]
        
        if history:
            parts.append("Conversation context:")
            parts.append(history)
            parts.append("")
        
        parts.append("Generate a clarification question to help understand the command.")
        parts.append("Respond with JSON including 'clarification_needed' in parameters.")
        
        return "\n".join(parts)
    
    @staticmethod
    def build_reference_resolution_prompt(
        reference_term: str,
        history: str,
        known_refs: dict
    ) -> str:
        """Build prompt specifically for resolving spatial references"""
        
        parts = [
            f'User mentioned: "{reference_term}"',
            "",
            "This appears to be a spatial reference. Resolve it to coordinates.",
            "",
        ]
        
        if known_refs:
            parts.append("Known reference mappings:")
            for term, coords in known_refs.items():
                parts.append(f'  "{term}" = ({coords[0]:.2f}, {coords[1]:.2f})')
            parts.append("")
        
        if history:
            parts.append("Recent locations from conversation:")
            parts.append(history)
            parts.append("")
        
        parts.append("Determine what location the user is referring to.")
        parts.append("Respond with JSON containing the resolved coordinates in 'goal'.")
        
        return "\n".join(parts)
    
    @staticmethod
    def extract_key_terms(user_input: str) -> dict:
        """Extract important terms from user input for better processing"""
        
        terms = {
            'spatial_refs': [],
            'location_labels': [],
            'action_verbs': [],
            'has_coordinates': False
        }
        
        user_lower = user_input.lower()
        
        # Spatial references
        spatial_keywords = [
            'there', 'here', 'back', 'previous', 'before', 
            'that place', 'this location', 'where we were',
            'last place', 'earlier'
        ]
        for keyword in spatial_keywords:
            if keyword in user_lower:
                terms['spatial_refs'].append(keyword)
        
        # Action verbs
        action_keywords = [
            'go', 'move', 'navigate', 'travel', 'drive',
            'return', 'come back', 'head to'
        ]
        for keyword in action_keywords:
            if keyword in user_lower:
                terms['action_verbs'].append(keyword)
        
        # Check for explicit coordinates
        import re
        coord_pattern = r'\d+\.?\d*\s*,\s*\d+\.?\d*'
        if re.search(coord_pattern, user_input):
            terms['has_coordinates'] = True
        
        # Common location labels
        location_keywords = [
            'kitchen', 'bedroom', 'living room', 'bathroom',
            'office', 'garage', 'entrance', 'hallway'
        ]
        for keyword in location_keywords:
            if keyword in user_lower:
                terms['location_labels'].append(keyword)
        
        return terms
```

**Update LLM Processor to use templates:**

```python
# In llm_processor.py, update build_context_aware_prompt method:

from .prompt_templates import PromptTemplates

def build_context_aware_prompt(
    self,
    user_input: str,
    conversation_history: str,
    spatial_references: Dict[str, Tuple[float, float, float]],
    current_location: Tuple[float, float, float] = None
) -> str:
    """Build context-aware prompt using optimized templates"""
    
    # Extract key terms to determine best template
    key_terms = PromptTemplates.extract_key_terms(user_input)
    
    # If heavy spatial references, use specialized prompt
    if key_terms['spatial_refs'] and not key_terms['has_coordinates']:
        prompt = PromptTemplates.build_reference_resolution_prompt(
            key_terms['spatial_refs'][0],
            conversation_history,
            spatial_references
        )
    else:
        # Use standard navigation prompt
        prompt = PromptTemplates.build_navigation_prompt(
            user_input,
            conversation_history,
            spatial_references,
            current_location
        )
    
    return prompt
```

**Action Items:**
- [ ] Create `prompt_templates.py`
- [ ] Update LLM processor to use templates
- [ ] Test with challenging commands:
  - "Go there"
  - "Take me back"
  - "Return to where we started"
- [ ] Measure confidence scores in responses
- [ ] Tune templates if confidence < 0.7

#### Task 2.4: Response Quality Validation (1 hour)

Create validation logic to ensure LLM responses are high quality.

**File:** `conversation_memory_node/conversation_memory_node/response_validator.py`

```python
#!/usr/bin/env python3
"""
Validation logic for LLM responses
"""

class ResponseValidator:
    """Validates and sanitizes LLM responses"""
    
    @staticmethod
    def validate_response(response: dict) -> tuple[bool, str]:
        """
        Validate LLM response structure and content
        
        Args:
            response: Parsed JSON from LLM
        
        Returns:
            (is_valid, error_message)
        """
        # Check required fields
        if 'action' not in response:
            return False, "Missing 'action' field"
        
        if 'parameters' not in response:
            return False, "Missing 'parameters' field"
        
        if 'explanation' not in response:
            return False, "Missing 'explanation' field"
        
        # Validate action type
        valid_actions = ['navigate', 'query', 'clarify', 'confirm', 'info']
        if response['action'] not in valid_actions:
            return False, f"Invalid action: {response['action']}"
        
        # Validate navigate action
        if response['action'] == 'navigate':
            params = response['parameters']
            goal = params.get('goal')
            
            # Must have either goal coordinates or location_label
            if not goal and not params.get('location_label'):
                return False, "Navigate action requires 'goal' or 'location_label'"
            
            # If goal exists, validate structure
            if goal:
                if not isinstance(goal, dict):
                    return False, "'goal' must be a dictionary"
                
                if 'x' not in goal or 'y' not in goal:
                    return False, "'goal' must contain 'x' and 'y' coordinates"
                
                try:
                    float(goal['x'])
                    float(goal['y'])
                except (ValueError, TypeError):
                    return False, "goal coordinates must be numeric"
        
        # Validate clarify action
        if response['action'] == 'clarify':
            params = response['parameters']
            if not params.get('clarification_needed'):
                return False, "Clarify action requires 'clarification_needed'"
        
        # Check confidence if present
        if 'confidence' in response:
            conf = response['confidence']
            if not isinstance(conf, (int, float)) or not (0 <= conf <= 1):
                return False, f"Invalid confidence value: {conf}"
        
        return True, ""
    
    @staticmethod
    def sanitize_coordinates(goal: dict, bounds: dict = None) -> dict:
        """
        Sanitize and bound-check coordinates
        
        Args:
            goal: Dictionary with x, y, z coordinates
            bounds: Optional dict with min/max values
        
        Returns:
            Sanitized goal dictionary
        """
        if bounds is None:
            bounds = {
                'x_min': -10.0, 'x_max': 10.0,
                'y_min': -10.0, 'y_max': 10.0,
                'z_min': 0.0, 'z_max': 2.0
            }
        
        sanitized = {
            'x': max(bounds['x_min'], min(bounds['x_max'], float(goal.get('x', 0.0)))),
            'y': max(bounds['y_min'], min(bounds['y_max'], float(goal.get('y', 0.0)))),
            'z': max(bounds['z_min'], min(bounds['z_max'], float(goal.get('z', 0.0))))
        }
        
        return sanitized
    
    @staticmethod
    def should_request_clarification(response: dict, user_input: str) -> tuple[bool, str]:
        """
        Determine if clarification should be requested
        
        Args:
            response: LLM response
            user_input: Original user input
        
        Returns:
            (should_clarify, reason)
        """
        # Low confidence threshold
        if response.get('confidence', 1.0) < 0.5:
            return True, "Low confidence in command understanding"
        
        # Action is explicitly clarify
        if response['action'] == 'clarify':
            return True, response['parameters'].get('clarification_needed', 'Unclear command')
        
        # Navigate without goal
        if response['action'] == 'navigate':
            params = response['parameters']
            if not params.get('goal') and not params.get('location_label'):
                return True, "Navigation destination unclear"
        
        # Very short input might be unclear
        if len(user_input.strip()) < 5:
            return True, "Command too brief to understand"
        
        return False, ""
```

**Integrate validator into memory manager:**

```python
# In memory_manager.py

from .response_validator import ResponseValidator

def handle_llm_command(self, user_input: str, llm_response: dict):
    """Execute LLM command with validation"""
    
    # Validate response
    is_valid, error_msg = ResponseValidator.validate_response(llm_response)
    
    if not is_valid:
        self.get_logger().error(f'Invalid LLM response: {error_msg}')
        self.add_conversation_entry(
            user_input=user_input,
            robot_response=f"Error: {error_msg}",
            command_type='error',
            success=False
        )
        return
    
    # Check if clarification needed
    should_clarify, reason = ResponseValidator.should_request_clarification(
        llm_response, user_input
    )
    
    if should_clarify:
        clarification_msg = f"I need clarification: {reason}"
        self.get_logger().info(f'Requesting clarification: {reason}')
        self.publish_robot_response(clarification_msg)
        self.add_conversation_entry(
            user_input=user_input,
            robot_response=clarification_msg,
            command_type='clarify',
            success=False
        )
        return
    
    # ... rest of existing handle_llm_command logic ...
    
    # Sanitize coordinates if navigate action
    if action == 'navigate' and params.get('goal'):
        params['goal'] = ResponseValidator.sanitize_coordinates(params['goal'])
```

**Action Items:**
- [ ] Create `response_validator.py`
- [ ] Integrate validator into command handling
- [ ] Test with invalid responses (manually craft bad JSON)
- [ ] Test with ambiguous commands
- [ ] Verify clarification requests work

### End of Day 2 Checklist

- [ ] LLM context injection working
- [ ] Prompt templates created and optimized
- [ ] Response validation implemented
- [ ] Integration tests passing
- [ ] Code committed: "Day 2: LLM integration with context"

---

## Day 3 (Wednesday): Voice Pipeline Integration

### Morning Session (4 hours)

#### Task 3.1: Connect to Existing Voice Control (2 hours)

Integrate memory system with your existing Whisper + Gemini voice pipeline.

**Review your existing voice control architecture:**

```
Voice Input → Whisper API → Transcription → Gemini API → Command JSON → Nav2
```

**New architecture with memory:**

```
Voice Input → Whisper API → Transcription → 
  ↓
Memory Manager (retrieve context) →
  ↓
Gemini API (with context) → Command JSON →
  ↓
Memory Manager (store interaction) → Nav2
```

**File:** `conversation_memory_node/launch/conversational_system.launch.py`

Create launch file that starts everything:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Get environment variables
    gemini_key = os.getenv('GEMINI_API_KEY', '')
    whisper_key = os.getenv('WHISPER_API_KEY', '')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'history_length',
            default_value='10',
            description='Number of conversation turns to maintain'
        ),
        
        # Conversation Memory Node
        Node(
            package='conversation_memory_node',
            executable='memory_manager',
            name='conversation_memory',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'history_length': LaunchConfiguration('history_length'),
                'db_path': '~/.ros/conversation_history.db'
            }],
            remappings=[
                ('/voice/transcription', '/voice/transcription'),
                ('/robot/pose', '/amcl_pose'),  # Or your localization topic
            ]
        ),
        
        # Your existing voice control node (from last semester)
        Node(
            package='voice_control',  # Your package name
            executable='voice_controller',  # Your executable
            name='voice_controller',
            output='screen',
            parameters=[{
                'gemini_api_key': gemini_key,
                'whisper_api_key': whisper_key,
            }]
        ),
        
        # rosbridge for web dashboard
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'port': 9090,
            }]
        ),
    ])
```

**Action Items:**
- [ ] Create launch file
- [ ] Test launch: `ros2 launch conversation_memory_node conversational_system.launch.py`
- [ ] Verify all nodes start
- [ ] Check topic connections: `ros2 topic list` and `ros2 topic info /voice/transcription`

#### Task 3.2: Bidirectional Communication Setup (1.5 hours)

Ensure memory manager can both receive transcriptions and send enriched commands back to voice system.

**File:** `conversation_memory_node/conversation_memory_node/memory_manager.py`

Add publishers and services for voice system integration:

```python
class ConversationMemoryNode(Node):
    def __init__(self):
        super().__init__('conversation_memory')
        
        # ... existing initialization ...
        
        # Publishers for voice system
        self.command_pub = self.create_publisher(
            String,
            '/conversation/processed_command',
            10
        )
        
        self.context_pub = self.create_publisher(
            String,
            '/conversation/context',
            10
        )
        
        # Service for on-demand context retrieval
        from std_srvs.srv import Trigger
        self.context_service = self.create_service(
            Trigger,
            '/conversation/get_context',
            self.get_context_service_callback
        )
        
        self.get_logger().info('Voice integration publishers and services ready')
    
    def publish_processed_command(self, llm_response: dict):
        """
        Publish processed command for other nodes to act on
        
        Args:
            llm_response: Parsed command from LLM
        """
        import json
        
        msg = String()
        msg.data = json.dumps(llm_response)
        self.command_pub.publish(msg)
        
        self.get_logger().debug(f'Published processed command: {llm_response["action"]}')
    
    def publish_context(self):
        """Publish current conversation context"""
        context = self.get_context_for_llm(max_turns=5)
        
        msg = String()
        msg.data = context
        self.context_pub.publish(msg)
        
        self.get_logger().debug('Published conversation context')
    
    def get_context_service_callback(self, request, response):
        """Service callback to provide context on demand"""
        context = self.get_context_for_llm(max_turns=10)
        response.success = True
        response.message = context
        
        self.get_logger().info('Context requested via service')
        return response
    
    def handle_llm_command(self, user_input: str, llm_response: dict):
        """Execute LLM command with voice system integration"""
        
        # ... existing validation code ...
        
        # Publish processed command for other systems
        self.publish_processed_command(llm_response)
        
        # ... rest of existing handle_llm_command logic ...
        
        # Publish updated context after storing new turn
        self.publish_context()
```

**Action Items:**
- [ ] Add publishers and service to memory manager
- [ ] Test publishing: `ros2 topic echo /conversation/processed_command`
- [ ] Test service: `ros2 service call /conversation/get_context std_srvs/srv/Trigger`
- [ ] Verify JSON format is correct

#### Task 3.3: Voice Response Integration (30 mins)

Ensure robot responses are properly formatted for text-to-speech.

**File:** `conversation_memory_node/conversation_memory_node/tts_formatter.py`

```python
#!/usr/bin/env python3
"""
Format robot responses for text-to-speech
"""

class TTSFormatter:
    """Formats responses to be more natural for voice output"""
    
    @staticmethod
    def format_for_speech(response: str, response_type: str = 'info') -> str:
        """
        Format text for more natural speech output
        
        Args:
            response: Raw response text
            response_type: Type of response (info, error, confirmation, clarify)
        
        Returns:
            Formatted speech-friendly text
        """
        # Remove coordinate details for speech (too technical)
        import re
        
        # Replace coordinate patterns with more natural language
        coord_pattern = r'\((-?\d+\.?\d*),\s*(-?\d+\.?\d*)\)'
        response = re.sub(coord_pattern, r'coordinates \1 by \2', response)
        
        # Add appropriate response prefixes
        prefixes = {
            'confirmation': ['Okay', 'Understood', 'Got it', 'Alright'],
            'error': ['Sorry', 'I apologize', 'Unfortunately'],
            'clarify': ['Let me clarify', 'Just to be clear', 'To make sure'],
            'info': ['', 'Sure', 'Here you go']
        }
        
        # Randomly select prefix for variety
        import random
        if response_type in prefixes:
            prefix_options = prefixes[response_type]
            if prefix_options[0]:  # Not empty string
                prefix = random.choice(prefix_options)
                response = f"{prefix}. {response}"
        
        # Ensure proper punctuation
        if not response.endswith(('.', '!', '?')):
            response += '.'
        
        return response
    
    @staticmethod
    def format_navigation_response(goal: dict, location_label: str = None) -> str:
        """Format navigation confirmation"""
        if location_label:
            return f"Navigating to the {location_label}"
        else:
            # Simplify coordinates for speech
            x, y = goal['x'], goal['y']
            if abs(x) < 0.5 and abs(y) < 0.5:
                return "Staying near my current position"
            else:
                direction = TTSFormatter._coords_to_direction(x, y)
                return f"Moving {direction}"
    
    @staticmethod
    def _coords_to_direction(x: float, y: float) -> str:
        """Convert coordinates to natural direction description"""
        import math
        
        angle = math.atan2(y, x)
        angle_deg = math.degrees(angle)
        
        # Normalize to 0-360
        if angle_deg < 0:
            angle_deg += 360
        
        # Convert to cardinal directions
        if angle_deg < 22.5 or angle_deg >= 337.5:
            return "to the right"
        elif angle_deg < 67.5:
            return "forward and to the right"
        elif angle_deg < 112.5:
            return "forward"
        elif angle_deg < 157.5:
            return "forward and to the left"
        elif angle_deg < 202.5:
            return "to the left"
        elif angle_deg < 247.5:
            return "backward and to the left"
        elif angle_deg < 292.5:
            return "backward"
        else:
            return "backward and to the right"
    
    @staticmethod
    def format_clarification_question(clarification: str) -> str:
        """Format clarification question naturally"""
        question_starters = [
            "Could you clarify",
            "I'm not sure I understand",
            "Can you help me understand",
            "Just to confirm"
        ]
        
        import random
        starter = random.choice(question_starters)
        
        return f"{starter}: {clarification}?"
```

**Update memory manager to use TTS formatting:**

```python
from .tts_formatter import TTSFormatter

def publish_robot_response(self, response: str, response_type: str = 'info'):
    """Publish formatted robot response"""
    
    # Format for speech
    speech_response = TTSFormatter.format_for_speech(response, response_type)
    
    # Publish
    msg = String()
    msg.data = speech_response
    self.response_pub.publish(msg)
    
    self.get_logger().info(f'Response: "{speech_response}"')
```

**Action Items:**
- [ ] Create TTS formatter
- [ ] Update response publishing to use formatter
- [ ] Test with various response types
- [ ] Listen to voice output if TTS is connected

### Afternoon Session (3 hours)

#### Task 3.4: End-to-End Voice Conversation Test (2 hours)

Test complete voice conversation flow with memory.

**Test Scenario Script:**

```markdown
# End-to-End Conversation Test Scenario

## Setup
1. Launch full system with digital twin
2. Verify microphone is connected
3. Clear conversation database for fresh start

## Test Conversation (5+ turns)

### Turn 1: Initial Navigation
**User:** "Move to the kitchen at coordinates 5, 3"
**Expected Response:** "Navigating to the kitchen"
**Verify:** 
- [ ] Database entry created
- [ ] Spatial reference "kitchen" stored at (5, 3)
- [ ] Robot starts navigation in Gazebo

### Turn 2: Another Navigation
**User:** "Go to the living room at 8, 7"
**Expected Response:** "Navigating to the living room"
**Verify:**
- [ ] New database entry
- [ ] Spatial reference "living room" stored at (8, 7)
- [ ] Navigation goal updated

### Turn 3: Reference Previous Location
**User:** "Go back to the kitchen"
**Expected Response:** "Returning to the kitchen"
**Verify:**
- [ ] LLM resolves "kitchen" to (5, 3) from memory
- [ ] Context history included kitchen mention
- [ ] Navigation goal set correctly

### Turn 4: Use Vague Reference
**User:** "Take me back there"
**Expected Response:** "Navigating to the living room" (most recent location)
**Verify:**
- [ ] "there" resolved to (8, 7)
- [ ] Context history showed living room as last location
- [ ] Spatial reference properly retrieved

### Turn 5: Complex Contextual Reference
**User:** "Go to where we were before that"
**Expected Response:** "Returning to the kitchen"
**Verify:**
- [ ] LLM used conversation history to find location before living room
- [ ] Resolved to (5, 3)
- [ ] Multiple turns of context were used

### Turn 6 (Bonus): Clarification Test
**User:** "Go over there" (without establishing context)
**Expected Response:** "I need clarification: which location are you referring to?"
**Verify:**
- [ ] Low confidence detected
- [ ] Clarification requested
- [ ] No navigation attempted

## Success Criteria
- [ ] All 5 main turns processed correctly
- [ ] Average response time < 3 seconds
- [ ] Spatial references resolved with 100% accuracy
- [ ] Database contains all conversation entries
- [ ] No crashes or exceptions
```

**Execute Test:**

```bash
# Terminal 1: Launch system
ros2 launch conversation_memory_node conversational_system.launch.py

# Terminal 2: Monitor memory node logs
ros2 node info /conversation_memory

# Terminal 3: Monitor database
watch -n 2 'sqlite3 ~/.ros/conversation_history.db "SELECT turn_number, user_input, robot_response FROM conversations ORDER BY timestamp DESC LIMIT 5;"'

# Terminal 4: Monitor topics
ros2 topic echo /conversation/processed_command

# Use your voice or publish test messages
ros2 topic pub /voice/transcription std_msgs/String "data: 'Move to the kitchen at 5, 3'" --once
```

**Action Items:**
- [ ] Execute complete test scenario
- [ ] Document response times for each turn
- [ ] Capture screenshots of database state
- [ ] Record any errors or unexpected behavior
- [ ] Calculate success rate

#### Task 3.5: Performance Optimization (1 hour)

Optimize for response time and memory usage.

**Performance Checklist:**

```python
# Add to memory_manager.py

class ConversationMemoryNode(Node):
    
    def __init__(self):
        # ... existing init ...
        
        # Performance tracking
        self.performance_metrics = {
            'total_turns': 0,
            'avg_processing_time': 0.0,
            'db_query_times': [],
            'llm_call_times': []
        }
        
        # Create timer for periodic optimization
        self.optimization_timer = self.create_timer(
            300.0,  # Every 5 minutes
            self.optimize_database
        )
    
    def transcription_callback(self, msg):
        """Handle transcription with performance tracking"""
        import time
        start_time = time.time()
        
        try:
            # ... existing processing ...
            pass
        finally:
            # Track processing time
            processing_time = time.time() - start_time
            self.update_performance_metrics(processing_time)
            
            if processing_time > 3.0:
                self.get_logger().warn(
                    f'Slow processing detected: {processing_time:.2f}s'
                )
    
    def update_performance_metrics(self, processing_time: float):
        """Update performance tracking"""
        self.performance_metrics['total_turns'] += 1
        
        # Update rolling average
        n = self.performance_metrics['total_turns']
        old_avg = self.performance_metrics['avg_processing_time']
        new_avg = ((old_avg * (n - 1)) + processing_time) / n
        self.performance_metrics['avg_processing_time'] = new_avg
        
        # Log every 10 turns
        if n % 10 == 0:
            self.get_logger().info(
                f'Performance: {n} turns, avg {new_avg:.2f}s/turn'
            )
    
    def optimize_database(self):
        """Periodic database optimization"""
        self.get_logger().info('Running database optimization...')
        
        try:
            # Vacuum database to reclaim space
            self.cursor.execute('VACUUM')
            
            # Analyze for query optimization
            self.cursor.execute('ANALYZE')
            
            # Archive old sessions
            self.archive_old_sessions(days=7)
            
            self.conn.commit()
            self.get_logger().info('Database optimized')
            
        except Exception as e:
            self.get_logger().error(f'Optimization failed: {e}')
    
    def archive_old_sessions(self, days: int = 7):
        """Archive sessions older than specified days"""
        from datetime import datetime, timedelta
        
        cutoff_date = (datetime.now() - timedelta(days=days)).isoformat()
        
        # Count sessions to archive
        self.cursor.execute('''
            SELECT COUNT(*) FROM sessions 
            WHERE end_time < ? AND is_active = 0
        ''', (cutoff_date,))
        
        count = self.cursor.fetchone()[0]
        
        if count > 0:
            # Move to archive table (create if not exists)
            self.cursor.execute('''
                CREATE TABLE IF NOT EXISTS conversations_archive
                AS SELECT * FROM conversations WHERE 1=0
            ''')
            
            self.cursor.execute('''
                INSERT INTO conversations_archive
                SELECT c.* FROM conversations c
                JOIN sessions s ON c.session_id = s.session_id
                WHERE s.end_time < ? AND s.is_active = 0
            ''', (cutoff_date,))
            
            self.cursor.execute('''
                DELETE FROM conversations WHERE session_id IN (
                    SELECT session_id FROM sessions 
                    WHERE end_time < ? AND is_active = 0
                )
            ''', (cutoff_date,))
            
            self.get_logger().info(f'Archived {count} old sessions')
```

**Optimization Strategies:**

1. **Database Indexing:**
```python
# Ensure indexes exist (already added in Day 1, but verify)
self.cursor.execute('''
    CREATE INDEX IF NOT EXISTS idx_session_timestamp 
    ON conversations(session_id, timestamp DESC)
''')

self.cursor.execute('''
    CREATE INDEX IF NOT EXISTS idx_turn_lookup
    ON conversations(session_id, turn_number)
''')
```

2. **LLM Caching:**
```python
# In llm_processor.py

class LLMContextProcessor:
    def __init__(self, api_key: str = None):
        # ... existing init ...
        
        # Response cache
        self.response_cache = {}
        self.cache_max_size = 100
    
    async def process_with_context(self, user_input: str, *args, **kwargs):
        """Process with caching"""
        
        # Create cache key from input
        cache_key = user_input.lower().strip()
        
        # Check cache
        if cache_key in self.response_cache:
            self.get_logger().debug(f'Cache hit for: {cache_key[:30]}...')
            return self.response_cache[cache_key]
        
        # Call API
        response = await self._call_gemini_api(user_input, *args, **kwargs)
        
        # Update cache
        if len(self.response_cache) >= self.cache_max_size:
            # Remove oldest entry (FIFO)
            self.response_cache.pop(next(iter(self.response_cache)))
        
        self.response_cache[cache_key] = response
        
        return response
```

3. **Connection Pooling:**
```python
# Use connection pooling for SQLite
import sqlite3
from contextlib import contextmanager

@contextmanager
def get_db_connection(db_path):
    """Context manager for database connections"""
    conn = sqlite3.connect(db_path, check_same_thread=False)
    conn.row_factory = sqlite3.Row  # Enable column access by name
    try:
        yield conn
    finally:
        conn.close()
```

**Action Items:**
- [ ] Add performance tracking
- [ ] Verify database indexes exist
- [ ] Implement LLM response caching
- [ ] Test optimization functions
- [ ] Measure performance improvement (target: <2s average)

### End of Day 3 Checklist

- [ ] Voice pipeline fully integrated
- [ ] Bidirectional communication working
- [ ] End-to-end conversation test passing (5+ turns)
- [ ] Performance optimizations implemented
- [ ] Average response time < 2.5 seconds
- [ ] Code committed: "Day 3: Voice integration & optimization"

---

## Day 4 (Thursday): Dashboard Integration & Visualization

### Morning Session (4 hours)

#### Task 4.1: WebSocket Integration (2 hours)

Connect conversation memory to React dashboard via rosbridge.

**File:** `web_dashboard/src/hooks/useConversationMemory.ts`

```typescript
import { useState, useEffect, useCallback, useRef } from 'react';
import ROSLIB from 'roslib';

interface ConversationTurn {
  timestamp: string;
  turnNumber: number;
  userInput: string;
  robotResponse: string;
  location: {
    x: number;
    y: number;
    z: number;
    label?: string;
  } | null;
  commandType: string;
  success: boolean;
}

interface ConversationContext {
  sessionId: string;
  turnCount: number;
  history: ConversationTurn[];
  lastUpdated: string;
}

export const useConversationMemory = (rosUrl: string = 'ws://localhost:9090') => {
  const [context, setContext] = useState<ConversationContext>({
    sessionId: '',
    turnCount: 0,
    history: [],
    lastUpdated: new Date().toISOString()
  });
  
  const [isConnected, setIsConnected] = useState(false);
  const [error, setError] = useState<string | null>(null);
  
  const rosRef = useRef<ROSLIB.Ros | null>(null);
  const contextSubRef = useRef<ROSLIB.Topic | null>(null);
  const commandSubRef = useRef<ROSLIB.Topic | null>(null);
  
  // Initialize ROS connection
  useEffect(() => {
    const ros = new ROSLIB.Ros({
      url: rosUrl
    });
    
    ros.on('connection', () => {
      console.log('Connected to ROS');
      setIsConnected(true);
      setError(null);
    });
    
    ros.on('error', (error) => {
      console.error('ROS connection error:', error);
      setError('Failed to connect to ROS');
      setIsConnected(false);
    });
    
    ros.on('close', () => {
      console.log('Disconnected from ROS');
      setIsConnected(false);
    });
    
    rosRef.current = ros;
    
    return () => {
      ros.close();
    };
  }, [rosUrl]);
  
  // Subscribe to conversation context updates
  useEffect(() => {
    if (!rosRef.current || !isConnected) return;
    
    const contextTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/conversation/context',
      messageType: 'std_msgs/String'
    });
    
    contextTopic.subscribe((message: any) => {
      try {
        // Parse context string into structured data
        const contextText = message.data;
        parseConversationContext(contextText);
      } catch (err) {
        console.error('Failed to parse context:', err);
      }
    });
    
    contextSubRef.current = contextTopic;
    
    return () => {
      contextTopic.unsubscribe();
    };
  }, [isConnected]);
  
  // Subscribe to processed commands
  useEffect(() => {
    if (!rosRef.current || !isConnected) return;
    
    const commandTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/conversation/processed_command',
      messageType: 'std_msgs/String'
    });
    
    commandTopic.subscribe((message: any) => {
      try {
        const command = JSON.parse(message.data);
        handleNewCommand(command);
      } catch (err) {
        console.error('Failed to parse command:', err);
      }
    });
    
    commandSubRef.current = commandTopic;
    
    return () => {
      commandTopic.unsubscribe();
    };
  }, [isConnected]);
  
  const parseConversationContext = (contextText: string) => {
    // Parse the formatted context string
    const lines = contextText.split('\n');
    const turns: ConversationTurn[] = [];
    
    let currentTurn: Partial<ConversationTurn> | null = null;
    
    for (const line of lines) {
      if (line.startsWith('Turn ')) {
        if (currentTurn && currentTurn.userInput) {
          turns.push(currentTurn as ConversationTurn);
        }
        
        const turnNum = parseInt(line.match(/\d+/)?.[0] || '0');
        currentTurn = {
          turnNumber: turnNum,
          timestamp: new Date().toISOString(),
          location: null,
          commandType: 'unknown',
          success: true,
          userInput: '',
          robotResponse: ''
        };
      } else if (line.trim().startsWith('User:') && currentTurn) {
        currentTurn.userInput = line.split('User:')[1].trim();
      } else if (line.trim().startsWith('Robot:') && currentTurn) {
        const robotLine = line.split('Robot:')[1].trim();
        const atMatch = robotLine.match(/at \(([^)]+)\)/);
        
        if (atMatch) {
          const [x, y] = atMatch[1].split(',').map(parseFloat);
          currentTurn.location = { x, y, z: 0 };
          currentTurn.robotResponse = robotLine.replace(atMatch[0], '').trim();
        } else {
          currentTurn.robotResponse = robotLine;
        }
      }
    }
    
    if (currentTurn && currentTurn.userInput) {
      turns.push(currentTurn as ConversationTurn);
    }
    
    setContext(prev => ({
      ...prev,
      history: turns,
      turnCount: turns.length,
      lastUpdated: new Date().toISOString()
    }));
  };
  
  const handleNewCommand = (command: any) => {
    // Update context with new command
    const newTurn: ConversationTurn = {
      timestamp: new Date().toISOString(),
      turnNumber: context.turnCount + 1,
      userInput: command.explanation || 'Voice command',
      robotResponse: `Action: ${command.action}`,
      location: command.parameters?.goal || null,
      commandType: command.action,
      success: true
    };
    
    setContext(prev => ({
      ...prev,
      history: [newTurn, ...prev.history].slice(0, 10), // Keep last 10
      turnCount: prev.turnCount + 1,
      lastUpdated: new Date().toISOString()
    }));
  };
  
  // Service call to get full context
  const requestFullContext = useCallback(async () => {
    if (!rosRef.current || !isConnected) {
      throw new Error('Not connected to ROS');
    }
    
    return new Promise<string>((resolve, reject) => {
      const service = new ROSLIB.Service({
        ros: rosRef.current!,
        name: '/conversation/get_context',
        serviceType: 'std_srvs/Trigger'
      });
      
      const request = new ROSLIB.ServiceRequest({});
      
      service.callService(request, (result: any) => {
        if (result.success) {
          parseConversationContext(result.message);
          resolve(result.message);
        } else {
          reject(new Error('Failed to get context'));
        }
      });
    });
  }, [isConnected]);
  
  return {
    context,
    isConnected,
    error,
    requestFullContext
  };
};
```

**Action Items:**
- [ ] Create useConversationMemory hook
- [ ] Install roslib: `npm install roslib`
- [ ] Test connection to rosbridge
- [ ] Verify topic subscriptions work

#### Task 4.2: Update Conversation Panel (1.5 hours)

Connect the conversation panel to real data.

**File:** `web_dashboard/src/components/ConversationPanel.tsx`

```typescript
import React, { useEffect } from 'react';
import { MessageCircle, RefreshCw, User, Bot } from 'lucide-react';
import { useConversationMemory } from '../hooks/useConversationMemory';

export const ConversationPanel: React.FC = () => {
  const { context, isConnected, error, requestFullContext } = useConversationMemory();
  
  useEffect(() => {
    // Request full context on mount
    if (isConnected) {
      requestFullContext().catch(console.error);
    }
  }, [isConnected, requestFullContext]);
  
  const formatTimestamp = (timestamp: string) => {
    return new Date(timestamp).toLocaleTimeString();
  };
  
  return (
    <div className="bg-white rounded-lg shadow-md p-4">
      <div className="flex items-center justify-between mb-4">
        <div className="flex items-center gap-2">
          <MessageCircle className="w-5 h-5 text-blue-600" />
          <h2 className="text-lg font-bold text-gray-800">
            Conversation History
          </h2>
          <span className="text-sm text-gray-500">
            ({context.turnCount} turns)
          </span>
        </div>
        
        <div className="flex items-center gap-2">
          {/* Connection status */}
          <div className={`w-2 h-2 rounded-full ${
            isConnected ? 'bg-green-500' : 'bg-red-500'
          }`} />
          <span className="text-xs text-gray-600">
            {isConnected ? 'Connected' : 'Disconnected'}
          </span>
          
          {/* Refresh button */}
          <button
            onClick={() => requestFullContext()}
            className="p-1 hover:bg-gray-100 rounded"
            disabled={!isConnected}
          >
            <RefreshCw className="w-4 h-4 text-gray-600" />
          </button>
        </div>
      </div>
      
      {error && (
        <div className="bg-red-50 border-l-4 border-red-500 p-3 mb-4 rounded">
          <p className="text-sm text-red-700">{error}</p>
        </div>
      )}
      
      <div className="space-y-3 max-h-[500px] overflow-y-auto">
        {context.history.length === 0 ? (
          <div className="text-center text-gray-500 py-8">
            <MessageCircle className="w-12 h-12 mx-auto mb-2 opacity-30" />
            <p>No conversation history yet</p>
            <p className="text-sm">Start talking to the robot!</p>
          </div>
        ) : (
          context.history.map((turn, idx) => (
            <div key={idx} className="space-y-2">
              {/* User message */}
              <div className="flex items-start gap-2">
                <div className="flex-shrink-0 w-8 h-8 bg-blue-100 rounded-full flex items-center justify-center">
                  <User className="w-5 h-5 text-blue-600" />
                </div>
                <div className="flex-1">
                  <div className="bg-blue-50 rounded-lg p-3">
                    <p className="text-sm text-gray-800">{turn.userInput}</p>
                  </div>
                  <p className="text-xs text-gray-500 mt-1">
                    Turn {turn.turnNumber} • {formatTimestamp(turn.timestamp)}
                  </p>
                </div>
              </div>
              
              {/* Robot response */}
              {turn.robotResponse && (
                <div className="flex items-start gap-2 ml-4">
                  <div className="flex-shrink-0 w-8 h-8 bg-green-100 rounded-full flex items-center justify-center">
                    <Bot className="w-5 h-5 text-green-600" />
                  </div>
                  <div className="flex-1">
                    <div className="bg-green-50 rounded-lg p-3">
                      <p className="text-sm text-gray-800">{turn.robotResponse}</p>
                      {turn.location && (
                        <div className="mt-2 text-xs text-gray-600">
                          📍 {turn.location.label || 
                            `(${turn.location.x.toFixed(2)}, ${turn.location.y.toFixed(2)})`}
                        </div>
                      )}
                      <div className="mt-1 flex items-center gap-2">
                        <span className={`inline-block px-2 py-0.5 rounded text-xs ${
                          turn.success 
                            ? 'bg-green-200 text-green-800' 
                            : 'bg-red-200 text-red-800'
                        }`}>
                          {turn.commandType}
                        </span>
                        {turn.success ? '✓' : '✗'}
                      </div>
                    </div>
                  </div>
                </div>
              )}
            </div>
          ))
        )}
      </div>
      
      {/* Session info footer */}
      {context.sessionId && (
        <div className="mt-4 pt-4 border-t border-gray-200">
          <div className="text-xs text-gray-500">
            Session: {context.sessionId}
            <span className="mx-2">•</span>
            Last updated: {formatTimestamp(context.lastUpdated)}
          </div>
        </div>
      )}
    </div>
  );
};
```

**Action Items:**
- [ ] Update ConversationPanel component
- [ ] Test real-time updates from ROS
- [ ] Verify conversation history displays correctly
- [ ] Test refresh button functionality

#### Task 4.3: Spatial Reference Visualization (30 mins)

Add visualization of known spatial references to dashboard.

**File:** `web_dashboard/src/components/SpatialReferenceMap.tsx`

```typescript
import React, { useState, useEffect } from 'react';
import { MapPin, Eye, EyeOff } from 'lucide-react';
import ROSLIB from 'roslib';

interface SpatialReference {
  term: string;
  x: number;
  y: number;
  label?: string;
  frequency: number;
}

export const SpatialReferenceMap: React.FC = () => {
  const [references, setReferences] = useState<SpatialReference[]>([]);
  const [showLabels, setShowLabels] = useState(true);
  
  // TODO: Subscribe to spatial references topic or service
  // For now using mock data structure
  
  return (
    <div className="bg-white rounded-lg shadow-md p-4">
      <div className="flex items-center justify-between mb-4">
        <div className="flex items-center gap-2">
          <MapPin className="w-5 h-5 text-purple-600" />
          <h2 className="text-lg font-bold text-gray-800">
            Known Locations
          </h2>
        </div>
        
        <button
          onClick={() => setShowLabels(!showLabels)}
          className="flex items-center gap-1 text-sm text-gray-600 hover:text-gray-800"
        >
          {showLabels ? <Eye className="w-4 h-4" /> : <EyeOff className="w-4 h-4" />}
          {showLabels ? 'Hide' : 'Show'} Labels
        </button>
      </div>
      
      <div className="space-y-2 max-h-64 overflow-y-auto">
        {references.length === 0 ? (
          <p className="text-sm text-gray-500 text-center py-4">
            No spatial references stored yet
          </p>
        ) : (
          references.map((ref, idx) => (
            <div
              key={idx}
              className="flex items-center justify-between p-2 bg-gray-50 rounded hover:bg-gray-100"
            >
              <div className="flex items-center gap-2">
                <MapPin className="w-4 h-4 text-purple-600" />
                <div>
                  <p className="font-medium text-sm text-gray-800">
                    "{ref.term}"
                    {ref.label && <span className="text-gray-500"> ({ref.label})</span>}
                  </p>
                  <p className="text-xs text-gray-600">
                    ({ref.x.toFixed(2)}, {ref.y.toFixed(2)})
                  </p>
                </div>
              </div>
              
              <div className="text-xs text-gray-500">
                Used {ref.frequency}x
              </div>
            </div>
          ))
        )}
      </div>
    </div>
  );
};
```

**Action Items:**
- [ ] Create spatial reference visualization component
- [ ] Add to dashboard layout
- [ ] Connect to ROS data (implement in next task)
- [ ] Test display with sample data

### Afternoon Session (3 hours)

#### Task 4.4: Real-Time Updates & Polish (2 hours)

Ensure dashboard updates smoothly in real-time.

**File:** `web_dashboard/src/App.tsx`

Update main app to include all conversation features:

```typescript
import React, { useState, useEffect } from 'react';
import { ConversationPanel } from './components/ConversationPanel';
import { ExplanationPanel } from './components/ExplanationPanel';
import { TwinComparisonPanel } from './components/TwinComparisonPanel';
import { SpatialReferenceMap } from './components/SpatialReferenceMap';
import { RobotStatus } from './components/RobotStatus';
import { MapVisualization } from './components/MapVisualization';
import { Settings, Power } from 'lucide-react';

function App() {
  const [rosConnected, setRosConnected] = useState(false);
  const [showSettings, setShowSettings] = useState(false);
  
  return (
    <div className="min-h-screen bg-gray-100">
      {/* Header */}
      <header className="bg-white shadow-sm border-b border-gray-200 px-6 py-4">
        <div className="flex items-center justify-between">
          <div>
            <h1 className="text-2xl font-bold text-gray-800">
              Intelligent Digital Twin with XAI
            </h1>
            <p className="text-sm text-gray-600 mt-1">
              Conversational Robot Control • Week 2
            </p>
          </div>
          
          <div className="flex items-center gap-4">
            {/* Connection Status */}
            <div className="flex items-center gap-2 px-3 py-2 bg-gray-50 rounded-lg">
              <div className={`w-2 h-2 rounded-full ${
                rosConnected ? 'bg-green-500' : 'bg-red-500'
              }`} />
              <span className="text-sm text-gray-700">
                {rosConnected ? 'Connected' : 'Disconnected'}
              </span>
            </div>
            
            {/* Settings Button */}
            <button
              onClick={() => setShowSettings(!showSettings)}
              className="p-2 hover:bg-gray-100 rounded-lg"
            >
              <Settings className="w-5 h-5 text-gray-600" />
            </button>
          </div>
        </div>
      </header>
      
      <div className="p-6">
        <div className="grid grid-cols-12 gap-6">
          {/* Left Column - Main Visualization */}
          <div className="col-span-8 space-y-6">
            <MapVisualization />
            <RobotStatus />
          </div>
          
          {/* Right Column - Conversation & Context */}
          <div className="col-span-4 space-y-6">
            <ConversationPanel />
            <SpatialReferenceMap />
            <ExplanationPanel />
          </div>
        </div>
      </div>
      
      {/* Settings Modal */}
      {showSettings && (
        <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
          <div className="bg-white rounded-lg shadow-xl p-6 max-w-md w-full">
            <h2 className="text-xl font-bold mb-4">Settings</h2>
            
            <div className="space-y-4">
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  ROS Bridge URL
                </label>
                <input
                  type="text"
                  defaultValue="ws://localhost:9090"
                  className="w-full px-3 py-2 border border-gray-300 rounded-lg"
                />
              </div>
              
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  History Length
                </label>
                <input
                  type="number"
                  defaultValue="10"
                  className="w-full px-3 py-2 border border-gray-300 rounded-lg"
                />
              </div>
            </div>
            
            <div className="mt-6 flex gap-2">
              <button
                onClick={() => setShowSettings(false)}
                className="flex-1 px-4 py-2 bg-gray-200 text-gray-800 rounded-lg hover:bg-gray-300"
              >
                Cancel
              </button>
              <button
                onClick={() => setShowSettings(false)}
                className="flex-1 px-4 py-2 bg-blue-600 text-white rounded-lg hover:bg-blue-700"
              >
                Save
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}

export default App;
```

**Action Items:**
- [ ] Update App.tsx with complete layout
- [ ] Test all panels display correctly
- [ ] Verify real-time updates work
- [ ] Test settings modal functionality

#### Task 4.5: Documentation & Screenshots (1 hour)

Document Week 2 work and capture evidence of success.

**Create:** `docs/weekly_logs/week2.md`

```markdown
# Week 2 Log: Conversational Memory Implementation

**Dates:** Jan 27 - Feb 2, 2025  
**Status:** ✓ Complete  
**Milestone:** 5+ turn conversational dialogue with spatial context

---

## Objectives Achieved

### Day 1: Database & Core Memory ✓
- [x] Enhanced database schema (3 tables: conversations, spatial_references, sessions)
- [x] Session management (start, end, resume)
- [x] Core memory operations (add, retrieve, format)
- [x] Spatial reference system (store, resolve, infer from history)
- [x] Comprehensive unit tests (100% pass rate)

### Day 2: LLM Integration ✓
- [x] Context-aware prompt engineering
- [x] Gemini API integration with conversation history
- [x] Prompt templates for different scenarios
- [x] Response validation and sanitization
- [x] Confidence scoring and clarification logic

### Day 3: Voice Pipeline Integration ✓
- [x] Bidirectional communication with voice system
- [x] TTS-friendly response formatting
- [x] End-to-end conversation test (5+ turns)
- [x] Performance optimization (avg < 2.5s)
- [x] Database optimization and archiving

### Day 4: Dashboard Integration ✓
- [x] WebSocket integration via rosbridge
- [x] Real-time conversation display
- [x] Spatial reference visualization
- [x] Connection status monitoring
- [x] Settings modal

---

## Key Metrics

### Performance
- **Average processing time:** 2.1 seconds (target: <2.5s) ✓
- **Context retrieval time:** 45ms (target: <100ms) ✓
- **LLM API latency:** 800ms (acceptable)
- **Database query time:** 15ms (excellent)

### Functionality
- **Context retention:** 100% (all 5 test turns)
- **Spatial reference resolution:** 100% accuracy
- **Clarification detection:** Working (triggered on ambiguous commands)
- **Session persistence:** Working (survived node restart)

### Test Results
```
End-to-End Conversation Test:
✓ Turn 1: "Go to kitchen at 5, 3" → Navigated, stored reference
✓ Turn 2: "Move to living room at 8, 7" → Navigated, stored reference  
✓ Turn 3: "Go back to kitchen" → Resolved from memory (5, 3)
✓ Turn 4: "Take me back there" → Resolved "there" to (8, 7)
✓ Turn 5: "Go where we were before that" → Resolved to (5, 3)
✓ Turn 6: "Go there" (no context) → Clarification requested

Success Rate: 6/6 (100%)
Average Response Time: 2.1s
```

---

## Technical Implementation Highlights

### 1. Database Schema
```sql
-- 3 tables with proper indexing
conversations: 14 fields including context_used, metadata
spatial_references: 8 fields with frequency tracking
sessions: 5 fields with active status
```

### 2. LLM Context Injection
```python
# Structured prompt with 5 sections:
1. System instructions
2. Conversation history (last 5 turns)
3. Known spatial references
4. Current robot position
5. User command

Total prompt size: ~500-800 tokens
```

### 3. Spatial Reference Resolution
```python
# Multi-strategy resolution:
1. Direct lookup in database
2. Pattern matching (variations)
3. History inference (previous, return)
4. Clarification request (fallback)
```

### 4. Response Validation
```python
# 5-stage validation:
1. Structure check (required fields)
2. Action type validation
3. Parameter validation  
4. Coordinate sanitization
5. Confidence thresholding
```

---

## Challenges & Solutions

### Challenge 1: LLM Prompt Length
**Issue:** Context history exceeded Gemini token limits (8192)
**Solution:** Limited history to 5 turns, used summarization for older turns
**Result:** Prompts now 600-800 tokens, well within limits

### Challenge 2: Spatial Reference Ambiguity
**Issue:** "there" could refer to multiple locations
**Solution:** Implemented recency-based resolution + clarification requests
**Result:** 100% disambiguation success in tests

### Challenge 3: Database Performance
**Issue:** Query times increased with conversation history
**Solution:** Added strategic indexes, periodic optimization, archiving
**Result:** Query times <20ms even with 100+ turns

### Challenge 4: WebSocket Connection Stability
**Issue:** Dashboard lost connection intermittently
**Solution:** Added reconnection logic, heartbeat monitoring
**Result:** Stable connection, <1% disconnection rate

---

## Code Statistics

- **Python code:** ~1,200 lines
- **TypeScript code:** ~800 lines
- **Database tables:** 3 (+ 1 archive)
- **ROS topics:** 6 published/subscribed
- **Test coverage:** 85% (core functions)
- **Git commits:** 12

---

## Example Conversations

### Conversation 1: Location References
```
User: "Go to the kitchen"
Robot: "Navigating to the kitchen" [stores at current position]

User: "Now go to the bedroom"  
Robot: "Moving to the bedroom"

User: "Return to the kitchen"
Robot: "Returning to the kitchen" [resolves from memory]
```

### Conversation 2: Spatial Context
```
User: "Move forward 3 meters"
Robot: "Moving forward" [stores position]

User: "Go back there"
Robot: "Returning to previous position" [resolves from history]
```

### Conversation 3: Clarification
```
User: "Go over there"
Robot: "I need clarification: which location are you referring to?"

User: "The kitchen"
Robot: "Navigating to the kitchen"
```

---

## Screenshots

### 1. Dashboard - Conversation Panel
![Conversation History](screenshots/week2_conversation_panel.png)
*Shows 5-turn conversation with user/robot messages and locations*

### 2. Database Contents
![Database State](screenshots/week2_database.png)
*SQLite browser showing conversations and spatial_references tables*

### 3. Performance Metrics
![Response Times](screenshots/week2_performance.png)
*Graph showing average response times across 20 test conversations*

### 4. Terminal Output
![Node Logs](screenshots/week2_terminal.png)
*ROS node logs showing context injection and LLM responses*

---

## Week 2 Deliverable Checklist

### Core Functionality ✓
- [x] 5+ turn conversation working
- [x] Spatial reference resolution ("there", "back", "previous")
- [x] Session management (start, end, resume)
- [x] Context injection into LLM prompts
- [x] Response validation and sanitization

### Integration ✓
- [x] Voice pipeline fully integrated
- [x] Dashboard displays real-time conversations
- [x] rosbridge WebSocket communication working
- [x] Database persists across restarts

### Performance ✓
- [x] Average response time < 2.5 seconds
- [x] Context retrieval < 100ms
- [x] 100% spatial reference resolution accuracy
- [x] No memory leaks or performance degradation

### Documentation ✓
- [x] Week 2 log complete with metrics
- [x] Code well-commented
- [x] API documentation updated
- [x] Test scenarios documented

---

## Lessons Learned

### What Worked Well
1. **Incremental testing:** Testing each component daily prevented integration nightmares
2. **Prompt templates:** Reusable templates made LLM responses more consistent
3. **Database indexing:** Early optimization prevented slowdowns
4. **Response validation:** Caught many edge cases before they became bugs

### What Could Improve
1. **Prompt engineering:** Still requires tuning for edge cases
2. **Error handling:** Need more graceful degradation for API failures
3. **Testing coverage:** Could add more integration tests
4. **Documentation:** API docs could be more detailed

### Skills Developed
- Advanced SQL query optimization
- LLM prompt engineering for conversational AI
- Real-time WebSocket communication
- Performance profiling and optimization

---

## Next Week Preview (Week 3)

**Focus:** XAI Navigation - Part 1 (Decision Logging)

**Key Tasks:**
1. Hook into Nav2 action server
2. Capture path planning decisions
3. Extract cost map data
4. Log obstacle detections
5. Create decision data structures

**Dependencies Met:**
- ✓ Conversation memory working (can store explanations)
- ✓ Database ready for decision logs
- ✓ LLM integrated (can generate explanations)

**Risk:** Nav2 callback hooks may be complex - budget extra time

---

## Supervisor Check-in Notes

**Discussed with Dr. Sujala:**
- ✓ Week 2 milestones achieved on schedule
- ✓ Performance metrics exceed targets
- ✓ Demonstration of 5-turn conversation successful
- Discussion point: Consider multi-language support in Week 6+

**Action Items from Meeting:**
- Continue with Week 3 as planned
- Document any Nav2 integration challenges
- Prepare mid-project progress presentation (Week 4 end)

---

**Confidence for Week 3:** 9/10 (strong foundation, clear path forward)
```

**Action Items:**
- [ ] Complete Week 2 log with all metrics
- [ ] Capture 4 screenshots:
  - Dashboard conversation panel
  - Database contents (SQLite browser)
  - Performance graph (can be simple chart)
  - Terminal logs
- [ ] Save screenshots to `docs/screenshots/`
- [ ] Update main README.md with Week 2 status

### End of Day 4 Checklist

- [ ] Dashboard fully integrated with ROS
- [ ] Real-time conversation display working
- [ ] Spatial reference visualization complete
- [ ] Week 2 documentation complete with screenshots
- [ ] All code committed: "Day 4: Dashboard integration & Week 2 complete"
- [ ] Git tag created: `git tag week2-complete`

---

## Days 5-7: Testing, Polish & Preparation

### Day 5 (Friday): Comprehensive Testing

#### Morning: Integration Testing (3 hours)

**Create comprehensive test suite:**

**File:** `conversation_memory_node/test/test_week2_integration.py`

```python
#!/usr/bin/env python3
"""
Comprehensive integration tests for Week 2
"""

import unittest
import subprocess
import time
import sqlite3
import os

class Week2IntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """Launch ROS system"""
        # Clear old database
        db_path = os.path.expanduser('~/.ros/conversation_history.db')
        if os.path.exists(db_path):
            os.remove(db_path)
        
        # Launch system (in background)
        cls.ros_process = subprocess.Popen([
            'ros2', 'launch', 'conversation_memory_node',
            'conversational_system.launch.py'
        ])
        
        # Wait for startup
        time.sleep(10)
    
    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS system"""
        cls.ros_process.terminate()
        cls.ros_process.wait()
    
    def test_01_database_creation(self):
        """Test database is created with correct schema"""
        db_path = os.path.expanduser('~/.ros/conversation_history.db')
        self.assertTrue(os.path.exists(db_path))
        
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        
        # Check tables exist
        cursor.execute(
            "SELECT name FROM sqlite_master WHERE type='table'"
        )
        tables = [row[0] for row in cursor.fetchall()]
        
        self.assertIn('conversations', tables)
        self.assertIn('spatial_references', tables)
        self.assertIn('sessions', tables)
        
        conn.close()
    
    def test_02_session_created(self):
        """Test session is created on startup"""
        db_path = os.path.expanduser('~/.ros/conversation_history.db')
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        
        cursor.execute("SELECT COUNT(*) FROM sessions WHERE is_active = 1")
        active_sessions = cursor.fetchone()[0]
        
        self.assertEqual(active_sessions, 1, "Should have 1 active session")
        
        conn.close()
    
    def test_03_five_turn_conversation(self):
        """Test 5-turn conversation with context"""
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        
        rclpy.init()
        
        class TestNode(Node):
            def __init__(self):
                super().__init__('test_node')
                self.pub = self.create_publisher(
                    String, '/voice/transcription', 10
                )
                self.responses = []
                self.sub = self.create_subscription(
                    String, '/robot/response', 
                    lambda msg: self.responses.append(msg.data),
                    10
                )
        
        node = TestNode()
        
        # Give time for connections
        time.sleep(2)
        
        # Turn 1
        msg = String()
        msg.data = "Go to kitchen at 5, 3"
        node.pub.publish(msg)
        time.sleep(3)
        
        # Turn 2
        msg.data = "Move to bedroom at 2, 8"
        node.pub.publish(msg)
        time.sleep(3)
        
        # Turn 3 - use reference
        msg.data = "Go back to the kitchen"
        node.pub.publish(msg)
        time.sleep(3)
        
        # Turn 4 - use "there"
        msg.data = "Take me back there"
        node.pub.publish(msg)
        time.sleep(3)
        
        # Turn 5 - complex reference
        msg.data = "Return to where we were before that"
        node.pub.publish(msg)
        time.sleep(3)
        
        # Check responses received
        self.assertGreaterEqual(
            len(node.responses), 5,
            f"Should have 5+ responses, got {len(node.responses)}"
        )
        
        node.destroy_node()
        rclpy.shutdown()
    
    def test_04_spatial_references_stored(self):
        """Test spatial references are stored correctly"""
        db_path = os.path.expanduser('~/.ros/conversation_history.db')
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        
        cursor.execute(
            "SELECT COUNT(*) FROM spatial_references"
        )
        ref_count = cursor.fetchone()[0]
        
        self.assertGreater(ref_count, 0, "Should have spatial references")
        
        # Check kitchen reference exists
        cursor.execute('''
            SELECT location_x, location_y FROM spatial_references
            WHERE reference_term LIKE '%kitchen%'
        ''')
        kitchen = cursor.fetchone()
        
        self.assertIsNotNone(kitchen, "Kitchen reference should exist")
        
        conn.close()
    
    def test_05_performance_metrics(self):
        """Test response times are within targets"""
        db_path = os.path.expanduser('~/.ros/conversation_history.db')
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        
        # Get all conversation timestamps
        cursor.execute('''
            SELECT timestamp FROM conversations 
            ORDER BY timestamp
        ''')
        timestamps = [row[0] for row in cursor.fetchall()]
        
        if len(timestamps) < 2:
            self.skipTest("Not enough data for timing test")
        
        # Calculate time between turns
        from datetime import datetime
        times = []
        for i in range(1, len(timestamps)):
            t1 = datetime.fromisoformat(timestamps[i-1])
            t2 = datetime.fromisoformat(timestamps[i])
            delta = (t2 - t1).total_seconds()
            times.append(delta)
        
        avg_time = sum(times) / len(times)
        
        self.assertLess(
            avg_time, 5.0,
            f"Average processing time {avg_time:.2f}s exceeds 5s target"
        )
        
        conn.close()

if __name__ == '__main__':
    # Run tests
    suite = unittest.TestLoader().loadTestsFromTestCase(Week2IntegrationTest)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Print summary
    print("\n" + "="*60)
    print("WEEK 2 INTEGRATION TEST SUMMARY")
    print("="*60)
    print(f"Tests run: {result.testsRun}")
    print(f"Successes: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print("="*60)
    
    # Exit with appropriate code
    exit(0 if result.wasSuccessful() else 1)
```

**Action Items:**
- [ ] Create integration test suite
- [ ] Run full test suite: `python3 test_week2_integration.py`
- [ ] Fix any failing tests
- [ ] Achieve 100% test pass rate
- [ ] Document test results in Week 2 log

#### Afternoon: User Testing (3 hours)

**Recruit 3-5 test users and conduct usability testing:**

**Test Protocol:**
1. Give 5-minute demo of system
2. Have user perform 5-turn conversation
3. Observe and take notes
4. Conduct brief survey (5-10 minutes)

**Survey Questions** (5-point Likert scale):
1. The robot understood my commands clearly
2. The conversation felt natural
3. The robot remembered previous locations correctly
4. Response times were acceptable
5. I would trust this system to control a robot

**Action Items:**
- [ ] Recruit 3-5 test users (classmates, lab members)
- [ ] Conduct 3-5 user testing sessions
- [ ] Collect survey responses
- [ ] Calculate average scores
- [ ] Document feedback in Week 2 log

### Day 6 (Saturday): Polish & Bug Fixes

#### Tasks (4 hours total):
- [ ] Fix any bugs discovered in testing
- [ ] Improve error messages for common issues
- [ ] Add input validation for edge cases
- [ ] Optimize database queries if needed
- [ ] Polish dashboard UI (animations, transitions)
- [ ] Update all documentation
- [ ] Clean up code (remove debug prints, add comments)

### Day 7 (Sunday): Week 3 Preparation

#### Morning (2 hours): Week 2 Retrospective

**Create:** `docs/week2_retrospective.md`

```markdown
# Week 2 Retrospective

## What Went Well ✓
1. Database schema well-designed from start
2. Incremental testing caught issues early
3. LLM integration smoother than expected
4. Dashboard integration straightforward

## What Could Improve ⚠️
1. Prompt engineering took longer than planned
2. WebSocket connection needed debugging
3. Test coverage could be higher
4. Documentation done at end vs. continuously

## Action Items for Future Weeks
- [ ] Write tests alongside features (not after)
- [ ] Document as you code
- [ ] Budget more time for prompt engineering
- [ ] Test WebSocket early and often

## Skills Gained
- SQLite optimization techniques
- Advanced prompt engineering
- Real-time web integration
- Performance profiling

## Confidence Level: 9/10
Strong foundation for Week 3. Ready to proceed.
```

#### Afternoon (2 hours): Week 3 Planning

**Review Week 3 objectives:**
- XAI Navigation - Part 1
- Nav2 decision logging
- Cost map extraction
- Explanation data structures

**Prepare:**
- [ ] Read Nav2 documentation on action servers
- [ ] Review your last semester's Nav2 integration code
- [ ] Sketch explanation data flow
- [ ] Identify potential integration points
- [ ] List questions for Dr. Sujala check-in

---

## Week 2 Final Checklist

### Deliverables ✓
- [ ] Working 5+ turn conversational system
- [ ] Spatial reference resolution (100% accuracy)
- [ ] Session management with persistence
- [ ] Dashboard with real-time updates
- [ ] Comprehensive documentation
- [ ] Test suite with >85% coverage
- [ ] User testing completed (3+ users)
- [ ] Screenshots captured
- [ ] Git repository updated

### Performance Metrics ✓
- [ ] Average response time < 2.5 seconds
- [ ] Context retrieval < 100ms
- [ ] Database queries < 20ms
- [ ] 100% spatial reference resolution
- [ ] User satisfaction > 4.0/5.0

### Code Quality ✓
- [ ] All functions documented
- [ ] Type hints added (Python)
- [ ] No debug print statements
- [ ] Consistent naming conventions
- [ ] Error handling implemented
- [ ] Logging configured properly

### Integration ✓
- [ ] Voice pipeline connected
- [ ] Dashboard displaying data
- [ ] rosbridge stable
- [ ] All ROS topics working
- [ ] Database persisting correctly

---

## Success Criteria Met

### MVP (Minimum Viable Product) ✓
- ✅ 3+ turn conversation with context
- ✅ Basic spatial reference resolution
- ✅ Database persistence
- ✅ Dashboard display

### Target Goals ✓
- ✅ 5+ turn conversation with spatial references
- ✅ Multiple reference resolution strategies
- ✅ Performance < 2.5s
- ✅ Real-time dashboard updates
- ✅ User testing positive

### Stretch Goals (Optional)
- ⚠️ 10+ turn conversation (tested but not required)
- ⚠️ Multi-language support (deferred to later)
- ✅ Automated testing suite
- ✅ Performance optimization

---

## Week 2 Sign-Off

**Student Assessment:**
- All objectives met ✓
- Performance exceeds targets ✓
- Ready for Week 3 ✓
- Confidence: 9/10

**Next Milestone:** Week 3 - XAI Navigation Decision Logging

**Estimated Completion:** February 9, 2025

---

*Week 2 Guide Complete. Proceed to Week 3 Implementation.*
