# Week 2 Preparation Guide

## Intelligent Digital Twin with XAI for Human-Robot Interaction

---

## Week 2 Focus: Conversation Memory Business Logic

**Primary Goal:** Implement complete conversation memory system with SQLite persistence, context injection, and Gemini integration.

---

## Prerequisites Checklist

Before starting Week 2, verify:

- [ ] Gemini API key configured in `backend/.env`
- [ ] OpenAI API key configured (for Whisper)
- [ ] All Week 1 packages build successfully
- [ ] Custom messages import without errors
- [ ] SQLite3 available on system (`sudo apt install sqlite3`)

---

## Week 2 Day-by-Day Plan

### Day 1: SQLite Implementation
**File:** `src/conversation_memory_pkg/conversation_memory_pkg/conversation_memory_node.py`

Tasks:
1. Complete `_store_conversation()` method with full schema
2. Implement `_get_history(n_turns=5)` with timestamp ordering
3. Add database cleanup (retain last 1000 entries)
4. Test SQLite thread safety with ROS2 callbacks

Expected Output:
```bash
sqlite3 ~/.ros/conversation_memory.db "SELECT * FROM conversation_history LIMIT 5;"
```

### Day 2: Context Building
**File:** Same as Day 1

Tasks:
1. Implement semantic location extraction from Nav2
2. Parse robot pose from `/amcl_pose` topic
3. Build context dictionary with history window
4. Publish to `/conversation/context` topic

Context Format:
```json
{
  "current_input": "Go to the kitchen",
  "conversation_history": [
    {"user": "Hello", "robot": "Hi, how can I help?", "time": "..."},
    ...
  ],
  "current_location": "Living Room",
  "current_pose": {"x": 1.2, "y": 3.4, "theta": 0.5},
  "timestamp": "2025-11-18T10:30:00Z"
}
```

### Day 3: Gemini Service Integration
**File:** `backend/app/services/gemini_service.py`

Tasks:
1. Create prompt template with conversation context
2. Configure structured output (Pydantic schema)
3. Parse intent classification response
4. Handle API rate limits and errors

Prompt Template:
```python
SYSTEM_PROMPT = """You are a robot navigation assistant.
Given the conversation history and current context, extract:
- user_intent: navigation_command | status_query | emergency_stop | general
- parameters: {destination, action, query_type}
- confidence: 0.0 to 1.0
"""
```

### Day 4: Backend Integration
**File:** `backend/app/main_ros2_endpoints.py`

Tasks:
1. Create `/api/conversation/send` endpoint
2. Connect conversation_memory_node to backend via rosbridge
3. Implement request/response flow:
   - User input → Backend → Whisper → conversation_memory_node
   - context → Gemini → parsed intent → Nav2 commands

### Day 5: Testing & Validation
Tasks:
1. Unit tests for SQLite operations
2. Integration test: Voice → Text → Intent → Navigation
3. Test context injection with various history sizes
4. Validate Gemini response parsing

Test Cases:
```python
def test_conversation_storage():
    # Insert 10 conversations
    # Retrieve last 5
    # Verify ordering
    pass

def test_context_building():
    # Mock sensor data
    # Build context
    # Validate JSON structure
    pass

def test_gemini_parsing():
    # Send known command
    # Verify intent classification
    # Check confidence score
    pass
```

### Day 6: Error Handling & Edge Cases
Tasks:
1. Handle missing history (first conversation)
2. Graceful degradation if Gemini API fails
3. Database corruption recovery
4. Rate limit backoff strategy

### Day 7: Documentation & Code Review
Tasks:
1. Document conversation memory API
2. Update CLAUDE.md with new endpoints
3. Create usage examples
4. Week 3 preparation

---

## Code Snippets for Week 2

### SQLite Full Schema
```python
self.cursor.execute('''
    CREATE TABLE IF NOT EXISTS conversation_history (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        timestamp TEXT NOT NULL,
        user_input TEXT NOT NULL,
        robot_response TEXT,
        intent TEXT,
        confidence REAL,
        location TEXT,
        pose_x REAL,
        pose_y REAL,
        pose_theta REAL,
        session_id TEXT
    )
''')
```

### Gemini Structured Output
```python
from pydantic import BaseModel
from typing import Optional

class RobotIntent(BaseModel):
    intent_type: str  # navigation_command, status_query, etc.
    destination: Optional[str]
    action: Optional[str]
    confidence: float
    explanation: str

# Usage (New SDK)
client = genai.Client(api_key=api_key)
result = client.models.generate_content(
    model='gemini-2.5-flash-preview-09-2025',
    contents=prompt,
    config=types.GenerateContentConfig(
        response_mime_type="application/json",
        response_schema=RobotIntent
    )
)
```

### Context Injection Pattern
```python
def build_gemini_prompt(context: dict) -> str:
    history_str = "\n".join([
        f"User: {h['user']}\nRobot: {h['robot']}"
        for h in context['conversation_history']
    ])

    return f"""
    Current Location: {context['current_location']}
    Robot Position: ({context['current_pose']['x']:.2f}, {context['current_pose']['y']:.2f})

    Recent Conversation:
    {history_str}

    Current User Input: {context['current_input']}

    Extract the user's intent and provide structured response.
    """
```

---

## Performance Targets for Week 2

| Metric | Target | How to Measure |
|--------|--------|----------------|
| SQLite write latency | < 10ms | Python time.time() |
| Context build time | < 50ms | ROS2 timer callbacks |
| Gemini response time | < 2s | API request/response |
| Memory usage (node) | < 100MB | `ros2 run` with memory profiler |
| Database size (1000 entries) | < 10MB | `ls -lh ~/.ros/conversation_memory.db` |

---

## Potential Blockers & Solutions

### Blocker 1: Gemini API Key Not Set
**Solution:**
```bash
# In backend/.env
GEMINI_API_KEY=your_key_here

# Test:
python3 -c "import google.generativeai as genai; print('OK')"
```

### Blocker 2: SQLite Thread Safety
**Solution:** Use connection per callback or connection pool
```python
# Option 1: check_same_thread=False (quick fix)
self.conn = sqlite3.connect(db_path, check_same_thread=False)

# Option 2: Connection per write (safer)
def _store_conversation(self, data):
    with sqlite3.connect(self.db_path) as conn:
        conn.execute(...)
```

### Blocker 3: ROS2 Topic Not Publishing
**Solution:** Verify with `ros2 topic echo`
```bash
ros2 topic echo /conversation/context --no-arr
```

### Blocker 4: Gemini Structured Output Fails
**Solution:** Check Pydantic schema matches expected format
```python
# Test schema independently
from pydantic import BaseModel
class Test(BaseModel):
    field: str
print(Test.schema_json())
```

---

## Files to Modify in Week 2

1. **Primary:**
   - `src/conversation_memory_pkg/conversation_memory_pkg/conversation_memory_node.py`
   - `backend/app/services/gemini_service.py`
   - `backend/app/main_ros2_endpoints.py`

2. **Testing:**
   - `src/conversation_memory_pkg/test/test_storage.py` (new)
   - `src/conversation_memory_pkg/test/test_context.py` (new)

3. **Configuration:**
   - `backend/.env` (API keys)
   - `project/.env` (frontend config if needed)

---

## Week 2 Success Criteria

By end of Week 2, the following should work:

1. **Conversation Storage:**
```bash
# User says "Go to the kitchen"
# Check database has entry
sqlite3 ~/.ros/conversation_memory.db \
  "SELECT user_input, intent FROM conversation_history ORDER BY id DESC LIMIT 1;"
# Output: Go to the kitchen|navigation_command
```

2. **Context Publishing:**
```bash
ros2 topic echo /conversation/context
# Should show JSON with history, location, intent
```

3. **Gemini Integration:**
```bash
curl -X POST http://localhost:8000/api/conversation/send \
  -H "Content-Type: application/json" \
  -d '{"input": "What is your location?"}'
# Response: {"intent": "status_query", "confidence": 0.92, ...}
```

4. **Full Pipeline:**
```
Voice Input → Whisper → Text → conversation_memory_node →
Context → Gemini → Intent → (Ready for Week 3: Nav2 commands)
```

---

## Research Papers for Week 2

Refer to `docs/literature_review/02_conversational_ai_llm_robots.md`:

1. **ROSGPT (Koubaa 2024)** - JSON serialization pattern
2. **Gemini 2.0 Flash Documentation** - Structured output
3. **SayCan Grounding** - Intent → Action mapping

---

## Commands for Week 2 Development

### Start Backend
```bash
cd backend
source /opt/ros/humble/setup.bash
source ~/ros2_navigation_project/install/setup.bash
python -m uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

### Start Conversation Memory Node
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_navigation_project/install/setup.bash
ros2 run conversation_memory_pkg conversation_memory_node
```

### Monitor Topics
```bash
ros2 topic list | grep conversation
ros2 topic echo /conversation/context
ros2 topic hz /conversation/context
```

### Database Inspection
```bash
sqlite3 ~/.ros/conversation_memory.db
.tables
.schema conversation_history
SELECT COUNT(*) FROM conversation_history;
```

---

## Integration with Existing System

Week 2 connects to Week 1 infrastructure:

```
[Existing Week 1]              [Week 2 Business Logic]
conversation_memory_node  →→→  SQLite storage, context building
                          →→→  Gemini intent parsing
                          →→→  JSON publishing

[Frontend]                     [Week 2 Updates]
ConversationPanel.tsx     →→→  Rosbridge subscription (not mocked)

[Backend]                      [Week 2 Additions]
gemini_service.py         →→→  Structured output parsing
main_ros2_endpoints.py    →→→  /api/conversation/send endpoint
```

---

## Week 3 Preview

After Week 2 conversation memory is complete:

- **Week 3 Focus:** XAI Navigation Explanations
- **Builds on:** Parsed intents from Gemini
- **New features:** SHAP feature importance, Nav2 feedback hooks
- **Integration:** Intent → Nav2 goal → Explanation generation

---

**Week 2 Ready to Start ✅**

All prerequisites verified. Begin with Day 1: SQLite implementation.
