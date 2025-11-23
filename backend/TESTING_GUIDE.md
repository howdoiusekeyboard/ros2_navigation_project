# Backend Testing Guide - Week 2 Conversational Memory

This guide will help you test the conversational memory backend implementation thoroughly before integrating with the frontend.

---

## Prerequisites Check

Run these commands to verify your environment:

```bash
# Check Python version (need 3.8+)
python3 --version

# Check if pip is available
python3 -m pip --version
```

---

## Step 1: Install Dependencies

### Option A: Using pip (Recommended)

```bash
cd /home/noob/ros2_navigation_project/backend

# Install aiosqlite (only new dependency)
python3 -m pip install aiosqlite==0.19.0

# Or install all dependencies
python3 -m pip install -r requirements.txt
```

### Option B: Using apt (if pip fails)

```bash
# Install aiosqlite from system packages
sudo apt update
sudo apt install python3-aiosqlite
```

### Option C: Create Virtual Environment (Best Practice)

```bash
cd /home/noob/ros2_navigation_project/backend

# Create virtual environment
python3 -m venv venv

# Activate it
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# When done testing, deactivate with:
# deactivate
```

---

## Step 2: Verify Installation

```bash
cd /home/noob/ros2_navigation_project/backend

# Test imports
python3 -c "import aiosqlite; print('✅ aiosqlite installed')"
python3 -c "from app.database.conversation_db import ConversationDatabase; print('✅ conversation_db imports successfully')"
python3 -c "from app.services.context_builder import ContextBuilder; print('✅ context_builder imports successfully')"
```

Expected output:
```
✅ aiosqlite installed
✅ conversation_db imports successfully
✅ context_builder imports successfully
```

---

## Step 3: Run Unit Tests

```bash
cd /home/noob/ros2_navigation_project/backend

# Run all database tests
python3 -m pytest tests/test_conversation_db.py -v

# Run with detailed output
python3 -m pytest tests/test_conversation_db.py -v -s

# Run specific test
python3 -m pytest tests/test_conversation_db.py::test_add_turn_basic -v
```

Expected output:
```
tests/test_conversation_db.py::test_database_initialization PASSED
tests/test_conversation_db.py::test_session_id_generation PASSED
tests/test_conversation_db.py::test_add_turn_basic PASSED
tests/test_conversation_db.py::test_add_turn_with_location PASSED
tests/test_conversation_db.py::test_multiple_turns_same_session PASSED
tests/test_conversation_db.py::test_get_history_with_limit PASSED
tests/test_conversation_db.py::test_session_isolation PASSED
tests/test_conversation_db.py::test_spatial_references PASSED
tests/test_conversation_db.py::test_spatial_reference_caching PASSED
tests/test_conversation_db.py::test_session_summary PASSED
tests/test_conversation_db.py::test_get_recent_sessions PASSED
tests/test_conversation_db.py::test_delete_session PASSED
tests/test_conversation_db.py::test_metadata_storage PASSED
tests/test_conversation_db.py::test_empty_session PASSED
tests/test_conversation_db.py::test_turn_number_increments PASSED

========================== 15 passed in 2.50s ==========================
```

---

## Step 4: Start Backend Server

### Terminal 1: Backend Server

```bash
cd /home/noob/ros2_navigation_project/backend

# Make sure .env file has GEMINI_API_KEY
cat .env | grep GEMINI_API_KEY

# Start server
python3 -m uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

Expected output:
```
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12345] using StatReload
INFO:     Started server process [12346]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

---

## Step 5: Manual API Testing

### Test 1: Health Check

```bash
# In a new terminal
curl http://localhost:8000/health
```

Expected response:
```json
{"status": "healthy", "version": "1.0.0"}
```

### Test 2: Create First Conversation Turn

```bash
curl -X POST http://localhost:8000/api/v1/execute_voice_command \
  -H "Content-Type: application/json" \
  -d '{
    "transcript": "go to position 2, 3"
  }'
```

Expected response (save the `session_id`):
```json
{
  "transcript": "go to position 2, 3",
  "parsed_command": {
    "action": "navigate",
    "parameters": {
      "x": 2.0,
      "y": 3.0,
      "theta": 0.0
    },
    "confidence": 0.92,
    "reasoning": "Navigation to specific coordinates"
  },
  "execution_status": "navigation_not_implemented",
  "session_id": "a1b2c3d4",
  "resolved_reference": null,
  "latency": {
    "gemini_ms": 245.2,
    "execution_ms": 0.5,
    "memory_ms": 0.8,
    "total_ms": 246.5
  }
}
```

### Test 3: Second Turn (With Context)

Replace `YOUR_SESSION_ID` with the session_id from Test 2:

```bash
curl -X POST http://localhost:8000/api/v1/execute_voice_command \
  -H "Content-Type: application/json" \
  -d '{
    "transcript": "now move to position 5, 1",
    "session_id": "YOUR_SESSION_ID"
  }'
```

### Test 4: Spatial Reference Resolution

This is the key test! Use the same session_id:

```bash
curl -X POST http://localhost:8000/api/v1/execute_voice_command \
  -H "Content-Type: application/json" \
  -d '{
    "transcript": "go back there",
    "session_id": "YOUR_SESSION_ID"
  }'
```

**Expected behavior:** Should resolve "there" to (5.0, 1.0) from the previous turn!

Check the response:
```json
{
  "parsed_command": {
    "parameters": {
      "x": 5.0,
      "y": 1.0
    }
  },
  "resolved_reference": "last mentioned location (turn 2)"
}
```

### Test 5: Get Conversation History

```bash
curl "http://localhost:8000/api/v1/conversation/history/YOUR_SESSION_ID?limit=10"
```

Expected response:
```json
{
  "session_id": "a1b2c3d4",
  "history": [
    {
      "id": 3,
      "timestamp": "2025-11-19T...",
      "turn_number": 3,
      "user_input": "go back there",
      "robot_response": "navigation_not_implemented: ...",
      "action_type": "navigate",
      "location": {
        "x": 5.0,
        "y": 1.0,
        "label": null
      },
      "confidence": 0.88
    },
    {
      "id": 2,
      "timestamp": "2025-11-19T...",
      "turn_number": 2,
      "user_input": "now move to position 5, 1",
      "robot_response": "navigation_not_implemented: ...",
      "action_type": "navigate",
      "location": {
        "x": 5.0,
        "y": 1.0,
        "label": null
      },
      "confidence": 0.92
    },
    {
      "id": 1,
      "timestamp": "2025-11-19T...",
      "turn_number": 1,
      "user_input": "go to position 2, 3",
      "robot_response": "navigation_not_implemented: ...",
      "action_type": "navigate",
      "location": {
        "x": 2.0,
        "y": 3.0,
        "label": null
      },
      "confidence": 0.92
    }
  ],
  "turn_count": 3
}
```

### Test 6: Get Session Summary

```bash
curl "http://localhost:8000/api/v1/conversation/summary/YOUR_SESSION_ID"
```

Expected response:
```json
{
  "session_id": "a1b2c3d4",
  "turn_count": 3,
  "start_time": "2025-11-19T...",
  "end_time": "2025-11-19T...",
  "avg_confidence": 0.907,
  "avg_latency_ms": 240.5,
  "navigation_count": 3
}
```

### Test 7: Get All Sessions

```bash
curl "http://localhost:8000/api/v1/conversation/sessions?limit=5"
```

### Test 8: Get Spatial References

```bash
curl "http://localhost:8000/api/v1/conversation/spatial_refs/YOUR_SESSION_ID"
```

### Test 9: Delete Session

```bash
curl -X DELETE "http://localhost:8000/api/v1/conversation/session/YOUR_SESSION_ID"
```

Expected response:
```json
{
  "message": "Session a1b2c3d4 deleted successfully",
  "deleted": true
}
```

---

## Step 6: Advanced Testing Scenarios

### Scenario 1: Location Labels

```bash
# Create a new conversation
SESSION=$(curl -s -X POST http://localhost:8000/api/v1/execute_voice_command \
  -H "Content-Type: application/json" \
  -d '{"transcript": "go to the kitchen at position 2, 3"}' | grep -o '"session_id":"[^"]*' | cut -d'"' -f4)

echo "Session ID: $SESSION"

# Reference the location
curl -X POST http://localhost:8000/api/v1/execute_voice_command \
  -H "Content-Type: application/json" \
  -d "{
    \"transcript\": \"now go to the bedroom at position 5, 1\",
    \"session_id\": \"$SESSION\"
  }"

# Use "go there" - should resolve to bedroom (most recent)
curl -X POST http://localhost:8000/api/v1/execute_voice_command \
  -H "Content-Type: application/json" \
  -d "{
    \"transcript\": \"go there\",
    \"session_id\": \"$SESSION\"
  }"

# Check spatial references
curl "http://localhost:8000/api/v1/conversation/spatial_refs/$SESSION"
```

Expected spatial_refs output:
```json
{
  "session_id": "...",
  "references": {
    "kitchen": {"x": 2.0, "y": 3.0},
    "bedroom": {"x": 5.0, "y": 1.0}
  }
}
```

### Scenario 2: "Go Back" Resolution

```bash
# Navigate to multiple locations
SESSION="test123"

curl -X POST http://localhost:8000/api/v1/execute_voice_command \
  -H "Content-Type: application/json" \
  -d "{\"transcript\": \"go to 1, 1\", \"session_id\": \"$SESSION\"}"

curl -X POST http://localhost:8000/api/v1/execute_voice_command \
  -H "Content-Type: application/json" \
  -d "{\"transcript\": \"go to 2, 2\", \"session_id\": \"$SESSION\"}"

curl -X POST http://localhost:8000/api/v1/execute_voice_command \
  -H "Content-Type: application/json" \
  -d "{\"transcript\": \"go to 3, 3\", \"session_id\": \"$SESSION\"}"

# "go back" should resolve to (2, 2) - second most recent
curl -X POST http://localhost:8000/api/v1/execute_voice_command \
  -H "Content-Type: application/json" \
  -d "{\"transcript\": \"go back\", \"session_id\": \"$SESSION\"}"
```

---

## Step 7: Performance Testing

### Test Latency

```bash
# Run 10 consecutive requests and measure latency
for i in {1..10}; do
  curl -s -X POST http://localhost:8000/api/v1/execute_voice_command \
    -H "Content-Type: application/json" \
    -d '{"transcript": "move forward"}' \
    | grep -o '"total_ms":[0-9.]*' | cut -d':' -f2
done
```

Expected: All values should be < 2000ms (target: ~200-500ms)

### Test Database Query Performance

```bash
# Create a script to add 100 turns
cat > /tmp/test_load.sh << 'EOF'
#!/bin/bash
SESSION_ID="loadtest"
for i in {1..100}; do
  curl -s -X POST http://localhost:8000/api/v1/execute_voice_command \
    -H "Content-Type: application/json" \
    -d "{\"transcript\": \"command $i\", \"session_id\": \"$SESSION_ID\"}" \
    > /dev/null
  echo "Added turn $i"
done
echo "Getting history..."
time curl -s "http://localhost:8000/api/v1/conversation/history/$SESSION_ID?limit=100" > /dev/null
EOF

bash /tmp/test_load.sh
```

Expected: History retrieval should still be < 10ms even with 100 turns

---

## Step 8: Database Inspection

```bash
# Check database file
ls -lh /home/noob/ros2_navigation_project/backend/data/conversations.db

# Connect to SQLite
sqlite3 backend/data/conversations.db

# Run queries
SELECT COUNT(*) FROM conversations;
SELECT session_id, COUNT(*) as turns FROM conversations GROUP BY session_id;
SELECT * FROM conversations ORDER BY timestamp DESC LIMIT 5;

# Check indexes
.indexes conversations

# Exit
.quit
```

---

## Step 9: Logs Analysis

```bash
# Watch backend logs in real-time
tail -f /var/log/backend.log  # (if configured)

# Or check uvicorn logs
# Logs appear in the terminal where you ran uvicorn
```

Look for:
- `[SESSION_ID] Executing voice command`
- `Context retrieval: X.Xms`
- `Resolved spatial reference: ...`
- `Turn X stored`

---

## Step 10: Error Testing

### Test Invalid Session ID

```bash
curl "http://localhost:8000/api/v1/conversation/history/INVALID_ID"
```

Expected: Returns empty history (not an error)

### Test Missing Transcript

```bash
curl -X POST http://localhost:8000/api/v1/execute_voice_command \
  -H "Content-Type: application/json" \
  -d '{}'
```

Expected: HTTP 400 error

### Test Database Connection

```bash
# Stop backend
# Delete database file
rm backend/data/conversations.db

# Restart backend
# Try a command
curl -X POST http://localhost:8000/api/v1/execute_voice_command \
  -H "Content-Type: application/json" \
  -d '{"transcript": "test"}'
```

Expected: Database should be auto-created and command should work

---

## Troubleshooting

### Issue: aiosqlite not found

```bash
# Install it
python3 -m pip install aiosqlite==0.19.0

# Or use system package
sudo apt install python3-aiosqlite
```

### Issue: Import errors

```bash
# Check Python path
python3 -c "import sys; print(sys.path)"

# Run from backend directory
cd /home/noob/ros2_navigation_project/backend
python3 -m uvicorn app.main:app --reload
```

### Issue: Port 8000 already in use

```bash
# Find process
sudo lsof -i :8000

# Kill it
kill -9 <PID>

# Or use different port
python3 -m uvicorn app.main:app --reload --port 8001
```

### Issue: Gemini API errors

```bash
# Check API key
cat backend/.env | grep GEMINI_API_KEY

# Test Gemini directly
python3 -c "
import os
from dotenv import load_dotenv
load_dotenv()
api_key = os.getenv('GEMINI_API_KEY')
print(f'API Key: {api_key[:10]}...' if api_key else 'NOT FOUND')
"
```

---

## Success Criteria Checklist

- [ ] All 15 unit tests pass
- [ ] Backend starts without errors
- [ ] `/health` endpoint responds
- [ ] First conversation turn creates session
- [ ] Subsequent turns use same session
- [ ] "go there" resolves to last location
- [ ] "go back" resolves to second-last location
- [ ] Conversation history retrieval works
- [ ] Session summary returns correct stats
- [ ] Spatial references are cached
- [ ] End-to-end latency < 2 seconds
- [ ] Database persists after backend restart
- [ ] Session deletion works (soft delete)
- [ ] Multiple concurrent sessions work correctly
- [ ] Database file size is reasonable (<1MB for 100 turns)

---

## Next Steps After Testing

Once all tests pass:

1. **Document any issues found** in TESTING_RESULTS.md
2. **Measure actual performance** (latency, memory usage)
3. **Test with real voice input** (using existing frontend)
4. **Proceed to Day 4** (frontend integration)

---

## Quick Test Script

Save this as `backend/test_backend.sh`:

```bash
#!/bin/bash
set -e

echo "=========================================="
echo "Week 2 Backend Testing Script"
echo "=========================================="
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Test 1: Dependencies
echo "Test 1: Checking dependencies..."
python3 -c "import aiosqlite" 2>/dev/null && echo -e "${GREEN}✓ aiosqlite installed${NC}" || echo -e "${RED}✗ aiosqlite missing${NC}"

# Test 2: Imports
echo ""
echo "Test 2: Checking imports..."
python3 -c "from app.database.conversation_db import ConversationDatabase" 2>/dev/null && echo -e "${GREEN}✓ conversation_db imports${NC}" || echo -e "${RED}✗ conversation_db import failed${NC}"

# Test 3: Unit Tests
echo ""
echo "Test 3: Running unit tests..."
python3 -m pytest tests/test_conversation_db.py -v --tb=short 2>&1 | grep -E "(PASSED|FAILED|ERROR)" && echo -e "${GREEN}✓ Unit tests completed${NC}" || echo -e "${RED}✗ Unit tests failed${NC}"

# Test 4: Backend Health (if running)
echo ""
echo "Test 4: Checking backend health..."
curl -s http://localhost:8000/health >/dev/null 2>&1 && echo -e "${GREEN}✓ Backend is running${NC}" || echo -e "${RED}✗ Backend not running (start with: python3 -m uvicorn app.main:app --reload)${NC}"

echo ""
echo "=========================================="
echo "Testing complete!"
echo "=========================================="
```

Make it executable and run:
```bash
chmod +x backend/test_backend.sh
./backend/test_backend.sh
```

---

**Happy Testing! 🧪**
