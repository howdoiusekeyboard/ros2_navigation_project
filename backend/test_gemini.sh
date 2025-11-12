#!/bin/bash
# Test script for Gemini command parsing

BACKEND_URL="http://localhost:8000"

echo "=================================================="
echo "Gemini Command Parsing Test Suite"
echo "=================================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

test_command() {
    local command="$1"
    echo -e "${BLUE}Testing:${NC} \"$command\""
    echo ""

    curl -s -X POST "${BACKEND_URL}/api/v1/parse" \
        -H "Content-Type: application/json" \
        -d "{\"command\": \"$command\"}" | jq '.'

    echo ""
    echo "---"
    echo ""
}

test_execute() {
    local command="$1"
    echo -e "${BLUE}Executing:${NC} \"$command\""
    echo ""

    curl -s -X POST "${BACKEND_URL}/api/v1/execute_voice_command" \
        -H "Content-Type: application/json" \
        -d "{\"transcript\": \"$command\", \"use_turtlesim\": true}" | jq '.'

    echo ""
    echo "---"
    echo ""
}

# Check if backend is running
echo "Checking backend health..."
curl -s "${BACKEND_URL}/health" | jq '.checks' || {
    echo "❌ Backend not running! Start it with: cd backend && ./run.sh"
    exit 1
}

echo ""
echo "=================================================="
echo "Test 1: Simple Commands (Regex Path)"
echo "=================================================="
echo ""

test_command "stop"
test_command "go forward"
test_command "turn left"

echo ""
echo "=================================================="
echo "Test 2: Complex Commands (Gemini Path)"
echo "=================================================="
echo ""

test_command "spin in a circle"
test_command "move forward 2 meters"
test_command "rotate 90 degrees clockwise"

echo ""
echo "=================================================="
echo "Test 3: Navigation Commands (Week 4)"
echo "=================================================="
echo ""

test_command "go to coordinates 3, 2"
test_command "navigate to position x=5 y=3"

echo ""
echo "=================================================="
echo "Test 4: Safety Validation"
echo "=================================================="
echo ""

test_command "move forward at 100 meters per second"
test_command "spin super fast"

echo ""
echo "=================================================="
echo "Test 5: Complete Pipeline (with Turtlesim)"
echo "=================================================="
echo ""
echo "NOTE: Make sure turtlesim is running:"
echo "  ros2 run turtlesim turtlesim_node"
echo ""
read -p "Press enter to continue..."

test_execute "spin in a circle"
sleep 3
test_execute "stop"

echo ""
echo "=================================================="
echo "✅ All tests complete!"
echo "=================================================="
echo ""
echo "Check backend logs for detailed parsing information."
echo ""
