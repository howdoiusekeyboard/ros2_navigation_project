#!/bin/bash
# Kill All Services Helper Script
# Gracefully stops all ROS2/backend/frontend processes

set +e  # Don't exit on errors

echo "🧹 Cleaning up all services..."
echo ""

# Function to check if process is running
check_process() {
    pgrep -f "$1" >/dev/null 2>&1
}

# Kill processes with increasing force
echo "Sending SIGTERM to processes..."
pkill -TERM -f "uvicorn app.main" 2>/dev/null && echo "  ✓ Terminating backend (uvicorn)"
pkill -TERM -f "turtlesim_node" 2>/dev/null && echo "  ✓ Terminating turtlesim"
pkill -TERM -f "rosbridge_websocket" 2>/dev/null && echo "  ✓ Terminating rosbridge"
pkill -TERM -f "rosapi_node" 2>/dev/null && echo "  ✓ Terminating rosapi"
pkill -TERM -f "vite" 2>/dev/null && echo "  ✓ Terminating vite dev server"

# Wait for graceful shutdown
echo ""
echo "Waiting for graceful shutdown (2 seconds)..."
sleep 2

# Force kill any remaining processes
echo ""
echo "Force killing any remaining processes..."
pkill -KILL -f "uvicorn app.main" 2>/dev/null
pkill -KILL -f "turtlesim_node" 2>/dev/null
pkill -KILL -f "rosbridge" 2>/dev/null
pkill -KILL -f "vite" 2>/dev/null

# Free up ports (requires sudo)
echo ""
echo "Freeing up ports (may require sudo)..."
if command -v fuser >/dev/null 2>&1; then
    sudo fuser -k 8000/tcp 2>/dev/null && echo "  ✓ Freed port 8000 (backend)" || echo "  ✓ Port 8000 already free"
    sudo fuser -k 9090/tcp 2>/dev/null && echo "  ✓ Freed port 9090 (rosbridge)" || echo "  ✓ Port 9090 already free"
    sudo fuser -k 5173/tcp 2>/dev/null && echo "  ✓ Freed port 5173 (frontend)" || echo "  ✓ Port 5173 already free"
else
    echo "  ⚠ fuser not available, skipping port cleanup"
fi

echo ""
echo "✅ All services stopped"
echo ""
echo "Verify with: lsof -i :8000,9090,5173"
