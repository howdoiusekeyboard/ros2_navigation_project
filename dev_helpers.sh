#!/bin/bash
# Development Helper Commands
# Provides convenient shortcuts for common development tasks

set -e

PROJECT_DIR="$HOME/ros2_navigation_project"

case "$1" in
    clean)
        echo "🧹 Cleaning all artifacts..."
        cd "$PROJECT_DIR"

        # Kill running processes first
        echo "Stopping all services..."
        pkill -f "uvicorn|turtlesim|rosbridge|vite" 2>/dev/null || true
        sleep 1

        # Clean build artifacts
        echo "Removing build/, install/, log/ directories..."
        rm -rf build/ install/ log/

        echo "✅ Cleaned successfully"
        ;;

    build)
        echo "🔨 Building workspace..."
        cd "$PROJECT_DIR"

        # Source ROS2 first
        if [ -f "/opt/ros/humble/setup.bash" ]; then
            source /opt/ros/humble/setup.bash
        fi

        # Build with symlink install
        colcon build --symlink-install

        if [ $? -eq 0 ]; then
            echo "✅ Build completed successfully"
            echo ""
            echo "Source the workspace with:"
            echo "  source install/setup.bash"
        else
            echo "❌ Build failed"
            exit 1
        fi
        ;;

    rebuild)
        echo "🔄 Clean rebuild..."
        cd "$PROJECT_DIR"

        # Clean first
        echo "Step 1: Cleaning..."
        rm -rf build/ install/ log/

        # Source ROS2
        if [ -f "/opt/ros/humble/setup.bash" ]; then
            source /opt/ros/humble/setup.bash
        fi

        # Build
        echo ""
        echo "Step 2: Building..."
        colcon build --symlink-install

        if [ $? -eq 0 ]; then
            echo "✅ Rebuild completed successfully"
        else
            echo "❌ Rebuild failed"
            exit 1
        fi
        ;;

    kill)
        echo "🛑 Stopping all services..."

        # Graceful termination
        pkill -TERM -f "uvicorn app.main" 2>/dev/null && echo "  Stopping backend..." || true
        pkill -TERM -f "turtlesim_node" 2>/dev/null && echo "  Stopping turtlesim..." || true
        pkill -TERM -f "rosbridge" 2>/dev/null && echo "  Stopping rosbridge..." || true
        pkill -TERM -f "vite" 2>/dev/null && echo "  Stopping frontend..." || true

        # Wait for graceful shutdown
        sleep 1

        # Force kill if needed
        pkill -KILL -f "uvicorn|turtlesim|rosbridge|vite" 2>/dev/null || true

        echo "✅ All services stopped"
        ;;

    status)
        echo "📊 Service Status:"
        echo ""

        # Check backend
        echo "Port 8000 (Backend):"
        if lsof -i :8000 2>/dev/null | grep -q LISTEN; then
            lsof -i :8000 2>/dev/null | grep LISTEN
        else
            echo "  ✅ Free"
        fi

        echo ""

        # Check rosbridge
        echo "Port 9090 (ROS Bridge):"
        if lsof -i :9090 2>/dev/null | grep -q LISTEN; then
            lsof -i :9090 2>/dev/null | grep LISTEN
        else
            echo "  ✅ Free"
        fi

        echo ""

        # Check frontend
        echo "Port 5173 (Frontend):"
        if lsof -i :5173 2>/dev/null | grep -q LISTEN; then
            lsof -i :5173 2>/dev/null | grep LISTEN
        else
            echo "  ✅ Free"
        fi

        echo ""

        # Check processes
        echo "Running processes:"
        ps aux | grep -E "(uvicorn|turtlesim|rosbridge|vite)" | grep -v grep || echo "  ✅ No services running"
        ;;

    logs)
        echo "📜 Viewing recent logs..."
        cd "$PROJECT_DIR"

        if [ -d "log" ]; then
            echo ""
            echo "Latest build log:"
            find log -name "*.log" -type f -printf '%T@ %p\n' | sort -n | tail -1 | cut -d' ' -f2- | xargs tail -50
        else
            echo "No log directory found. Run 'colcon build' first."
        fi
        ;;

    *)
        echo "Development Helper Commands"
        echo "============================"
        echo ""
        echo "Usage: ./dev_helpers.sh {command}"
        echo ""
        echo "Commands:"
        echo "  clean    - Remove build artifacts and kill processes"
        echo "  build    - Run colcon build --symlink-install"
        echo "  rebuild  - Clean + build (full rebuild)"
        echo "  kill     - Stop all running services"
        echo "  status   - Check port usage and running processes"
        echo "  logs     - View recent build logs"
        echo ""
        echo "Examples:"
        echo "  ./dev_helpers.sh clean    # Clean everything"
        echo "  ./dev_helpers.sh rebuild  # Full rebuild"
        echo "  ./dev_helpers.sh status   # Check what's running"
        ;;
esac
