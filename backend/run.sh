#!/bin/bash

# Backend Server Startup Script

set -e

echo "========================================="
echo "Voice-Controlled Robot Backend Server"
echo "========================================="

# Source ROS2 Humble (required for rclpy)
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "Sourcing ROS2 Humble..."
    source /opt/ros/humble/setup.bash
else
    echo "ERROR: ROS2 Humble not found at /opt/ros/humble"
    echo "Install with: sudo apt install ros-humble-desktop"
    exit 1
fi

# Check if virtual environment exists and is valid
if [ ! -f "venv/bin/activate" ]; then
    echo "Virtual environment not found or invalid. Creating..."
    rm -rf venv
    python3 -m venv venv --system-site-packages
    echo "Installing dependencies..."
    source venv/bin/activate
    pip install --upgrade pip
    pip install -r requirements.txt
else
    source venv/bin/activate
fi

# Check if .env exists
if [ ! -f ".env" ]; then
    echo "ERROR: .env file not found"
    echo "Please copy .env.example to .env and add your API keys"
    echo "  cp .env.example .env"
    echo "  nano .env  # Add OPENAI_API_KEY and GEMINI_API_KEY"
    exit 1
fi

# Check if API keys are set
if ! grep -q "OPENAI_API_KEY=sk-" .env && ! grep -q "OPENAI_API_KEY=\$" .env; then
    echo "WARNING: OPENAI_API_KEY not set in .env"
    echo "Get your key from: https://platform.openai.com/api-keys"
fi

if ! grep -q "GEMINI_API_KEY=AI" .env && ! grep -q "GEMINI_API_KEY=\$" .env; then
    echo "WARNING: GEMINI_API_KEY not set in .env"
    echo "Get your key from: https://aistudio.google.com/app/apikey"
fi

echo ""
echo "Starting FastAPI server..."
echo "API Documentation: http://localhost:8000/docs"
echo "Health Check: http://localhost:8000/health"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Trap Ctrl+C for clean shutdown
trap 'echo ""; echo "Shutting down backend server..."; exit 0' INT TERM

# Run server without --reload to fix double Ctrl+C issue
# Note: Auto-reload disabled for clean signal handling
# For development with auto-reload: add --reload flag (requires 2x Ctrl+C)
python3 -m uvicorn app.main:app --host 0.0.0.0 --port 8000
