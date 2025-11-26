#!/bin/bash

# Backend Server Startup Script

set -e

echo "========================================="
echo "Voice-Controlled Robot Backend Server"
echo "========================================="

# Check if virtual environment exists and is valid
if [ ! -f "venv/bin/activate" ]; then
    echo "Virtual environment not found or invalid. Creating..."
    rm -rf venv
    python3 -m venv venv
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

# Run server
python3 -m uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
