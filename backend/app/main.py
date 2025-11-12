"""
FastAPI Backend Server for Voice-Controlled Robot System

Provides:
- Whisper API speech-to-text transcription
- Gemini command parsing
- ROS2 integration for robot control
- WebSocket for real-time communication
- Database for context/memory storage
"""

from fastapi import FastAPI, UploadFile, File, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from loguru import logger
import sys
from datetime import datetime

from app.config import settings
from app.ros2_client.robot_controller import ros2_manager
from app.services.gemini_service import initialize_gemini, get_parser

# Configure logging
logger.remove()
logger.add(
    sys.stdout,
    level=settings.log_level,
    format="<green>{time:YYYY-MM-DD HH:mm:ss}</green> | <level>{level: <8}</level> | <cyan>{name}</cyan>:<cyan>{function}</cyan> - <level>{message}</level>"
)

# Initialize FastAPI app
app = FastAPI(
    title="Voice-Controlled Robot API",
    description="Backend server for LLM-based voice control of mobile robots",
    version="1.0.0",
    docs_url="/docs",  # Swagger UI at /docs
    redoc_url="/redoc"  # ReDoc at /redoc
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ==================== Health & Status ====================

@app.get("/")
async def root():
    """Root endpoint - API information"""
    return {
        "service": "Voice-Controlled Robot API",
        "version": "1.0.0",
        "status": "operational",
        "timestamp": datetime.utcnow().isoformat(),
        "endpoints": {
            "health": "/health",
            "docs": "/docs",
            "transcribe": "/api/v1/transcribe",
            "parse_command": "/api/v1/parse",
            "websocket": "/ws"
        }
    }


@app.get("/health")
async def health_check():
    """Health check endpoint for monitoring"""
    try:
        # Check ROS2
        ros2_status = "not_initialized"
        try:
            if ros2_manager.robot_controller:
                ros2_status = "connected"
        except:
            ros2_status = "error"

        # Check Gemini
        gemini_status = "not_initialized"
        try:
            from app.services.gemini_service import gemini_parser
            if gemini_parser:
                gemini_status = "connected"
        except:
            gemini_status = "error"

        # Check API keys
        api_status = "ok"
        if not settings.openai_api_key:
            api_status = "missing_openai_key"
        elif not settings.gemini_api_key:
            api_status = "missing_gemini_key"

        return {
            "status": "healthy",
            "timestamp": datetime.utcnow().isoformat(),
            "checks": {
                "database": "not_implemented",  # TODO Week 3
                "ros2": ros2_status,
                "gemini": gemini_status,
                "apis": api_status
            }
        }
    except Exception as e:
        logger.error(f"Health check failed: {e}")
        raise HTTPException(status_code=500, detail="Service unhealthy")


# ==================== Speech-to-Text (Whisper) ====================

@app.post("/api/v1/transcribe")
async def transcribe_audio(audio: UploadFile = File(...)):
    """
    Transcribe audio file using OpenAI Whisper API

    Args:
        audio: Audio file (mp3, mp4, mpeg, mpga, m4a, wav, webm)

    Returns:
        {
            "transcript": str,
            "language": str,
            "confidence": float,
            "duration": float
        }
    """
    try:
        logger.info(f"Received audio file: {audio.filename}, size: {audio.size} bytes")

        # Import OpenAI client
        from openai import OpenAI
        client = OpenAI(api_key=settings.openai_api_key)

        # Read audio file
        audio_data = await audio.read()

        # Save temporarily (Whisper API requires file path or file object)
        import tempfile
        with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as temp_audio:
            temp_audio.write(audio_data)
            temp_audio_path = temp_audio.name

        # Call Whisper API
        with open(temp_audio_path, "rb") as audio_file:
            transcript_response = client.audio.transcriptions.create(
                model="whisper-1",
                file=audio_file,
                response_format="verbose_json"  # Get detailed response
            )

        # Clean up temp file
        import os
        os.unlink(temp_audio_path)

        # Extract results
        result = {
            "transcript": transcript_response.text,
            "language": getattr(transcript_response, 'language', 'en'),
            "confidence": 1.0,  # Whisper doesn't provide confidence, use 1.0
            "duration": getattr(transcript_response, 'duration', 0.0)
        }

        logger.info(f"Transcription successful: '{result['transcript']}'")
        return result

    except Exception as e:
        logger.error(f"Transcription failed: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Transcription failed: {str(e)}"
        )


# ==================== Command Parsing (Gemini) ====================

@app.post("/api/v1/parse")
async def parse_command(data: dict):
    """
    Parse natural language command into structured robot action

    Args:
        {
            "command": str,  # User command text
            "context": dict  # Optional robot state/history
        }

    Returns:
        {
            "action": str,  # "twist", "navigate", "stop", etc.
            "parameters": dict,
            "confidence": float,
            "reasoning": str  # Why this interpretation
        }
    """
    try:
        command = data.get("command")
        context = data.get("context", {})

        if not command:
            raise HTTPException(status_code=400, detail="Missing 'command' field")

        logger.info(f"Parsing command: '{command}'")

        # Get Gemini parser
        parser = get_parser()

        # Parse command
        result = parser.parse_command(command, context)

        # Return structured result
        return {
            "action": result.action.value,
            "parameters": result.parameters,
            "confidence": result.confidence,
            "reasoning": result.reasoning
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Command parsing failed: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Parsing failed: {str(e)}"
        )


# ==================== WebSocket for Real-Time Communication ====================

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """
    WebSocket endpoint for real-time voice streaming

    Protocol:
    1. Client connects
    2. Client sends audio chunks
    3. Server transcribes with Whisper
    4. Server parses with Gemini
    5. Server sends result back
    """
    await websocket.accept()
    logger.info(f"WebSocket client connected: {websocket.client}")

    try:
        while True:
            # Receive message from client
            data = await websocket.receive_json()

            message_type = data.get("type")

            if message_type == "ping":
                # Keepalive
                await websocket.send_json({"type": "pong"})

            elif message_type == "audio_chunk":
                # TODO: Handle streaming audio
                await websocket.send_json({
                    "type": "error",
                    "message": "Audio streaming not yet implemented"
                })

            else:
                await websocket.send_json({
                    "type": "error",
                    "message": f"Unknown message type: {message_type}"
                })

    except WebSocketDisconnect:
        logger.info(f"WebSocket client disconnected: {websocket.client}")
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        await websocket.close()


# ==================== Robot Control (ROS2) ====================

@app.post("/api/v1/robot/twist")
async def robot_twist(data: dict):
    """
    Send Twist (velocity) command to robot

    Request body:
        {
            "linear_x": float,  # m/s
            "angular_z": float,  # rad/s
            "use_turtlesim": bool  # Optional: use /turtle1/cmd_vel
        }
    """
    try:
        controller = ros2_manager.get_controller()

        linear_x = data.get('linear_x', 0.0)
        angular_z = data.get('angular_z', 0.0)
        use_turtlesim = data.get('use_turtlesim', False)

        controller.publish_twist(
            linear_x=linear_x,
            angular_z=angular_z,
            use_turtlesim=use_turtlesim
        )

        return {
            "status": "success",
            "command": {
                "linear_x": linear_x,
                "angular_z": angular_z,
                "topic": "/turtle1/cmd_vel" if use_turtlesim else "/cmd_vel"
            }
        }

    except Exception as e:
        logger.error(f"Robot control failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/v1/robot/stop")
async def robot_stop(data: dict = {}):
    """Emergency stop - set all velocities to zero"""
    try:
        controller = ros2_manager.get_controller()
        use_turtlesim = data.get('use_turtlesim', False)

        controller.stop(use_turtlesim=use_turtlesim)

        return {"status": "stopped"}

    except Exception as e:
        logger.error(f"Emergency stop failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/v1/robot/state")
async def robot_state():
    """Get current robot state (pose, velocity)"""
    try:
        controller = ros2_manager.get_controller()

        return {
            "pose": controller.get_current_pose(),
            "velocity": controller.get_current_velocity()
        }

    except Exception as e:
        logger.error(f"Failed to get robot state: {e}")
        raise HTTPException(status_code=500, detail=str(e))


# ==================== Voice Command Pipeline ====================

@app.post("/api/v1/execute_voice_command")
async def execute_voice_command(data: dict):
    """
    Complete voice command pipeline: Transcript → Gemini → Robot Control

    Request body:
        {
            "transcript": str,  # Voice command text from Whisper
            "use_turtlesim": bool,  # Optional: use turtlesim topics
            "context": dict  # Optional: robot state/history
        }

    Returns:
        {
            "transcript": str,
            "parsed_command": {...},
            "execution_status": str,
            "latency": {
                "gemini_ms": float,
                "execution_ms": float,
                "total_ms": float
            }
        }
    """
    import time
    start_time = time.time()

    try:
        transcript = data.get('transcript')
        use_turtlesim = data.get('use_turtlesim', False)
        context = data.get('context', {})

        if not transcript:
            raise HTTPException(status_code=400, detail="Missing 'transcript' field")

        logger.info(f"Executing voice command: '{transcript}'")

        # Step 1: Parse with Gemini
        gemini_start = time.time()
        parser = get_parser()
        parsed_command = parser.parse_command(transcript, context)
        gemini_time = (time.time() - gemini_start) * 1000

        logger.info(f"Parsed: action={parsed_command.action.value}, confidence={parsed_command.confidence:.2f}")

        # Step 2: Execute on robot
        execution_start = time.time()
        controller = ros2_manager.get_controller()
        execution_status = "unknown"

        if parsed_command.action.value == "twist":
            # Send velocity command
            params = parsed_command.parameters
            controller.publish_twist(
                linear_x=params.get('linear_x', 0.0),
                angular_z=params.get('angular_z', 0.0),
                use_turtlesim=use_turtlesim
            )
            execution_status = "executed"

        elif parsed_command.action.value == "stop":
            # Emergency stop
            controller.stop(use_turtlesim=use_turtlesim)
            execution_status = "stopped"

        elif parsed_command.action.value == "move_forward":
            # Move forward (using constant velocity for duration)
            params = parsed_command.parameters
            distance = params.get('distance', 1.0)
            # Approximate: distance / speed = time
            speed = 0.15  # m/s
            duration = abs(distance / speed)
            controller.move_forward(speed=speed, duration=duration)
            execution_status = "moving_forward"

        elif parsed_command.action.value == "move_backward":
            # Move backward
            params = parsed_command.parameters
            distance = params.get('distance', 1.0)
            speed = 0.15
            duration = abs(distance / speed)
            controller.publish_twist(linear_x=-speed, angular_z=0.0, use_turtlesim=use_turtlesim)
            execution_status = "moving_backward"

        elif parsed_command.action.value == "rotate":
            # Rotate in place
            params = parsed_command.parameters
            angle = params.get('angle', 1.57)  # Default 90 degrees
            angular_speed = 1.0  # rad/s
            duration = abs(angle / angular_speed)
            controller.rotate(angular_speed=angular_speed if angle > 0 else -angular_speed, duration=duration)
            execution_status = "rotating"

        elif parsed_command.action.value == "navigate":
            # Navigation (Week 4 - not yet implemented)
            execution_status = "navigation_not_implemented"
            logger.warning("Navigate action requires Nav2 integration (Week 4)")

        else:
            execution_status = "unknown_action"
            logger.warning(f"Unknown action: {parsed_command.action}")

        execution_time = (time.time() - execution_start) * 1000
        total_time = (time.time() - start_time) * 1000

        return {
            "transcript": transcript,
            "parsed_command": {
                "action": parsed_command.action.value,
                "parameters": parsed_command.parameters,
                "confidence": parsed_command.confidence,
                "reasoning": parsed_command.reasoning
            },
            "execution_status": execution_status,
            "latency": {
                "gemini_ms": round(gemini_time, 2),
                "execution_ms": round(execution_time, 2),
                "total_ms": round(total_time, 2)
            }
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Voice command execution failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))


# ==================== Startup & Shutdown Events ====================

@app.on_event("startup")
async def startup_event():
    """Initialize services on startup"""
    logger.info("=" * 60)
    logger.info("Voice-Controlled Robot Backend Server Starting")
    logger.info("=" * 60)
    logger.info(f"Host: {settings.host}:{settings.port}")
    logger.info(f"Debug Mode: {settings.debug}")
    logger.info(f"CORS Origins: {settings.cors_origins}")
    logger.info(f"Database: {settings.database_url}")
    logger.info("=" * 60)

    # Initialize ROS2
    try:
        logger.info("Initializing ROS2 Manager...")
        ros2_manager.initialize()
        logger.info("✅ ROS2 Manager initialized successfully")
    except Exception as e:
        logger.error(f"❌ Failed to initialize ROS2: {e}")
        logger.warning("Backend will continue without ROS2 support")

    # Initialize Gemini
    try:
        if settings.gemini_api_key:
            logger.info("Initializing Gemini Command Parser...")
            initialize_gemini(api_key=settings.gemini_api_key, model_name="gemini-2.0-flash-exp")
            logger.info("✅ Gemini Command Parser initialized successfully")
        else:
            logger.warning("⚠️ GEMINI_API_KEY not set - command parsing will fail")
    except Exception as e:
        logger.error(f"❌ Failed to initialize Gemini: {e}")
        logger.warning("Backend will continue without Gemini support")

    # TODO: Initialize:
    # - Database connection


@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup on shutdown"""
    logger.info("=" * 60)
    logger.info("Voice-Controlled Robot Backend Server Shutting Down")
    logger.info("=" * 60)

    # Shutdown ROS2
    try:
        logger.info("Shutting down ROS2 Manager...")
        ros2_manager.shutdown()
        logger.info("✅ ROS2 Manager shut down successfully")
    except Exception as e:
        logger.error(f"❌ Error shutting down ROS2: {e}")

    # TODO: Cleanup:
    # - Close database connections
    # - Save state if needed


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "app.main:app",
        host=settings.host,
        port=settings.port,
        reload=settings.debug,  # Auto-reload on code changes in debug mode
        log_level=settings.log_level.lower()
    )
