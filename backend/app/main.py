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
from typing import List, Dict, Any, Optional

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
    Complete voice command pipeline with conversation memory:
    Transcript → Context Retrieval → Gemini → Robot Control → Memory Storage

    Request body:
        {
            "transcript": str,  # Voice command text from Whisper
            "use_turtlesim": bool,  # Optional: use turtlesim topics
            "session_id": str,  # Optional: conversation session ID
            "context": dict  # Optional: additional context (legacy)
        }

    Returns:
        {
            "transcript": str,
            "parsed_command": {...},
            "execution_status": str,
            "session_id": str,
            "latency": {
                "gemini_ms": float,
                "execution_ms": float,
                "memory_ms": float,
                "total_ms": float
            }
        }
    """
    import time
    from app.database.conversation_db import get_conversation_db
    from app.services.context_builder import get_context_builder

    start_time = time.time()

    try:
        transcript = data.get('transcript')
        use_turtlesim = data.get('use_turtlesim', False)
        session_id = data.get('session_id')

        if not transcript:
            raise HTTPException(status_code=400, detail="Missing 'transcript' field")

        # Get or create session
        conversation_db = get_conversation_db()
        await conversation_db.connect()

        if not session_id:
            session_id = conversation_db.generate_session_id()
            logger.info(f"Created new session: {session_id}")

        logger.info(f"[{session_id}] Executing voice command: '{transcript}'")

        # Step 1: Retrieve conversation context
        memory_start = time.time()
        context_builder = get_context_builder(conversation_db)

        # Build context string for LLM
        conversation_context = await context_builder.build_context_for_llm(
            session_id=session_id,
            max_turns=5,
            include_spatial_refs=True
        )

        # Check for spatial references in user input
        has_spatial_ref = context_builder.has_spatial_reference(transcript)
        explicit_coords = context_builder.extract_coordinates(transcript)

        memory_time_1 = (time.time() - memory_start) * 1000
        logger.debug(f"Context retrieval: {memory_time_1:.2f}ms")

        # Step 2: Parse with Gemini (with context)
        gemini_start = time.time()
        parser = get_parser()
        parsed_command = parser.parse_command(transcript, context=conversation_context)
        gemini_time = (time.time() - gemini_start) * 1000

        logger.info(
            f"[{session_id}] Parsed: action={parsed_command.action.value}, "
            f"confidence={parsed_command.confidence:.2f}"
        )

        # Step 2.5: Resolve spatial references if needed
        resolved_location = None
        if has_spatial_ref and not explicit_coords:
            resolution = await context_builder.resolve_spatial_reference(
                reference_text=transcript,
                session_id=session_id
            )
            if resolution:
                x, y, description = resolution
                logger.info(f"[{session_id}] Resolved spatial reference: {description}")

                # Update parsed command with resolved coordinates
                if parsed_command.action.value == "navigate" or not parsed_command.parameters.get('x'):
                    parsed_command.parameters['x'] = x
                    parsed_command.parameters['y'] = y
                    resolved_location = (x, y, description)

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

        # Step 3: Store conversation turn in database
        memory_store_start = time.time()

        # Extract location info
        location_x = None
        location_y = None
        location_label = None

        if parsed_command.action.value == "navigate":
            location_x = parsed_command.parameters.get('x')
            location_y = parsed_command.parameters.get('y')
            # Extract location labels from transcript
            location_mentions = context_builder.extract_location_mentions(transcript)
            if location_mentions:
                location_label = location_mentions[0]

        # Store the conversation turn
        robot_response = f"{execution_status}: {parsed_command.reasoning or ''}"
        if resolved_location:
            robot_response += f" (resolved to: {resolved_location[2]})"

        await conversation_db.add_turn(
            session_id=session_id,
            user_input=transcript,
            robot_response=robot_response,
            action_type=parsed_command.action.value,
            location_x=location_x,
            location_y=location_y,
            location_label=location_label,
            confidence=parsed_command.confidence,
            latency_ms=int(gemini_time),
            metadata={
                "execution_status": execution_status,
                "use_turtlesim": use_turtlesim,
                "resolved_reference": resolved_location[2] if resolved_location else None
            }
        )

        memory_time_2 = (time.time() - memory_store_start) * 1000
        total_memory_time = memory_time_1 + memory_time_2
        total_time = (time.time() - start_time) * 1000

        logger.info(
            f"[{session_id}] Complete: total={total_time:.0f}ms "
            f"(gemini={gemini_time:.0f}ms, memory={total_memory_time:.0f}ms)"
        )

        return {
            "transcript": transcript,
            "parsed_command": {
                "action": parsed_command.action.value,
                "parameters": parsed_command.parameters,
                "confidence": parsed_command.confidence,
                "reasoning": parsed_command.reasoning
            },
            "execution_status": execution_status,
            "session_id": session_id,  # Return session_id to client
            "resolved_reference": resolved_location[2] if resolved_location else None,
            "latency": {
                "gemini_ms": round(gemini_time, 2),
                "execution_ms": round(execution_time, 2),
                "memory_ms": round(total_memory_time, 2),
                "total_ms": round(total_time, 2)
            }
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Voice command execution failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))


# ==================== Conversation Memory Endpoints ====================

@app.get("/api/v1/conversation/history/{session_id}")
async def get_conversation_history(
    session_id: str,
    limit: int = 20,
    include_metadata: bool = False
):
    """
    Get conversation history for a session.

    Query Parameters:
        - limit: Maximum number of turns to retrieve (default: 20)
        - include_metadata: Include metadata field (default: false)

    Returns:
        {
            "session_id": str,
            "history": [
                {
                    "id": int,
                    "timestamp": str,
                    "turn_number": int,
                    "user_input": str,
                    "robot_response": str,
                    "action_type": str,
                    "location": {...},
                    "confidence": float
                },
                ...
            ],
            "turn_count": int
        }
    """
    try:
        from app.database.conversation_db import get_conversation_db

        conversation_db = get_conversation_db()
        await conversation_db.connect()

        history = await conversation_db.get_history(
            session_id=session_id,
            limit=limit,
            include_metadata=include_metadata
        )

        # Format response
        formatted_history = []
        for turn in history:
            formatted_turn = {
                "id": turn["id"],
                "timestamp": turn["timestamp"],
                "turn_number": turn["turn_number"],
                "user_input": turn["user_input"],
                "robot_response": turn["robot_response"],
                "action_type": turn["action_type"],
                "location": {
                    "x": turn["location_x"],
                    "y": turn["location_y"],
                    "label": turn["location_label"]
                } if turn["location_x"] is not None else None,
                "confidence": turn["confidence"],
                "latency_ms": turn["latency_ms"]
            }

            if include_metadata and "metadata" in turn:
                formatted_turn["metadata"] = turn["metadata"]

            formatted_history.append(formatted_turn)

        return {
            "session_id": session_id,
            "history": formatted_history,
            "turn_count": len(formatted_history)
        }

    except Exception as e:
        logger.error(f"Failed to retrieve conversation history: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/v1/conversation/sessions")
async def get_recent_sessions(limit: int = 10):
    """
    Get list of recent conversation sessions.

    Query Parameters:
        - limit: Maximum number of sessions (default: 10)

    Returns:
        {
            "sessions": [
                {
                    "session_id": str,
                    "turn_count": int,
                    "start_time": str,
                    "end_time": str,
                    "avg_confidence": float,
                    "avg_latency_ms": float,
                    "navigation_count": int
                },
                ...
            ]
        }
    """
    try:
        from app.database.conversation_db import get_conversation_db

        conversation_db = get_conversation_db()
        await conversation_db.connect()

        sessions = await conversation_db.get_recent_sessions(limit=limit)

        return {"sessions": sessions}

    except Exception as e:
        logger.error(f"Failed to retrieve sessions: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/v1/conversation/summary/{session_id}")
async def get_session_summary(session_id: str):
    """
    Get summary statistics for a conversation session.

    Returns:
        {
            "session_id": str,
            "turn_count": int,
            "start_time": str,
            "end_time": str,
            "avg_confidence": float,
            "avg_latency_ms": float,
            "navigation_count": int
        }
    """
    try:
        from app.database.conversation_db import get_conversation_db

        conversation_db = get_conversation_db()
        await conversation_db.connect()

        summary = await conversation_db.get_session_summary(session_id)

        if not summary:
            raise HTTPException(status_code=404, detail=f"Session {session_id} not found")

        return summary

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Failed to retrieve session summary: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.delete("/api/v1/conversation/session/{session_id}")
async def delete_conversation_session(session_id: str):
    """
    Delete a conversation session (soft delete).

    Returns:
        {
            "message": str,
            "deleted": bool
        }
    """
    try:
        from app.database.conversation_db import get_conversation_db

        conversation_db = get_conversation_db()
        await conversation_db.connect()

        deleted = await conversation_db.delete_session(session_id)

        if not deleted:
            raise HTTPException(status_code=404, detail=f"Session {session_id} not found")

        return {
            "message": f"Session {session_id} deleted successfully",
            "deleted": True
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Failed to delete session: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/v1/conversation/spatial_refs/{session_id}")
async def get_spatial_references(session_id: str):
    """
    Get all spatial references (labeled locations) for a session.

    Returns:
        {
            "session_id": str,
            "references": {
                "kitchen": {"x": 2.0, "y": 3.0},
                "bedroom": {"x": 5.0, "y": 1.0},
                ...
            }
        }
    """
    try:
        from app.database.conversation_db import get_conversation_db

        conversation_db = get_conversation_db()
        await conversation_db.connect()

        refs = await conversation_db.get_spatial_references(session_id)

        # Format for JSON response
        formatted_refs = {
            label: {"x": x, "y": y}
            for label, (x, y, _) in refs.items()
        }

        return {
            "session_id": session_id,
            "references": formatted_refs
        }

    except Exception as e:
        logger.error(f"Failed to retrieve spatial references: {e}")
        raise HTTPException(status_code=500, detail=str(e))


# ==================== Navigation Decision Endpoints (Week 3) ====================

@app.post("/api/v1/navigation/decisions/sync")
async def sync_navigation_decisions(decisions: List[Dict[str, Any]]):
    """
    Sync navigation decisions from ROS2 XAI Navigator node.

    Called periodically by BackendSync service.

    Request body:
        [
            {
                "decision_id": int,
                "decision_type": str,
                "timestamp": float,
                "session_id": str,
                "goal": {"x": float, "y": float},
                "data": {...}
            },
            ...
        ]

    Returns:
        {
            "success": bool,
            "synced_count": int
        }
    """
    try:
        # Validate that we have a list
        if not isinstance(decisions, list):
            raise HTTPException(status_code=422, detail="Request body must be a list of decisions")

        # Allow empty list (no decisions to sync)
        if len(decisions) == 0:
            logger.debug("Sync called with empty decision list")
            return {
                "success": True,
                "synced_count": 0
            }

        from app.database.navigation_db import get_navigation_db

        nav_db = await get_navigation_db()
        count = await nav_db.store_decisions_batch(decisions)

        logger.info(f"Synced {count} navigation decisions")
        return {
            "success": True,
            "synced_count": count
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Failed to sync navigation decisions: {e}")
        logger.exception("Full traceback:")  # Log full traceback for debugging
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/v1/navigation/decisions")
async def get_navigation_decisions(
    limit: int = 50,
    session_id: str = None
):
    """
    Get recent navigation decisions.

    Query Parameters:
        - limit: Maximum decisions to return (default: 50)
        - session_id: Optional filter by session

    Returns:
        {
            "success": bool,
            "decisions": [...],
            "count": int
        }
    """
    try:
        from app.database.navigation_db import get_navigation_db

        nav_db = await get_navigation_db()
        decisions = await nav_db.get_recent_decisions(limit, session_id)

        return {
            "success": True,
            "decisions": decisions,
            "count": len(decisions)
        }

    except Exception as e:
        logger.error(f"Failed to get navigation decisions: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/v1/navigation/decisions/type/{decision_type}")
async def get_decisions_by_type(decision_type: str, limit: int = 100):
    """
    Get decisions of a specific type.

    Path Parameters:
        - decision_type: Type to filter (goal_sent, goal_reached, path_changed, etc.)

    Query Parameters:
        - limit: Maximum decisions to return (default: 100)

    Returns:
        {
            "success": bool,
            "decision_type": str,
            "decisions": [...],
            "count": int
        }
    """
    try:
        from app.database.navigation_db import get_navigation_db

        nav_db = await get_navigation_db()
        decisions = await nav_db.get_decisions_by_type(decision_type, limit)

        return {
            "success": True,
            "decision_type": decision_type,
            "decisions": decisions,
            "count": len(decisions)
        }

    except Exception as e:
        logger.error(f"Failed to get decisions by type: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/v1/navigation/path_changes")
async def get_path_changes(limit: int = 50):
    """
    Get recent path change events.

    Returns:
        {
            "success": bool,
            "path_changes": [...],
            "count": int
        }
    """
    try:
        from app.database.navigation_db import get_navigation_db

        nav_db = await get_navigation_db()
        changes = await nav_db.get_path_changes(limit)

        return {
            "success": True,
            "path_changes": changes,
            "count": len(changes)
        }

    except Exception as e:
        logger.error(f"Failed to get path changes: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/v1/navigation/obstacles")
async def get_obstacle_events(limit: int = 50):
    """
    Get recent obstacle detection events.

    Returns:
        {
            "success": bool,
            "obstacles": [...],
            "count": int
        }
    """
    try:
        from app.database.navigation_db import get_navigation_db

        nav_db = await get_navigation_db()
        obstacles = await nav_db.get_obstacle_events(limit)

        return {
            "success": True,
            "obstacles": obstacles,
            "count": len(obstacles)
        }

    except Exception as e:
        logger.error(f"Failed to get obstacle events: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/v1/navigation/statistics")
async def get_navigation_statistics():
    """
    Get navigation database statistics.

    Returns:
        {
            "success": bool,
            "statistics": {
                "total_decisions": int,
                "by_type": {...},
                "path_changes": int,
                "obstacle_events": int,
                "latest_timestamp": float
            }
        }
    """
    try:
        from app.database.navigation_db import get_navigation_db

        nav_db = await get_navigation_db()
        stats = await nav_db.get_statistics()

        return {
            "success": True,
            "statistics": stats
        }

    except Exception as e:
        logger.error(f"Failed to get navigation statistics: {e}")
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
            # Use Gemini 2.5 Flash (September 2025 preview) for better performance
            initialize_gemini(api_key=settings.gemini_api_key, model_name="gemini-2.5-flash-preview-09-2025")
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
