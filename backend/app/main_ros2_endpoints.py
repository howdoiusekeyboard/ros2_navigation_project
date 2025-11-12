# ROS2 Control Endpoints (to be added to main.py)

# ==================== Robot Control (ROS2) ====================

@app.post("/api/v1/robot/twist")
async def robot_twist(data: dict):
    """
    Send Twist (velocity) command to robot

    Args:
        {
            "linear_x": float,  # m/s
            "angular_z": float,  # rad/s
            "use_turtlesim": bool  # Optional: use /turtle1/cmd_vel
        }

    Returns:
        {"status": "success", "command": {...}}
    """
    try:
        # Get robot controller
        controller = ros2_manager.get_controller()

        # Extract parameters
        linear_x = data.get('linear_x', 0.0)
        angular_z = data.get('angular_z', 0.0)
        use_turtlesim = data.get('use_turtlesim', False)

        # Send command
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
    """
    Emergency stop - set all velocities to zero

    Args:
        {
            "use_turtlesim": bool  # Optional
        }

    Returns:
        {"status": "stopped"}
    """
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
    """
    Get current robot state (pose, velocity, etc.)

    Returns:
        {
            "pose": {...},
            "velocity": {...}
        }
    """
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
    Complete voice command pipeline:
    Audio → Whisper → Gemini → Robot Control

    Args:
        {
            "audio": base64 string OR
            "transcript": string (if already transcribed),
            "use_turtlesim": bool  # Optional
        }

    Returns:
        {
            "transcript": str,
            "parsed_command": {...},
            "execution_status": str,
            "latency": {
                "whisper_ms": float,
                "gemini_ms": float,
                "total_ms": float
            }
        }
    """
    import time
    start_time = time.time()

    try:
        use_turtlesim = data.get('use_turtlesim', False)

        # Step 1: Get transcript (either provided or transcribe audio)
        transcript = data.get('transcript')
        whisper_time = 0.0

        if not transcript:
            # TODO: Transcribe audio from base64
            raise HTTPException(status_code=400, detail="Either 'transcript' or 'audio' required")

        # Step 2: Parse command with Gemini
        gemini_start = time.time()
        # TODO: Implement Gemini parsing
        parsed_command = {
            "action": "unknown",
            "parameters": {},
            "confidence": 0.0
        }
        gemini_time = (time.time() - gemini_start) * 1000

        # Step 3: Execute on robot
        controller = ros2_manager.get_controller()

        if parsed_command['action'] == 'twist':
            params = parsed_command.get('parameters', {})
            controller.publish_twist(
                linear_x=params.get('linear', 0.0),
                angular_z=params.get('angular', 0.0),
                use_turtlesim=use_turtlesim
            )
            execution_status = "executed"

        elif parsed_command['action'] == 'stop':
            controller.stop(use_turtlesim=use_turtlesim)
            execution_status = "stopped"

        else:
            execution_status = "unknown_command"

        total_time = (time.time() - start_time) * 1000

        return {
            "transcript": transcript,
            "parsed_command": parsed_command,
            "execution_status": execution_status,
            "latency": {
                "whisper_ms": whisper_time,
                "gemini_ms": gemini_time,
                "total_ms": total_time
            }
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Voice command execution failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))
