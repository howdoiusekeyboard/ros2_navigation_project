"""
Gemini Command Parsing Service

Uses Google Gemini 2.0 Flash to parse natural language commands
into structured robot actions with safety validation.
"""

import google.generativeai as genai
from typing import Dict, Any, Optional, List
from pydantic import BaseModel, Field, validator
from loguru import logger
from enum import Enum


# ==================== Command Schema ====================

class ActionType(str, Enum):
    """Supported robot actions"""
    TWIST = "twist"              # Velocity control (linear + angular)
    NAVIGATE = "navigate"        # Navigate to position
    STOP = "stop"                # Emergency stop
    ROTATE = "rotate"            # Rotate in place
    MOVE_FORWARD = "move_forward"  # Move forward distance
    MOVE_BACKWARD = "move_backward"  # Move backward distance
    UNKNOWN = "unknown"          # Unable to parse


class TwistParameters(BaseModel):
    """Parameters for twist (velocity) command"""
    linear_x: float = Field(0.0, description="Linear velocity in m/s (-0.5 to 0.5)")
    angular_z: float = Field(0.0, description="Angular velocity in rad/s (-2.0 to 2.0)")
    duration: Optional[float] = Field(None, description="Duration in seconds (optional)")

    @validator('linear_x')
    def validate_linear(cls, v):
        if not -0.5 <= v <= 0.5:
            raise ValueError(f"Linear velocity {v} out of range [-0.5, 0.5] m/s")
        return v

    @validator('angular_z')
    def validate_angular(cls, v):
        if not -2.0 <= v <= 2.0:
            raise ValueError(f"Angular velocity {v} out of range [-2.0, 2.0] rad/s")
        return v


class NavigateParameters(BaseModel):
    """Parameters for navigation command"""
    x: float = Field(description="Target x position in meters")
    y: float = Field(description="Target y position in meters")
    theta: float = Field(0.0, description="Target orientation in radians")
    frame_id: str = Field("map", description="Reference frame")

    @validator('x', 'y')
    def validate_position(cls, v):
        if not -10.0 <= v <= 10.0:
            raise ValueError(f"Position {v} out of range [-10.0, 10.0] meters")
        return v


class RotateParameters(BaseModel):
    """Parameters for rotation command"""
    angle: float = Field(description="Rotation angle in radians (positive = counterclockwise)")

    @validator('angle')
    def validate_angle(cls, v):
        if not -6.28 <= v <= 6.28:  # -2π to 2π
            raise ValueError(f"Angle {v} out of range [-2π, 2π] radians")
        return v


class MoveParameters(BaseModel):
    """Parameters for linear movement"""
    distance: float = Field(description="Distance to move in meters")

    @validator('distance')
    def validate_distance(cls, v):
        if not -5.0 <= v <= 5.0:
            raise ValueError(f"Distance {v} out of range [-5.0, 5.0] meters")
        return v


class RobotCommand(BaseModel):
    """Structured robot command from Gemini"""
    action: ActionType = Field(description="Type of action to perform")
    parameters: Dict[str, Any] = Field(default_factory=dict, description="Action parameters")
    confidence: float = Field(0.0, description="Confidence score (0-1)")
    reasoning: Optional[str] = Field(None, description="Why this interpretation")


# ==================== Gemini Parser ====================

class GeminiCommandParser:
    """
    Gemini-based natural language command parser with safety validation.

    Uses Gemini 2.0 Flash for cost-effective parsing with structured output.
    """

    # Safety limits (TurtleBot3 Burger)
    MAX_LINEAR_SPEED = 0.22  # m/s (official limit)
    MAX_ANGULAR_SPEED = 2.84  # rad/s (official limit)
    MAX_DISTANCE = 5.0  # meters (safety limit)
    MAX_ANGLE = 6.28  # radians (2π, full rotation)

    def __init__(self, api_key: str, model_name: str = "gemini-2.0-flash-exp"):
        """
        Initialize Gemini parser

        Args:
            api_key: Google AI API key
            model_name: Gemini model to use (default: gemini-2.0-flash-exp)
        """
        self.api_key = api_key
        self.model_name = model_name

        # Configure Gemini
        genai.configure(api_key=api_key)

        # Create model with generation config
        self.generation_config = {
            "temperature": 0.3,  # Low temperature for consistent parsing
            "top_p": 0.95,
            "top_k": 40,
            "max_output_tokens": 512,
        }

        self.model = genai.GenerativeModel(
            model_name=model_name,
            generation_config=self.generation_config
        )

        logger.info(f"Gemini Command Parser initialized: {model_name}")

    def parse_command(self, command: str, context: Optional[Dict[str, Any]] = None) -> RobotCommand:
        """
        Parse natural language command into structured robot action

        Args:
            command: User's natural language command
            context: Optional context (robot state, history, etc.)

        Returns:
            RobotCommand with action, parameters, and confidence
        """
        try:
            logger.info(f"Parsing command: '{command}'")

            # Try regex-based simple command detection first (fast path)
            simple_result = self._try_simple_command(command)
            if simple_result:
                logger.info(f"Matched simple command: {simple_result.action}")
                return simple_result

            # Use Gemini for complex commands
            gemini_result = self._parse_with_gemini(command, context)

            # Validate and apply safety limits
            validated_result = self._validate_command(gemini_result)

            logger.info(f"Parsed command: action={validated_result.action}, confidence={validated_result.confidence:.2f}")
            return validated_result

        except Exception as e:
            logger.error(f"Command parsing failed: {e}")
            return RobotCommand(
                action=ActionType.UNKNOWN,
                parameters={},
                confidence=0.0,
                reasoning=f"Error: {str(e)}"
            )

    def _try_simple_command(self, command: str) -> Optional[RobotCommand]:
        """
        Try regex-based pattern matching for simple commands (fast path)

        Returns None if command is complex and needs Gemini.
        """
        import re

        cmd_lower = command.lower().strip()

        # Stop commands
        if re.search(r'\b(stop|halt|freeze|brake)\b', cmd_lower):
            return RobotCommand(
                action=ActionType.STOP,
                parameters={},
                confidence=0.95,
                reasoning="Matched stop keyword"
            )

        # Simple forward/backward
        if re.search(r'\b(forward|ahead)\b', cmd_lower) and not re.search(r'\d', cmd_lower):
            return RobotCommand(
                action=ActionType.TWIST,
                parameters={"linear_x": 0.15, "angular_z": 0.0},
                confidence=0.85,
                reasoning="Matched simple forward command"
            )

        if re.search(r'\b(backward|back|reverse)\b', cmd_lower) and not re.search(r'\d', cmd_lower):
            return RobotCommand(
                action=ActionType.TWIST,
                parameters={"linear_x": -0.15, "angular_z": 0.0},
                confidence=0.85,
                reasoning="Matched simple backward command"
            )

        # Simple rotation
        if re.search(r'\b(turn|rotate|spin)\s+(left|right)\b', cmd_lower) and not re.search(r'\d', cmd_lower):
            if 'right' in cmd_lower:
                angular = -1.0
            else:
                angular = 1.0

            return RobotCommand(
                action=ActionType.TWIST,
                parameters={"linear_x": 0.0, "angular_z": angular},
                confidence=0.85,
                reasoning=f"Matched simple rotation command"
            )

        # Complex command - needs Gemini
        return None

    def _parse_with_gemini(self, command: str, context: Optional[Dict[str, Any]] = None) -> RobotCommand:
        """
        Parse command using Gemini API with structured output
        """
        # Build prompt with JSON schema
        prompt = self._build_parsing_prompt(command, context)

        # Call Gemini
        response = self.model.generate_content(prompt)

        # Extract JSON from response
        import json
        import re

        response_text = response.text.strip()

        # Try to extract JSON from markdown code blocks or raw text
        json_match = re.search(r'```(?:json)?\s*(\{.*?\})\s*```', response_text, re.DOTALL)
        if json_match:
            json_str = json_match.group(1)
        else:
            # Try to find JSON directly
            json_match = re.search(r'\{.*\}', response_text, re.DOTALL)
            if json_match:
                json_str = json_match.group(0)
            else:
                raise ValueError(f"No JSON found in Gemini response: {response_text}")

        # Parse JSON
        parsed_data = json.loads(json_str)

        # Create RobotCommand
        return RobotCommand(
            action=ActionType(parsed_data.get('action', 'unknown')),
            parameters=parsed_data.get('parameters', {}),
            confidence=parsed_data.get('confidence', 0.5),
            reasoning=parsed_data.get('reasoning', '')
        )

    def _build_parsing_prompt(self, command: str, context: Optional[Dict[str, Any]] = None) -> str:
        """Build prompt for Gemini with command schema and conversation context"""

        context_str = ""
        if context:
            # Handle conversation history if provided
            if isinstance(context, str):
                # Context is already formatted string (from context_builder)
                context_str = f"\n\n{context}\n"
            elif isinstance(context, dict):
                # Legacy dict format
                if 'conversation_history' in context:
                    context_str += f"\n\n{context['conversation_history']}\n"
                if 'robot_state' in context:
                    context_str += f"\nCurrent robot state:\n{context['robot_state']}\n"

        prompt = f"""You are a robot command interpreter with conversation memory. Parse the user's natural language command into a structured JSON format.{context_str}

User command: "{command}"

Available actions:
- "twist": Send velocity command (linear_x in m/s, angular_z in rad/s)
- "navigate": Navigate to position (x, y in meters, theta in radians)
- "stop": Emergency stop (no parameters)
- "rotate": Rotate in place (angle in radians, positive=counterclockwise)
- "move_forward": Move forward (distance in meters)
- "move_backward": Move backward (distance in meters)
- "unknown": Cannot parse command

Safety limits:
- Linear velocity: -0.22 to 0.22 m/s
- Angular velocity: -2.84 to 2.84 rad/s
- Distance: -5.0 to 5.0 meters
- Angle: -6.28 to 6.28 radians

Examples:

Command: "move forward 2 meters"
Output:
{{
  "action": "move_forward",
  "parameters": {{"distance": 2.0}},
  "confidence": 0.95,
  "reasoning": "Clear forward movement command with specific distance"
}}

Command: "spin in a circle"
Output:
{{
  "action": "twist",
  "parameters": {{"linear_x": 0.15, "angular_z": 1.0}},
  "confidence": 0.90,
  "reasoning": "Circle motion requires forward velocity with rotation"
}}

Command: "go to coordinates 3, 2"
Output:
{{
  "action": "navigate",
  "parameters": {{"x": 3.0, "y": 2.0, "theta": 0.0}},
  "confidence": 0.92,
  "reasoning": "Navigation to specific coordinates"
}}

Command: "rotate 90 degrees clockwise"
Output:
{{
  "action": "rotate",
  "parameters": {{"angle": -1.57}},
  "confidence": 0.95,
  "reasoning": "90 degrees = π/2 radians, clockwise is negative"
}}

**IMPORTANT: Spatial Reference Resolution**
If conversation history shows previous locations, resolve references like "there", "back", "previous location":

Command: "go there" (with history showing last location was x=2.5, y=3.1)
Output:
{{
  "action": "navigate",
  "parameters": {{"x": 2.5, "y": 3.1, "theta": 0.0}},
  "confidence": 0.88,
  "reasoning": "Resolved 'there' to last mentioned location from conversation history"
}}

Command: "go to the kitchen" (with known locations showing kitchen at x=2.0, y=3.0)
Output:
{{
  "action": "navigate",
  "parameters": {{"x": 2.0, "y": 3.0, "theta": 0.0}},
  "confidence": 0.90,
  "reasoning": "Using known location 'kitchen' from conversation context"
}}

Now parse this command and return ONLY the JSON output:
"""

        return prompt

    def _validate_command(self, command: RobotCommand) -> RobotCommand:
        """
        Validate command and apply safety limits

        Clamps values to safe ranges and adjusts confidence if modified.
        """
        params = command.parameters.copy()
        modified = False

        # Validate twist parameters
        if command.action == ActionType.TWIST:
            if 'linear_x' in params:
                original = params['linear_x']
                params['linear_x'] = max(-self.MAX_LINEAR_SPEED, min(self.MAX_LINEAR_SPEED, params['linear_x']))
                if params['linear_x'] != original:
                    logger.warning(f"Clamped linear_x from {original} to {params['linear_x']}")
                    modified = True

            if 'angular_z' in params:
                original = params['angular_z']
                params['angular_z'] = max(-self.MAX_ANGULAR_SPEED, min(self.MAX_ANGULAR_SPEED, params['angular_z']))
                if params['angular_z'] != original:
                    logger.warning(f"Clamped angular_z from {original} to {params['angular_z']}")
                    modified = True

        # Validate navigation parameters
        elif command.action == ActionType.NAVIGATE:
            for coord in ['x', 'y']:
                if coord in params:
                    original = params[coord]
                    params[coord] = max(-10.0, min(10.0, params[coord]))
                    if params[coord] != original:
                        logger.warning(f"Clamped {coord} from {original} to {params[coord]}")
                        modified = True

        # Validate movement parameters
        elif command.action in [ActionType.MOVE_FORWARD, ActionType.MOVE_BACKWARD]:
            if 'distance' in params:
                original = params['distance']
                params['distance'] = max(-self.MAX_DISTANCE, min(self.MAX_DISTANCE, params['distance']))
                if params['distance'] != original:
                    logger.warning(f"Clamped distance from {original} to {params['distance']}")
                    modified = True

        # Validate rotation parameters
        elif command.action == ActionType.ROTATE:
            if 'angle' in params:
                original = params['angle']
                params['angle'] = max(-self.MAX_ANGLE, min(self.MAX_ANGLE, params['angle']))
                if params['angle'] != original:
                    logger.warning(f"Clamped angle from {original} to {params['angle']}")
                    modified = True

        # Reduce confidence if values were clamped
        confidence = command.confidence
        if modified:
            confidence = max(0.5, confidence - 0.2)
            logger.info(f"Reduced confidence to {confidence:.2f} due to safety clamping")

        return RobotCommand(
            action=command.action,
            parameters=params,
            confidence=confidence,
            reasoning=command.reasoning
        )


# ==================== Singleton Instance ====================

# Global parser instance (initialized in main.py startup)
gemini_parser: Optional[GeminiCommandParser] = None


def initialize_gemini(api_key: str, model_name: str = "gemini-2.0-flash-exp"):
    """Initialize global Gemini parser"""
    global gemini_parser
    gemini_parser = GeminiCommandParser(api_key=api_key, model_name=model_name)
    logger.info("✅ Gemini Command Parser initialized")


def get_parser() -> GeminiCommandParser:
    """Get global Gemini parser instance"""
    if gemini_parser is None:
        raise RuntimeError("Gemini parser not initialized. Call initialize_gemini() first.")
    return gemini_parser
