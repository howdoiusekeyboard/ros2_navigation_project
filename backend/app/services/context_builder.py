"""
Context builder for conversation memory injection into LLM prompts.

This module formats conversation history and resolves spatial references
for context-aware command processing.
"""

import re
from typing import Dict, List, Optional, Tuple

from app.database.conversation_db import ConversationDatabase


class ContextBuilder:
    """
    Builds context dictionaries from conversation history for LLM injection.

    Responsibilities:
    - Format conversation history for LLM prompts
    - Resolve spatial references ("there", "back", "previous location")
    - Extract mentioned locations from user input
    """

    # Spatial reference patterns for matching
    SPATIAL_PATTERNS = {
        "there": ["there", "that place", "that location", "that spot"],
        "here": ["here", "right here", "this spot", "this place"],
        "back": ["back", "go back", "return", "come back"],
        "previous": ["previous", "before", "last place", "earlier", "prior"],
        "start": ["start", "starting point", "where we started", "origin"]
    }

    def __init__(self, database: ConversationDatabase):
        """
        Initialize context builder.

        Args:
            database: ConversationDatabase instance for history retrieval
        """
        self.db = database

    async def build_context_for_llm(
        self,
        session_id: str,
        max_turns: int = 5,
        include_spatial_refs: bool = True
    ) -> str:
        """
        Build formatted context string for LLM prompt injection.

        Args:
            session_id: Session to retrieve context for
            max_turns: Maximum conversation turns to include
            include_spatial_refs: Whether to include known location references

        Returns:
            Formatted context string ready for LLM prompt
        """
        history = await self.db.get_history(session_id, limit=max_turns)

        if not history:
            return "No previous conversation in this session."

        context_lines = ["=== CONVERSATION HISTORY ==="]

        # Reverse to show chronological order (oldest to newest)
        for turn in reversed(history):
            context_lines.append(f"\nTurn {turn['turn_number']}:")
            context_lines.append(f"  User: {turn['user_input']}")

            if turn['robot_response']:
                response = turn['robot_response']

                # Add location context if available
                if turn['location_label']:
                    response += f" [at {turn['location_label']}]"
                elif turn['location_x'] is not None:
                    response += f" [at ({turn['location_x']:.1f}, {turn['location_y']:.1f})]"

                context_lines.append(f"  Robot: {response}")

        # Add spatial references if requested
        if include_spatial_refs:
            refs = await self.db.get_spatial_references(session_id)
            if refs:
                context_lines.append("\n=== KNOWN LOCATIONS ===")
                for label, (x, y, display_label) in refs.items():
                    context_lines.append(f"  {display_label}: ({x:.1f}, {y:.1f})")

        return "\n".join(context_lines)

    async def build_context_dict(
        self,
        session_id: str,
        max_turns: int = 5
    ) -> Dict:
        """
        Build structured context dictionary (alternative to string format).

        Returns:
            Dict with 'history', 'spatial_references', 'turn_count'
        """
        history = await self.db.get_history(session_id, limit=max_turns)
        spatial_refs = await self.db.get_spatial_references(session_id)

        # Format history as list of dicts
        formatted_history = []
        for turn in reversed(history):
            formatted_history.append({
                "turn": turn["turn_number"],
                "user": turn["user_input"],
                "robot": turn["robot_response"],
                "location": {
                    "x": turn["location_x"],
                    "y": turn["location_y"],
                    "label": turn["location_label"]
                } if turn["location_x"] is not None else None
            })

        return {
            "history": formatted_history,
            "spatial_references": {
                label: {"x": x, "y": y}
                for label, (x, y, _) in spatial_refs.items()
            },
            "turn_count": len(history)
        }

    async def resolve_spatial_reference(
        self,
        reference_text: str,
        session_id: str,
        current_location: Optional[Tuple[float, float]] = None
    ) -> Optional[Tuple[float, float, str]]:
        """
        Resolve a spatial reference term to coordinates.

        Args:
            reference_text: Text containing spatial reference (e.g., "go there")
            session_id: Current session ID
            current_location: Current robot position (x, y) if available

        Returns:
            Tuple of (x, y, description) or None if unresolvable
        """
        text_lower = reference_text.lower().strip()

        # 1. Check for explicit location labels in database
        spatial_refs = await self.db.get_spatial_references(session_id)

        # Direct label match (e.g., "kitchen", "bedroom")
        if text_lower in spatial_refs:
            x, y, label = spatial_refs[text_lower]
            return (x, y, f"labeled location '{label}'")

        # 2. Check for spatial reference patterns
        detected_ref = self._extract_spatial_reference(text_lower)

        if detected_ref:
            coords = await self._resolve_pattern(
                detected_ref, session_id, current_location
            )
            if coords:
                x, y, desc = coords
                return (x, y, desc)

        # 3. Check if any spatial reference contains this term
        for label, (x, y, display_label) in spatial_refs.items():
            if label in text_lower or display_label.lower() in text_lower:
                return (x, y, f"labeled location '{display_label}'")

        return None

    def _extract_spatial_reference(self, text: str) -> Optional[str]:
        """
        Extract spatial reference keyword from text.

        Returns:
            Reference type ("there", "back", "previous", etc.) or None
        """
        for ref_type, patterns in self.SPATIAL_PATTERNS.items():
            for pattern in patterns:
                if pattern in text:
                    return ref_type
        return None

    async def _resolve_pattern(
        self,
        pattern: str,
        session_id: str,
        current_location: Optional[Tuple[float, float]] = None
    ) -> Optional[Tuple[float, float, str]]:
        """
        Resolve a spatial reference pattern to coordinates.

        Args:
            pattern: Pattern type ("there", "back", "previous", "start")
            session_id: Session ID
            current_location: Current robot location

        Returns:
            (x, y, description) or None
        """
        history = await self.db.get_history(session_id, limit=10)

        if not history:
            return None

        if pattern == "there":
            # Most recent mentioned location
            for turn in history:
                if turn["location_x"] is not None:
                    return (
                        turn["location_x"],
                        turn["location_y"],
                        f"last mentioned location (turn {turn['turn_number']})"
                    )

        elif pattern == "back" or pattern == "previous":
            # Second most recent location (before current)
            locations = [
                (t["location_x"], t["location_y"], t["turn_number"])
                for t in history
                if t["location_x"] is not None
            ]

            if len(locations) >= 2:
                x, y, turn_num = locations[1]  # Second most recent
                return (x, y, f"previous location (turn {turn_num})")
            elif len(locations) == 1:
                # Only one previous location, return it
                x, y, turn_num = locations[0]
                return (x, y, f"previous location (turn {turn_num})")

        elif pattern == "start":
            # First location in session
            for turn in reversed(history):
                if turn["location_x"] is not None:
                    return (
                        turn["location_x"],
                        turn["location_y"],
                        f"starting location (turn {turn['turn_number']})"
                    )

        elif pattern == "here":
            # Current location
            if current_location:
                return (
                    current_location[0],
                    current_location[1],
                    "current location"
                )

        return None

    def extract_location_mentions(self, text: str) -> List[str]:
        """
        Extract potential location labels from user input.

        Args:
            text: User input text

        Returns:
            List of detected location labels (e.g., ["kitchen", "bedroom"])
        """
        # Common location labels
        location_keywords = [
            "kitchen", "bedroom", "living room", "bathroom", "office",
            "garage", "hallway", "entrance", "dining room", "study",
            "basement", "attic", "balcony", "patio", "garden",
            "lobby", "corridor", "storage", "closet", "pantry"
        ]

        text_lower = text.lower()
        found_locations = []

        for location in location_keywords:
            if location in text_lower:
                found_locations.append(location)

        return found_locations

    def has_spatial_reference(self, text: str) -> bool:
        """
        Check if text contains any spatial reference patterns.

        Returns:
            True if spatial reference detected
        """
        return self._extract_spatial_reference(text.lower()) is not None

    def extract_coordinates(self, text: str) -> Optional[Tuple[float, float]]:
        """
        Extract explicit coordinates from user input.

        Matches patterns like:
        - "2, 3"
        - "position 2.5, 3.1"
        - "coordinates (2, 3)"
        - "x=2 y=3"

        Returns:
            (x, y) tuple or None if no coordinates found
        """
        # Truncate text to max 250 characters to completely prevent polynomial ReDoS 
        # backtracking on excessively long malicious inputs.
        text = text[:250]
        
        # Pattern 1: "2, 3" or "2.5, 3.1"
        coord_pattern_1 = r'(-?\d+(?:\.\d+)?)\s{0,5},\s{0,5}(-?\d+(?:\.\d+)?)'

        # Pattern 2: "x=2 y=3" or "x:2 y:3"
        coord_pattern_2 = r'x\s{0,5}[=:]\s{0,5}(-?\d+(?:\.\d+)?)\s{1,5}y\s{0,5}[=:]\s{0,5}(-?\d+(?:\.\d+)?)'

        # Try pattern 1
        match = re.search(coord_pattern_1, text)
        if match:
            x = float(match.group(1))
            y = float(match.group(2))
            return (x, y)

        # Try pattern 2
        match = re.search(coord_pattern_2, text.lower())
        if match:
            x = float(match.group(1))
            y = float(match.group(2))
            return (x, y)

        return None

    async def get_conversation_summary(self, session_id: str) -> str:
        """
        Get a brief summary of the conversation session.

        Returns:
            Human-readable summary string
        """
        summary = await self.db.get_session_summary(session_id)

        if not summary:
            return "No conversation data available"

        lines = [
            f"Session {session_id}:",
            f"  • {summary['turn_count']} conversation turns",
            f"  • {summary['navigation_count']} navigation commands",
        ]

        if summary['avg_confidence']:
            lines.append(
                f"  • Average confidence: {summary['avg_confidence']:.0%}"
            )

        if summary['avg_latency_ms']:
            lines.append(
                f"  • Average latency: {summary['avg_latency_ms']:.0f}ms"
            )

        return "\n".join(lines)


# Singleton for easy access
_context_builder_instance: Optional[ContextBuilder] = None


def get_context_builder(database: ConversationDatabase) -> ContextBuilder:
    """
    Get or create ContextBuilder singleton.

    Args:
        database: ConversationDatabase instance

    Returns:
        ContextBuilder instance
    """
    global _context_builder_instance

    if _context_builder_instance is None:
        _context_builder_instance = ContextBuilder(database)

    return _context_builder_instance
