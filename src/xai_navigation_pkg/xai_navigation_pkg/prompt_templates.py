#!/usr/bin/env python3
"""
Prompt Template Library for Navigation Explanations

Based on research from:
- Miller (2019): Contrastive, selective, causal explanations
- Anjomshoae et al. (2021): User-centered XAI for HRI
- Ehsan et al. (2022): Self-explaining robot architecture
"""

from typing import Dict, List, Optional
from dataclasses import dataclass


@dataclass
class PromptTemplate:
    """Template for explanation generation."""
    name: str
    decision_type: str
    template: str
    max_tokens: int = 150
    temperature: float = 0.3
    description: str = ""


class PromptTemplateLibrary:
    """
    Library of specialized prompt templates for different explanation types.
    
    Each template follows Miller (2019) principles:
    - Contrastive: "I did X instead of Y"
    - Selective: Focus on most important factors
    - Causal: "This happened because..."
    - Social: Appropriate for human understanding
    """
    
    def __init__(self):
        """Initialize template library."""
        self.templates: Dict[str, PromptTemplate] = {}
        self._load_default_templates()
    
    def _load_default_templates(self):
        """Load default explanation templates."""
        
        # === Path Selection Explanation ===
        self.templates['path_changed'] = PromptTemplate(
            name="Path Change Explanation",
            decision_type="path_changed",
            description="Explains why the robot changed its planned path",
            template="""
You are an autonomous mobile robot explaining your navigation decisions to a human user.

CONTEXT:
{CONTEXT}

DECISION EVENT:
{DECISION_DATA}

Task: Explain in 1-2 clear sentences why you changed your path. 

Requirements:
1. Use first person ("I changed my path...")
2. Be specific about the reason (obstacle, better route, etc.)
3. Mention the practical impact (distance, safety, time)
4. Be concise and user-friendly
5. Do NOT include technical jargon or coordinates unless critical

Example good explanations:
- "I changed my path because there's an obstacle blocking the original route. The new path adds 0.5 meters but avoids the obstruction."
- "I found a more efficient route that's 0.3 meters shorter, so I adjusted my path to save time."

Your explanation:""",
            max_tokens=100,
            temperature=0.3
        )
        
        # === Obstacle Avoidance Explanation ===
        self.templates['obstacle_detected'] = PromptTemplate(
            name="Obstacle Avoidance Explanation",
            decision_type="obstacle_detected",
            description="Explains why the robot stopped or avoided an obstacle",
            template="""
You are an autonomous mobile robot explaining why you stopped or changed direction.

CONTEXT:
{CONTEXT}

OBSTACLE EVENT:
{DECISION_DATA}

Task: Explain in 1-2 clear sentences why you stopped or changed course due to an obstacle.

Requirements:
1. Use first person ("I stopped because...")
2. Describe the obstacle's location simply (e.g., "ahead", "to my left")
3. Explain your action (stopped, went around, waited)
4. Reassure safety if appropriate
5. Be conversational, not technical

Example good explanations:
- "I stopped because there's an obstacle directly ahead of me. I'm waiting for it to clear before continuing."
- "I detected an obstacle on my left side, so I adjusted my path to go around it on the right."

Your explanation:""",
            max_tokens=100,
            temperature=0.3
        )
        
        # === Goal Unreachable Explanation ===
        self.templates['goal_aborted'] = PromptTemplate(
            name="Goal Abortion Explanation",
            decision_type="goal_aborted",
            description="Explains why the robot couldn't reach the goal",
            template="""
You are an autonomous mobile robot explaining why you couldn't reach your destination.

CONTEXT:
{CONTEXT}

FAILURE EVENT:
{DECISION_DATA}

Task: Explain in 2-3 clear sentences why you couldn't reach the goal and what alternatives exist.

Requirements:
1. Use first person ("I couldn't reach...")
2. Clearly state the reason (blocked, unreachable, unsafe)
3. Mention how close you got (if relevant)
4. Offer alternatives or ask for guidance
5. Be apologetic but professional

Example good explanations:
- "I couldn't reach the requested location because it's blocked by an obstacle. I got within 0.8 meters. Would you like me to wait here or go somewhere else?"
- "The goal location is unreachable due to obstacles in the way. I tried 3 times but couldn't find a safe path. Should I try a different location?"

Your explanation:""",
            max_tokens=150,
            temperature=0.3
        )
        
        # === Goal Reached Explanation ===
        self.templates['goal_reached'] = PromptTemplate(
            name="Goal Success Explanation",
            decision_type="goal_reached",
            description="Confirms successful arrival at goal",
            template="""
You are an autonomous mobile robot confirming you reached your destination.

CONTEXT:
{CONTEXT}

SUCCESS EVENT:
{DECISION_DATA}

Task: Confirm arrival in 1 short sentence. Be professional and ready for next command.

Requirements:
1. Use first person ("I've arrived at...")
2. Be brief and confirmatory
3. Indicate readiness for next task
4. Stay conversational

Example good confirmations:
- "I've arrived at the goal location."
- "I reached the destination successfully."

Your confirmation:""",
            max_tokens=50,
            temperature=0.2
        )
        
        # === Recovery Behavior Explanation ===
        self.templates['recovery_behavior'] = PromptTemplate(
            name="Recovery Behavior Explanation",
            decision_type="recovery_behavior",
            description="Explains recovery actions when stuck",
            template="""
You are an autonomous mobile robot explaining a recovery action you're taking.

CONTEXT:
{CONTEXT}

RECOVERY EVENT:
{DECISION_DATA}

Task: Explain in 1-2 sentences what recovery action you're taking and why.

Requirements:
1. Use first person ("I'm trying to...")
2. Describe the recovery action (backing up, rotating, clearing costmap)
3. Explain the problem you're addressing
4. Keep it simple and reassuring

Example good explanations:
- "I'm backing up because I got too close to an obstacle. I'll try a different approach."
- "I'm rotating to clear my sensor data and find a better path forward."

Your explanation:""",
            max_tokens=100,
            temperature=0.3
        )
        
        # === Planning Failure Explanation ===
        self.templates['planning_failed'] = PromptTemplate(
            name="Planning Failure Explanation",
            decision_type="planning_failed",
            description="Explains why path planning failed",
            template="""
You are an autonomous mobile robot explaining why you couldn't plan a path.

CONTEXT:
{CONTEXT}

PLANNING FAILURE:
{DECISION_DATA}

Task: Explain in 1-2 sentences why you couldn't create a path to the goal.

Requirements:
1. Use first person ("I couldn't find a path...")
2. Explain the reason (too far, blocked, invalid goal)
3. Suggest what the user might do (move goal, clear obstacles)
4. Be helpful and solution-oriented

Example good explanations:
- "I couldn't find a path to that location because it appears to be blocked or outside the mapped area. Could you choose a different location?"
- "The goal is too far for me to reach with my current battery level. I can go about halfway there."

Your explanation:""",
            max_tokens=120,
            temperature=0.3
        )
        
        # === Wait/Pause Explanation ===
        self.templates['waiting'] = PromptTemplate(
            name="Waiting Explanation",
            decision_type="waiting",
            description="Explains why the robot is waiting",
            template="""
You are an autonomous mobile robot explaining why you're paused.

CONTEXT:
{CONTEXT}

WAIT EVENT:
{DECISION_DATA}

Task: Explain in 1 sentence why you're waiting.

Requirements:
1. Use first person ("I'm waiting for...")
2. State the reason clearly (person passing, door opening, obstacle clearing)
3. Be patient and professional

Example good explanations:
- "I'm waiting for a person to pass before I continue."
- "I'm pausing until the path ahead is clear."

Your explanation:""",
            max_tokens=50,
            temperature=0.2
        )

        # === Goal Received Explanation ===
        self.templates['goal_received'] = PromptTemplate(
            name="Goal Received Explanation",
            decision_type="goal_received",
            description="Explains acceptance of a new navigation goal",
            template="""
You are an autonomous mobile robot confirming a new navigation goal.

CONTEXT:
{CONTEXT}

GOAL EVENT:
{DECISION_DATA}

Task: Confirm the new goal in 1 short sentence.

Requirements:
1. Use first person ("I'm heading to...")
2. Mention the coordinates or location if available
3. Be concise and professional
4. Confirm you are starting navigation

Example good confirmations:
- "I've received a new goal at coordinates (2.5, 1.0) and I'm starting navigation now."
- "Heading to the new destination."

Your confirmation:""",
            max_tokens=60,
            temperature=0.2
        )
    
    def get_template(self, decision_type: str) -> Optional[PromptTemplate]:
        """
        Get template for decision type.
        
        Args:
            decision_type: Type of navigation decision
            
        Returns:
            PromptTemplate or None if not found
        """
        return self.templates.get(decision_type)
    
    def add_custom_template(self, template: PromptTemplate):
        """Add a custom template to the library."""
        self.templates[template.decision_type] = template
    
    def list_templates(self) -> List[str]:
        """List all available template types."""
        return list(self.templates.keys())
    
    def get_template_info(self, decision_type: str) -> Optional[Dict[str, str]]:
        """Get information about a template."""
        template = self.templates.get(decision_type)
        if template:
            return {
                'name': template.name,
                'decision_type': template.decision_type,
                'description': template.description,
                'max_tokens': template.max_tokens,
                'temperature': template.temperature
            }
        return None
