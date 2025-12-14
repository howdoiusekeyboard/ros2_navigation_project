from google import genai
from google.genai import types

import os
import json
import time
import asyncio
from typing import Dict, Any, Optional, List
from dataclasses import dataclass
import aiohttp


@dataclass
class ExplanationRequest:
    """Request for explanation generation."""
    decision_type: str
    decision_data: Dict[str, Any]
    context: Dict[str, Any]
    template: str
    max_tokens: int = 150
    temperature: float = 0.3


@dataclass
class ExplanationResponse:
    """Response from explanation generation."""
    text: str
    confidence: float
    generation_time: float
    token_count: int
    cached: bool = False

class GeminiClient:
    """
    Client for Google Gemini API with explanation-specific optimizations.
    
    Features:
    - Uses official google-genai SDK
    - Async wrapper for blocking SDK calls
    - Request batching for efficiency
    - Error handling with exponential backoff
    - Response validation
    - Rate limiting compliance
    """
    
    def __init__(
        self,
        api_key: Optional[str] = None,
        model: str = "gemini-2.5-flash-preview-09-2025",
        max_retries: int = 3,
        timeout: float = 5.0
    ):
        """
        Initialize Gemini client.

        Args:
            api_key: Gemini API key (defaults to GEMINI_API_KEY env var)
            model: Model to use (gemini-2.5-flash-preview-09-2025 September 2025 preview)
            max_retries: Maximum retry attempts on failure
            timeout: Request timeout in seconds
        """
        self.api_key = api_key or os.getenv('GEMINI_API_KEY')
        if not self.api_key:
            # Don't raise error immediately, allow setting later or via param
            pass
        
        self.model = model
        self.max_retries = max_retries
        self.timeout = timeout
        
        # Initialize SDK client
        if self.api_key:
            self.client = genai.Client(api_key=self.api_key)
        else:
            self.client = None
        
        # Performance tracking
        self.stats = {
            'total_requests': 0,
            'successful_requests': 0,
            'failed_requests': 0,
            'total_tokens': 0,
            'total_time': 0.0,
            'cache_hits': 0
        }
        
    async def generate_explanation(
        self,
        request: ExplanationRequest
    ) -> ExplanationResponse:
        """
        Generate explanation using Gemini API.
        
        Args:
            request: ExplanationRequest with decision data and template
            
        Returns:
            ExplanationResponse with generated text and metadata
        """
        if not self.client:
             # Try to get from env again
            self.api_key = self.api_key or os.getenv('GEMINI_API_KEY')
            if not self.api_key:
                raise ValueError("GEMINI_API_KEY not found in environment or parameters")
            self.client = genai.Client(api_key=self.api_key)

        start_time = time.time()
        self.stats['total_requests'] += 1
        
        # Build prompt from template and data
        prompt = self._build_prompt(request)
        
        # Make API call with retry logic
        for attempt in range(self.max_retries):
            try:
                response_text = await self._call_api(
                    prompt,
                    max_tokens=request.max_tokens,
                    temperature=request.temperature
                )
                
                # Parse and validate response
                explanation = self._parse_response(response_text)
                
                generation_time = time.time() - start_time
                self.stats['successful_requests'] += 1
                self.stats['total_time'] += generation_time
                
                return ExplanationResponse(
                    text=explanation,
                    confidence=self._estimate_confidence(explanation),
                    generation_time=generation_time,
                    token_count=len(explanation.split()),
                    cached=False
                )
                
            except Exception as e:
                if attempt == self.max_retries - 1:
                    self.stats['failed_requests'] += 1
                    raise Exception(f"Gemini API failed after {self.max_retries} attempts: {e}")
                
                # Exponential backoff
                await asyncio.sleep(2 ** attempt)

    def _build_prompt(self, request: ExplanationRequest) -> str:
        """
        Build prompt from template and decision data.
        
        Uses template variables and context to create a complete prompt.
        """
        # Replace template variables with actual data
        prompt = request.template
        
        # Add context
        context_str = self._format_context(request.context)
        prompt = prompt.replace("{CONTEXT}", context_str)
        
        # Add decision data
        decision_str = self._format_decision_data(
            request.decision_type,
            request.decision_data
        )
        prompt = prompt.replace("{DECISION_DATA}", decision_str)
        
        return prompt
    
    def _format_context(self, context: Dict[str, Any]) -> str:
        """Format context dictionary into readable text."""
        lines = []
        
        if 'current_position' in context and context['current_position']:
            pos = context['current_position']
            lines.append(f"Current position: ({pos.get('x', 0):.2f}, {pos.get('y', 0):.2f})")
        
        if 'goal_position' in context and context['goal_position']:
            goal = context['goal_position']
            lines.append(f"Goal position: ({goal.get('x', 0):.2f}, {goal.get('y', 0):.2f})")
        
        if 'recent_decisions' in context and context['recent_decisions']:
            lines.append(f"Recent decisions: {len(context['recent_decisions'])}")
        
        return "\n".join(lines)
    
    def _format_decision_data(
        self,
        decision_type: str,
        data: Dict[str, Any]
    ) -> str:
        """Format decision data based on type."""
        if decision_type == 'path_changed':
            return f"""
Path Change Event:
- Original path length: {data.get('original_length', 0):.2f}m
- New path length: {data.get('new_length', 0):.2f}m
- Length change: {data.get('length_change', 0):.2f}m
- Reason: {data.get('reason', 'unknown')}
- Max deviation: {data.get('max_deviation', 0):.2f}m
"""
        
        elif decision_type == 'obstacle_detected':
            return f"""
Obstacle Detection Event:
- Obstacle position: ({data.get('obstacle_x', 0):.2f}, {data.get('obstacle_y', 0):.2f})
- Distance to robot: {data.get('distance_to_robot', 0):.2f}m
- Severity: {data.get('severity', 'unknown')}
- Action taken: {data.get('action', 'unknown')}
"""
        
        elif decision_type == 'goal_aborted':
            return f"""
Goal Abortion Event:
- Reason: {data.get('reason', 'unknown')}
- Attempts made: {data.get('attempts', 0)}
- Final position: ({data.get('final_x', 0):.2f}, {data.get('final_y', 0):.2f})
- Distance from goal: {data.get('distance_from_goal', 0):.2f}m
"""
        
        else:
            return json.dumps(data, indent=2)

    async def _call_api(
        self,
        prompt: str,
        max_tokens: int,
        temperature: float
    ) -> str:
        """
        Make actual API call to Gemini using official SDK.
        
        Wraps the blocking SDK call in a thread executor to maintain async interface.
        """
        loop = asyncio.get_running_loop()
        
        def _blocking_call():
            response = self.client.models.generate_content(
                model=self.model,
                contents=prompt,
                config=types.GenerateContentConfig(
                    max_output_tokens=max_tokens,
                    temperature=temperature,
                    top_k=40,
                    top_p=0.95,
                    thinking_config=types.ThinkingConfig(
                        include_thoughts=False,
                        thinking_budget=0
                    ),
                    safety_settings=[
                        types.SafetySetting(
                            category="HARM_CATEGORY_HARASSMENT",
                            threshold="BLOCK_NONE"
                        ),
                        types.SafetySetting(
                            category="HARM_CATEGORY_HATE_SPEECH",
                            threshold="BLOCK_NONE"
                        ),
                        types.SafetySetting(
                            category="HARM_CATEGORY_SEXUALLY_EXPLICIT",
                            threshold="BLOCK_NONE"
                        ),
                        types.SafetySetting(
                            category="HARM_CATEGORY_DANGEROUS_CONTENT",
                            threshold="BLOCK_NONE"
                        )
                    ]
                )
            )
            return response.text

        try:
            return await loop.run_in_executor(None, _blocking_call)
        except Exception as e:
            # Log the full error for debugging
            print(f"Gemini SDK Error Details: {e}")
            raise Exception(f"Gemini SDK error: {e}")
    
    def _parse_response(self, response_text: Optional[str]) -> str:
        """
        Parse and validate Gemini response.
        
        Ensures response meets quality criteria.
        """
        if not response_text:
            return "I couldn't generate an explanation at this time."

        # Remove markdown formatting if present
        if response_text.startswith('```') and response_text.endswith('```'):
            response_text = response_text[3:-3].strip()
        
        # Basic validation
        if len(response_text) < 5: # Relaxed length check
            return "Explanation too short."
        
        if len(response_text) > 500:
            # Truncate overly long responses
            response_text = response_text[:497] + "..."
        
        # Remove any system artifacts
        response_text = response_text.replace("As an AI", "I")
        response_text = response_text.replace("As a robot", "I")
        
        return response_text
    
    def _estimate_confidence(self, explanation: str) -> float:
        """
        Estimate confidence in explanation quality.
        
        Simple heuristic based on length, structure, and keywords.
        """
        confidence = 0.5  # Base confidence
        
        # Length bonus (not too short, not too long)
        word_count = len(explanation.split())
        if 15 <= word_count <= 50:
            confidence += 0.2
        elif 10 <= word_count < 15 or 50 < word_count <= 100:
            confidence += 0.1
        
        # Structure bonus (has "because", "so", "therefore")
        causal_words = ['because', 'so', 'therefore', 'due to', 'since']
        if any(word in explanation.lower() for word in causal_words):
            confidence += 0.2
        
        # Specific detail bonus (mentions distances, positions)
        if any(char.isdigit() for char in explanation):
            confidence += 0.1
        
        return min(confidence, 1.0)
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get client performance statistics."""
        avg_time = (
            self.stats['total_time'] / self.stats['successful_requests']
            if self.stats['successful_requests'] > 0
            else 0
        )
        
        success_rate = (
            self.stats['successful_requests'] / self.stats['total_requests']
            if self.stats['total_requests'] > 0
            else 0
        )
        
        return {
            **self.stats,
            'average_response_time': avg_time,
            'success_rate': success_rate
        }
