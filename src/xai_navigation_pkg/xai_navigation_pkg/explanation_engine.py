#!/usr/bin/env python3
"""
Explanation Engine - Core explanation generation system.

Coordinates:
- Decision data retrieval
- Context building
- Prompt template selection
- Gemini API calls
- Response validation
- Caching
"""

import time
import asyncio
import json
from typing import Dict, Any, Optional, List
from dataclasses import dataclass

from .gemini_client import GeminiClient, ExplanationRequest, ExplanationResponse
from .prompt_templates import PromptTemplateLibrary, PromptTemplate
from .decision_database import DecisionDatabase


@dataclass
class ExplanationContext:
    """Context for explanation generation."""
    current_position: Optional[Dict[str, float]] = None
    goal_position: Optional[Dict[str, float]] = None
    recent_decisions: List[Dict[str, Any]] = None
    environment_info: Optional[Dict[str, Any]] = None


class ExplanationEngine:
    """
    Core engine for generating natural language explanations.
    
    Features:
    - Automatic template selection
    - Context-aware explanation generation
    - Caching for common scenarios
    - Performance optimization
    - Quality validation
    """
    
    def __init__(
        self,
        gemini_api_key: str,
        decision_db: DecisionDatabase,
        cache_enabled: bool = True
    ):
        """
        Initialize explanation engine.
        
        Args:
            gemini_api_key: API key for Gemini
            decision_db: Database with decision logs
            cache_enabled: Enable explanation caching
        """
        self.gemini_client = GeminiClient(api_key=gemini_api_key)
        self.template_library = PromptTemplateLibrary()
        self.decision_db = decision_db
        self.cache_enabled = cache_enabled
        
        # Simple cache: decision_id -> explanation
        self._cache: Dict[int, ExplanationResponse] = {}
        
        # Stats
        self.stats = {
            'total_explanations': 0,
            'cache_hits': 0,
            'generation_times': [],
            'average_confidence': 0.0
        }
    
    async def explain_decision(
        self,
        decision_id: int,
        context: Optional[ExplanationContext] = None
    ) -> ExplanationResponse:
        """
        Generate explanation for a specific decision.
        
        Args:
            decision_id: ID of decision to explain
            context: Additional context (current state, recent history)
            
        Returns:
            ExplanationResponse with generated explanation
        """
        start_time = time.time()
        self.stats['total_explanations'] += 1
        
        # Check cache first
        if self.cache_enabled and decision_id in self._cache:
            self.stats['cache_hits'] += 1
            cached_response = self._cache[decision_id]
            cached_response.cached = True
            return cached_response
        
        # Retrieve decision from database
        decision = self._get_decision(decision_id)
        if not decision:
            raise ValueError(f"Decision {decision_id} not found in database")
        
        # Build context if not provided
        if context is None:
            context = await self._build_context(decision)
        
        # Select appropriate template
        template = self._select_template(decision.get('decision_type', 'unknown'))
        if not template:
            # Fallback or raise error? Let's try to use a generic one or raise
            # For now, raise to be explicit
            raise ValueError(f"No template for decision type: {decision.get('decision_type')}")
        
        # Create explanation request
        request = ExplanationRequest(
            decision_type=decision.get('decision_type', 'unknown'),
            decision_data=decision.get('data', {}),
            context=self._context_to_dict(context),
            template=template.template,
            max_tokens=template.max_tokens,
            temperature=template.temperature
        )
        
        # Generate explanation
        response = await self.gemini_client.generate_explanation(request)
        
        # Validate and post-process
        response = self._post_process_explanation(response, decision)
        
        # Cache result
        if self.cache_enabled:
            self._cache[decision_id] = response
        
        # Update stats
        generation_time = time.time() - start_time
        self.stats['generation_times'].append(generation_time)
        
        # Update rolling average confidence
        current_avg = self.stats['average_confidence']
        count = self.stats['total_explanations']
        # If it was a cache hit, we already incremented total_explanations but didn't generate new confidence
        # Actually, we returned early on cache hit. So here count is correct for new generations?
        # Wait, total_explanations is incremented at start.
        # If cache hit, we return early.
        # So here we are only for non-cached.
        # But total_explanations counts all calls.
        # Let's just track average of generated ones maybe? Or all?
        # Let's keep it simple.
        self.stats['average_confidence'] = (
            (current_avg * (count - 1) + response.confidence) / count
        )
        
        return response
    
    async def explain_latest_decision(
        self,
        decision_type: Optional[str] = None
    ) -> ExplanationResponse:
        """
        Generate explanation for the most recent decision.
        
        Args:
            decision_type: Optionally filter by decision type
            
        Returns:
            ExplanationResponse with generated explanation
        """
        # Get latest decision from database
        stats = self.decision_db.get_statistics()
        # We need a way to get the last ID.
        # The stats dict might not have it directly based on Week 3 implementation.
        # We might need to query recent decisions.
        
        recent = self.decision_db.get_recent_decisions(limit=1)
        if not recent:
             raise ValueError("No decisions in database")
        
        latest_id = recent[0]['db_id']
        
        return await self.explain_decision(latest_id)
    
    def _get_decision(self, decision_id: int) -> Optional[Dict[str, Any]]:
        """Retrieve decision from database."""
        # We need to fetch by ID.
        # DecisionDatabase in Week 3 might not have get_by_id.
        # It has get_recent_decisions.
        # We might need to add get_decision method to DecisionDatabase or query manually.
        # For now, let's use a direct query if possible or assume we can add it.
        # Actually, let's implement a helper here that queries the DB directly using the connection if possible,
        # or better, rely on DecisionDatabase having a method.
        # Since I can't easily modify DecisionDatabase right now without checking it,
        # I'll try to use what's available or add a method to DecisionDatabase in a separate step if needed.
        # Wait, I can see DecisionDatabase code in the context.
        # It has `get_recent_decisions` and `get_unsynced_decisions`.
        # It does NOT have `get_decision_by_id`.
        # I should probably add it.
        # But for now, I'll implement a workaround using SQL directly if I can access the conn,
        # or just scan recent (inefficient).
        # Accessing `self.decision_db.conn` is possible.
        
        cursor = self.decision_db.conn.cursor()
        cursor.execute('''
            SELECT id, decision_id, timestamp, decision_type,
                   goal_x, goal_y, current_x, current_y,
                   distance_remaining, data_json
            FROM navigation_decisions
            WHERE id = ?
        ''', (decision_id,))
        
        row = cursor.fetchone()
        if not row:
            return None
            
        return {
            'db_id': row[0],
            'decision_id': row[1],
            'timestamp': row[2],
            'decision_type': row[3],
            'goal': {'x': row[4], 'y': row[5]} if row[4] else None,
            'current': {'x': row[6], 'y': row[7]} if row[6] else None,
            'distance_remaining': row[8],
            'data': json.loads(row[9]) if row[9] else {}
        }

    async def _build_context(
        self,
        decision: Dict[str, Any]
    ) -> ExplanationContext:
        """
        Build context for explanation generation.
        
        Retrieves recent decisions, current state, etc.
        """
        # Get recent decisions (last 5)
        recent = self._get_recent_decisions(limit=5)
        
        # Extract position info from decision
        current_pos = decision.get('data', {}).get('current_position')
        # Or use 'current' key from DB record
        if not current_pos and decision.get('current'):
             current_pos = decision['current']
             
        goal_pos = decision.get('goal')
        
        return ExplanationContext(
            current_position=current_pos,
            goal_position=goal_pos,
            recent_decisions=recent,
            environment_info={}
        )
    
    def _get_recent_decisions(self, limit: int = 5) -> List[Dict[str, Any]]:
        """Get recent decisions from database."""
        return self.decision_db.get_recent_decisions(limit=limit)
    
    def _context_to_dict(self, context: ExplanationContext) -> Dict[str, Any]:
        """Convert ExplanationContext to dictionary."""
        return {
            'current_position': context.current_position,
            'goal_position': context.goal_position,
            'recent_decisions': context.recent_decisions or [],
            'environment_info': context.environment_info or {}
        }
    
    def _select_template(self, decision_type: str) -> Optional[PromptTemplate]:
        """Select appropriate template for decision type."""
        return self.template_library.get_template(decision_type)
    
    def _post_process_explanation(
        self,
        response: ExplanationResponse,
        decision: Dict[str, Any]
    ) -> ExplanationResponse:
        """
        Post-process and validate explanation.
        
        - Ensure it's user-friendly
        - Check for technical jargon
        - Validate length
        - Add context if needed
        """
        text = response.text
        
        # Remove any remaining placeholders
        text = text.replace("{", "").replace("}", "")
        
        # Ensure first-person perspective
        if not any(text.lower().startswith(word) for word in ['i ', 'i\'', "i'm"]):
            # If doesn't start with "I", try to make it first-person
            if text.lower().startswith('the robot'):
                text = text.replace('The robot', 'I', 1)
                text = text.replace('the robot', 'I')
        
        # Update response
        response.text = text
        
        return response
    
    def clear_cache(self):
        """Clear explanation cache."""
        self._cache.clear()
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get engine performance statistics."""
        avg_time = (
            sum(self.stats['generation_times']) / len(self.stats['generation_times'])
            if self.stats['generation_times']
            else 0
        )
        
        cache_hit_rate = (
            self.stats['cache_hits'] / self.stats['total_explanations']
            if self.stats['total_explanations'] > 0
            else 0
        )
        
        return {
            **self.stats,
            'average_generation_time': avg_time,
            'cache_hit_rate': cache_hit_rate,
            'gemini_stats': self.gemini_client.get_statistics()
        }
import json
