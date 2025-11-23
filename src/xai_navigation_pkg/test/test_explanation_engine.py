#!/usr/bin/env python3
"""Tests for ExplanationEngine."""

import pytest
import asyncio
import os
from unittest.mock import Mock, patch, AsyncMock

from xai_navigation_pkg.explanation_engine import ExplanationEngine, ExplanationContext
from xai_navigation_pkg.gemini_client import ExplanationResponse
from xai_navigation_pkg.decision_database import DecisionDatabase


class TestExplanationEngine:
    """Test suite for ExplanationEngine."""
    
    @pytest.fixture
    def mock_db(self):
        """Create mock database."""
        db = Mock(spec=DecisionDatabase)
        db.get_statistics.return_value = {
            'total_decisions': 10,
            'last_decision_id': 10
        }
        # Mock connection cursor for _get_decision
        db.conn = Mock()
        cursor = Mock()
        db.conn.cursor.return_value = cursor
        
        # Setup fetchone return value for _get_decision
        # id, decision_id, timestamp, decision_type, goal_x, goal_y, current_x, current_y, distance, data
        cursor.fetchone.return_value = (
            1, 1, 1234567890, 'path_changed', 
            5.0, 3.0, 0.0, 0.0, 10.0, 
            '{"original_length": 5.0, "new_length": 5.5, "length_change": 0.5, "reason": "obstacle_avoidance", "max_deviation": 0.8}'
        )
        
        return db
    
    @pytest.fixture
    def engine(self, mock_db):
        """Create explanation engine with mock dependencies."""
        # Use a fake API key for testing
        return ExplanationEngine(
            gemini_api_key='test_key_123',
            decision_db=mock_db,
            cache_enabled=True
        )
    
    @pytest.mark.asyncio
    async def test_explain_path_change(self, engine):
        """Test explanation generation for path change."""
        # Mock Gemini response
        mock_response = ExplanationResponse(
            text="I changed my path because there's an obstacle blocking the original route. The new path adds 0.5 meters but avoids the obstruction.",
            confidence=0.85,
            generation_time=0.8,
            token_count=25,
            cached=False
        )
        
        with patch.object(
            engine.gemini_client,
            'generate_explanation',
            return_value=mock_response
        ):
            result = await engine.explain_decision(1)
        
        assert result.text is not None
        assert 'path' in result.text.lower()
        assert result.confidence > 0.7
        assert result.generation_time < 2.0
    
    @pytest.mark.asyncio
    async def test_caching(self, engine):
        """Test that explanations are properly cached."""
        mock_response = ExplanationResponse(
            text="I've arrived at the goal location.",
            confidence=0.95,
            generation_time=0.5,
            token_count=7,
            cached=False
        )
        
        with patch.object(
            engine.gemini_client,
            'generate_explanation',
            return_value=mock_response
        ) as mock_generate:
            # First call - should hit API
            result1 = await engine.explain_decision(1)
            assert mock_generate.call_count == 1
            
            # Second call - should use cache
            result2 = await engine.explain_decision(1)
            assert mock_generate.call_count == 1  # Not called again
            assert result2.cached is True
    
    def test_template_selection(self, engine):
        """Test that correct templates are selected."""
        template = engine._select_template('path_changed')
        assert template is not None
        assert template.decision_type == 'path_changed'
        
        template = engine._select_template('nonexistent_type')
        assert template is None

if __name__ == '__main__':
    pytest.main([__file__, '-v'])
