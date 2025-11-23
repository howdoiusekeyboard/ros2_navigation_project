#!/usr/bin/env python3
"""
Test script for PromptTemplateLibrary.
"""

import unittest
from xai_navigation_pkg.prompt_templates import PromptTemplateLibrary

class TestPromptTemplates(unittest.TestCase):
    def setUp(self):
        self.library = PromptTemplateLibrary()
        
    def test_load_defaults(self):
        """Test that default templates are loaded."""
        templates = self.library.list_templates()
        self.assertIn('path_changed', templates)
        self.assertIn('obstacle_detected', templates)
        self.assertIn('goal_aborted', templates)
        self.assertIn('goal_reached', templates)
        self.assertIn('recovery_behavior', templates)
        self.assertIn('planning_failed', templates)
        self.assertIn('waiting', templates)
        
    def test_get_template(self):
        """Test retrieving a specific template."""
        template = self.library.get_template('path_changed')
        self.assertIsNotNone(template)
        self.assertEqual(template.decision_type, 'path_changed')
        self.assertIn('CONTEXT', template.template)
        self.assertIn('DECISION_DATA', template.template)
        
    def test_get_template_info(self):
        """Test retrieving template info."""
        info = self.library.get_template_info('obstacle_detected')
        self.assertIsNotNone(info)
        self.assertEqual(info['decision_type'], 'obstacle_detected')
        self.assertIsInstance(info['max_tokens'], int)
        
    def test_missing_template(self):
        """Test retrieving a non-existent template."""
        template = self.library.get_template('non_existent')
        self.assertIsNone(template)

if __name__ == '__main__':
    unittest.main()
