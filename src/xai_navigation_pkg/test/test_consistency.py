#!/usr/bin/env python3
import os
import sys
import asyncio
import numpy as np
from typing import List

# Add package to path
sys.path.append(os.path.join(os.path.dirname(__file__), '../xai_navigation_pkg'))

from gemini_client import GeminiClient, ExplanationRequest
from prompt_templates import PromptTemplateLibrary

async def run_consistency_test():
    print("==================================================")
    print("      XAI Consistency Check Suite")
    print("==================================================")
    
    api_key = os.getenv('GEMINI_API_KEY')
    if not api_key:
        print("Error: GEMINI_API_KEY not set.")
        return

    client = GeminiClient(api_key=api_key)
    templates = PromptTemplateLibrary()
    
    # Test Scenario: Path Change
    scenario = {
        "type": "path_changed",
        "data": {
            "original_length": 12.0,
            "new_length": 14.5,
            "length_change": 2.5,
            "reason": "dynamic_obstacle",
            "max_deviation": 1.5
        },
        "context": {
            "current_position": {"x": 5.0, "y": 5.0},
            "goal_position": {"x": 10.0, "y": 10.0}
        }
    }
    
    iterations = 5
    print(f"Running {iterations} iterations for scenario: {scenario['type']}...")
    
    explanations = []
    
    template = templates.get_template(scenario['type'])
    request = ExplanationRequest(
        decision_type=scenario['type'],
        decision_data=scenario['data'],
        context=scenario['context'],
        template=template.template,
        temperature=0.3 # Low temperature for consistency
    )
    
    for i in range(iterations):
        print(f"Generating {i+1}/{iterations}...", end='\r')
        try:
            response = await client.generate_explanation(request)
            explanations.append(response.text)
        except Exception as e:
            print(f"\nError: {e}")
            
    print("\n\nGenerated Explanations:")
    for i, exp in enumerate(explanations):
        print(f"{i+1}. {exp}")
        
    # Analyze Consistency
    lengths = [len(exp.split()) for exp in explanations]
    avg_len = np.mean(lengths)
    std_len = np.std(lengths)
    
    print("\n--- Metrics ---")
    print(f"Average Word Count: {avg_len:.1f}")
    print(f"Std Dev Word Count: {std_len:.1f}")
    
    # Keyword check
    keywords = ["detour", "obstacle", "longer", "path"]
    print("\nKeyword Presence:")
    for kw in keywords:
        count = sum(1 for exp in explanations if kw in exp.lower())
        print(f"- '{kw}': {count}/{iterations} ({count/iterations*100:.0f}%)")
        
    if std_len < 5.0:
        print("\nPASS: Explanations are consistent in length.")
    else:
        print("\nWARN: Explanations show high variance in length.")

if __name__ == "__main__":
    asyncio.run(run_consistency_test())
