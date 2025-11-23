#!/usr/bin/env python3
import os
import sys
import asyncio
import json
from datetime import datetime
from typing import List, Dict, Any

# Add package to path
sys.path.append(os.path.join(os.path.dirname(__file__), '../xai_navigation_pkg'))

from gemini_client import GeminiClient, ExplanationRequest
from prompt_templates import PromptTemplateLibrary

async def run_test():
    print("==================================================")
    print("      XAI User Comprehension Test Suite")
    print("==================================================")
    
    api_key = os.getenv('GEMINI_API_KEY')
    if not api_key:
        print("Error: GEMINI_API_KEY not set.")
        return

    client = GeminiClient(api_key=api_key)
    templates = PromptTemplateLibrary()
    
    scenarios = [
        {
            "name": "Obstacle Avoidance",
            "type": "obstacle_detected",
            "data": {
                "obstacle_x": 2.5,
                "obstacle_y": 1.0,
                "distance_to_robot": 1.2,
                "severity": "high",
                "action": "stop_and_replan"
            },
            "context": {
                "current_position": {"x": 1.0, "y": 1.0},
                "goal_position": {"x": 5.0, "y": 5.0}
            }
        },
        {
            "name": "Path Detour",
            "type": "path_changed",
            "data": {
                "original_length": 10.0,
                "new_length": 15.0,
                "length_change": 5.0,
                "reason": "dynamic_obstacle",
                "max_deviation": 2.5
            },
            "context": {
                "current_position": {"x": 0.0, "y": 0.0},
                "goal_position": {"x": 10.0, "y": 0.0}
            }
        },
        {
            "name": "Goal Aborted",
            "type": "goal_aborted",
            "data": {
                "reason": "path_blocked",
                "attempts": 3,
                "final_x": 2.0,
                "final_y": 2.0,
                "distance_from_goal": 8.0
            },
            "context": {
                "current_position": {"x": 2.0, "y": 2.0},
                "goal_position": {"x": 10.0, "y": 10.0}
            }
        }
    ]
    
    results = []
    
    for i, scenario in enumerate(scenarios):
        print(f"\n--- Scenario {i+1}/{len(scenarios)}: {scenario['name']} ---")
        print(f"Data: {json.dumps(scenario['data'], indent=2)}")
        
        print("\nGenerating explanation...")
        
        try:
            template = templates.get_template(scenario['type'])
            request = ExplanationRequest(
                decision_type=scenario['type'],
                decision_data=scenario['data'],
                context=scenario['context'],
                template=template.template
            )
            
            response = await client.generate_explanation(request)
            
            print("\n[ROBOT EXPLANATION]:")
            print(f"\"{response.text}\"")
            print(f"(Confidence: {response.confidence:.2f}, Time: {response.generation_time:.2f}s)")
            
            while True:
                try:
                    rating = input("\nRate clarity (1-5): ")
                    rating = int(rating)
                    if 1 <= rating <= 5:
                        break
                    print("Please enter a number between 1 and 5.")
                except ValueError:
                    print("Invalid input.")
            
            feedback = input("Optional feedback: ")
            
            results.append({
                "scenario": scenario['name'],
                "explanation": response.text,
                "rating": rating,
                "feedback": feedback,
                "metrics": {
                    "confidence": response.confidence,
                    "generation_time": response.generation_time
                }
            })
            
        except Exception as e:
            print(f"Error generating explanation: {e}")
    
    # Save results
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"comprehension_test_{timestamp}.json"
    with open(filename, 'w') as f:
        json.dump(results, f, indent=2)
        
    print(f"\nTest complete. Results saved to {filename}")
    
    # Calculate average rating
    avg_rating = sum(r['rating'] for r in results) / len(results) if results else 0
    print(f"Average Rating: {avg_rating:.2f}/5.0")

if __name__ == "__main__":
    asyncio.run(run_test())
