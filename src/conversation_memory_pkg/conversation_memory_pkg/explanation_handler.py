#!/usr/bin/env python3
"""
Explanation Handler for Conversation Memory

Bridges XAI explanations with conversation memory and voice output.
Allows users to ask "Why did you do that?" and get explanations.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from typing import Optional, Dict, Any


class ExplanationHandler(Node):
    """
    Handles explanation requests in conversation.
    
    Subscribes to:
    - /navigation/explanation_detailed - XAI explanations
    - /conversation/user_input - User questions
    
    Publishes to:
    - /conversation/robot_response - Explanation to TTS
    """
    
    def __init__(self):
        super().__init__('explanation_handler')
        
        # Declare parameters
        self.declare_parameter('auto_speak_explanations', False)
        
        # Subscribe to explanations
        self.explanation_sub = self.create_subscription(
            String,
            '/navigation/explanation_detailed',
            self._explanation_callback,
            10
        )
        
        # Subscribe to user input
        self.user_input_sub = self.create_subscription(
            String,
            '/conversation/user_input',
            self._user_input_callback,
            10
        )
        
        # Publish robot responses
        self.response_pub = self.create_publisher(
            String,
            '/conversation/robot_response',
            10
        )
        
        # Store latest explanation
        self.latest_explanation: Optional[Dict[str, Any]] = None
        self.explanation_history = []
        
        self.get_logger().info('Explanation Handler initialized')
    
    def _explanation_callback(self, msg: String):
        """Store incoming explanation."""
        try:
            explanation_data = json.loads(msg.data)
            self.latest_explanation = explanation_data
            self.explanation_history.append(explanation_data)
            
            # Keep only last 10
            if len(self.explanation_history) > 10:
                self.explanation_history.pop(0)
            
            # Automatically speak explanation if enabled
            if self.get_parameter('auto_speak_explanations').value:
                text = explanation_data.get('text', "No explanation text available.")
                self._speak_explanation(text)
                
        except Exception as e:
            self.get_logger().error(f"Failed to process explanation: {e}")
    
    def _user_input_callback(self, msg: String):
        """Handle user questions that might be asking for explanation."""
        user_text = msg.data.lower()
        
        # Check if user is asking for explanation
        explanation_triggers = [
            'why did you',
            'why are you',
            'what happened',
            'explain',
            'what are you doing',
            'why',
            'how come'
        ]
        
        if any(trigger in user_text for trigger in explanation_triggers):
            self._provide_explanation(user_text)
    
    def _provide_explanation(self, user_question: str):
        """Provide explanation in response to user question."""
        if not self.latest_explanation:
            response = "I haven't made any navigation decisions recently."
        else:
            # Get explanation text
            explanation_text = self.latest_explanation.get('text', '')
            decision_type = self.latest_explanation.get('decision_type', '')
            
            # Contextualize based on question
            if 'why' in user_question and 'stop' in user_question:
                # User asking specifically about stopping
                if 'obstacle' in explanation_text.lower():
                    response = explanation_text
                else:
                    response = "I stopped because: " + explanation_text
            else:
                # General explanation request
                response = explanation_text
        
        self._speak_explanation(response)
    
    def _speak_explanation(self, text: str):
        """Send explanation to TTS."""
        msg = String()
        msg.data = text
        self.response_pub.publish(msg)
        self.get_logger().info(f"Speaking: {text}")


def main(args=None):
    rclpy.init(args=args)
    node = ExplanationHandler()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
