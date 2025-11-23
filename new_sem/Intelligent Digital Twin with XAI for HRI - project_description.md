I'm developing an intelligent robotic system for my undergraduate special project at BITS Pilani Dubai under Dr. Sujala D. Shetty that combines three cutting-edge technologies:

1. CONVERSATIONAL MEMORY: Building a multi-turn dialogue system that remembers past interactions and understands spatial references like "go back to where you were before." This extends my previous ROS2 voice control project with context-aware conversations.

2. EXPLAINABLE AI (XAI) NAVIGATION: Creating a system where the robot explains its navigation decisions in natural language. When the robot changes its path or avoids obstacles, it will tell you WHY in plain English, making autonomous behavior transparent and trustworthy.

3. DIGITAL TWIN ANOMALY DETECTION: Running a parallel simulation (digital twin) in Gazebo that mirrors the real robot's expected behavior. By comparing real sensor data against the twin, I'm using machine learning to detect when something goes wrong (sensor failures, unexpected obstacles, hardware issues).

TECHNICAL STACK:
- ROS2 Humble (Robot Operating System)
- React TypeScript dashboard for visualization
- Gemini LLM for natural language processing and explanations
- Whisper API for voice recognition
- Gazebo for digital twin simulation
- Python ML models (scikit-learn/PyTorch) for anomaly detection

TIMELINE: 8 weeks (January-March 2025)

GOALS:
- Enable 5+ turn conversations with spatial context
- Generate real-time natural language explanations for robot decisions
- Achieve >80% accuracy in anomaly detection
- Create an integrated web dashboard showing all three components

This project addresses a gap in current research by integrating conversational AI, explainable robotics, and digital twin technology—three areas that typically exist separately. It builds on my proven ROS2 foundation from last semester and aligns with my supervisor's research in XAI, IoT, and Big Data.

I need help with:
- Implementation guidance (currently starting Week 1)
- Debugging ROS2 integration issues
- Prompt engineering for context-aware LLM conversations
- ML model design for anomaly detection
- System architecture decisions
- Literature review integration into methodology