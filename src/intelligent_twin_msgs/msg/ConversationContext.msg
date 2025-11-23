# Conversation context for LLM command parsing
# Contains history and current robot state

std_msgs/Header header

# Conversation history (last N turns)
string[] user_inputs
string[] robot_responses

# Current robot state
string current_location          # Semantic location (kitchen, hallway)
geometry_msgs/Pose current_pose  # Exact x, y, theta

# Parsed command information
string user_intent               # Parsed intent from Gemini
float32 confidence               # Confidence score (0-1)
string raw_transcript            # Original Whisper output
