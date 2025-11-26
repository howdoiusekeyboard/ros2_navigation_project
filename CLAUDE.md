# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a complete ROS 2 Humble autonomous navigation system with an integrated React-based web dashboard. The project demonstrates the full navigation pipeline: SLAM mapping → localization → path planning, designed for TurtleBot3 in Gazebo simulation.

**Intelligent Systems Layer:** This project extends Nav2 with:
- **Explainable AI (XAI)**: Natural language explanations of navigation decisions via Gemini 2.0 Flash
- **Decision Logging**: SQLite database capturing all Nav2 events with zero-latency impact
- **Conversation Memory**: Multi-turn dialogue with spatial reference resolution ("go back to the kitchen")
- **Backend API**: FastAPI server providing unified data layer and voice command parsing
- **Digital Twin**: Parallel simulation monitoring with ML-based anomaly detection

**Project Context:** 8-week undergraduate research project integrating conversational AI, explainable robotics, and digital twin technology.

## Build System & Common Commands

### ROS 2 Workspace

```bash
# Build all packages
colcon build --symlink-install

# Build specific package
colcon build --packages-select <package_name>

# Source workspace (required in each terminal)
source install/setup.bash

# Set TurtleBot3 model (required)
export TURTLEBOT3_MODEL=burger
```

### Testing

```bash
# Run tests for all packages
colcon test

# Run tests for specific package
colcon test --packages-select <package_name>

# View test results
colcon test-result --all
```

### Web Dashboard

```bash
# Start complete system (rosbridge + turtlesim + web server)
./start_robot_dashboard.sh

# Manual web dashboard development
cd project
npm install  # or bun install
npm run dev  # starts at http://localhost:5173
```

## Architecture & Key Concepts

### Navigation Pipeline Flow

The system operates in distinct phases that must be understood when modifying code:

1. **SLAM Mapping Phase** (`cartographer_slam`): Creates occupancy grid maps using Google Cartographer. Outputs .pgm/.yaml map files to `map_server/config/`.

2. **Localization Phase** (`localization_server`): Uses AMCL (particle filter) to estimate robot pose against the saved map. Publishes the critical `map → odom` transform.

3. **Planning Phase** (`path_planner_server`): Four coordinated Nav2 servers work together:
   - **Planner Server**: Global path planning using NavfnPlanner (Dijkstra/A*)
   - **Controller Server**: Local trajectory generation using DWB (Dynamic Window Approach)
   - **BT Navigator**: Orchestrates planning/control via behavior trees
   - **Behavior Server**: Handles recovery behaviors (spin, backup, wait)

### Coordinate Frame Hierarchy

Understanding this hierarchy is critical when debugging transforms or adding sensors:

```
map (global, fixed reference)
 └─> odom (published by AMCL, corrects for drift)
      └─> base_footprint / base_link (robot center)
           └─> laser (sensor frame)
```

### Lifecycle Management Pattern

All major nodes use Nav2's `lifecycle_manager` for coordinated startup/shutdown. When adding new nodes to the navigation stack, they must be added to the appropriate lifecycle manager's node list.

### Costmap Architecture

Two costmaps with distinct purposes:
- **Global Costmap** (planner_server.yaml): Large, map-frame, slow update (1Hz), uses static map layer
- **Local Costmap** (controller.yaml): Small rolling window (3m × 3m), odom-frame, fast update (5Hz), no static layer

Both use layered costmaps: static layer → obstacle layer → inflation layer

## Intelligent Systems Architecture

This project extends the standard Nav2 stack with an XAI (Explainable AI) observation layer that captures navigation decisions and generates natural language explanations.

### Data Flow Overview

```
Voice Input → Backend (Gemini) → ROS2 cmd_vel → Nav2 Stack
                                                    ↓
                                         Nav2Monitor (XAI Package)
                                                    ↓
                                        DecisionDatabase (Local SQLite)
                                                    ↓
                                          BackendSync (Async HTTP)
                                                    ↓
                                      Backend NavigationDB (Unified Storage)
                                                    ↓
                                    Web Dashboard / Explanation Generation
```

### The XAI Navigation Pipeline ("Passive Observer" Pattern)

**Critical Design Principle:** The XAI layer observes navigation but never blocks it. All logging operations are async with <10ms latency impact.

#### Component Details

**1. Nav2Monitor** (`xai_navigation_pkg/nav2_monitor.py`)
- Wraps the `/navigate_to_pose` action client as a "man-in-the-middle"
- Captures events: goal_sent, feedback (throttled to 5Hz), goal_reached/aborted/canceled
- Does NOT modify Nav2 behavior - read-only observation
- Entry point: `_emit_decision()` method triggers all downstream logging

**2. DecisionDatabase** (`xai_navigation_pkg/decision_database.py`)
- Local SQLite with WAL (Write-Ahead Logging) mode at `~/.ros/navigation_decisions.db`
- Tables:
  - `navigation_decisions`: Main event log (decision_type, pose, goal, nearest_obstacle_dist, active_behavior, synced_to_backend)
  - `path_changes`: Path deviation events (when deviation > threshold)
  - `obstacle_events`: Obstacle detections with severity
  - `telemetry_snapshots`: Battery, CPU, WiFi, motor currents (1Hz for anomaly detection)
- Auto-pruning: Keeps last 7 days of synced data
- Fast writes (<10ms) critical for zero-latency requirement

**3. CostmapProcessor** (`xai_navigation_pkg/costmap_processor.py`)
- Subscribes to `/local_costmap/costmap` and `/global_costmap/costmap`
- Analyzes occupancy grids for obstacles along planned path
- Updates `nearest_obstacle_dist` field in decision logs
- Provides context for obstacle-related explanations

**4. PathAnalyzer** (`xai_navigation_pkg/path_analyzer.py`)
- Subscribes to `/plan` (global) and `/local_plan`
- Detects path changes when deviation > `deviation_threshold` (0.3m default)
- Logs to `path_changes` table with reason inference (obstacle_avoidance, optimization, minor_adjustment)

**5. ObstacleDetector** (`xai_navigation_pkg/obstacle_detector.py`)
- Correlates costmap data with robot pose
- Severity classification: critical (<0.3m), warning (<1.0m)
- Logs to `obstacle_events` table with `action_taken` field

**6. BackendSync** (`xai_navigation_pkg/backend_sync.py`)
- Background thread (5s interval, configurable via `sync_interval` parameter)
- Batch syncs unsynced decisions (50 per batch) to backend API
- Endpoint: `POST /api/v1/navigation/decisions/sync`
- Handles offline mode: Queues locally, syncs when backend returns
- Retry logic: Exponential backoff on failure

### Configuration & Parameters

**Critical File:** `src/xai_navigation_pkg/config/xai_params.yaml`

Key parameters (NO magic numbers in code):
- `critical_distance: 0.3` - Obstacle detection threshold (meters)
- `warning_distance: 1.0` - Caution zone (meters)
- `deviation_threshold: 0.3` - Path change sensitivity (meters)
- `sync_interval: 5.0` - Backend sync frequency (seconds)
- `feedback_throttle: 0.2` - Max feedback rate, 5Hz (seconds)
- `backend_url: http://localhost:8000` - Backend API endpoint
- `enable_logging: true` - Master switch for decision logging
- `enable_explanations: true` - Enable Gemini explanation generation

**Why These Thresholds Matter:** They directly affect explanation quality. Too sensitive = noise, too lenient = missed events.

## Backend System Architecture

The FastAPI backend provides a unified HTTP/WebSocket API layer that bridges the web dashboard, voice control, and ROS2 systems.

**Location:** `backend/`
**Tech Stack:** FastAPI + Async SQLite + Gemini 2.0 Flash + Loguru logging

### Core Services

**1. Gemini Command Parser** (`app/services/gemini_service.py`)
- **Model:** `gemini-2.0-flash-exp` (low latency, cost-effective)
- **Temperature:** 0.3 (deterministic parsing for robot commands)
- **Input:** Natural language text from voice transcription
- **Output:** `RobotCommand(action, parameters, confidence, reasoning)`
- **Safety Validation:** Clamps velocities/distances to TurtleBot3 Burger limits
  - Linear velocity: -0.22 to 0.22 m/s
  - Angular velocity: -2.84 to 2.84 rad/s
- **Fallback:** Regex-based parser for offline operation (supports: stop, spin, forward, backward)
- **Context-Aware:** Integrates conversation history for spatial reference resolution

**2. Conversation Database** (`app/database/conversation_db.py`)
- **Storage:** Async SQLite at `~/.ros/backend_data/conversation_history.db`
- **Schema:** session_id, turn_number, user_text, bot_text, location_x/y, location_label, timestamp
- **Purpose:** Powers "go to the kitchen" → resolves to coordinates from previous navigation
- **Context Builder:** Generates LLM-ready conversation summaries with spatial references

**3. Navigation Database** (`app/database/navigation_db.py`)
- **Storage:** Mirrors the ROS2 DecisionDatabase schema
- **Receives:** Synced decisions from XAI Navigator via batch uploads
- **Provides:** REST API for dashboard queries, filtering, and statistics

### Key API Endpoints

**Voice Command Pipeline:**
```
POST /api/v1/execute_voice_command
Input: {transcript, session_id}
Output: {parsed_command, execution_status, latency_ms}
```
Complete pipeline: transcript → context retrieval → Gemini → robot control → memory storage

**Navigation Decision Sync (Week 3):**
```
POST /api/v1/navigation/decisions/sync - Batch upload from ROS2 (50 decisions/request)
GET /api/v1/navigation/decisions - Query decisions (pagination, session filter)
GET /api/v1/navigation/decisions/type/{decision_type} - Filter by type
GET /api/v1/navigation/path_changes - Path deviation events
GET /api/v1/navigation/obstacles - Obstacle detection events
GET /api/v1/navigation/statistics - Aggregated stats (total decisions, avg confidence, etc.)
```

**Conversation Memory:**
```
GET /api/v1/conversation/history/{session_id} - Turn-by-turn history
GET /api/v1/conversation/spatial_refs/{session_id} - Labeled locations ("kitchen" → (2.0, 3.0))
GET /api/v1/conversation/sessions - Recent sessions with stats
```

**Health & Status:**
```
GET /health - System health (ROS2 connection, Gemini API, database status)
GET /docs - Swagger UI API documentation
```

### Environment Configuration

**Critical Gotcha:** Gemini API key must be set in **TWO locations**:
1. `backend/.env` for voice command parsing
2. XAI Navigator launch args for explanation generation

**Backend .env Structure:**
```env
GEMINI_API_KEY=AIza...                             # Command parsing + explanations
OPENAI_API_KEY=sk-...                              # Whisper transcription (optional)
DATABASE_URL=sqlite+aiosqlite:///~/.ros/backend_data/app.db
CORS_ORIGINS=["http://localhost:5173"]             # Web dashboard origin
LOG_LEVEL=INFO                                     # DEBUG for development
ROS_DOMAIN_ID=0                                    # Must match ROS2 network
```

**Startup:** `./backend/run.sh` handles venv setup, dependency installation, and uvicorn launch

## Explanation System (Week 4 Integration)

The explanation system generates natural language descriptions of navigation decisions using Gemini 2.0 Flash.

### How Explanations Work

```
User asks "Why did you turn?" (via dashboard or ROS service)
    ↓
Backend ExplanationEngine queries NavigationDatabase
    ↓
Retrieves: recent decisions, costmap snapshots, path changes, obstacles
    ↓
PromptTemplateLibrary selects template based on decision_type
    ↓
GeminiClient generates natural language explanation
    ↓
Post-processing: Ensures first-person perspective, removes jargon
    ↓
Explanation published to /navigation/explanation topic
    ↓
conversation_memory_pkg reads and converts to speech (TTS)
```

### Key Components

**1. ExplanationEngine** (`xai_navigation_pkg/explanation_engine.py`)
- **Caching:** Stores explanations by decision_id to avoid duplicate API calls
- **Context Building:** Gathers last 5 decisions + robot state + environment info
- **Template Selection:** Matches decision_type to prompt template
- **Stats Tracking:** Average latency, confidence, cache hit rate

**2. PromptTemplateLibrary** (`xai_navigation_pkg/prompt_templates.py`)
- Templates for: `goal_sent`, `goal_reached`, `path_changed`, `obstacle_detected`, `goal_aborted`
- Each template includes: system role, example outputs, safety constraints
- **First-person enforcement:** "I turned" not "The robot turned"

**3. GeminiClient** (`xai_navigation_pkg/gemini_client.py`)
- **Retry Logic:** 3 attempts with exponential backoff
- **Timeout:** 5s per request
- **Error Handling:** Fallback to template-based explanation if Gemini fails
- **Rate Limiting:** Max 10 requests/minute (configurable)

### Explanation Quality Guidelines

**Good Explanation:**
> "I turned right because I detected a static obstacle 0.8 meters ahead. The obstacle was blocking my planned path to the goal, so I requested a new path that goes around it."

**Bad Explanation (too technical):**
> "The local costmap indicated lethal cost cells at grid coordinates (45, 67) within the trajectory bounds. DWB controller generated an alternative trajectory with increased path cost but maintained feasibility."

Post-processing catches technical jargon and converts to first-person, user-friendly language.

### Testing Explanations

```bash
# 1. Navigate robot to trigger decisions
# 2. Query latest explanation:
ros2 topic echo /navigation/explanation --once

# 3. Check explanation cache:
curl http://localhost:8000/api/v1/navigation/decisions | jq '.decisions[] | select(.decision_type=="path_changed")'

# 4. Force regenerate explanation:
ros2 service call /xai_navigator/explain_latest std_srvs/srv/Trigger
```

## Database Architecture & Sync Flow

### Two-Database Architecture

**Why Two Databases?**
- **Local (ROS2 side):** Fast writes, zero network dependency, survives backend crashes
- **Backend (unified):** Cross-system queries, dashboard access, long-term storage

**Local Database:** `~/.ros/navigation_decisions.db`
- SQLite with WAL mode for concurrent reads during writes
- Written by: XAI Navigator Node
- Read by: ExplanationEngine (for context), BackendSync (for upload)
- Cleanup: Auto-prunes synced decisions older than 7 days

**Backend Database:** `~/.ros/backend_data/navigation_history.db`
- SQLite with async operations (aiosqlite)
- Written by: Backend API sync endpoint
- Read by: Dashboard, REST API queries
- No auto-pruning (long-term archive)

### Sync Flow Details

**Phase 1: Decision Capture (ROS2)**
```python
# In XAI Navigator Node
def _on_decision(self, decision: Dict[str, Any]):
    # 1. Log to local database (< 10ms)
    db_id = self.decision_db.log_decision(decision, session_id)

    # 2. Trigger analysis (async, doesn't block)
    self.path_analyzer.analyze(decision)
    self.obstacle_detector.check(decision)
```

**Phase 2: Background Sync (Every 5s)**
```python
# In BackendSync thread
def _do_sync(self):
    # 1. Get unsynced decisions (up to 50)
    unsynced = self.decision_db.get_unsynced_decisions(50)

    # 2. POST to backend
    response = requests.post('/api/v1/navigation/decisions/sync', json=unsynced)

    # 3. Mark as synced only if 200 OK
    if response.status_code == 200:
        db_ids = [d['db_id'] for d in unsynced]
        self.decision_db.mark_synced(db_ids)
```

**Phase 3: Backend Storage**
```python
# In Backend API
@app.post("/api/v1/navigation/decisions/sync")
async def sync_decisions(decisions: list):
    nav_db = await get_navigation_db()
    count = await nav_db.store_decisions_batch(decisions)
    return {"success": True, "synced_count": count}
```

### Handling Network Failures

**Scenario:** Backend is down, robot keeps navigating.

**Behavior:**
1. Local database continues capturing all decisions (no data loss)
2. BackendSync logs errors: `stats['sync_errors']` increments
3. Unsynced decisions queue up (marked `synced_to_backend=0`)
4. When backend returns, next sync uploads entire backlog (50 at a time)

**Recovery Test:**
```bash
# 1. Start full system
# 2. Navigate robot to generate decisions
# 3. Kill backend: pkill -f uvicorn
# 4. Navigate more
# 5. Check local database has unsynced records:
sqlite3 ~/.ros/navigation_decisions.db \
  "SELECT COUNT(*) FROM navigation_decisions WHERE synced_to_backend=0;"
# 6. Restart backend
# 7. Within 5s, check backend logs for sync message
# 8. Verify unsynced count drops to 0
```

## Configuration File Relationships

Understanding these relationships is essential when modifying parameters:

- **Launch files** (`.launch.py`) reference parameter files by package path using `get_package_share_directory()`
- **Parameter files** (`.yaml`) must match node names and namespaces exactly
- **Behavior tree XML** path in `bt_navigator.yaml` must be absolute after installation
- **Map files** come in pairs: `.pgm` (image) + `.yaml` (metadata with origin, resolution)
- **setup.py** in each package defines what gets installed (launch files, configs, maps)

## Critical Configuration Notes

### TurtleBot3 Model Consistency
All configurations assume `TURTLEBOT3_MODEL=burger`. Robot footprint parameters (radius: 0.15m, inflation: 0.35m) are tuned for this model. Changing models requires updating costmap parameters in both planner_server.yaml and controller.yaml.

### Simulation Time
All nodes use `use_sim_time: True` for Gazebo. When deploying to real hardware, this must be changed to `False` across all launch files.

### Frame ID Consistency
Some config files reference `base_footprint` while others use `base_link`. TurtleBot3 uses `base_footprint` as the primary robot frame. Ensure consistency when adding new nodes.

### Behavior Tree XML Path
The `default_nav_to_pose_bt_xml` parameter in `bt_navigator.yaml` requires an absolute path post-installation. It's currently passed via launch file, but direct YAML edits will fail if not using the installed path.

## Package Structure & Responsibilities

### cartographer_slam
- **Purpose**: Real-time SLAM mapping
- **Key file**: `config/cartographer.lua` - Cartographer tuning parameters (resolution, scan matching)
- **Launch**: `launch/cartographer.launch.py` - Starts cartographer_node + occupancy_grid_node
- **Output**: Generates maps saved to `map_server/config/`

### map_server
- **Purpose**: Serves static pre-built maps
- **Key file**: `config/turtlebot_area.yaml` - Map metadata (origin: [-4.31, -5.33], resolution: 0.05m)
- **Launch**: `launch/nav2_map_server.launch.py` - Uses lifecycle management for reliable startup

### localization_server
- **Purpose**: AMCL-based robot localization
- **Key file**: `config/amcl_config.yaml` - Particle filter parameters (200-8000 particles, 60 laser beams)
- **Launch**: `launch/localization.launch.py` - Starts both map_server AND amcl together
- **Initial pose**: Default at x=-4.44, y=2.32, yaw=0.33 (configurable via launch args)

### path_planner_server
- **Purpose**: Complete Nav2 navigation stack
- **Key files**:
  - `config/planner_server.yaml` - Global planning + global costmap
  - `config/controller.yaml` - Local planning + local costmap + velocity limits
  - `config/bt_navigator.yaml` - Behavior tree plugin configuration
  - `config/behavior.xml` - Navigation behavior tree logic with recovery sequence
  - `config/recovery.yaml` - Recovery behavior parameters
- **Launch**: `launch/pathplanner.launch.py` - Orchestrates all four Nav2 servers with lifecycle management

### project/ (Web Dashboard)
- **Tech stack**: React 18 + TypeScript + Vite + Tailwind CSS
- **ROS integration**: rosbridge_websocket on port 9090
- **Pages**: RobotDashboard, SpeechProcessing, MemoryManagement, CommandControl, Settings
- **Control topic**: Publishes to `/turtle1/cmd_vel`

### xai_navigation_pkg
- **Purpose:** Explainable AI decision logging and explanation generation
- **Key modules:**
  - `xai_navigator_node.py` - Main orchestrator node (console script entry point)
  - `nav2_monitor.py` - Nav2 action client wrapper for observation
  - `decision_database.py` - Local SQLite storage with WAL mode
  - `explanation_engine.py` - Gemini-based explanation generation
  - `backend_sync.py` - Async sync to backend API
  - `costmap_processor.py` - Obstacle detection from costmaps
  - `path_analyzer.py` - Path deviation detection
  - `prompt_templates.py` - LLM prompt templates for explanations
  - `gemini_client.py` - Gemini API integration with retry logic
- **Configuration:** `config/xai_params.yaml` - All tuning parameters
- **Launch:** `launch/xai_navigator.launch.py` - Main XAI node with configurable parameters
- **Critical:** Must run alongside Nav2 stack. Observes but does not control navigation.
- **Testing:** `test/` directory with comprehensive test suite (integration, decision_database, nav2_monitor, explanation_engine)

### backend/ (FastAPI Server)
- **Purpose:** Unified backend for voice control, conversation memory, and XAI data
- **Tech:** FastAPI + SQLite (aiosqlite) + Gemini 2.0 Flash + Loguru logging
- **Key files:**
  - `app/main.py` - Main API with all endpoints (1091 lines)
  - `app/services/gemini_service.py` - Command parser with safety validation
  - `app/services/context_builder.py` - Conversation context for LLM
  - `app/database/conversation_db.py` - Multi-turn dialogue storage
  - `app/database/navigation_db.py` - Navigation decision archive
  - `app/ros2_client/robot_controller.py` - ROS2 publisher for cmd_vel
- **Launch:** `./run.sh` (handles venv setup and uvicorn)
- **API Docs:** http://localhost:8000/docs (Swagger UI)

### conversation_memory_pkg
- **Purpose:** Multi-turn dialogue with spatial reference resolution
- **Key modules:**
  - `conversation_memory_node.py` - Main ROS2 node for conversation state
  - `explanation_handler.py` - Converts text explanations to speech (TTS)
- **Integration:** Reads `/navigation/explanation` topic, speaks via TTS
- **Database:** Uses backend conversation_db for persistent storage

### intelligent_twin_msgs
- **Purpose:** Custom ROS2 message definitions (CMake package, not Python)
- **Messages:**
  - `NavigationDecision.msg` - Decision with simple + detailed explanations
  - `ConversationContext.msg` - Dialogue state with spatial context
  - `AnomalyAlert.msg` - Digital twin anomaly detection results
  - `SensorComparison.msg` - Real vs. simulated sensor comparison
- **Usage:** Shared across XAI, conversation memory, and digital twin packages

### digital_twin_pkg
- **Purpose:** Parallel simulation monitoring with anomaly detection
- **Key modules:**
  - `digital_twin_monitor_node.py` - Dual robot setup with Isolation Forest + SHAP
  - `command_synchronizer_node.py` - Ensures both robots receive same commands
- **Launch:** `launch/dual_robot.launch.py` - Spawns two Gazebo worlds
- **Status:** Experimental, not required for basic XAI operation

## Development Patterns

### Adding a New Navigation Node
1. Add node to appropriate package's `setup.py` entry_points
2. Add node name to lifecycle_manager's `node_names` list in launch file
3. Create parameter file in `config/` directory
4. Load parameters in launch file using `Node` parameter argument
5. Install config file via `data_files` in `setup.py`

### Modifying Costmap Behavior
- **Obstacle detection sensitivity**: Adjust `obstacle_range` and `raytrace_range` in costmap configs
- **Safety margins**: Modify `inflation_radius` in inflation_layer parameters
- **Performance**: Balance `update_frequency` vs computational load

### Changing Path Planning Behavior
- **Path smoothness**: Adjust `tolerance` in NavfnPlanner plugin
- **Speed/agility tradeoff**: Modify `max_vel_x`, `max_vel_theta` in controller.yaml
- **Trajectory evaluation**: Tune DWB critics in controller.yaml (PathAlign, GoalAlign, PathDist, GoalDist weights)
- **Recovery logic**: Edit behavior.xml to change retry sequences, or modify recovery.yaml for behavior parameters

### Map Resolution Consistency
Current system uses 0.05m resolution throughout (cartographer.lua, map YAML files, costmap configs). Changing this requires coordinated updates across all configuration files.

### Adding a New Decision Type

When you want to log a new type of navigation event:

1. **Define decision type string** (use snake_case):
   ```python
   # Example: "recovery_behavior_triggered"
   ```

2. **Emit from XAI Navigator Node:**
   ```python
   def _on_recovery_behavior(self, msg):
       self.nav2_monitor._emit_decision('recovery_behavior_triggered', {
           'behavior_type': msg.behavior,
           'timestamp': time.time(),
           'current_x': self.current_pose.x,
           'current_y': self.current_pose.y
       })
   ```

3. **Add prompt template** (`prompt_templates.py`):
   ```python
   self.templates['recovery_behavior_triggered'] = PromptTemplate(
       name="recovery_behavior",
       template="Explain why the robot executed a recovery behavior...",
       max_tokens=150,
       temperature=0.4
   )
   ```

4. **Test with database query:**
   ```bash
   sqlite3 ~/.ros/navigation_decisions.db \
     "SELECT * FROM navigation_decisions WHERE decision_type='recovery_behavior_triggered';"
   ```

5. **Verify backend sync:**
   ```bash
   curl http://localhost:8000/api/v1/navigation/decisions/type/recovery_behavior_triggered
   ```

### Modifying Explanation Quality

**Problem:** Explanations are too technical or generic.

**Solutions:**

1. **Adjust prompt template** (`prompt_templates.py`):
   - Add more examples of good vs. bad explanations
   - Emphasize first-person perspective
   - Add domain-specific vocabulary constraints

2. **Tune Gemini parameters** (`explanation_engine.py`):
   ```python
   # More creative (less deterministic)
   temperature=0.7  # Default: 0.4

   # Longer explanations
   max_tokens=300  # Default: 150
   ```

3. **Improve context** (`explanation_engine.py` → `_build_context`):
   - Include more recent decisions (increase from 5 to 10)
   - Add costmap visualization data
   - Include robot's current "state of mind" (battery, errors)

4. **Add post-processing rules** (`explanation_engine.py` → `_post_process_explanation`):
   ```python
   # Example: Replace technical terms
   text = text.replace('occupancy grid', 'map')
   text = text.replace('lethal cost', 'blocked area')
   ```

## Data Flow Examples

### Example 1: Voice Command to Robot Motion

```
1. User says: "Move forward 2 meters"
   ↓ (Web Speech API)
2. Dashboard transcribes: "move forward 2 meters"
   ↓ (HTTP POST /api/v1/execute_voice_command)
3. Backend parses with Gemini:
   {
     "action": "move_forward",
     "parameters": {"distance": 2.0},
     "confidence": 0.95
   }
   ↓ (ROS2 cmd_vel publishing)
4. Robot controller publishes Twist message
   ↓ (Nav2 execution)
5. Robot moves forward 2 meters
   ↓ (XAI Monitor observes)
6. Decision logged: {type: "feedback", distance_remaining: 1.5, ...}
   ↓ (Backend sync after 5s)
7. Backend stores in navigation_db
   ↓ (Available for queries)
8. Dashboard shows in decision timeline
```

**Total latency:** ~1.5-2.5s (Gemini: ~800ms, ROS2: ~100ms, execution: variable)

### Example 2: Obstacle Detection to Explanation

```
1. Robot navigating to goal (x=3.0, y=2.0)
   ↓ (Gazebo obstacle spawned)
2. Costmap updates with new obstacle
   ↓ (CostmapProcessor analyzes)
3. ObstacleDetector: distance=0.7m, severity="warning"
   ↓ (Decision logged)
4. DecisionDatabase stores: {type: "obstacle_detected", nearest_obstacle_dist: 0.7, ...}
   ↓ (Nav2 replans path)
5. PathAnalyzer: deviation=1.2m > threshold
   ↓ (path_changes table)
6. User asks: "Why did you turn?"
   ↓ (Backend /api/v1/navigation/decisions)
7. ExplanationEngine retrieves last 5 decisions
   ↓ (PromptTemplateLibrary selects "path_changed" template)
8. GeminiClient generates:
   "I changed my path because I detected an obstacle 0.7 meters ahead.
    I planned a new route around it to reach the goal."
   ↓ (Published to /navigation/explanation)
9. ConversationMemory converts to speech (TTS)
   ↓ (Audio output)
10. User hears explanation
```

**Total explanation latency:** ~2-3s (DB query: ~50ms, Gemini: ~1.5s, TTS: ~500ms)

### Example 3: Spatial Reference Resolution

```
1. User navigates: "Go to x=2.0, y=3.0"
   ↓ (Backend stores)
2. ConversationDB logs: location_x=2.0, location_y=3.0, location_label="kitchen"
   ↓ (Later...)
3. User says: "Go back to the kitchen"
   ↓ (Context builder)
4. ContextBuilder.build_context_for_llm(session_id) returns:
   "Previous locations mentioned: kitchen (2.0, 3.0)"
   ↓ (Gemini parser)
5. GeminiCommandParser resolves "kitchen" → (2.0, 3.0)
   ↓ (Command parameters updated)
6. Robot navigates to (2.0, 3.0)
```

**Spatial memory enables natural conversation without repeating coordinates.**

## Running the System

### Full Navigation Stack
Requires 4 terminals (in order):
```bash
# Terminal 1: Gazebo simulation
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Localization (map_server + AMCL)
ros2 launch localization_server localization.launch.py

# Terminal 3: Path planning (Nav2 stack)
ros2 launch path_planner_server pathplanner.launch.py

# Terminal 4: Visualization
rviz2
# MUST set initial pose with "2D Pose Estimate" tool first
# Then send goals with "Nav2 Goal" tool
```

### SLAM Mapping (to create new maps)
```bash
# Terminal 1: Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Cartographer
ros2 launch cartographer_slam cartographer.launch.py

# Terminal 3: Teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 4: RViz for visualization
rviz2
```

### Web Dashboard Demo
```bash
# All-in-one startup script
./start_robot_dashboard.sh
# Opens browser at http://localhost:5173
# Requires rosbridge_server, ros-humble-turtlesim
```

### Full Intelligent Navigation Stack (XAI + Nav2)

**Prerequisites:**
```bash
# Set environment variables
export TURTLEBOT3_MODEL=burger
export GEMINI_API_KEY="your_key_here"

# Ensure backend is configured
cd backend
source venv/bin/activate
pip install -r requirements.txt
```

Requires 6 terminals (in order):

**Terminal 1: Gazebo Simulation**
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2: Localization (AMCL + Map Server)**
```bash
ros2 launch localization_server localization.launch.py
```

**Terminal 3: Path Planning (Nav2 Stack)**
```bash
ros2 launch path_planner_server pathplanner.launch.py
```

**Terminal 4: XAI Navigator (Decision Logging + Explanations)**
```bash
ros2 launch xai_navigation_pkg xai_navigator.launch.py \
    enable_logging:=true \
    enable_explanations:=true \
    backend_url:=http://localhost:8000 \
    gemini_api_key:=$GEMINI_API_KEY
```

**Terminal 5: Backend Server**
```bash
cd backend
./run.sh
# Or manually: uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

**Terminal 6: Web Dashboard**
```bash
cd project
bun run dev --host
# Or: npm run dev -- --host
```

**Verification Steps:**
1. Check ROS topics: `ros2 topic list | grep navigation`
   - Should see `/navigation/decision`, `/navigation/explanation`
2. Check backend health: `curl http://localhost:8000/health`
   - Should show `ros2: connected`, `gemini: connected`
3. Check database: `ls -lh ~/.ros/navigation_decisions.db`
   - Should grow in size as navigation occurs
4. Send test goal in RViz with "Nav2 Goal" tool
5. Check backend logs for sync messages: Look for "Synced N navigation decisions"

**Common Startup Issues:**

1. **XAI Navigator fails to start:**
   - Check GEMINI_API_KEY is set: `echo $GEMINI_API_KEY`
   - Check Nav2 is running: `ros2 action list | grep navigate_to_pose`
   - Check logs: `ros2 node info /xai_navigator_node`

2. **Backend sync not working:**
   - Verify backend is reachable: `curl http://localhost:8000/health`
   - Check firewall: `sudo ufw status`
   - Check XAI logs for "sync_errors" counter

3. **Explanations not generating:**
   - Verify Gemini API key is valid (test with: `backend/test_gemini.sh`)
   - Check decision database has data: `sqlite3 ~/.ros/navigation_decisions.db "SELECT COUNT(*) FROM navigation_decisions;"`
   - Check explanation engine logs in backend

## Important Debugging Notes

### AMCL Localization Issues
- **Symptom**: Robot pose jumps or diverges
- **Solution**: Set better initial pose estimate in RViz using "2D Pose Estimate"
- **Config**: Increase `min_particles` or `max_particles` in amcl_config.yaml
- **Visual check**: View `/particle_cloud` in RViz to see particle spread

### Navigation Failures
- **Symptom**: "No valid path found" or robot gets stuck
- **Global planner**: Check if goal is in free space on global costmap
- **Local planner**: Verify local costmap is not completely occupied (increase `obstacle_range`)
- **Recovery**: Behavior tree attempts 6 retries with costmap clearing, spinning, and waiting

### Transform Errors (TF)
- **Symptom**: "Transform from X to Y does not exist"
- **Check**: Verify AMCL is running (provides map→odom)
- **Check**: Verify simulation provides odom→base_footprint and sensor frames
- **Tool**: `ros2 run tf2_tools view_frames` to generate frame tree PDF

### Performance Issues
- **AMCL**: Reduce `max_particles` if CPU usage too high
- **Costmaps**: Decrease update frequencies or reduce costmap size
- **Cartographer**: Adjust `num_laser_scans` or `num_point_clouds` in cartographer.lua

### XAI Decision Logging Issues

**Symptom:** No decisions appearing in local database

**Debug Steps:**
1. Verify XAI node is running: `ros2 node list | grep xai_navigator`
2. Check Nav2 action server: `ros2 action list | grep navigate_to_pose`
3. Check topic connections: `ros2 node info /xai_navigator_node`
4. Send test goal in RViz, check logs: `ros2 topic echo /navigation/decision`
5. Check database file exists: `ls -lh ~/.ros/navigation_decisions.db`
6. Query database directly:
   ```bash
   sqlite3 ~/.ros/navigation_decisions.db \
     "SELECT decision_type, timestamp FROM navigation_decisions ORDER BY timestamp DESC LIMIT 5;"
   ```

**Common Causes:**
- Nav2 not running (no action server to monitor)
- Incorrect topic names (check remappings in launch file)
- Database write permissions (`chmod 644 ~/.ros/navigation_decisions.db`)
- XAI node crashed (check: `ros2 node list`)

### Backend Sync Failures

**Symptom:** XAI navigator logs show `sync_errors` incrementing

**Debug Steps:**
1. Check backend health: `curl http://localhost:8000/health`
2. Check backend logs: `tail -f backend/logs/app.log`
3. Test sync endpoint manually:
   ```bash
   curl -X POST http://localhost:8000/api/v1/navigation/decisions/sync \
     -H "Content-Type: application/json" \
     -d '[{"decision_id": 1, "decision_type": "test", "timestamp": 123456, "data": {}}]'
   ```
4. Check firewall: `sudo ufw status`
5. Check network connectivity: `ping localhost`

**Common Causes:**
- Backend not running (check: `pgrep -f uvicorn`)
- Incorrect backend_url in xai_params.yaml (default: http://localhost:8000)
- Backend database locked (restart backend)
- Port 8000 blocked by firewall

### Explanation Generation Issues

**Symptom:** `/navigation/explanation` topic empty or generic responses

**Debug Steps:**
1. Verify Gemini API key:
   ```bash
   echo $GEMINI_API_KEY
   # Test with: cd backend && ./test_gemini.sh
   ```
2. Check decision database has context:
   ```bash
   sqlite3 ~/.ros/navigation_decisions.db \
     "SELECT decision_type, data_json FROM navigation_decisions WHERE decision_type='path_changed' LIMIT 1;"
   ```
3. Check explanation engine logs in backend: `grep "explanation" backend/logs/app.log`
4. Test explanation API directly:
   ```bash
   curl http://localhost:8000/api/v1/navigation/decisions | jq '.decisions[0]'
   ```

**Common Causes:**
- Invalid/expired Gemini API key (get new one: https://aistudio.google.com/app/apikey)
- No recent decisions to explain (navigate robot first)
- Gemini API rate limits (check: `stats['gemini_stats']` in logs)
- Explanation cache serving stale data (clear: restart XAI node)
- Temperature too high/low (check xai_params.yaml: should be 0.3-0.4)

### Conversation Memory / Spatial References

**Symptom:** "Go to the kitchen" doesn't resolve to coordinates

**Debug Steps:**
1. Check conversation database exists: `ls -lh ~/.ros/backend_data/conversation_history.db`
2. Verify session has location labels:
   ```bash
   curl http://localhost:8000/api/v1/conversation/spatial_refs/SESSION_ID_HERE
   ```
3. Check context builder logs: `grep "spatial_reference" backend/logs/app.log`
4. Test command with explicit coordinates first: "Navigate to x=2.0, y=3.0"

**Common Causes:**
- No prior navigation to that location (must navigate first to create label)
- Incorrect session_id (check: `/api/v1/conversation/sessions`)
- Backend database corruption (delete and reinit: `rm ~/.ros/backend_data/*.db`)
- Location label not stored in conversation turn metadata

## Configuration File Cheat Sheet

Quick reference for common configuration tasks:

| What You Want to Change | File to Edit | Parameter Name |
|------------------------|-------------|----------------|
| Obstacle detection sensitivity | `src/xai_navigation_pkg/config/xai_params.yaml` | `critical_distance`, `warning_distance` |
| Backend sync frequency | `src/xai_navigation_pkg/config/xai_params.yaml` | `sync_interval` |
| Path change sensitivity | `src/xai_navigation_pkg/config/xai_params.yaml` | `deviation_threshold` |
| Gemini API key (explanations) | `backend/.env` | `GEMINI_API_KEY` |
| Backend API URL | `src/xai_navigation_pkg/config/xai_params.yaml` OR launch args | `backend_url` |
| Database cleanup age | `src/xai_navigation_pkg/xai_navigation_pkg/decision_database.py` | `cleanup_old(days=7)` |
| Explanation detail level | `src/xai_navigation_pkg/config/xai_params.yaml` | `explanation_level: "simple"/"detailed"/"debug"` |
| Command parsing model | `backend/app/main.py` startup | `initialize_gemini(model_name="...")` |
| ROS2 topics to monitor | `src/xai_navigation_pkg/xai_navigation_pkg/xai_navigator_node.py` | Subscriptions in `__init__` |
| Nav2 costmap resolution | `src/path_planner_server/config/planner_server.yaml` | `resolution: 0.05` |
| TurtleBot3 model | Environment variable | `export TURTLEBOT3_MODEL=burger` |

### Environment Variables Required

**For XAI Navigator:**
```bash
export TURTLEBOT3_MODEL=burger  # Required
export GEMINI_API_KEY="your_key"  # For explanations
```

**For Backend:**
```bash
# In backend/.env file:
GEMINI_API_KEY=your_key_here  # For command parsing
OPENAI_API_KEY=your_key_here  # For Whisper transcription (optional)
DATABASE_URL=sqlite+aiosqlite:///~/.ros/backend_data/app.db
CORS_ORIGINS=["http://localhost:5173","http://localhost:3000"]
LOG_LEVEL=INFO
```

## Package Dependencies

When adding new functionality, be aware of these dependency patterns:
- All packages depend on: `rclpy`, `launch`, `launch_ros`
- Navigation packages depend on: `nav2_common`, `nav2_bringup`
- SLAM packages depend on: `cartographer_ros`
- Web dashboard depends on: `rosbridge_server` (install: `sudo apt install ros-humble-rosbridge-server`)

Dependencies are declared in `package.xml` and must match versions (currently ROS 2 Humble).

### XAI & Backend Dependencies

**xai_navigation_pkg requires:**
- ROS2 packages: `rclpy`, `nav2_msgs`, `geometry_msgs`, `nav_msgs`, `action_msgs`
- Python packages:
  - `sqlite3` (Python standard library)
  - `google-generativeai>=0.3.0` - Gemini SDK for explanation generation
  - `requests` - HTTP client for backend sync
  - `aiohttp` - Async HTTP (optional, for async operations)
- Custom messages: `intelligent_twin_msgs`

**Install Gemini SDK in ROS2 workspace:**
```bash
pip3 install google-generativeai
```

**backend/ requires:**
- Web framework: `fastapi>=0.109.0`, `uvicorn[standard]`
- Database: `aiosqlite`, `sqlalchemy>=2.0`
- AI APIs: `google-generativeai>=0.3.0`, `openai` (optional, for Whisper)
- Utilities: `loguru`, `pydantic>=2.0`, `python-dotenv`, `python-multipart`
- Testing: `pytest`, `pytest-asyncio`

**Install backend dependencies:**
```bash
cd backend
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

**conversation_memory_pkg requires:**
- Same as xai_navigation_pkg base dependencies
- TTS library (system-level): `espeak` or `festival`

**intelligent_twin_msgs requires:**
- Build type: `ament_cmake` (not ament_python)
- Dependencies: `rosidl_default_generators`, `rosidl_default_runtime`

### Version Compatibility

- **ROS2:** Humble (Ubuntu 22.04)
- **Python:** 3.10+ (required for FastAPI type hints and structural pattern matching)
- **Gemini SDK:** `google-generativeai>=0.3.0`
- **FastAPI:** `>=0.109.0`
- **Node.js:** 18+ or Bun 1.0+ (for web dashboard)
- **SQLite:** 3.35+ (for WAL mode support)

**Critical Version Notes:**
- Python 3.10 required for backend `match` statements and modern type hints
- Gemini SDK 0.3.0+ required for `gemini-2.0-flash-exp` model access
- SQLite 3.35+ required for WAL mode concurrent access patterns
