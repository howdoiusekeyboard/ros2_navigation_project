# Project Brief: Intelligent Digital Twin with XAI for HRI

**Student:** Kushagra Golash (2022A7PS0226U)  
**Supervisor:** Dr. Sujala D. Shetty, Associate Professor & Head, Computer Science Dept.  
**Institution:** BITS Pilani, Dubai Campus  
**Project Type:** Undergraduate Special Project (Final Year)  
**Duration:** 8 Weeks (January 20 - March 16, 2025)  
**Status:** Week 1 - Foundation Phase

---

## Executive Summary

This project develops an **intelligent robotic system** that makes autonomous robots more trustworthy, interactive, and self-aware by combining three state-of-the-art technologies:

1. **Conversational Memory** - Multi-turn dialogue with spatial context awareness
2. **Explainable AI Navigation** - Natural language explanations for robot decisions  
3. **Digital Twin Anomaly Detection** - ML-based behavioral deviation monitoring

The system extends proven ROS2 voice control work from last semester, adding intelligence layers that bridge the gap between human intent and robot action through transparent, conversational interaction.

---

## Research Context & Motivation

### Problem Statement
Current robotic systems suffer from three key limitations:
1. **No conversation memory** - Each voice command is isolated, requiring users to repeat context
2. **Opaque decision-making** - Users don't know WHY robots choose certain paths or actions
3. **No self-awareness** - Robots can't detect when their behavior deviates from expectations

### Novel Contribution
This project uniquely **integrates** three typically separate research areas:
- XAI for robotics (usually focuses only on explainability)
- Conversational AI (usually in chatbots, not physical robots)
- Digital twins (usually in industrial settings, not with real-time AI explanation)

The integration creates a system where explanations are conversational, context-aware, and can reference both past interactions and current anomalies.

---

## Technical Architecture

### System Components

```
┌─────────────────────────────────────────────────────────────┐
│                  WEB DASHBOARD (React + TypeScript)          │
│  • Conversation History Panel                                │
│  • Navigation Explanation Display                            │
│  • Digital Twin Comparison View                              │
│  • Real-time Anomaly Alerts                                  │
└────────────────┬────────────────────────────────────────────┘
                 │ WebSocket (rosbridge)
┌────────────────▼────────────────────────────────────────────┐
│              ROS2 MIDDLEWARE (Humble Distribution)           │
└─────┬──────────────────┬──────────────────┬─────────────────┘
      │                  │                  │
┌─────▼─────────┐  ┌────▼──────────┐  ┌───▼──────────────────┐
│ CONVERSATION  │  │ XAI NAVIGATOR │  │ DIGITAL TWIN MONITOR │
│ MEMORY NODE   │  │ NODE          │  │ NODE                 │
│               │  │               │  │                      │
│ • SQLite DB   │  │ • Nav2 hooks  │  │ • Gazebo sync       │
│ • Context LLM │  │ • Gemini API  │  │ • Sensor compare    │
│ • References  │  │ • Explanation │  │ • ML anomaly model  │
└───────────────┘  └───────────────┘  └──────────────────────┘
```

### Technology Stack

**Core Platform:**
- ROS2 Humble (Robot Operating System)
- TurtleBot3 platform
- Gazebo 11 simulation
- Nav2 navigation stack

**AI/ML Services:**
- Gemini API (LLM for conversation & explanations)
- Whisper API (speech recognition)
- scikit-learn/PyTorch (anomaly detection models)

**Frontend:**
- React 18 + TypeScript
- Tailwind CSS
- rosbridge_websocket
- Recharts for visualization

**Data Management:**
- SQLite (conversation history)
- ROS2 bags (sensor data logging)
- CSV exports (ML training datasets)

---

## Three Core Components

### 1. Conversational Memory System

**Goal:** Enable natural, context-aware multi-turn dialogue

**Key Features:**
- Maintain 5-10 turn conversation history
- Resolve spatial references: "go back there", "the place from before"
- Session persistence across robot restarts
- Location-tagged memory entries

**Example Interaction:**
```
User: "Go to the kitchen"
Robot: [navigates] "I've arrived at the kitchen"
User: "Now go back to where you were before"
Robot: [retrieves context] "Returning to the living room at (2.5, 1.2)"
```

**Technical Implementation:**
- SQLite database with schema: {timestamp, user_input, robot_response, location_x, location_y, context}
- Context injection into LLM prompts
- Reference resolution algorithm for spatial terms

**Week 2 Deliverable:** Working 5+ turn dialogue with spatial context

---

### 2. Explainable AI Navigation Module

**Goal:** Generate natural language explanations for robot navigation decisions

**Key Features:**
- Real-time explanation generation (<2s latency)
- Explains path planning, obstacle avoidance, goal modifications
- Dashboard visualization of reasoning
- Conversational integration (explanations stored in memory)

**Example Explanations:**
```
Scenario: Obstacle detected during navigation
Explanation: "I changed my path because there's an obstacle at 
(3.0, 2.1). The new route adds 0.5m distance but avoids the 
blocked area. I'm now taking a path around the left side."

Scenario: Goal unreachable
Explanation: "I cannot reach the requested location because it's 
blocked by furniture. The closest I can get is 0.8m away. Would 
you like me to go there instead?"
```

**Technical Implementation:**
- Hooks into Nav2 action server callbacks
- Captures: original_path, modified_path, cost_maps, obstacle_positions
- Gemini API with specialized prompts for explanation generation
- Template-based structure with dynamic content

**Weeks 3-4 Deliverable:** Real-time natural language navigation explanations

---

### 3. Digital Twin Anomaly Detection

**Goal:** Detect behavioral deviations using parallel simulation baseline

**Key Features:**
- Parallel Gazebo digital twin (expected behavior)
- Real-time sensor comparison (real vs. twin)
- ML-based anomaly scoring (>80% target accuracy)
- Automatic anomaly explanations

**How It Works:**
```
1. Command sent to both real robot and digital twin
2. Both execute navigation simultaneously
3. Monitor compares sensor streams:
   - Odometry (position/velocity)
   - Laser scans (environment perception)
   - IMU (orientation)
   - Command velocities
4. ML model scores deviation (0-1 scale)
5. If score > threshold (0.7): Alert + Explanation
```

**Example Anomaly Detection:**
```
Alert: "Anomaly detected (score: 0.82)"
Type: "Position drift"
Explanation: "The real robot is 0.35m behind where it should be. 
This could indicate wheel slippage or motor issues. Digital twin 
reached (5.0, 3.2) but real robot is at (4.65, 3.1)."
```

**Technical Implementation:**
- Feature engineering: position_deviation, velocity_mismatch, scan_divergence
- ML model: Isolation Forest (simple, unsupervised, fast)
- Training data: 2+ hours of "normal operation" from twin
- Real-time scoring at 10Hz

**Weeks 5-6 Deliverable:** Working anomaly detection with >80% accuracy

---

## 8-Week Development Timeline

### Week 1: Foundation & Architecture ✓ (Current)
- Literature review (20 papers)
- System architecture design
- ROS2 workspace setup (4 packages)
- Digital twin simulation configured
- Dashboard UI extended

### Week 2: Conversational Memory
- Database schema implementation
- Context retrieval system
- LLM prompt engineering for context
- Spatial reference resolution
- **Milestone:** 5+ turn dialogue working

### Week 3: XAI Navigation - Part 1
- Nav2 decision logging
- Cost map data capture
- Explanation data structures
- **Milestone:** Decision capture system operational

### Week 4: XAI Navigation - Part 2
- Explanation generation engine (Gemini)
- Dashboard visualization
- Latency optimization
- **Milestone:** Real-time NL explanations

### Week 5: Digital Twin - Part 1
- Parallel Gazebo synchronization
- Sensor data collection pipeline
- Baseline "normal operation" dataset
- **Milestone:** 2+ hours baseline data collected

### Week 6: Digital Twin - Part 2
- Feature engineering
- ML model training (Isolation Forest)
- Real-time anomaly scoring
- **Milestone:** >80% anomaly detection accuracy

### Week 7: System Integration (CRITICAL)
- Integrate all three components
- Dashboard polish
- End-to-end testing
- Performance optimization
- **Milestone:** Fully working unified system

### Week 8: Evaluation & Documentation
- Performance metrics collection
- User comprehension surveys
- Final report writing
- Demo video creation
- **Milestone:** Complete project submission

---

## Literature Review Summary (20 Papers)

### Category 1: XAI in Human-Robot Interaction (7 papers)

**Key Papers:**
1. **Anjomshoae et al. (2021)** - User-centered XAI framework for HRI
   - *Applied to:* Designing comprehensible explanations for non-experts
   
2. **Ehsan et al. (2022)** - Self-explaining social robots architecture
   - *Applied to:* Real-time explanation generation approach

3. **Uruj et al. (2025)** - GPT-4/LLaMA integration for HRI (Supervisor's recent work)
   - *Applied to:* LLM integration patterns with ROS2

4. **Arrieta et al. (2020)** - Comprehensive XAI taxonomy
   - *Applied to:* Method selection and categorization

**Application to Project:**
- Explanation design principles (contrastive, selective, causal)
- Real-time generation architectures
- User comprehension metrics

---

### Category 2: Conversational AI with Memory (6 papers)

**Key Papers:**
1. **Dondrup et al. (2021)** - Long-term memory in HRI
   - *Applied to:* Incremental learning of user preferences

2. **Scheggia et al. (2025)** - LLM challenges in companion robots
   - *Applied to:* Avoiding common pitfalls (turn-taking, context loss)

3. **Okon et al. (2025)** - Memory-enhanced conversational AI
   - *Applied to:* Storage and retrieval architectures

4. **Miller (2019)** - Social science perspective on AI explanations
   - *Applied to:* Understanding human explanation expectations

**Application to Project:**
- Context window management strategies
- Spatial reference resolution techniques
- Memory persistence patterns

---

### Category 3: Digital Twin Anomaly Detection (7 papers)

**Key Papers:**
1. **Yuan et al. (2024)** - Physics-informed hybrid convolutional autoencoder
   - *Applied to:* ML architecture for robot anomaly detection

2. **Alaluss et al. (2023)** - ML-based digital twin for soccer robots
   - *Applied to:* ROS-specific implementation patterns

3. **Chen et al. (2023)** - Curriculum learning for anomaly detection
   - *Applied to:* Training strategy with synthetic twin data

4. **Eckhart & Ekelhart (2021)** - Anomaly detection framework for digital twins
   - *Applied to:* Distinguishing normal variation from true anomalies

**Application to Project:**
- Feature selection for sensor comparison
- Threshold tuning strategies
- Weakly-supervised training approaches

---

## Research Gaps Addressed

1. **Integration Gap:** XAI, conversational AI, and digital twins typically exist separately
2. **Modality Gap:** Most XAI focuses on visual explanations; our conversational interface is more natural
3. **Real-time Explanation Gap:** Anomaly detection usually alerts but doesn't explain via dialogue
4. **ROS2 Gap:** Limited work specifically on XAI with modern ROS2 systems

---

## Success Criteria

### Minimum Viable Product (Must Have)
- ✅ 3+ turn conversation with context
- ✅ Basic navigation explanations
- ✅ Digital twin running in parallel
- ✅ Simple threshold-based anomaly detection
- ✅ Dashboard showing all components

### Target Goals (Should Have)
- ✅ 5+ turn conversation with spatial references
- ✅ Detailed explanations with reasoning
- ✅ ML-based anomaly detection (>80% accuracy)
- ✅ Polished dashboard with real-time updates
- ✅ User evaluation with positive feedback

### Stretch Goals (Nice to Have)
- ✅ 10+ turn conversation history
- ✅ Proactive suggestions based on past interactions
- ✅ Multiple anomaly types detected
- ✅ Explanation quality metrics (BLEU scores)

---

## Evaluation Metrics

### Quantitative Metrics

**Conversational Memory:**
- Context retention rate: % of correctly resolved references
- Average retrieval time: <100ms target
- Database query efficiency

**XAI Navigation:**
- Explanation generation latency: <2s target
- Explanation consistency: BLEU score between similar scenarios
- Coverage: % of navigation decisions explained

**Anomaly Detection:**
- Precision: True positives / (True positives + False positives)
- Recall: True positives / (True positives + False negatives)
- F1-Score: Harmonic mean of precision and recall
- Target: >80% on all metrics

### Qualitative Metrics

**User Comprehension Survey (5-point Likert scale):**
1. "I understood why the robot made its decisions"
2. "The explanations were helpful and clear"
3. "The conversation felt natural and contextual"
4. "I trust the robot's decision-making more after explanations"
5. "The anomaly alerts helped me understand robot issues"

**Target:** Average score >4.0/5.0 (n=5 test users minimum)

**System Usability Scale (SUS):**
- Standard 10-question SUS questionnaire
- Target: Score >70 (above average)

---

## Risk Mitigation

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| LLM API latency >2s | High | Medium | Pre-cache common explanations; use streaming responses |
| Anomaly false positives | Medium | High | Conservative thresholds; manual validation phase |
| Digital twin drift over time | Medium | Medium | Periodic recalibration; focus on relative differences |
| Integration complexity Week 7 | High | Medium | Weekly integration checkpoints; modular design |
| Limited test subjects | Low | Medium | Self-evaluation + minimum 3 lab members |
| Gazebo performance issues | Medium | Low | Reduce simulation fidelity; use headless mode |

---

## Supervisor Research Alignment

### Dr. Sujala D. Shetty's Research Areas:
1. **Explainable AI** ✓ (Core component of project)
2. **Big Data** ✓ (Sensor data collection, ML training datasets)
3. **Deep Neural Networks** ✓ (Potential for Week 6 ML model)
4. **IoT** ✓ (Robot as IoT device, sensor data streams)
5. **NLP** ✓ (Conversational memory, explanation generation)

### Recent Relevant Publications:
1. **Uruj et al. (2025)** - IEEE Access paper on LLM integration with speech for HRI
   - Directly applicable to conversation memory implementation
   
2. **Choudhary et al. (2025)** - ICCI-25 Best Paper on ROS-Flask-LLM integration
   - Provides implementation patterns for this project

3. **Goswami et al. (2024)** - CINS2024 paper on voice-controlled navigation
   - Foundation work this project extends

---

## Previous Semester Foundation

### ROS2 Voice Control Project (Sem 1, 2024-25)

**What Was Built:**
- Complete ROS2 Humble workspace
- Voice input → Whisper API → transcription
- Gemini LLM → structured command JSON
- Nav2 integration for autonomous navigation
- React TypeScript dashboard with real-time updates
- WebSocket communication via rosbridge

**Key Achievements:**
- 97.3% voice recognition accuracy (5 languages)
- <2s voice-to-action latency
- 98.2% navigation accuracy
- Working demo with full system integration

**What This Project Adds:**
- Conversational memory (not just single commands)
- Explanation generation (not just action execution)
- Digital twin monitoring (not just real robot)

**Technical Continuity:**
- Same ROS2 workspace structure
- Same LLM APIs (Gemini, Whisper)
- Same dashboard framework (React + TypeScript)
- Same robot platform (TurtleBot3)

---

## Current Status (Week 1 Complete)

### Completed Deliverables ✓
1. **Documentation:**
   - 20-paper literature review
   - System architecture document
   - Integration plan for Week 7
   - Week 1 development log

2. **ROS2 Workspace:**
   - 4 packages created and building successfully
   - 4 custom message types defined
   - Node skeletons implemented
   - All dependencies configured

3. **Digital Twin:**
   - Parallel Gazebo simulation launching
   - Command synchronizer working
   - Both robots moving identically
   - Separate namespaces configured

4. **Dashboard:**
   - 3 new panels implemented (UI only)
   - Updated layout (8-4 column grid)
   - Components rendering without errors
   - Ready for data integration

### Week 1 Metrics
- **Code written:** ~800 lines (Python + TypeScript)
- **Build time:** 45 seconds (full workspace)
- **Test success rate:** 100% (5/5 tests passed)
- **Documentation:** 3 documents, 2500+ words
- **Progress vs. plan:** 100%

---

## Technical Specifications

### ROS2 Node Details

**1. conversation_memory_node**
```yaml
Subscribed Topics:
  - /voice/transcription (std_msgs/String)
  - /robot/pose (geometry_msgs/PoseStamped)
Published Topics:
  - /conversation/context (intelligent_twin_msgs/ConversationContext)
Services:
  - /conversation/query_reference (string → Point)
Database: SQLite (~/.ros/conversation_history.db)
```

**2. xai_navigator_node**
```yaml
Subscribed Topics:
  - /plan (nav_msgs/Path)
  - /local_costmap/costmap (nav_msgs/OccupancyGrid)
Published Topics:
  - /navigation/explanation (std_msgs/String)
  - /navigation/decision_data (intelligent_twin_msgs/NavigationDecision)
Action Clients:
  - /navigate_to_pose (nav2_msgs/NavigateToPose)
```

**3. digital_twin_monitor_node**
```yaml
Subscribed Topics:
  - /real/odom, /twin/odom (nav_msgs/Odometry)
  - /real/scan, /twin/scan (sensor_msgs/LaserScan)
  - /real/cmd_vel, /twin/cmd_vel (geometry_msgs/Twist)
Published Topics:
  - /anomaly/alert (intelligent_twin_msgs/AnomalyAlert)
  - /anomaly/score (std_msgs/Float32)
ML Model: Isolation Forest (scikit-learn)
Comparison Rate: 10Hz
```

### Custom Message Types

**ConversationContext.msg:**
```
std_msgs/Header header
string[] history_turns
geometry_msgs/Point[] location_references
string current_context
```

**NavigationDecision.msg:**
```
std_msgs/Header header
nav_msgs/Path original_path
nav_msgs/Path modified_path
geometry_msgs/Point obstacle_position
float32 cost_difference
string decision_type
```

**AnomalyAlert.msg:**
```
std_msgs/Header header
float32 anomaly_score
string anomaly_type
string explanation
SensorDifference sensor_data
```

---

## Project Resources

### GitHub Repository Structure
```
intelligent-digital-twin-xai/
├── README.md
├── docs/
│   ├── literature_review.md
│   ├── architecture.md
│   ├── integration_plan.md
│   ├── weekly_logs/
│   └── api_documentation.md
├── ros2_ws/
│   └── src/
│       ├── conversation_memory_node/
│       ├── xai_navigator_node/
│       ├── digital_twin_monitor_node/
│       ├── intelligent_twin_msgs/
│       └── voice_control/
├── web_dashboard/
│   ├── src/
│   │   └── components/
│   └── package.json
└── models/
    └── anomaly_detector.pkl
```

### Key Development Tools
- **IDE:** VS Code with ROS extensions
- **Version Control:** Git + GitHub
- **Testing:** pytest (Python), Jest (TypeScript)
- **Documentation:** Markdown + Mermaid diagrams
- **Visualization:** RViz2, Gazebo, custom React dashboard

---

## Help Needed Areas

### Immediate (Week 2)
1. **Prompt engineering** for context injection into Gemini API
2. **Spatial reference resolution** algorithm design
3. **SQLite optimization** for real-time conversation queries

### Medium-term (Weeks 3-4)
1. **Nav2 callback hooks** - best practices for intercepting decisions
2. **Explanation template design** - balancing detail vs. brevity
3. **Dashboard state management** - Redux vs. Context API

### Later (Weeks 5-6)
1. **Feature engineering** for sensor difference comparison
2. **ML model selection** - Isolation Forest vs. alternatives
3. **Threshold tuning** strategies for anomaly detection

### Integration (Week 7)
1. **Performance optimization** - reducing end-to-end latency
2. **Error handling** - graceful degradation strategies
3. **Testing automation** - integration test suite design

---

## Contact & Collaboration

**Student:**
- Name: Kushagra Golash
- Email: f20220226@dubai.bits-pilani.ac.in
- GitHub: [Your GitHub username]

**Supervisor:**
- Dr. Sujala D. Shetty
- Associate Professor & Head, Computer Science
- Email: sujala@dubai.bits-pilani.ac.in
- Research: XAI, IoT, Big Data, NLP

**Institution:**
- BITS Pilani, Dubai Campus
- Academic Block, Dubai International Academic City
- Website: https://www.bits-pilani.ac.in/dubai/

---

## Expected Outcomes

### Academic Deliverables
1. Final technical report (15-20 pages, IEEE format)
2. Literature review document (integrated methodology)
3. Complete codebase (GitHub, well-documented)
4. Technical documentation (architecture, API, deployment)
5. Demo video (3-5 minutes, narrated)
6. Presentation slides (15-20 slides)

### Technical Artifacts
1. Working ROS2 system (all 3 components integrated)
2. Trained ML model (anomaly detection, >80% accuracy)
3. Web dashboard (real-time visualization)
4. Conversation database (with test data)
5. Evaluation results (quantitative + qualitative)

### Potential Research Outputs
1. Conference paper submission (ICRA, IROS, HRI)
2. Technical blog post series
3. Open-source release (after supervisor approval)
4. Poster presentation at campus research day

---

## Project Timeline At-a-Glance

```
Week 1 ████████ Foundation & Architecture ✓
Week 2 ████████ Conversational Memory
Week 3 ████░░░░ XAI Navigation Part 1
Week 4 ████░░░░ XAI Navigation Part 2
Week 5 ████░░░░ Digital Twin Setup
Week 6 ████░░░░ Anomaly Detection ML
Week 7 ████░░░░ INTEGRATION (Critical)
Week 8 ████░░░░ Evaluation & Documentation
       ────────────────────────────────────
       Jan 20                      Mar 16
```

**Current Progress:** Week 1 Complete (12.5% of timeline)  
**On Track:** Yes ✓  
**Next Milestone:** Week 2 - 5-turn dialogue by Feb 2  
**Critical Milestone:** Week 7 - Full integration by Mar 9

---

## Closing Notes

This project represents a significant integration challenge, combining conversational AI, explainable robotics, and digital twin technology into a cohesive system. The 8-week timeline is ambitious but achievable given:

1. **Strong foundation:** Proven ROS2 + LLM system from last semester
2. **Incremental approach:** Weekly milestones with clear deliverables
3. **Pragmatic scope:** Focus on integration over individual component perfection
4. **Risk awareness:** Week 7 identified as critical, with buffer strategies

The project aligns perfectly with Dr. Sujala's research interests (XAI, NLP, IoT) and advances the state-of-the-art in human-robot interaction through novel integration of existing technologies.

**Success depends on:** Maintaining weekly momentum, early identification of blockers, and focus on the critical Week 7 integration phase.

---

**Document Version:** 1.0  
**Last Updated:** Week 1 End (January 26, 2025)  
**Next Update:** Week 2 End (February 2, 2025)

---

*This brief is intended for Claude AI Pro project context. For detailed implementation guides, see the weekly implementation documents.*