# 8-Week Project Scope of Work
## Intelligent Digital Twin with Explainable AI for Human-Robot Interaction

**Student:** Kushagra Golash (2022A7PS0226U)  
**Supervisor:** Dr. Sujala D. Shetty  
**Timeline:** 8 Weeks  
**Base Platform:** ROS2 Humble + React Dashboard (from previous semester)

---

## Project Objectives

Build an intelligent robotic system that:
1. Maintains conversational context across multiple interactions
2. Explains navigation decisions in natural language
3. Detects behavioral anomalies using a digital twin baseline

---

## Core Deliverables

### 1. **Conversational Memory System**
- Multi-turn dialogue with context retention (5-10 conversation history)
- Commands like "go back there" or "repeat the last task"
- Session persistence across robot restarts

### 2. **Explainable Navigation Module**
- Natural language explanations for path planning decisions
- Real-time explanation generation ("I chose this path because...")
- Dashboard visualization of reasoning (cost maps + text)

### 3. **Digital Twin Anomaly Detector**
- Parallel Gazebo simulation as expected behavior baseline
- ML model comparing real vs. expected sensor streams
- Alert system for detected anomalies with explanations

### 4. **Integrated Web Dashboard**
- Extended React UI showing: conversation history, navigation explanations, anomaly alerts
- Real-time data visualization from both physical and twin robots

### 5. **Documentation Package**
- Literature review (20 papers - already complete)
- Technical documentation (architecture, API, deployment)
- Final report (methodology, results, evaluation)
- Demo video (3-5 minutes)

---

## 8-Week Development Timeline

### **Week 1: Foundation & Architecture Design**
**Goal:** Design system architecture and set up development environment

**Tasks:**
- Finalize literature review integration into project document
- Design system architecture (components, data flow, interfaces)
- Set up extended ROS2 workspace with new packages:
  - `conversation_memory` node
  - `xai_navigator` node  
  - `digital_twin_monitor` node
- Create data models for conversation history storage
- Set up parallel Gazebo instance for digital twin

**Deliverable:** System architecture diagram + development environment ready

---

### **Week 2: Conversational Memory Implementation**
**Goal:** Enable multi-turn dialogue with context awareness

**Tasks:**
- Implement conversation history database (SQLite/JSON)
- Extend LLM prompt engineering for context injection
- Add memory retrieval to Gemini API calls
- Implement spatial memory (location references: "there", "before")
- Create conversation session management

**Test:** "Go to the kitchen. Now go back there. What did you just do?"

**Deliverable:** Working conversational memory with 5+ turn context

---

### **Week 3: Explainable Navigation - Part 1**
**Goal:** Extract and structure navigation decision data

**Tasks:**
- Hook into Nav2 action server to capture:
  - Path planning decisions
  - Cost map evaluations
  - Obstacle avoidance triggers
- Create explanation data structure (decision type, factors, alternatives)
- Implement basic explanation templates
- Log navigation decisions with timestamps

**Test:** Trigger path replan and capture the decision factors

**Deliverable:** Navigation decision logging system operational

---

### **Week 4: Explainable Navigation - Part 2**
**Goal:** Generate natural language explanations

**Tasks:**
- Design explanation prompt templates for Gemini
- Integrate explanation generation with voice response pipeline
- Implement explanation types:
  - Path selection: "Why this route?"
  - Obstacle avoidance: "Why did you stop/turn?"
  - Goal modification: "Why can't you reach there?"
- Add explanation visualization to web dashboard
- Optimize explanation latency (<2s)

**Test:** Ask "Why did you choose that path?" after navigation

**Deliverable:** Real-time natural language navigation explanations

---

### **Week 5: Digital Twin Setup & Data Collection**
**Goal:** Establish baseline digital twin simulation

**Tasks:**
- Configure parallel Gazebo simulation (identical environment)
- Synchronize command execution to both real and twin robots
- Implement sensor data collection pipeline:
  - Odometry
  - Laser scan
  - IMU (if available)
  - Command velocities
- Create data logging system (ROS bags + CSV exports)
- Collect baseline "normal operation" dataset (minimum 2 hours)

**Test:** Execute 20 navigation commands, verify data collection from both

**Deliverable:** Digital twin simulation + baseline dataset

---

### **Week 6: Anomaly Detection Model Development**
**Goal:** Train and deploy ML model for anomaly detection

**Tasks:**
- Feature engineering from sensor differences (twin vs real)
- Train anomaly detection model (start simple: Isolation Forest/AutoEncoder)
- Implement real-time anomaly scoring
- Define anomaly thresholds and alert triggers
- Create anomaly explanation generator
- Integrate with ROS2 monitoring node

**Test:** Simulate anomalies (block sensor, add friction) and verify detection

**Deliverable:** Working anomaly detection with >80% accuracy

---

### **Week 7: Integration & Dashboard Enhancement**
**Goal:** Complete system integration and polish UX

**Tasks:**
- Integrate all three components into unified system
- Enhance React dashboard:
  - Conversation history panel
  - Real-time explanation display
  - Twin vs Real comparison view
  - Anomaly alert system with explanations
- Add error handling and recovery mechanisms
- Implement system health monitoring
- Optimize performance (latency, resource usage)
- User testing with 3-5 test scenarios

**Test:** Complete end-to-end workflow with all features active

**Deliverable:** Fully integrated system with polished dashboard

---

### **Week 8: Evaluation, Documentation & Presentation**
**Goal:** Complete documentation and prepare final presentation

**Tasks:**
- **Days 1-2:** Performance evaluation
  - Conversation accuracy (context retention rate)
  - Explanation quality (user comprehension survey - 3-5 users)
  - Anomaly detection metrics (precision, recall, F1)
- **Days 3-4:** Documentation
  - Final report writing (15-20 pages)
  - Technical documentation (README, API docs, deployment guide)
  - Code cleanup and commenting
- **Days 5-6:** Presentation materials
  - Create demo video (3-5 minutes)
  - Prepare presentation slides (15-20 slides)
  - Practice demonstration scenarios
- **Day 7:** Buffer for revisions and final polish

**Deliverable:** Complete project package ready for submission

---

## Technical Stack

### **Core Technologies** (Already Established)
- ROS2 Humble
- Gazebo Simulation
- Nav2 Navigation Stack
- React + TypeScript + Tailwind CSS
- rosbridge_websocket
- Python 3.10+

### **New Additions for This Project**
- **LLM:** Gemini API (context-aware prompts)
- **Database:** SQLite (conversation history) or Redis (if real-time needed)
- **ML Framework:** scikit-learn / PyTorch (anomaly detection)
- **Data Processing:** pandas, numpy
- **Visualization:** Recharts (React), Plotly (optional)

---

## Risk Mitigation Plan

| Risk | Impact | Mitigation Strategy |
|------|--------|-------------------|
| LLM API latency too high | High | Pre-cache common explanations; use streaming responses |
| Anomaly detection false positives | Medium | Start with conservative thresholds; manual validation |
| Digital twin drift | Medium | Periodic recalibration; focus on relative differences |
| Integration complexity | High | Weekly integration checkpoints; modular design |
| Limited evaluation subjects | Low | Self-evaluation + 3 lab members minimum |

---

## Success Criteria

### **Minimum Viable Product (Must Have)**
- ✅ 3+ turn conversation with context
- ✅ Basic navigation explanations ("I'm taking this path")
- ✅ Digital twin running in parallel
- ✅ Simple anomaly detection (threshold-based)
- ✅ Dashboard showing all components

### **Target Goals (Should Have)**
- ✅ 5+ turn conversation with spatial references
- ✅ Detailed explanations (reasons, alternatives considered)
- ✅ ML-based anomaly detection with >80% accuracy
- ✅ Polished dashboard with real-time updates
- ✅ User evaluation with positive feedback

### **Stretch Goals (Nice to Have)**
- ✅ 10+ turn conversation history
- ✅ Proactive suggestions based on past interactions
- ✅ Multiple anomaly types detected
- ✅ Explanation quality metrics (BLEU score for consistency)

---

## Weekly Check-in Format (With Dr. Sujala)

**15-minute weekly meetings covering:**
1. **Progress:** What was completed (demo if possible)
2. **Challenges:** Blockers encountered and proposed solutions
3. **Next Week:** Concrete goals for upcoming week
4. **Quick Decision:** Any design decisions needing supervisor input

---

## Evaluation Metrics (Week 8)

### **Quantitative**
- Conversation context retention: % of correctly resolved references
- Explanation latency: Average time to generate (<2s target)
- Anomaly detection: Precision, Recall, F1-score
- System performance: CPU/Memory usage, response times

### **Qualitative**
- User comprehension survey (5-point Likert scale)
- Explanation clarity ratings
- Overall system usability (SUS score)

---

## Final Deliverables Checklist

- [ ] Complete codebase on GitHub (well-documented, README)
- [ ] Literature review document (integrated in final report)
- [ ] Final report (15-20 pages, IEEE format)
- [ ] Technical documentation (architecture, API, deployment)
- [ ] Presentation slides (15-20 slides)
- [ ] Demo video (3-5 minutes, narrated)
- [ ] Evaluation results and analysis
- [ ] (Optional) Poster for future conferences

---

## Notes

**Why This Scope is Achievable:**
1. **Builds on proven work:** Your ROS2 + LLM system already works
2. **Incremental complexity:** Each week adds one major component
3. **Parallel work possible:** Twin setup (Week 5) doesn't block explanation work (Week 4)
4. **Buffer built-in:** Week 8 has flexibility for overruns
5. **Supervisor expertise:** All three areas align with Dr. Sujala's research

**Pragmatic Simplifications:**
- Using existing Nav2 instead of custom planner
- Simple anomaly detection (not deep RL or complex models)
- 5-10 turn memory (not unlimited history)
- Single robot platform (not multi-robot fleet)
- Gazebo twin (not full physics-based simulation)

---

**Sign-off:** This scope balances ambition with feasibility for an 8-week undergraduate special project, building meaningfully on your proven technical capabilities while contributing novel integration insights aligned with Dr. Sujala's XAI and IoT research focus.