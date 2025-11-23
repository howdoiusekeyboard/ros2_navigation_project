# Literature Review: Intelligent Digital Twin with XAI for HRI

**Project:** Intelligent Digital Twin with Explainable AI (XAI) for Human-Robot Interaction
**Student:** BITS Pilani Dubai Campus, 4th Year BE
**Supervisor:** Dr. Sujala D. Shetty
**Date:** November 2025

---

## Overview

This literature review consolidates **25 papers** and **technical implementations** from 2024-2025, organized into five categories that directly inform our system architecture. Each paper is mapped to specific Week (2-7) implementation tasks.

## Document Structure

```
docs/literature_review/
├── README.md                                    # This file
├── 01_xai_human_robot_interaction.md           # 7 papers
├── 02_conversational_ai_llm_robots.md          # 6 papers + GitHub repos
├── 03_digital_twin_robotics.md                 # 7 papers
├── 04_anomaly_detection_methods.md             # 5 papers
├── 05_technical_implementations.md             # ROSGPT, ROSA, Gemini API
└── 06_actionable_insights.md                   # Direct applications
```

---

## Paper-to-Week Mapping

| Week | Focus Area | Key Papers | Implementation Target |
|------|-----------|------------|----------------------|
| **2** | Conversation Memory | ROSGPT, Gemini Structured Output, ROS-LLM | SQLite + context injection |
| **3** | XAI Navigation | BehaviorTree XAI, HCXAI Review, Groot | Explanation generation |
| **4** | Digital Twin Setup | ROS2 DT Architecture, Synchronization | Dual robot + command sync |
| **5** | Twin Monitoring | Multimodal Anomaly Detection, Sensor Fusion | Feature engineering |
| **6** | ML Model | Isolation Forest + Autoencoder, SHAP/LIME | Anomaly detection training |
| **7** | Integration | Alt et al. XUI Evaluation, NASA ROSA | End-to-end pipeline |

---

## Quick Reference: Core Technical Patterns

### 1. LLM → ROS2 Command Pattern (ROSGPT)
```
User Input → REST API → ChatGPT/Gemini → JSON → /voice_cmd topic → Parser → Robot
```
**Why it matters:** Decouples inference from execution via JSON serialization.

### 2. Gemini Structured Output (Google AI 2025)
```python
from pydantic import BaseModel
class RobotCommand(BaseModel):
    action: str
    parameters: dict
    confidence: float

# Gemini returns JSON matching schema exactly
response = model.generate_content(
    config={"response_json_schema": RobotCommand.model_json_schema()}
)
```
**Why it matters:** Guarantees format compliance for safety-critical robot commands.

### 3. BehaviorTree Explainability (Nav2)
```
Nav2 uses BehaviorTree.CPP for orchestration
→ Hierarchical structure is inherently interpretable
→ Hook into BT nodes for explanation generation
→ Groot2 for visualization
```
**Why it matters:** Nav2 already has explainable architecture - leverage it.

### 4. Digital Twin Synchronization (~20ms latency)
```
/cmd_vel (main)
    ↓
Command Synchronizer
    ↓
/real/cmd_vel    /twin/cmd_vel
    ↓                ↓
Physical Robot   Gazebo Twin
```
**Why it matters:** Provides behavioral baseline for anomaly detection.

### 5. Anomaly Detection: Autoencoder + Isolation Forest
```
Sensor Data → Autoencoder → Latent Features → Isolation Forest → Anomaly Score
```
**Why it matters:** Hybrid approach handles high-dimensional features with unsupervised learning.

---

## Supervisor Alignment: Dr. Sujala's Research

Dr. Sujala D. Shetty's recent publications (548 citations, 23 years experience):

1. **Voice-Controlled Bot Navigation** - CINS2024
   - Direct foundation for this project

2. **Enhancing Human-Robot Collaboration with ROS and Flask** - ICCI-25 (Best Paper)
   - LLM + ROS integration pattern

3. **GPT-4 and LLaMA with Speech Processing for HRI** - IEEE Access 2025
   - Comparative analysis relevant to Gemini choice

**Research Areas:** Big Data, AI, IoT, NLP, Web Services - All covered in our project.

---

## Critical Gaps in Existing Literature

1. **No single paper combines all three:** Conversation memory + XAI + Digital Twin
   - **Our contribution:** Integrated system with all components

2. **Limited ROS2 Humble implementations**
   - Most papers use ROS1 or ROS2 Foxy
   - **Our contribution:** Modern ROS2 Humble stack

3. **Gemini underrepresented**
   - Most papers focus on GPT-3.5/GPT-4
   - **Our contribution:** Cost-effective Gemini 2.0 Flash integration

4. **User studies lacking**
   - Technical demos without human evaluation
   - **Our contribution:** 3-5 participant comprehension study

---

## Recommended Reading Order

**Day 1 (Today):**
1. ROSGPT README - Architecture pattern
2. Gemini Structured Output docs - Technical implementation
3. Multimodal Anomaly Detection - Methodology

**Day 2 (Architecture):**
4. BehaviorTree.CPP / Groot documentation
5. ROS2 Digital Twin Architecture paper
6. HCXAI Annual Review

**Week 2-3 (Implementation):**
7. Conversation memory patterns (ROS-LLM, NASA ROSA)
8. XAI for robot failures (trust repair mechanisms)

**Week 6 (ML):**
9. Isolation Forest + Autoencoder hybrid approaches
10. SHAP/LIME for feature importance

---

## Next Steps

1. Read detailed papers in `01_xai_human_robot_interaction.md` through `05_technical_implementations.md`
2. Extract specific algorithms/metrics for Week 2-7 implementation
3. Cite these papers in your final thesis
4. Discuss with Dr. Sujala which papers align best with BITS requirements

---

**Total Papers Reviewed:** 25
**GitHub Repositories Analyzed:** 6 (ROSGPT, ROS-LLM, ROSA, ChatMobile, etc.)
**Technical Documentation:** Gemini API, Nav2, BehaviorTree.CPP
**Conference Proceedings:** HRI 2024/2025, CIRP 2024, IEEE Access, Nature Scientific Data

This literature foundation is **research-backed, not speculative** - every architectural decision traces to published work from 2024-2025.
