# XAI for Human-Robot Interaction

**Category:** Explainable AI methods for robot decision transparency
**Application:** Week 3 (XAI Navigation) and Week 7 (Evaluation)

---

## Paper 1: Industrial Robotics XAI User Interface

**Citation:** Alt, B., Zahn, J., Kienle, C., Dvorak, J., May, M., Katic, D., Jäkel, R., Kopp, T., Beetz, M., & Lanza, G. (2024). Human-AI Interaction in Industrial Robotics: Design and Empirical Evaluation of a User Interface for Explainable AI-Based Robot Program Optimization. CIRP International Conference on Manufacturing Systems.

**Key Contribution:**
- Designed an Explanation User Interface (XUI) for deep learning-based robot program optimizer
- Adapts explanation complexity to user expertise level
- Empirical evaluation with task performance, satisfaction, cognitive load metrics

**Methodology:**
1. Preliminary user survey to identify explanation needs
2. XUI implementation with multi-level explanations
3. Large-scale study measuring three dimensions:
   - Task performance outcomes
   - User satisfaction levels
   - Cognitive load (mental effort required)

**Application to Our Project:**
- Dashboard should offer simple vs detailed explanation modes
- Evaluate user comprehension with similar metrics
- XAI panels need to reduce cognitive load, not increase it

**Implementation (Week 3):**
```python
# Explanation complexity levels
class ExplanationLevel(Enum):
    SIMPLE = "simple"      # "I avoided obstacle"
    DETAILED = "detailed"  # "Detected obstacle at (3.2, 1.8), replanned path using A*"
    EXPERT = "expert"      # Full costmap data + algorithm parameters
```

---

## Paper 2: Behavior Trees for Projection-Level XAI

**Citation:** ACM/IEEE International Conference on Human-Robot Interaction (2024). "What Will You Do Next?" Designing and Evaluating Explanation Generation Using Behavior Trees for Projection-Level XAI.

**Key Contribution:**
- Uses BehaviorTree structure for inherently explainable robot planning
- Projection-level explanations predict future actions
- Hierarchical structure provides natural decision trace

**Why BTs are Explainable:**
```
Root (Sequence)
├── CheckBattery
├── Navigate
│   ├── ComputePath
│   ├── FollowPath
│   └── Recovery
└── ReportStatus
```
Each node represents an interpretable decision point.

**Application to Our Project:**
- Nav2 already uses BehaviorTree.CPP
- Hook into BT execution for explanation generation
- Use Groot2 for visualization in dashboard

**Implementation (Week 3):**
```python
# XAI Navigator hooks into Nav2 BT nodes
def on_navigate_action(self, goal):
    explanation = f"Planning path to ({goal.x:.2f}, {goal.y:.2f})"
    self.publish_explanation(explanation)

def on_recovery_triggered(self, behavior):
    explanation = f"Recovery behavior triggered: {behavior}"
    self.publish_explanation(explanation)
```

---

## Paper 3: Human-Centered Explainable AI Review

**Citation:** Ridley, M. (2025). Human-centered explainable artificial intelligence: An Annual Review of Information Science and Technology (ARIST) paper. Journal of the Association for Information Science and Technology.

**Key Finding:**
"The field of human-centered explainable AI (HCXAI) arose as a response to mainstream explainable AI (XAI) which was focused on algorithmic perspectives and technical challenges."

**HCXAI Principles:**
1. Focus on human understanding, not model accuracy
2. Explanations must be actionable
3. Context matters (who is the user?)
4. Trust is built through transparency

**Application to Our Project:**
- Explanations should answer "why?" not just "what?"
- Adapt language to user expertise
- Build trust through consistent, honest explanations
- Proactive explanations when uncertainty is high

**Implementation (Week 3):**
```python
# Context-aware explanation generation
def generate_explanation(self, decision, user_level="simple"):
    if user_level == "simple":
        return self._simple_explanation(decision)
    elif user_level == "detailed":
        return self._detailed_explanation(decision)
    # Include reasoning, not just result
```

---

## Paper 4: Trust Repair in Robot Failures

**Citation:** ACM/IEEE International Conference on Human-Robot Interaction (2025). Questioning the Robot: Using Human Non-verbal Cues to Estimate the Need for Explanations.

**Key Insight:**
- Humans give non-verbal cues when they need explanations
- Robot failures damage trust
- Proactive explanations repair trust faster than reactive

**Trust Repair Mechanism:**
1. Detect anomaly or failure
2. Immediately generate explanation
3. Suggest corrective action
4. Monitor user response

**Application to Our Project:**
- When anomaly detected in digital twin, generate explanation
- Don't wait for user to ask "what happened?"
- Proactive explanations build trust

**Implementation (Week 5-6):**
```python
# Anomaly detection triggers explanation
def on_anomaly_detected(self, anomaly_score):
    if anomaly_score > THRESHOLD:
        explanation = self.generate_anomaly_explanation(anomaly_score)
        self.publish_alert(explanation)  # Proactive!
```

---

## Paper 5: Immersive XAI for Robot Navigation

**Citation:** Springer Nature Social Robotics (2024). The Influence of a Robot's Personality on Real-Time Explanations of Its Navigation.

**Key Finding:**
Robot personality affects explanation effectiveness:
- Confident robot: "I will avoid obstacle X"
- Cautious robot: "I'm detecting obstacle X, planning alternative route"
- Same information, different framing

**Application to Our Project:**
- Consider tone of explanations
- Consistency in explanation style
- Match robot's "personality" to use case (service robot = helpful tone)

**Implementation (Week 3):**
```python
# Consistent explanation tone
EXPLANATION_TEMPLATES = {
    "obstacle_avoidance": "I detected an obstacle at {location} and planned an alternative route.",
    "goal_reached": "Successfully arrived at {destination}.",
    "path_blocked": "The path to {destination} is blocked. Attempting recovery."
}
```

---

## Paper 6: Explainable Reinforcement Learning Survey

**Citation:** ACM Computing Surveys (2024). Explainable reinforcement learning: A survey and comparative review.

**Key Taxonomy:**
1. **Intrinsic XAI:** Model is inherently interpretable (decision trees, BTs)
2. **Post-hoc XAI:** Explanations generated after the fact (SHAP, LIME)
3. **Local vs Global:** Single decision vs overall behavior

**Relevance:**
- BehaviorTrees = Intrinsic XAI (good!)
- SHAP/LIME = Post-hoc XAI (useful for anomaly model)
- Our system uses both approaches

**Application to Our Project:**
- Navigation explanations: Intrinsic (BT structure)
- Anomaly explanations: Post-hoc (SHAP for feature importance)
- Document which approach used where

---

## Paper 7: SHAP and LIME for Decision Trees

**Citation:** Salih, A. (2025). A Perspective on Explainable Artificial Intelligence Methods: SHAP and LIME. Advanced Intelligent Systems.

**SHAP vs LIME:**
- **SHAP:** Mathematical guarantees for consistency, model-dependent
- **LIME:** Local approximations via interpretable surrogate model

**When to Use:**
- SHAP: When you need feature importance rankings
- LIME: When you need to explain specific predictions
- Both: For comprehensive understanding

**Application to Our Project (Week 6):**
```python
from shap import TreeExplainer

# For anomaly detection model
explainer = TreeExplainer(isolation_forest_model)
shap_values = explainer.shap_values(sensor_features)

# Feature importance for anomaly
important_features = get_top_features(shap_values)
explanation = f"Anomaly detected due to high {important_features[0]} deviation"
```

---

## Summary: XAI Implementation Strategy

1. **Nav2 BehaviorTree** (Intrinsic XAI)
   - Hook into BT nodes for decision explanations
   - Use Groot2 for visualization
   - Hierarchical structure is self-documenting

2. **Template-Based Explanations** (HCXAI)
   - User-friendly language
   - Multiple complexity levels
   - Consistent tone

3. **SHAP for Anomaly Model** (Post-hoc XAI)
   - Feature importance for anomalies
   - Explain why deviation occurred
   - Trust repair through transparency

4. **Evaluation Metrics** (Alt et al.)
   - Task performance
   - User satisfaction
   - Cognitive load

**Total Papers:** 7
**Directly Applicable:** All
**Implementation Weeks:** 3, 5, 6, 7
