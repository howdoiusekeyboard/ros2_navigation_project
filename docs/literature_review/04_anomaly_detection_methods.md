# Anomaly Detection Methods for Mobile Robots

**Category:** ML-based detection of behavioral deviations
**Application:** Week 5 (Feature Engineering) and Week 6 (Model Training)

---

## Paper 1: Multimodal Anomaly Detection for Mobile Robots

**Citation:** Yoo, Y., Lee, C., & Zhang, B. (2024). Multimodal Anomaly Detection based on Deep Auto-Encoder for Object Slip Perception of Mobile Manipulation Robots. arXiv:2403.03563.

**Key Innovation:**
- Deep autoencoder for multimodal sensor fusion
- RGB, depth, audio, force-torque data combined
- Anomaly = reconstruction error in latent space

**Methodology:**
```
Sensor Data → Encoder → Latent Representation → Decoder → Reconstruction
                               ↓
                    Compare input vs output
                               ↓
                    High difference = Anomaly
```

**Why Autoencoder:**
- Learns "normal" operation patterns
- No labeled anomaly data needed (unsupervised)
- Handles high-dimensional sensor fusion

**Evaluation:**
- Real-world household environment
- Various object types and robot behaviors
- Robust to visual and auditory noise

**Application to Our Project:**
```python
# Our digital twin comparison is simpler but follows same principle
def compute_anomaly_score(self):
    real_features = self.extract_features(real_sensors)
    twin_features = self.extract_features(twin_sensors)

    # Deviation = anomaly score
    deviation = np.linalg.norm(real_features - twin_features)
    return deviation
```

---

## Paper 2: Isolation Forest + Autoencoder Hybrid

**Citation:** ResearchGate (2024). Anomaly Detection using combination of Autoencoder and Isolation Forest.

**Key Finding:**
"Hybrid approach enhances anomaly detection where individual methods might not perform well."

**Two-Stage Pipeline:**
1. **Autoencoder:** Learn compact latent representation
2. **Isolation Forest:** Detect outliers in latent space

**Why Combine:**
- Autoencoder reduces dimensionality
- Isolation Forest is fast and interpretable
- Together: scalable + accurate

**Isolation Forest Properties:**
- O(n log n) training time
- Unsupervised (no labels needed)
- Handles high-dimensional data
- Returns anomaly scores (not just binary)

**Application to Our Project (Week 6):**
```python
from sklearn.ensemble import IsolationForest

# Train on normal operation data
def train_anomaly_model(self, normal_data):
    # Features: [position_diff, velocity_diff, scan_diff, ...]
    self.model = IsolationForest(
        n_estimators=100,
        contamination=0.1,  # Expect 10% anomalies
        random_state=42
    )
    self.model.fit(normal_data)

def predict_anomaly(self, current_features):
    # Returns -1 for anomaly, 1 for normal
    score = self.model.score_samples(current_features)
    return score[0]  # Lower = more anomalous
```

---

## Paper 3: LSTM-Autoencoder for Time Series

**Citation:** IEEE (2024). Sensor Anomaly Detection in Nuclear Power Plant Using Deep LSTM Denoising Autoencoder and Isolation Forest.

**Key Insight:**
- LSTM captures temporal patterns
- Denoising autoencoder handles sensor noise
- Combined: robust time-series anomaly detection

**Architecture:**
```
Time Series → LSTM Encoder → Latent → LSTM Decoder → Reconstruction
                                 ↓
                        Isolation Forest
                                 ↓
                          Anomaly Score
```

**Application to Our Project:**
For time-series sensor data (odometry over time), LSTM could capture drift patterns. However, for Week 6 timeline, simpler features may suffice:
```python
# Simple temporal features (no LSTM needed)
def extract_temporal_features(self, history_window=10):
    features = {
        'position_drift': self.compute_drift(history_window),
        'velocity_variance': self.compute_variance(history_window),
        'scan_consistency': self.compute_scan_consistency(history_window)
    }
    return features
```

---

## Paper 4: Multi-Domain Anomaly Dataset

**Citation:** Nature Scientific Data (2025). Multi-Domain Indoor Dataset for Visual Place Recognition and Anomaly Detection by Mobile Robots.

**Benchmark Results:**
- Isolation Forest (IF): Strong baseline
- One-Class SVM: Good for small datasets
- Autoencoder (AE): Best for high-dimensional

**Key Quote:**
"IF has the ability to detect anomalies represented by high-dimensional features."

**Dataset Characteristics:**
- Indoor environment (like our simulation)
- Mobile robot navigation
- Place recognition + anomaly detection

**Application to Our Project:**
Isolation Forest is validated for this exact use case. Our choice is scientifically justified.

**Evaluation Metrics:**
- Precision, Recall, F1-score
- ROC-AUC curve
- False Positive Rate (critical for user trust)

**Implementation (Week 6):**
```python
from sklearn.metrics import precision_recall_fscore_support, roc_auc_score

def evaluate_model(self, test_data, test_labels):
    predictions = self.model.predict(test_data)
    precision, recall, f1, _ = precision_recall_fscore_support(
        test_labels, predictions, average='binary'
    )
    auc = roc_auc_score(test_labels, self.model.score_samples(test_data))
    return {'precision': precision, 'recall': recall, 'f1': f1, 'auc': auc}
```

---

## Paper 5: SHAP for Feature Importance

**Citation:** Advanced Intelligent Systems (2025). A Perspective on Explainable Artificial Intelligence Methods: SHAP and LIME.

**Why SHAP for Anomaly Explanation:**
- Mathematical guarantees for consistency
- Shows which features caused anomaly
- Essential for XAI component

**SHAP Values:**
```
Feature Importance:
- position_diff: 0.45 (most important)
- velocity_diff: 0.30
- scan_diff: 0.25

Explanation: "Anomaly detected primarily due to position deviation"
```

**Application to Our Project (Week 6):**
```python
import shap

def explain_anomaly(self, anomaly_features):
    explainer = shap.TreeExplainer(self.model)
    shap_values = explainer.shap_values(anomaly_features)

    # Get top contributing features
    feature_importance = dict(zip(self.feature_names, shap_values[0]))
    sorted_features = sorted(feature_importance.items(), key=lambda x: abs(x[1]), reverse=True)

    explanation = f"Anomaly caused by: {sorted_features[0][0]} ({sorted_features[0][1]:.2f})"
    return explanation
```

---

## Summary: Anomaly Detection Pipeline

**Week 5: Feature Engineering**
```python
features = [
    'real_x - twin_x',           # Position X difference
    'real_y - twin_y',           # Position Y difference
    'real_theta - twin_theta',   # Orientation difference
    'real_linear_vel - twin_linear_vel',  # Velocity difference
    'real_angular_vel - twin_angular_vel', # Angular velocity diff
    'scan_diff_mean',            # Average laser scan difference
    'scan_diff_max',             # Max laser scan difference
    'cmd_vel_match',             # Did both receive same command?
]
```

**Week 6: Model Training**
```python
# 1. Collect normal operation data (2+ hours)
normal_data = collect_baseline_data()

# 2. Train Isolation Forest
model = IsolationForest(n_estimators=100, contamination=0.1)
model.fit(normal_data)

# 3. Validate on test set
test_data, test_labels = get_test_data()
metrics = evaluate_model(model, test_data, test_labels)
# Target: >80% accuracy, >85% precision

# 4. Deploy for real-time scoring
def realtime_anomaly_check(current_features):
    score = model.score_samples(current_features.reshape(1, -1))
    is_anomaly = score[0] < THRESHOLD
    if is_anomaly:
        explanation = explain_anomaly(current_features)
        publish_alert(explanation)
```

**Total Papers:** 5
**Primary Method:** Isolation Forest (fast, unsupervised, proven)
**Backup Method:** Autoencoder + IF hybrid (if time permits)
**Explainability:** SHAP for feature importance
**Target Accuracy:** >80% (Week 6 goal)
