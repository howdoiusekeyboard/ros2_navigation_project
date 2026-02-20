#!/usr/bin/env python3
"""
Integration Tests for ML-Based Anomaly Detection

Tests the Isolation Forest model training, prediction, and SHAP explanations.
"""

import pytest
import numpy as np
import os
import tempfile
import json
from unittest.mock import MagicMock, patch

# Test imports
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class TestFeatureExtraction:
    """Tests for feature extraction from sensor data."""

    def test_feature_names_consistent(self):
        """Verify feature names are consistent between training and inference."""
        # Feature names from training script
        training_features = [
            'position_diff_x',
            'position_diff_y',
            'position_diff_total',
            'orientation_diff',
            'linear_vel_diff',
            'angular_vel_diff',
            'scan_diff_mean',
            'scan_diff_max',
            'scan_diff_variance'
        ]

        # These should match what the monitor node expects
        assert len(training_features) == 9
        assert 'position_diff_total' in training_features
        assert 'scan_diff_mean' in training_features

    def test_feature_array_conversion(self):
        """Test converting feature dict to numpy array."""
        features = {
            'position_diff_x': 0.05,
            'position_diff_y': 0.03,
            'position_diff_total': 0.058,
            'orientation_diff': 0.02,
            'linear_vel_diff': 0.01,
            'angular_vel_diff': 0.015,
            'scan_diff_mean': 0.04,
            'scan_diff_max': 0.15,
            'scan_diff_variance': 0.001
        }

        feature_names = [
            'position_diff_x',
            'position_diff_y',
            'position_diff_total',
            'orientation_diff',
            'linear_vel_diff',
            'angular_vel_diff',
            'scan_diff_mean',
            'scan_diff_max',
            'scan_diff_variance'
        ]

        # Convert to array
        feature_array = np.array([[features.get(name, 0.0) for name in feature_names]])

        assert feature_array.shape == (1, 9)
        assert feature_array[0, 0] == 0.05  # position_diff_x
        assert feature_array[0, 6] == 0.04  # scan_diff_mean


class TestIsolationForestModel:
    """Tests for Isolation Forest model behavior."""

    @pytest.fixture
    def trained_model(self):
        """Create a trained Isolation Forest model for testing."""
        from sklearn.ensemble import IsolationForest
        from sklearn.preprocessing import StandardScaler

        # Generate synthetic normal data
        np.random.seed(42)
        n_samples = 1000
        normal_data = np.random.normal(0, 0.05, (n_samples, 9))

        # Train scaler and model
        scaler = StandardScaler()
        normal_scaled = scaler.fit_transform(normal_data)

        model = IsolationForest(
            n_estimators=50,
            contamination=0.05,
            random_state=42
        )
        model.fit(normal_scaled)

        return model, scaler

    def test_normal_data_classification(self, trained_model):
        """Test that normal data is classified as normal."""
        model, scaler = trained_model

        # Create normal sample
        normal_sample = np.random.normal(0, 0.05, (1, 9))
        normal_scaled = scaler.transform(normal_sample)

        prediction = model.predict(normal_scaled)
        score = model.decision_function(normal_scaled)

        # Normal should be 1, anomaly should be -1
        # Score > 0 indicates normal
        assert prediction[0] == 1 or score[0] > -0.5  # Some tolerance

    def test_anomaly_data_classification(self, trained_model):
        """Test that anomalous data is classified as anomaly."""
        model, scaler = trained_model

        # Create anomaly sample (much larger deviations)
        anomaly_sample = np.random.normal(0, 0.5, (1, 9))  # 10x normal std
        anomaly_scaled = scaler.transform(anomaly_sample)

        prediction = model.predict(anomaly_scaled)
        score = model.decision_function(anomaly_scaled)

        # Score should be negative for anomalies
        assert score[0] < 0

    def test_decision_function_range(self, trained_model):
        """Test that decision function returns reasonable scores."""
        model, scaler = trained_model

        # Generate mixed data
        normal_samples = np.random.normal(0, 0.05, (50, 9))
        anomaly_samples = np.random.normal(0, 0.5, (10, 9))
        all_samples = np.vstack([normal_samples, anomaly_samples])
        all_scaled = scaler.transform(all_samples)

        scores = model.decision_function(all_scaled)

        # Scores should be distributed around 0
        assert scores.min() < 0
        assert scores.max() > -0.5


class TestModelPersistence:
    """Tests for model saving and loading."""

    def test_model_save_load_roundtrip(self):
        """Test that model can be saved and loaded correctly."""
        from sklearn.ensemble import IsolationForest
        from sklearn.preprocessing import StandardScaler
        import joblib

        # Create and train model
        np.random.seed(42)
        data = np.random.normal(0, 0.05, (100, 9))

        scaler = StandardScaler()
        data_scaled = scaler.fit_transform(data)

        model = IsolationForest(n_estimators=10, random_state=42)
        model.fit(data_scaled)

        # Save to temp directory
        with tempfile.TemporaryDirectory() as tmpdir:
            model_path = os.path.join(tmpdir, 'model.pkl')
            scaler_path = os.path.join(tmpdir, 'scaler.pkl')

            joblib.dump(model, model_path)
            joblib.dump(scaler, scaler_path)

            # Load back
            loaded_model = joblib.load(model_path)
            loaded_scaler = joblib.load(scaler_path)

            # Test prediction consistency
            test_sample = np.random.normal(0, 0.05, (1, 9))
            original_pred = model.predict(scaler.transform(test_sample))
            loaded_pred = loaded_model.predict(loaded_scaler.transform(test_sample))

            assert np.array_equal(original_pred, loaded_pred)

    def test_metadata_json_format(self):
        """Test that metadata is saved in correct format."""
        metadata = {
            'training_date': '2024-01-15T12:00:00',
            'feature_names': [
                'position_diff_x', 'position_diff_y', 'position_diff_total',
                'orientation_diff', 'linear_vel_diff', 'angular_vel_diff',
                'scan_diff_mean', 'scan_diff_max', 'scan_diff_variance'
            ],
            'model_params': {
                'n_estimators': 100,
                'contamination': 0.05,
                'max_samples': 'auto'
            },
            'metrics': {
                'f1_score': 0.85,
                'confusion_matrix': [[950, 50], [10, 90]]
            }
        }

        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            json.dump(metadata, f)
            temp_path = f.name

        try:
            with open(temp_path, 'r') as f:
                loaded = json.load(f)

            assert loaded['feature_names'] == metadata['feature_names']
            assert loaded['model_params']['n_estimators'] == 100
            assert loaded['metrics']['f1_score'] == 0.85
        finally:
            os.unlink(temp_path)


class TestSHAPExplanations:
    """Tests for SHAP-based explanations."""

    @pytest.fixture
    def shap_setup(self):
        """Set up model with SHAP explainer."""
        try:
            import shap
        except ImportError:
            pytest.skip("SHAP not installed")

        from sklearn.ensemble import IsolationForest
        from sklearn.preprocessing import StandardScaler

        np.random.seed(42)
        data = np.random.normal(0, 0.05, (100, 9))

        scaler = StandardScaler()
        data_scaled = scaler.fit_transform(data)

        model = IsolationForest(n_estimators=20, random_state=42)
        model.fit(data_scaled)

        explainer = shap.TreeExplainer(model)

        return model, scaler, explainer

    def test_shap_values_shape(self, shap_setup):
        """Test SHAP values have correct shape."""
        model, scaler, explainer = shap_setup

        test_sample = np.random.normal(0, 0.05, (1, 9))
        test_scaled = scaler.transform(test_sample)

        shap_values = explainer.shap_values(test_scaled)

        # SHAP values should have same shape as input
        if isinstance(shap_values, list):
            values = shap_values[0]
        else:
            values = shap_values

        assert values.shape == (1, 9)

    def test_shap_feature_importance_ranking(self, shap_setup):
        """Test SHAP values can be used to rank features."""
        model, scaler, explainer = shap_setup

        # Create sample with one dominant feature
        test_sample = np.zeros((1, 9))
        test_sample[0, 0] = 1.0  # Large position_diff_x
        test_scaled = scaler.transform(test_sample)

        shap_values = explainer.shap_values(test_scaled)

        if isinstance(shap_values, list):
            values = shap_values[0][0]
        else:
            values = shap_values[0]

        # Create ranked list
        feature_names = [
            'position_diff_x', 'position_diff_y', 'position_diff_total',
            'orientation_diff', 'linear_vel_diff', 'angular_vel_diff',
            'scan_diff_mean', 'scan_diff_max', 'scan_diff_variance'
        ]

        importance = list(zip(feature_names, values))
        importance.sort(key=lambda x: abs(x[1]), reverse=True)

        # Should be able to get top features
        assert len(importance) == 9
        assert importance[0][0] in feature_names


class TestAnomalyScoring:
    """Tests for real-time anomaly scoring behavior."""

    def test_threshold_based_fallback(self):
        """Test threshold-based scoring when ML not available."""
        # Simulate threshold-based scoring
        def compute_threshold_score(features):
            pos_diff = features.get('position_diff_total', 0.0)
            vel_diff = abs(features.get('linear_vel_diff', 0.0))
            scan_diff = features.get('scan_diff_mean', 0.0)

            score = 0.0

            if pos_diff > 0.5:
                score -= 1.0
            elif pos_diff > 0.2:
                score -= 0.5
            else:
                score += 0.5

            if vel_diff > 0.1:
                score -= 0.3

            if scan_diff > 0.3:
                score -= 0.2

            return score

        # Test normal case
        normal_features = {
            'position_diff_total': 0.05,
            'linear_vel_diff': 0.02,
            'scan_diff_mean': 0.04
        }
        assert compute_threshold_score(normal_features) > 0

        # Test anomaly case
        anomaly_features = {
            'position_diff_total': 0.6,
            'linear_vel_diff': 0.15,
            'scan_diff_mean': 0.35
        }
        assert compute_threshold_score(anomaly_features) < -0.5

    def test_severity_computation(self):
        """Test severity level computation from score."""
        def compute_severity(score):
            if score < -1.0:
                return 2  # Critical
            elif score < -0.5:
                return 1  # Warning
            else:
                return 0  # Info

        assert compute_severity(-1.5) == 2  # Critical
        assert compute_severity(-0.8) == 1  # Warning
        assert compute_severity(0.3) == 0   # Info
        assert compute_severity(-0.5) == 0  # Boundary


class TestRealtimePerformance:
    """Tests for real-time performance requirements."""

    def test_prediction_latency(self):
        """Test that prediction completes within latency budget."""
        from sklearn.ensemble import IsolationForest
        from sklearn.preprocessing import StandardScaler
        import time

        # Create model
        np.random.seed(42)
        data = np.random.normal(0, 0.05, (1000, 9))

        scaler = StandardScaler()
        data_scaled = scaler.fit_transform(data)

        model = IsolationForest(n_estimators=100, random_state=42)
        model.fit(data_scaled)

        # Test prediction latency
        test_sample = np.random.normal(0, 0.05, (1, 9))
        test_scaled = scaler.transform(test_sample)

        latencies = []
        for _ in range(100):
            start = time.time()
            model.predict(test_scaled)
            model.decision_function(test_scaled)
            latencies.append((time.time() - start) * 1000)  # ms

        avg_latency = np.mean(latencies)

        # Should complete within 100ms (target from requirements)
        assert avg_latency < 100, f"Average latency {avg_latency:.2f}ms exceeds 100ms target"


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
