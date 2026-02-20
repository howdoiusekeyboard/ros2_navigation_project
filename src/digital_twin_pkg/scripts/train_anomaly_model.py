#!/usr/bin/env python3
"""
Anomaly Detection Model Training Script

Trains an Isolation Forest model for digital twin anomaly detection.
Uses collected sensor comparison data to learn normal operation patterns.

Usage:
    python3 train_anomaly_model.py [--data-path PATH] [--output-path PATH]

Example:
    python3 train_anomaly_model.py --data-path ~/.ros/digital_twin_data/training_data.csv
"""

import argparse
import json
import os
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, Tuple

import numpy as np
import pandas as pd

try:
    import joblib
    from sklearn.ensemble import IsolationForest
    from sklearn.preprocessing import StandardScaler
    from sklearn.model_selection import train_test_split
    from sklearn.metrics import classification_report, confusion_matrix, f1_score
except ImportError:
    print("ERROR: scikit-learn is required. Install with: pip install scikit-learn")
    sys.exit(1)


# Feature names must match digital_twin_monitor_node.py
FEATURE_NAMES = [
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


def generate_synthetic_training_data(n_normal: int = 5000, n_anomaly: int = 500) -> pd.DataFrame:
    """
    Generate synthetic training data for demonstration.

    In production, this should be replaced with real data collected during
    normal robot operation.

    Args:
        n_normal: Number of normal operation samples
        n_anomaly: Number of anomaly samples (for validation)

    Returns:
        DataFrame with features and labels
    """
    print(f"Generating synthetic training data: {n_normal} normal, {n_anomaly} anomaly samples")

    np.random.seed(42)

    # Normal operation data - small differences between real and twin
    normal_data = {
        'position_diff_x': np.random.normal(0, 0.05, n_normal),
        'position_diff_y': np.random.normal(0, 0.05, n_normal),
        'orientation_diff': np.random.normal(0, 0.02, n_normal),
        'linear_vel_diff': np.random.normal(0, 0.02, n_normal),
        'angular_vel_diff': np.random.normal(0, 0.03, n_normal),
        'scan_diff_mean': np.abs(np.random.normal(0.05, 0.03, n_normal)),
        'scan_diff_max': np.abs(np.random.normal(0.15, 0.08, n_normal)),
        'scan_diff_variance': np.abs(np.random.normal(0.01, 0.005, n_normal)),
        'is_anomaly': np.zeros(n_normal)
    }

    # Calculate total position diff
    normal_data['position_diff_total'] = np.sqrt(
        normal_data['position_diff_x']**2 + normal_data['position_diff_y']**2
    )

    # Anomaly data - larger deviations
    anomaly_data = {
        'position_diff_x': np.random.normal(0, 0.3, n_anomaly),
        'position_diff_y': np.random.normal(0, 0.3, n_anomaly),
        'orientation_diff': np.random.normal(0, 0.15, n_anomaly),
        'linear_vel_diff': np.random.normal(0, 0.1, n_anomaly),
        'angular_vel_diff': np.random.normal(0, 0.15, n_anomaly),
        'scan_diff_mean': np.abs(np.random.normal(0.25, 0.15, n_anomaly)),
        'scan_diff_max': np.abs(np.random.normal(0.6, 0.3, n_anomaly)),
        'scan_diff_variance': np.abs(np.random.normal(0.08, 0.04, n_anomaly)),
        'is_anomaly': np.ones(n_anomaly)
    }

    anomaly_data['position_diff_total'] = np.sqrt(
        anomaly_data['position_diff_x']**2 + anomaly_data['position_diff_y']**2
    )

    # Combine into DataFrames
    normal_df = pd.DataFrame(normal_data)
    anomaly_df = pd.DataFrame(anomaly_data)

    # Full dataset
    df = pd.concat([normal_df, anomaly_df], ignore_index=True)

    # Ensure correct column order
    cols = FEATURE_NAMES + ['is_anomaly']
    df = df[cols]

    return df


def load_training_data(data_path: str) -> pd.DataFrame:
    """Load training data from CSV file."""
    if not os.path.exists(data_path):
        print(f"WARNING: Data file not found at {data_path}")
        print("Generating synthetic training data instead...")
        return generate_synthetic_training_data()

    df = pd.read_csv(data_path)

    # Validate columns
    missing_cols = set(FEATURE_NAMES) - set(df.columns)
    if missing_cols:
        print(f"ERROR: Missing columns: {missing_cols}")
        sys.exit(1)

    print(f"Loaded {len(df)} samples from {data_path}")
    return df


def train_model(
    df: pd.DataFrame,
    contamination: float = 0.05,
    n_estimators: int = 100,
    test_size: float = 0.2
) -> Tuple[IsolationForest, StandardScaler, Dict]:
    """
    Train Isolation Forest model.

    Args:
        df: Training data with features and optional 'is_anomaly' column
        contamination: Expected proportion of anomalies (0-0.5)
        n_estimators: Number of trees in the forest
        test_size: Proportion of data for testing

    Returns:
        Tuple of (trained model, fitted scaler, evaluation metrics)
    """
    print(f"\n{'='*60}")
    print("TRAINING ISOLATION FOREST MODEL")
    print(f"{'='*60}")

    # Prepare features
    X = df[FEATURE_NAMES].values

    # Check if we have labels for evaluation
    has_labels = 'is_anomaly' in df.columns
    if has_labels:
        y = df['is_anomaly'].values
        print(f"Dataset: {len(df)} samples, {int(y.sum())} anomalies ({y.mean()*100:.1f}%)")
    else:
        y = None
        print(f"Dataset: {len(df)} samples (unsupervised - no labels)")

    # Split data
    if has_labels and test_size > 0:
        X_train, X_test, y_train, y_test = train_test_split(
            X, y, test_size=test_size, random_state=42, stratify=y
        )
        print(f"Train: {len(X_train)}, Test: {len(X_test)}")
    else:
        X_train, X_test = X, X
        y_train = y_test = y

    # Fit scaler on training data
    print("\nFitting StandardScaler...")
    scaler = StandardScaler()
    X_train_scaled = scaler.fit_transform(X_train)
    X_test_scaled = scaler.transform(X_test)

    # Train Isolation Forest
    print(f"\nTraining Isolation Forest (n_estimators={n_estimators}, contamination={contamination})...")
    model = IsolationForest(
        n_estimators=n_estimators,
        contamination=contamination,
        max_samples='auto',
        random_state=42,
        n_jobs=-1
    )

    # Train on normal data only if we have labels
    if has_labels:
        # Filter to only normal samples for training
        normal_mask = y_train == 0
        X_train_normal = X_train_scaled[normal_mask]
        print(f"Training on {len(X_train_normal)} normal samples")
        model.fit(X_train_normal)
    else:
        model.fit(X_train_scaled)

    print("Training complete!")

    # Evaluate on test set
    metrics = {}
    if has_labels and len(X_test) > 0:
        print(f"\n{'='*60}")
        print("EVALUATION RESULTS")
        print(f"{'='*60}")

        # Predictions: -1 for anomaly, 1 for normal
        y_pred_test = model.predict(X_test_scaled)
        # Convert to 0/1: 1 for anomaly, 0 for normal
        y_pred_binary = (y_pred_test == -1).astype(int)

        # Calculate metrics
        metrics['f1_score'] = f1_score(y_test, y_pred_binary)
        metrics['confusion_matrix'] = confusion_matrix(y_test, y_pred_binary).tolist()

        print(f"\nF1 Score: {metrics['f1_score']:.4f}")
        print(f"\nConfusion Matrix:")
        print(f"                Predicted")
        print(f"              Normal  Anomaly")
        cm = metrics['confusion_matrix']
        print(f"Actual Normal  {cm[0][0]:5d}    {cm[0][1]:5d}")
        print(f"Actual Anomaly {cm[1][0]:5d}    {cm[1][1]:5d}")

        print(f"\nClassification Report:")
        print(classification_report(y_test, y_pred_binary, target_names=['Normal', 'Anomaly']))

        # Check if meets target
        if metrics['f1_score'] >= 0.85:
            print(f"\n[PASS] Model achieves target F1 >= 0.85")
        else:
            print(f"\n[WARNING] Model F1 {metrics['f1_score']:.4f} < 0.85 target")
            print("Consider: collecting more training data, adjusting contamination, or hyperparameter tuning")

    return model, scaler, metrics


def save_model(
    model: IsolationForest,
    scaler: StandardScaler,
    metrics: Dict,
    output_path: str
):
    """Save trained model, scaler, and metadata."""
    output_dir = Path(output_path)
    output_dir.mkdir(parents=True, exist_ok=True)

    model_file = output_dir / 'isolation_forest_model.pkl'
    scaler_file = output_dir / 'feature_scaler.pkl'
    metadata_file = output_dir / 'metadata.json'

    # Save model and scaler
    joblib.dump(model, model_file)
    joblib.dump(scaler, scaler_file)

    # Save metadata
    metadata = {
        'training_date': datetime.now().isoformat(),
        'feature_names': FEATURE_NAMES,
        'model_params': {
            'n_estimators': model.n_estimators,
            'contamination': model.contamination,
            'max_samples': str(model.max_samples)
        },
        'metrics': metrics,
        'scikit_learn_version': __import__('sklearn').__version__
    }

    with open(metadata_file, 'w') as f:
        json.dump(metadata, f, indent=2)

    print(f"\n{'='*60}")
    print("MODEL SAVED")
    print(f"{'='*60}")
    print(f"Model:    {model_file}")
    print(f"Scaler:   {scaler_file}")
    print(f"Metadata: {metadata_file}")


def main():
    parser = argparse.ArgumentParser(
        description='Train Isolation Forest model for digital twin anomaly detection'
    )
    parser.add_argument(
        '--data-path',
        type=str,
        default=os.path.expanduser('~/.ros/digital_twin_data/training_data.csv'),
        help='Path to training data CSV'
    )
    parser.add_argument(
        '--output-path',
        type=str,
        default=os.path.expanduser('~/.ros/digital_twin_models'),
        help='Directory to save trained model'
    )
    parser.add_argument(
        '--contamination',
        type=float,
        default=0.05,
        help='Expected proportion of anomalies (default: 0.05)'
    )
    parser.add_argument(
        '--n-estimators',
        type=int,
        default=100,
        help='Number of trees in Isolation Forest (default: 100)'
    )
    parser.add_argument(
        '--generate-synthetic',
        action='store_true',
        help='Generate synthetic training data instead of loading from file'
    )

    args = parser.parse_args()

    print(f"\n{'='*60}")
    print("DIGITAL TWIN ANOMALY DETECTION - MODEL TRAINING")
    print(f"{'='*60}")
    print(f"Data path:     {args.data_path}")
    print(f"Output path:   {args.output_path}")
    print(f"Contamination: {args.contamination}")
    print(f"Estimators:    {args.n_estimators}")

    # Load or generate data
    if args.generate_synthetic:
        df = generate_synthetic_training_data()
    else:
        df = load_training_data(args.data_path)

    # Train model
    model, scaler, metrics = train_model(
        df,
        contamination=args.contamination,
        n_estimators=args.n_estimators
    )

    # Save model
    save_model(model, scaler, metrics, args.output_path)

    print(f"\n{'='*60}")
    print("TRAINING COMPLETE")
    print(f"{'='*60}")
    print("\nTo use the model, run the digital twin monitor node:")
    print("  ros2 run digital_twin_pkg digital_twin_monitor")
    print("\nThe model will be automatically loaded from:")
    print(f"  {args.output_path}")


if __name__ == '__main__':
    main()
