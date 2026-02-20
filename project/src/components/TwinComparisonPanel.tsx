/**
 * Twin Comparison Panel Component
 *
 * Displays real robot vs digital twin comparison including:
 * - Position and orientation differences
 * - Velocity mismatches
 * - Laser scan deviations
 * - Anomaly score and alerts
 *
 * Live data via rosbridge:
 * - /twin/sensor_diff (std_msgs/String JSON)
 * - /anomaly/score (std_msgs/Float32)
 * - /anomaly/alert (std_msgs/String JSON)
 */

import React, { useState, useEffect } from 'react';
import {
  GitCompare,
  RefreshCw,
  AlertTriangle,
  CheckCircle,
  XCircle,
  Activity,
} from 'lucide-react';
import { rosService, type TwinSensorDiff, type AnomalyAlert } from '../services/rosService';

interface SensorComparison {
  positionDiffX: number;
  positionDiffY: number;
  positionDiffTotal: number;
  orientationDiff: number;
  linearVelDiff: number;
  angularVelDiff: number;
  scanDiffMean: number;
  scanDiffMax: number;
  scanDiffVariance: number;
}

interface AnomalyStatus {
  score: number;
  isAnomaly: boolean;
  severity: 0 | 1 | 2; // 0=info, 1=warning, 2=critical
  explanation: string;
  recommendedAction: string;
}

const defaultSensorComparison: SensorComparison = {
  positionDiffX: 0,
  positionDiffY: 0,
  positionDiffTotal: 0,
  orientationDiff: 0,
  linearVelDiff: 0,
  angularVelDiff: 0,
  scanDiffMean: 0,
  scanDiffMax: 0,
  scanDiffVariance: 0,
};

const defaultAnomalyStatus: AnomalyStatus = {
  score: 0,
  isAnomaly: false,
  severity: 0,
  explanation: 'Waiting for digital twin telemetry...',
  recommendedAction: 'Continue monitoring',
};

export const TwinComparisonPanel: React.FC = () => {
  const [comparison, setComparison] = useState<SensorComparison>(defaultSensorComparison);
  const [anomalyStatus, setAnomalyStatus] = useState<AnomalyStatus>(defaultAnomalyStatus);
  const [isLoading, setIsLoading] = useState(false);
  const [lastUpdateTime, setLastUpdateTime] = useState(new Date());

  // Subscribe to ROS topics
  useEffect(() => {
    rosService.onTwinSensorDiff((diff: TwinSensorDiff) => {
      setLastUpdateTime(new Date());
      setComparison({
        positionDiffX: diff.position_diff_x ?? 0,
        positionDiffY: diff.position_diff_y ?? 0,
        positionDiffTotal: diff.position_diff_total ?? 0,
        orientationDiff: diff.orientation_diff ?? 0,
        linearVelDiff: diff.linear_vel_diff ?? 0,
        angularVelDiff: diff.angular_vel_diff ?? 0,
        scanDiffMean: diff.scan_diff_mean ?? 0,
        scanDiffMax: diff.scan_diff_max ?? 0,
        scanDiffVariance: diff.scan_diff_variance ?? 0,
      });
    });

    rosService.onAnomalyScore((score: number) => {
      setLastUpdateTime(new Date());
      setAnomalyStatus((prev) => ({
        ...prev,
        score,
        // If no explicit alert received, treat as normal
        isAnomaly: prev.isAnomaly,
      }));
    });

    rosService.onAnomalyAlert((alert: AnomalyAlert) => {
      setLastUpdateTime(new Date());
      setAnomalyStatus({
        score: alert.score,
        isAnomaly: alert.is_anomaly,
        severity: (alert.severity ?? 1) as 0 | 1 | 2,
        explanation: alert.explanation || 'Anomaly detected',
        recommendedAction: alert.recommended_action || 'Investigate',
      });
    });
  }, []);

  const refreshData = () => {
    setIsLoading(true);
    setTimeout(() => {
      setIsLoading(false);
      setLastUpdateTime(new Date());
    }, 500);
  };

  const getSeverityIcon = (severity: number) => {
    switch (severity) {
      case 0:
        return <CheckCircle className="w-5 h-5 text-green-400" />;
      case 1:
        return <AlertTriangle className="w-5 h-5 text-yellow-400" />;
      case 2:
        return <XCircle className="w-5 h-5 text-red-400" />;
      default:
        return <Activity className="w-5 h-5 text-gray-400" />;
    }
  };

  const getSeverityColor = (severity: number) => {
    switch (severity) {
      case 0:
        return 'border-green-500 bg-green-900/20';
      case 1:
        return 'border-yellow-500 bg-yellow-900/20';
      case 2:
        return 'border-red-500 bg-red-900/20';
      default:
        return 'border-gray-500 bg-gray-900/20';
    }
  };

  const getValueColor = (value: number, threshold: number) => {
    if (value <= threshold * 0.5) return 'text-green-400';
    if (value <= threshold) return 'text-yellow-400';
    return 'text-red-400';
  };

  const formatValue = (value: number, decimals: number = 3) => {
    return value.toFixed(decimals);
  };

  return (
    <div className="bg-gray-800 rounded-lg p-4 shadow-lg">
      <div className="flex items-center justify-between mb-4">
        <div className="flex items-center">
          <GitCompare className="w-5 h-5 text-purple-400 mr-2" />
          <h2 className="text-lg font-semibold text-white">Digital Twin Comparison</h2>
        </div>
        <button
          onClick={refreshData}
          className="p-2 rounded hover:bg-gray-700 transition-colors"
          disabled={isLoading}
        >
          <RefreshCw className={`w-4 h-4 text-gray-400 ${isLoading ? 'animate-spin' : ''}`} />
        </button>
      </div>

      {/* Anomaly Status Banner */}
      <div className={`mb-4 p-3 rounded-lg border ${getSeverityColor(anomalyStatus.severity)}`}>
        <div className="flex items-center justify-between">
          <div className="flex items-center">
            {getSeverityIcon(anomalyStatus.severity)}
            <span className="ml-2 font-medium text-white">
              {anomalyStatus.isAnomaly ? 'ANOMALY DETECTED' : 'System Normal'}
            </span>
          </div>
          <span className="text-sm">
            Score:{' '}
            <span
              className={
                anomalyStatus.score < -0.5
                  ? 'text-red-400'
                  : anomalyStatus.score < 0
                  ? 'text-yellow-400'
                  : 'text-green-400'
              }
            >
              {formatValue(anomalyStatus.score, 2)}
            </span>
          </span>
        </div>
        <div className="mt-2 text-sm text-gray-300">{anomalyStatus.explanation}</div>
        <div className="mt-1 text-xs text-gray-400">
          Action: <span className="text-blue-300">{anomalyStatus.recommendedAction}</span>
        </div>
      </div>

      {/* Sensor Comparison Grid */}
      <div className="grid grid-cols-2 gap-3">
        {/* Position Differences */}
        <div className="bg-gray-700 rounded-lg p-3">
          <h3 className="text-xs font-semibold text-gray-400 mb-2">Position Diff</h3>
          <div className="space-y-1">
            <div className="flex justify-between text-xs">
              <span className="text-gray-400">X:</span>
              <span className={getValueColor(Math.abs(comparison.positionDiffX), 0.1)}>
                {formatValue(comparison.positionDiffX)}m
              </span>
            </div>
            <div className="flex justify-between text-xs">
              <span className="text-gray-400">Y:</span>
              <span className={getValueColor(Math.abs(comparison.positionDiffY), 0.1)}>
                {formatValue(comparison.positionDiffY)}m
              </span>
            </div>
            <div className="flex justify-between text-xs font-medium">
              <span className="text-gray-400">Total:</span>
              <span className={getValueColor(comparison.positionDiffTotal, 0.2)}>
                {formatValue(comparison.positionDiffTotal)}m
              </span>
            </div>
          </div>
        </div>

        {/* Velocity Differences */}
        <div className="bg-gray-700 rounded-lg p-3">
          <h3 className="text-xs font-semibold text-gray-400 mb-2">Velocity Diff</h3>
          <div className="space-y-1">
            <div className="flex justify-between text-xs">
              <span className="text-gray-400">Linear:</span>
              <span className={getValueColor(comparison.linearVelDiff, 0.05)}>
                {formatValue(comparison.linearVelDiff)}m/s
              </span>
            </div>
            <div className="flex justify-between text-xs">
              <span className="text-gray-400">Angular:</span>
              <span className={getValueColor(comparison.angularVelDiff, 0.1)}>
                {formatValue(comparison.angularVelDiff)}rad/s
              </span>
            </div>
            <div className="flex justify-between text-xs">
              <span className="text-gray-400">Orientation:</span>
              <span className={getValueColor(Math.abs(comparison.orientationDiff), 0.1)}>
                {formatValue(comparison.orientationDiff)}rad
              </span>
            </div>
          </div>
        </div>

        {/* Laser Scan Differences */}
        <div className="bg-gray-700 rounded-lg p-3 col-span-2">
          <h3 className="text-xs font-semibold text-gray-400 mb-2">Laser Scan Diff</h3>
          <div className="grid grid-cols-3 gap-2">
            <div className="text-center">
              <div className="text-xs text-gray-400">Mean</div>
              <div className={`text-sm ${getValueColor(comparison.scanDiffMean, 0.1)}`}>
                {formatValue(comparison.scanDiffMean)}m
              </div>
            </div>
            <div className="text-center">
              <div className="text-xs text-gray-400">Max</div>
              <div className={`text-sm ${getValueColor(comparison.scanDiffMax, 0.3)}`}>
                {formatValue(comparison.scanDiffMax)}m
              </div>
            </div>
            <div className="text-center">
              <div className="text-xs text-gray-400">Variance</div>
              <div className={`text-sm ${getValueColor(comparison.scanDiffVariance, 0.01)}`}>
                {formatValue(comparison.scanDiffVariance, 4)}
              </div>
            </div>
          </div>
        </div>
      </div>

      {/* Last Update Time */}
      <div className="mt-3 text-xs text-gray-500 text-center">
        Last updated: {lastUpdateTime.toLocaleTimeString()}
      </div>

      <div className="mt-2 text-xs text-gray-500 text-center">
        Live digital twin telemetry via rosbridge
      </div>
    </div>
  );
};
