/**
 * Anomaly Alert Panel Component
 *
 * Displays real-time anomaly alerts from the digital twin system including:
 * - Severity-colored alert banner
 * - SHAP feature importance visualization
 * - Action buttons (Stop Robot, Monitor, Dismiss)
 * - Anomaly history timeline
 *
 * Integrates with /anomaly/alert topic via rosbridge.
 */

import React, { useState, useEffect } from 'react';
import {
  AlertTriangle,
  AlertOctagon,
  CheckCircle,
  XCircle,
  Activity,
  History,
  StopCircle,
  Eye,
  X,
  Brain,
  TrendingUp,
  TrendingDown,
} from 'lucide-react';
import { rosService, AnomalyAlert } from '../services/rosService';

interface HistoryEntry {
  timestamp: Date;
  alert: AnomalyAlert;
}

export const AnomalyAlertPanel: React.FC = () => {
  const [currentAlert, setCurrentAlert] = useState<AnomalyAlert | null>(null);
  const [anomalyScore, setAnomalyScore] = useState<number>(0);
  const [alertHistory, setAlertHistory] = useState<HistoryEntry[]>([]);
  const [showHistory, setShowHistory] = useState(false);
  const [dismissed, setDismissed] = useState(false);
  const [isMonitoring, setIsMonitoring] = useState(true);

  // Subscribe to ROS topics
  useEffect(() => {
    rosService.onAnomalyScoreUpdate((score) => {
      setAnomalyScore(score);
    });

    rosService.onAnomalyAlertUpdate((alert) => {
      setCurrentAlert(alert);
      setDismissed(false);

      // Add to history
      setAlertHistory((prev) => {
        const newHistory = [
          { timestamp: new Date(), alert },
          ...prev.slice(0, 9), // Keep last 10
        ];
        return newHistory;
      });
    });
  }, []);

  const handleStopRobot = () => {
    rosService.emergencyStop();
    console.log('Emergency stop triggered from anomaly panel');
  };

  const handleDismiss = () => {
    setDismissed(true);
  };

  const toggleMonitoring = () => {
    setIsMonitoring(!isMonitoring);
  };

  const getSeverityConfig = (severity: number) => {
    switch (severity) {
      case 2:
        return {
          bgColor: 'bg-red-900/40',
          borderColor: 'border-red-500',
          textColor: 'text-red-400',
          icon: <AlertOctagon className="w-6 h-6 text-red-400 animate-pulse" />,
          label: 'CRITICAL',
          pulse: true,
        };
      case 1:
        return {
          bgColor: 'bg-yellow-900/30',
          borderColor: 'border-yellow-500',
          textColor: 'text-yellow-400',
          icon: <AlertTriangle className="w-6 h-6 text-yellow-400" />,
          label: 'WARNING',
          pulse: false,
        };
      default:
        return {
          bgColor: 'bg-green-900/20',
          borderColor: 'border-green-500',
          textColor: 'text-green-400',
          icon: <CheckCircle className="w-6 h-6 text-green-400" />,
          label: 'NORMAL',
          pulse: false,
        };
    }
  };

  const formatTimestamp = (date: Date) => {
    return date.toLocaleTimeString();
  };

  const formatValue = (value: number, decimals: number = 2) => {
    return value.toFixed(decimals);
  };

  // Get display data
  const displayAlert = currentAlert;
  const severity = displayAlert?.severity ?? (anomalyScore < -1 ? 2 : anomalyScore < -0.5 ? 1 : 0);
  const config = getSeverityConfig(severity);

  // Don't show if dismissed and not critical
  if (dismissed && severity < 2) {
    return (
      <div className="bg-gray-800 rounded-lg p-3 shadow-lg">
        <div className="flex items-center justify-between">
          <div className="flex items-center space-x-2">
            <Activity className="w-5 h-5 text-gray-400" />
            <span className="text-sm text-gray-400">Anomaly monitoring active</span>
          </div>
          <div className="flex items-center space-x-2">
            <span className="text-xs text-gray-500">Score: {formatValue(anomalyScore)}</span>
            <button
              onClick={() => setDismissed(false)}
              className="text-xs text-blue-400 hover:text-blue-300"
            >
              Show Details
            </button>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="bg-gray-800 rounded-lg shadow-lg overflow-hidden">
      {/* Main Alert Banner */}
      <div className={`p-4 border-l-4 ${config.borderColor} ${config.bgColor}`}>
        <div className="flex items-start justify-between">
          <div className="flex items-center space-x-3">
            {config.icon}
            <div>
              <div className="flex items-center space-x-2">
                <span className={`font-bold ${config.textColor}`}>{config.label}</span>
                {displayAlert?.using_ml && (
                  <span className="flex items-center px-2 py-0.5 bg-blue-600 rounded text-xs text-white">
                    <Brain className="w-3 h-3 mr-1" />
                    ML
                  </span>
                )}
              </div>
              <div className="text-sm text-gray-300 mt-1">
                {displayAlert?.explanation || 'System monitoring active'}
              </div>
            </div>
          </div>

          <div className="flex items-center space-x-2">
            <span className={`text-lg font-mono ${config.textColor}`}>
              {formatValue(displayAlert?.score ?? anomalyScore)}
            </span>
          </div>
        </div>

        {/* Recommended Action */}
        {displayAlert?.recommended_action && (
          <div className="mt-3 text-sm">
            <span className="text-gray-400">Action: </span>
            <span className="text-blue-300">{displayAlert.recommended_action}</span>
          </div>
        )}

        {/* SHAP Feature Importance */}
        {displayAlert?.shap_importance && displayAlert.shap_importance.length > 0 && (
          <div className="mt-4 pt-3 border-t border-gray-700">
            <div className="flex items-center text-xs text-gray-400 mb-2">
              <Brain className="w-3 h-3 mr-1" />
              Feature Contributions (SHAP)
            </div>
            <div className="space-y-2">
              {displayAlert.shap_importance.slice(0, 5).map((item, idx) => (
                <div key={idx} className="flex items-center text-xs">
                  <div className="w-28 truncate text-gray-400 flex items-center">
                    {item.impact > 0 ? (
                      <TrendingUp className="w-3 h-3 mr-1 text-red-400" />
                    ) : (
                      <TrendingDown className="w-3 h-3 mr-1 text-blue-400" />
                    )}
                    {item.feature.replace(/_/g, ' ')}
                  </div>
                  <div className="flex-1 mx-2">
                    <div className="bg-gray-700 rounded-full h-2 overflow-hidden">
                      <div
                        className={`h-2 rounded-full transition-all duration-300 ${
                          item.impact > 0 ? 'bg-red-500' : 'bg-blue-500'
                        }`}
                        style={{
                          width: `${Math.min(Math.abs(item.impact) * 100, 100)}%`,
                          marginLeft: item.impact < 0 ? 'auto' : 0,
                        }}
                      />
                    </div>
                  </div>
                  <span
                    className={`w-12 text-right ${
                      item.impact > 0 ? 'text-red-400' : 'text-blue-400'
                    }`}
                  >
                    {item.impact > 0 ? '+' : ''}
                    {formatValue(item.impact)}
                  </span>
                </div>
              ))}
            </div>
          </div>
        )}

        {/* Action Buttons */}
        <div className="mt-4 flex items-center space-x-2">
          {severity >= 1 && (
            <button
              onClick={handleStopRobot}
              className="flex items-center px-3 py-1.5 bg-red-600 hover:bg-red-700 rounded text-sm text-white transition-colors"
            >
              <StopCircle className="w-4 h-4 mr-1" />
              Stop Robot
            </button>
          )}
          <button
            onClick={toggleMonitoring}
            className={`flex items-center px-3 py-1.5 rounded text-sm transition-colors ${
              isMonitoring
                ? 'bg-blue-600 hover:bg-blue-700 text-white'
                : 'bg-gray-600 hover:bg-gray-700 text-gray-300'
            }`}
          >
            <Eye className="w-4 h-4 mr-1" />
            {isMonitoring ? 'Monitoring' : 'Paused'}
          </button>
          {severity < 2 && (
            <button
              onClick={handleDismiss}
              className="flex items-center px-3 py-1.5 bg-gray-600 hover:bg-gray-700 rounded text-sm text-gray-300 transition-colors"
            >
              <X className="w-4 h-4 mr-1" />
              Dismiss
            </button>
          )}
          <button
            onClick={() => setShowHistory(!showHistory)}
            className="flex items-center px-3 py-1.5 bg-gray-600 hover:bg-gray-700 rounded text-sm text-gray-300 transition-colors ml-auto"
          >
            <History className="w-4 h-4 mr-1" />
            History ({alertHistory.length})
          </button>
        </div>
      </div>

      {/* Alert History */}
      {showHistory && (
        <div className="border-t border-gray-700 max-h-48 overflow-y-auto">
          {alertHistory.length === 0 ? (
            <div className="p-4 text-center text-gray-500 text-sm">No alerts recorded yet</div>
          ) : (
            <div className="divide-y divide-gray-700">
              {alertHistory.map((entry, idx) => {
                const entryConfig = getSeverityConfig(entry.alert.severity);
                return (
                  <div
                    key={idx}
                    className={`p-3 flex items-center space-x-3 ${
                      idx === 0 ? 'bg-gray-700/30' : ''
                    }`}
                  >
                    <div className={entryConfig.textColor}>
                      {entry.alert.severity === 2 ? (
                        <XCircle className="w-4 h-4" />
                      ) : entry.alert.severity === 1 ? (
                        <AlertTriangle className="w-4 h-4" />
                      ) : (
                        <CheckCircle className="w-4 h-4" />
                      )}
                    </div>
                    <div className="flex-1 min-w-0">
                      <div className="text-xs text-gray-400 truncate">
                        {entry.alert.explanation}
                      </div>
                      <div className="text-xs text-gray-500">
                        Score: {formatValue(entry.alert.score)}
                      </div>
                    </div>
                    <div className="text-xs text-gray-500">{formatTimestamp(entry.timestamp)}</div>
                  </div>
                );
              })}
            </div>
          )}
        </div>
      )}
    </div>
  );
};
