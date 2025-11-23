/**
 * Command Feedback Component
 *
 * Displays voice command execution results with:
 * - Transcribed command
 * - Parsed action and parameters
 * - Execution status
 * - Latency breakdown (Gemini, execution, total)
 * - Confidence score
 */

import { CheckCircle2, XCircle, Clock, Brain, Zap, AlertTriangle } from 'lucide-react';
import type { CommandExecutionResult } from '../services/speechService';

interface CommandFeedbackProps {
  result: CommandExecutionResult | null;
  className?: string;
}

export const CommandFeedback: React.FC<CommandFeedbackProps> = ({ result, className = '' }) => {
  if (!result) {
    return null;
  }

  const { transcript, parsed_command, execution_status, latency } = result;
  const { action, parameters, confidence, reasoning } = parsed_command;

  // Determine status color and icon
  const getStatusDisplay = (status: string) => {
    if (status === 'executed' || status === 'stopped') {
      return {
        icon: <CheckCircle2 className="w-5 h-5 text-green-500" />,
        color: 'text-green-600',
        bg: 'bg-green-50',
        border: 'border-green-200',
        label: status === 'executed' ? 'Executed' : 'Stopped'
      };
    }

    if (status.includes('not_implemented') || status === 'unknown_action') {
      return {
        icon: <AlertTriangle className="w-5 h-5 text-yellow-500" />,
        color: 'text-yellow-600',
        bg: 'bg-yellow-50',
        border: 'border-yellow-200',
        label: 'Not Implemented'
      };
    }

    if (status.includes('moving') || status.includes('rotating')) {
      return {
        icon: <Zap className="w-5 h-5 text-blue-500" />,
        color: 'text-blue-600',
        bg: 'bg-blue-50',
        border: 'border-blue-200',
        label: 'Executing'
      };
    }

    return {
      icon: <XCircle className="w-5 h-5 text-red-500" />,
      color: 'text-red-600',
      bg: 'bg-red-50',
      border: 'border-red-200',
      label: 'Error'
    };
  };

  const statusDisplay = getStatusDisplay(execution_status);

  // Get confidence color
  const getConfidenceColor = (conf: number) => {
    if (conf >= 0.8) return 'text-green-600';
    if (conf >= 0.6) return 'text-yellow-600';
    return 'text-red-600';
  };

  // Format parameters for display
  const formatParameters = (params: any) => {
    if (!params || Object.keys(params).length === 0) {
      return 'None';
    }

    return Object.entries(params)
      .map(([key, value]) => {
        if (typeof value === 'number') {
          return `${key}: ${value.toFixed(2)}`;
        }
        return `${key}: ${value}`;
      })
      .join(', ');
  };

  // Latency performance indicator
  const getLatencyColor = (ms: number) => {
    if (ms < 500) return 'text-green-600';
    if (ms < 1000) return 'text-yellow-600';
    return 'text-orange-600';
  };

  return (
    <div className={`bg-white rounded-lg shadow-md border border-gray-200 p-4 ${className}`}>
      {/* Header */}
      <div className="flex items-center justify-between mb-4">
        <h3 className="text-lg font-semibold text-gray-800">Command Execution</h3>
        <div className={`flex items-center gap-2 px-3 py-1 rounded-full ${statusDisplay.bg} border ${statusDisplay.border}`}>
          {statusDisplay.icon}
          <span className={`text-sm font-medium ${statusDisplay.color}`}>
            {statusDisplay.label}
          </span>
        </div>
      </div>

      {/* Transcript */}
      <div className="mb-4">
        <h4 className="text-sm font-medium text-gray-600 mb-1">Voice Command:</h4>
        <div className="bg-gray-50 rounded px-3 py-2">
          <p className="text-gray-800 italic">"{transcript}"</p>
        </div>
      </div>

      {/* Parsed Command */}
      <div className="mb-4 grid grid-cols-1 md:grid-cols-2 gap-4">
        <div>
          <h4 className="text-sm font-medium text-gray-600 mb-2 flex items-center gap-2">
            <Brain className="w-4 h-4" />
            Parsed Action:
          </h4>
          <div className="bg-blue-50 rounded px-3 py-2 border border-blue-200">
            <p className="text-blue-800 font-mono text-sm">{action}</p>
          </div>
        </div>

        <div>
          <h4 className="text-sm font-medium text-gray-600 mb-2">Parameters:</h4>
          <div className="bg-gray-50 rounded px-3 py-2">
            <p className="text-gray-700 text-sm font-mono">{formatParameters(parameters)}</p>
          </div>
        </div>
      </div>

      {/* Confidence & Reasoning */}
      <div className="mb-4">
        <div className="flex items-center justify-between mb-2">
          <h4 className="text-sm font-medium text-gray-600">AI Confidence:</h4>
          <span className={`text-lg font-bold ${getConfidenceColor(confidence)}`}>
            {(confidence * 100).toFixed(0)}%
          </span>
        </div>

        {/* Confidence Bar */}
        <div className="w-full bg-gray-200 rounded-full h-2 mb-2">
          <div
            className={`h-2 rounded-full transition-all ${
              confidence >= 0.8
                ? 'bg-green-500'
                : confidence >= 0.6
                ? 'bg-yellow-500'
                : 'bg-red-500'
            }`}
            style={{ width: `${confidence * 100}%` }}
          />
        </div>

        {reasoning && (
          <div className="bg-gray-50 rounded px-3 py-2 mt-2">
            <p className="text-gray-600 text-xs">{reasoning}</p>
          </div>
        )}
      </div>

      {/* Latency Metrics */}
      <div className="border-t border-gray-200 pt-4">
        <h4 className="text-sm font-medium text-gray-600 mb-3 flex items-center gap-2">
          <Clock className="w-4 h-4" />
          Performance Metrics:
        </h4>

        <div className="grid grid-cols-3 gap-4">
          {/* Gemini Time */}
          <div className="text-center">
            <p className="text-xs text-gray-500 mb-1">Gemini Parsing</p>
            <p className={`text-lg font-bold ${getLatencyColor(latency.gemini_ms)}`}>
              {latency.gemini_ms.toFixed(0)}ms
            </p>
          </div>

          {/* Execution Time */}
          <div className="text-center">
            <p className="text-xs text-gray-500 mb-1">Robot Execution</p>
            <p className={`text-lg font-bold ${getLatencyColor(latency.execution_ms)}`}>
              {latency.execution_ms.toFixed(0)}ms
            </p>
          </div>

          {/* Total Time */}
          <div className="text-center">
            <p className="text-xs text-gray-500 mb-1">Total</p>
            <p className={`text-lg font-bold ${getLatencyColor(latency.total_ms)}`}>
              {latency.total_ms.toFixed(0)}ms
            </p>
          </div>
        </div>

        {/* Performance Assessment */}
        <div className="mt-3 text-center">
          {latency.total_ms < 500 ? (
            <p className="text-xs text-green-600 font-medium">⚡ Excellent Performance</p>
          ) : latency.total_ms < 1000 ? (
            <p className="text-xs text-yellow-600 font-medium">✓ Good Performance</p>
          ) : (
            <p className="text-xs text-orange-600 font-medium">⚠️ Slow Response</p>
          )}
        </div>
      </div>
    </div>
  );
};

export default CommandFeedback;
