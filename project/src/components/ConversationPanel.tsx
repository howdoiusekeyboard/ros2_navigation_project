/**
 * Conversation Panel Component
 *
 * Displays real conversation history with the robot including:
 * - User inputs and robot responses
 * - Location context (coordinates + labels)
 * - Confidence scores and latency metrics
 * - Spatial reference resolution feedback
 *
 * Week 2: Integrated with conversation_memory backend via REST API
 */

import React, { useState, useEffect, useCallback } from 'react';
import { MessageSquare, MapPin, Clock, Brain, RefreshCw, Zap, Target } from 'lucide-react';
import { conversationService, ConversationTurn } from '../services/conversationService';

export const ConversationPanel: React.FC = () => {
  const [history, setHistory] = useState<ConversationTurn[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [sessionId, setSessionId] = useState<string | null>(null);
  const [autoRefresh, setAutoRefresh] = useState(true);

  // Load conversation history
  const loadHistory = useCallback(async () => {
    try {
      setIsLoading(true);
      setError(null);

      const currentSession = conversationService.getCurrentSessionId();
      setSessionId(currentSession);

      if (!currentSession) {
        setHistory([]);
        return;
      }

      const result = await conversationService.getConversationHistory(currentSession, 20);
      setHistory(result.history);
    } catch (err: any) {
      console.error('Failed to load conversation history:', err);
      setError(err.message || 'Failed to load history');
    } finally {
      setIsLoading(false);
    }
  }, []);

  // Initial load and auto-refresh
  useEffect(() => {
    loadHistory();

    // Auto-refresh every 3 seconds if enabled
    let interval: NodeJS.Timeout | null = null;
    if (autoRefresh) {
      interval = setInterval(loadHistory, 3000);
    }

    return () => {
      if (interval) clearInterval(interval);
    };
  }, [loadHistory, autoRefresh]);

  // Get confidence color class
  const getConfidenceColor = (confidence: number) => {
    if (confidence >= 0.9) return 'text-green-400';
    if (confidence >= 0.7) return 'text-yellow-400';
    return 'text-red-400';
  };

  // Get action type color and icon
  const getActionInfo = (actionType: string) => {
    switch (actionType) {
      case 'navigate':
        return { color: 'text-blue-400', label: 'Navigation' };
      case 'twist':
        return { color: 'text-purple-400', label: 'Velocity' };
      case 'stop':
        return { color: 'text-red-400', label: 'Stop' };
      case 'rotate':
        return { color: 'text-cyan-400', label: 'Rotation' };
      case 'move_forward':
        return { color: 'text-green-400', label: 'Forward' };
      case 'move_backward':
        return { color: 'text-orange-400', label: 'Backward' };
      default:
        return { color: 'text-gray-400', label: actionType };
    }
  };

  return (
    <div className="bg-gray-800 rounded-lg p-4 shadow-lg">
      {/* Header */}
      <div className="flex items-center justify-between mb-4">
        <div className="flex items-center">
          <MessageSquare className="w-5 h-5 text-blue-400 mr-2" />
          <h2 className="text-lg font-semibold text-white">Conversation Memory</h2>
        </div>
        <div className="flex items-center gap-2">
          {/* Auto-refresh toggle */}
          <label className="flex items-center gap-1 text-xs text-gray-400 cursor-pointer">
            <input
              type="checkbox"
              checked={autoRefresh}
              onChange={(e) => setAutoRefresh(e.target.checked)}
              className="rounded text-blue-500"
            />
            Auto
          </label>
          {/* Manual refresh button */}
          <button
            onClick={loadHistory}
            className="p-2 rounded hover:bg-gray-700 transition-colors"
            disabled={isLoading}
            title="Refresh conversation history"
          >
            <RefreshCw className={`w-4 h-4 text-gray-400 ${isLoading ? 'animate-spin' : ''}`} />
          </button>
        </div>
      </div>

      {/* Session Info */}
      {sessionId && (
        <div className="mb-3 px-2 py-1 bg-gray-700/50 rounded text-xs text-gray-400">
          Session: <span className="text-blue-400 font-mono">{sessionId}</span>
          <span className="ml-2">• {history.length} turn{history.length !== 1 ? 's' : ''}</span>
        </div>
      )}

      {/* Error Display */}
      {error && (
        <div className="mb-4 p-3 bg-red-900/30 border border-red-500 rounded text-red-300 text-sm">
          {error}
        </div>
      )}

      {/* Empty State */}
      {!isLoading && !error && history.length === 0 && (
        <div className="py-8 text-center text-gray-500">
          <MessageSquare className="w-12 h-12 mx-auto mb-3 opacity-30" />
          <p>No conversation history yet.</p>
          <p className="text-sm">Start talking to your robot!</p>
        </div>
      )}

      {/* Conversation History */}
      <div className="space-y-3 max-h-96 overflow-y-auto">
        {history.map((turn) => {
          const actionInfo = getActionInfo(turn.action_type);

          return (
            <div key={turn.id} className="bg-gray-700 rounded-lg p-3 hover:bg-gray-700/80 transition-colors">
              {/* Turn Header */}
              <div className="flex items-center justify-between text-xs text-gray-400 mb-2">
                <div className="flex items-center gap-2">
                  <span className="px-1.5 py-0.5 bg-gray-600 rounded text-gray-300">
                    Turn {turn.turn_number}
                  </span>
                  <div className="flex items-center">
                    <Clock className="w-3 h-3 mr-1" />
                    {conversationService.formatTimestamp(turn.timestamp)}
                  </div>
                </div>
                <span className={`font-medium ${getConfidenceColor(turn.confidence)}`}>
                  {(turn.confidence * 100).toFixed(0)}%
                </span>
              </div>

              {/* User Input */}
              <div className="mb-2">
                <div className="text-white">
                  <span className="text-blue-300 font-medium">👤 You:</span>{' '}
                  {turn.user_input}
                </div>
              </div>

              {/* Robot Response */}
              <div className="mb-2">
                <div className="text-gray-200">
                  <span className="text-green-300 font-medium">🤖 Robot:</span>{' '}
                  {turn.robot_response || 'Command executed'}
                </div>
              </div>

              {/* Metadata Row */}
              <div className="flex flex-wrap items-center gap-2 text-xs">
                {/* Action Type */}
                <span className={`inline-flex items-center px-2 py-0.5 rounded bg-gray-600 ${actionInfo.color}`}>
                  <Zap className="w-3 h-3 mr-1" />
                  {actionInfo.label}
                </span>

                {/* Location */}
                {turn.location && (
                  <span className="inline-flex items-center px-2 py-0.5 rounded bg-purple-900/50 text-purple-300">
                    <MapPin className="w-3 h-3 mr-1" />
                    {turn.location.label || `(${turn.location.x?.toFixed(1)}, ${turn.location.y?.toFixed(1)})`}
                  </span>
                )}

                {/* Latency */}
                {turn.latency_ms && (
                  <span className="text-gray-500">
                    {conversationService.formatDuration(turn.latency_ms)}
                  </span>
                )}
              </div>
            </div>
          );
        })}
      </div>

      {/* Footer */}
      <div className="mt-4 text-xs text-gray-500 text-center">
        <span className="inline-flex items-center gap-1">
          <Target className="w-3 h-3" />
          Week 2: Live Conversation Memory
        </span>
        {autoRefresh && <span className="ml-2">• Auto-refreshing every 3s</span>}
      </div>
    </div>
  );
};
