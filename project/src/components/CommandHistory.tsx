import React, { useState, useEffect } from 'react';
import { CheckCircle2, Clock, XCircle, RefreshCw, Trash2 } from 'lucide-react';
import { backendService } from '../services/backendService';

interface ConversationTurn {
  id: number;
  turn_number: number;
  user_input: string;
  robot_response: string;
  action_type: string;
  timestamp: string;
  confidence: number;
  latency_ms: number;
  location?: {
    x: number;
    y: number;
    label: string;
  } | null;
}

export const CommandHistory: React.FC = () => {
  const [history, setHistory] = useState<ConversationTurn[]>([]);
  const [loading, setLoading] = useState(true);
  const [refreshing, setRefreshing] = useState(false);

  const loadHistory = async () => {
    try {
      setRefreshing(true);
      const result = await backendService.getConversationHistory();
      // Backend returns "history", not "turns"
      setHistory(result.history || []);
    } catch (error) {
      console.error('Failed to load conversation history:', error);
    } finally {
      setLoading(false);
      setRefreshing(false);
    }
  };

  const handleClearHistory = () => {
    if (confirm('Clear all conversation history? This will start a new session.')) {
      backendService.clearCurrentSession();
      setHistory([]);
    }
  };

  useEffect(() => {
    loadHistory();

    // Auto-refresh every 10 seconds
    const interval = setInterval(loadHistory, 10000);
    return () => clearInterval(interval);
  }, []);

  if (loading) {
    return (
      <div className="bg-slate-800 rounded-lg p-6">
        <h2 className="text-xl font-bold mb-4">Command History</h2>
        <p className="text-slate-400">Loading history...</p>
      </div>
    );
  }

  return (
    <div className="bg-slate-800 rounded-lg p-6">
      <div className="flex justify-between items-center mb-4">
        <h2 className="text-xl font-bold">Command History</h2>
        <div className="flex gap-2">
          <button
            onClick={loadHistory}
            disabled={refreshing}
            className="px-3 py-1 bg-blue-600 hover:bg-blue-700 rounded-md text-sm flex items-center gap-2 disabled:opacity-50"
            title="Refresh history"
          >
            <RefreshCw className={`h-4 w-4 ${refreshing ? 'animate-spin' : ''}`} />
            Refresh
          </button>
          <button
            onClick={handleClearHistory}
            className="px-3 py-1 bg-red-600 hover:bg-red-700 rounded-md text-sm flex items-center gap-2"
            title="Clear history"
          >
            <Trash2 className="h-4 w-4" />
            Clear
          </button>
        </div>
      </div>

      {history.length === 0 ? (
        <div className="text-center py-8 text-slate-400">
          <p>No commands yet. Start by sending a voice command or text input.</p>
          <p className="text-sm mt-2">Session ID: {backendService.getCurrentSessionId()}</p>
        </div>
      ) : (
        <div className="overflow-x-auto">
          <table className="min-w-full">
            <thead>
              <tr className="border-b border-slate-700 text-left">
                <th className="pb-3 pr-6 text-slate-400 font-medium">Command</th>
                <th className="pb-3 pr-6 text-slate-400 font-medium">Time</th>
                <th className="pb-3 pr-6 text-slate-400 font-medium">Status</th>
                <th className="pb-3 pr-6 text-slate-400 font-medium">Duration</th>
              </tr>
            </thead>
            <tbody>
              {history.map((turn) => {
                const duration = turn.latency_ms
                  ? `${(turn.latency_ms / 1000).toFixed(1)}s`
                  : '-';
                const confidence = `${(turn.confidence * 100).toFixed(0)}%`;

                return (
                  <tr key={turn.id} className="border-b border-slate-700">
                    <td className="py-3 pr-6">
                      <div className="font-mono text-sm">{turn.user_input}</div>
                      <div className="text-xs text-slate-500 mt-1">
                        {turn.action_type} ({confidence} confidence)
                      </div>
                      {turn.location && turn.location.label && (
                        <div className="text-xs text-blue-400 mt-1">
                          📍 {turn.location.label} ({turn.location.x.toFixed(2)}, {turn.location.y.toFixed(2)})
                        </div>
                      )}
                    </td>
                    <td className="py-3 pr-6 text-slate-400 whitespace-nowrap text-sm">
                      {new Date(turn.timestamp).toLocaleString()}
                    </td>
                    <td className="py-3 pr-6">
                      <div className="flex items-center text-green-400 text-sm">
                        <CheckCircle2 className="h-4 w-4 mr-1" />
                        <span>{turn.action_type}</span>
                      </div>
                    </td>
                    <td className="py-3 pr-6 text-slate-400 text-sm">
                      {duration}
                    </td>
                  </tr>
                );
              })}
            </tbody>
          </table>
        </div>
      )}
    </div>
  );
};
