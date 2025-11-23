import React, { useEffect, useState } from 'react';
import { Brain, Activity, Clock, CheckCircle, AlertTriangle } from 'lucide-react';
import { rosService, Explanation } from '../services/rosService';

export const ExplanationPanel: React.FC = () => {
  const [explanation, setExplanation] = useState<Explanation | null>(null);

  useEffect(() => {
    rosService.onExplanationReceived((newExplanation) => {
      setExplanation(newExplanation);
    });
  }, []);

  if (!explanation) {
    return (
      <div className="bg-white rounded-xl shadow-sm p-6 border border-gray-100 h-full flex flex-col items-center justify-center text-gray-400">
        <Brain className="w-12 h-12 mb-3 opacity-20" />
        <p>Waiting for navigation decisions...</p>
      </div>
    );
  }

  return (
    <div className="bg-white rounded-xl shadow-sm p-6 border border-gray-100 h-full flex flex-col">
      <div className="flex items-center justify-between mb-4">
        <div className="flex items-center gap-2">
          <Brain className="w-5 h-5 text-indigo-600" />
          <h2 className="text-lg font-semibold text-gray-800">Live Reasoning</h2>
        </div>
        <div className="flex items-center gap-2 text-xs font-medium px-2.5 py-1 rounded-full bg-indigo-50 text-indigo-700">
          <Activity className="w-3 h-3" />
          {explanation.decision_type.replace('_', ' ').toUpperCase()}
        </div>
      </div>

      <div className="flex-1 bg-gray-50 rounded-lg p-4 mb-4 border border-gray-100">
        <p className="text-gray-700 leading-relaxed text-lg">
          "{explanation.text}"
        </p>
      </div>

      <div className="grid grid-cols-3 gap-4">
        <div className="flex flex-col gap-1">
          <span className="text-xs text-gray-500 flex items-center gap-1">
            <CheckCircle className="w-3 h-3" /> Confidence
          </span>
          <span className="font-semibold text-gray-800">
            {(explanation.confidence * 100).toFixed(0)}%
          </span>
        </div>
        <div className="flex flex-col gap-1">
          <span className="text-xs text-gray-500 flex items-center gap-1">
            <Clock className="w-3 h-3" /> Gen Time
          </span>
          <span className="font-semibold text-gray-800">
            {explanation.metrics.generation_time.toFixed(2)}s
          </span>
        </div>
        <div className="flex flex-col gap-1">
          <span className="text-xs text-gray-500 flex items-center gap-1">
            <AlertTriangle className="w-3 h-3" /> Tokens
          </span>
          <span className="font-semibold text-gray-800">
            {explanation.metrics.token_count}
          </span>
        </div>
      </div>
    </div>
  );
};
