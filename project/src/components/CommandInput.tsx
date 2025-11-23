/**
 * Command Input Component
 *
 * Provides voice and text input for robot commands.
 * Integrates with backend Whisper API and Gemini AI for complete pipeline.
 */

import React, { useState, useEffect } from 'react';
import { Mic, Send, Square, MessageSquare, Trash2 } from 'lucide-react';
import { speechService, type CommandExecutionResult } from '../services/speechService';
import { backendService } from '../services/backendService';
import { conversationService } from '../services/conversationService';
import { CommandFeedback } from './CommandFeedback';

export const CommandInput: React.FC = () => {
  const [command, setCommand] = useState('');
  const [isRecording, setIsRecording] = useState(false);
  const [status, setStatus] = useState<'idle' | 'listening' | 'processing' | 'executing'>('idle');
  const [lastResult, setLastResult] = useState<CommandExecutionResult | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [useTurtlesim, setUseTurtlesim] = useState(false);
  const [useCompletePipeline, setUseCompletePipeline] = useState(true);

  // Conversation Memory State
  const [sessionId, setSessionId] = useState<string | null>(
    conversationService.getCurrentSessionId()
  );
  const [turnCount, setTurnCount] = useState(0);

  // Setup service callbacks
  useEffect(() => {
    // Speech recognition result callback
    speechService.onResult((result) => {
      console.log('Transcript received:', result.transcript);
      setCommand(result.transcript);
      setError(null);
    });

    // Command execution result callback
    speechService.onCommandExecution((result) => {
      console.log('Command executed:', result);
      setLastResult(result);
      setError(null);
    });

    // Error callback
    speechService.onError((errorMsg) => {
      console.error('Speech service error:', errorMsg);
      setError(errorMsg);
      setIsRecording(false);
    });

    // Status change callback
    speechService.onStatusChange((newStatus) => {
      console.log('Status changed:', newStatus);
      setStatus(newStatus);

      if (newStatus === 'idle') {
        setIsRecording(false);
      }
    });

    // Apply settings
    speechService.setUseTurtlesim(useTurtlesim);
    speechService.setUseCompletePipeline(useCompletePipeline);
  }, [useTurtlesim, useCompletePipeline]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (command.trim()) {
      console.log('Text command submitted:', command);
      setError(null);

      try {
        // Execute command via backend with session tracking
        const result = await backendService.executeVoiceCommand(
          command,
          useTurtlesim,
          sessionId || undefined  // Pass session ID for conversation memory
        );

        // Update session tracking from response
        if (result.session_id) {
          setSessionId(result.session_id);
          conversationService.setCurrentSessionId(result.session_id);
          setTurnCount(prev => prev + 1);
        }

        setLastResult(result);
        setCommand('');
      } catch (error: any) {
        setError(error.message || 'Failed to execute command');
      }
    }
  };

  // Clear conversation session
  const handleClearSession = async () => {
    try {
      if (sessionId) {
        await conversationService.deleteSession(sessionId);
      }
      conversationService.clearCurrentSession();
      setSessionId(null);
      setTurnCount(0);
      setLastResult(null);
      console.log('Session cleared');
    } catch (error: any) {
      console.error('Failed to clear session:', error);
    }
  };

  const toggleRecording = async () => {
    if (isRecording) {
      // Stop recording
      speechService.stopListening();
    } else {
      // Start recording
      setError(null);
      setLastResult(null);

      try {
        await speechService.startListening();
        setIsRecording(true);
      } catch (error: any) {
        setError(error.message || 'Failed to start recording');
      }
    }
  };

  const getStatusDisplay = () => {
    switch (status) {
      case 'listening':
        return { text: 'Listening...', color: 'text-red-400' };
      case 'processing':
        return { text: 'Processing...', color: 'text-yellow-400' };
      case 'executing':
        return { text: 'Executing...', color: 'text-blue-400' };
      default:
        return { text: 'Ready', color: 'text-green-400' };
    }
  };

  const statusDisplay = getStatusDisplay();

  return (
    <div className="space-y-4">
      {/* Command Input Card */}
      <div className="bg-slate-800 rounded-lg p-6">
        <div className="flex items-center justify-between mb-4">
          <h2 className="text-xl font-bold">Command Input</h2>
          <div className="flex items-center gap-2">
            <span className={`text-sm font-medium ${statusDisplay.color}`}>
              {statusDisplay.text}
            </span>
          </div>
        </div>

        {/* Session Indicator - Conversation Memory */}
        {sessionId && (
          <div className="flex items-center justify-between mb-4 p-3 bg-slate-700/50 rounded-lg">
            <div className="flex items-center gap-3">
              <MessageSquare className="h-5 w-5 text-blue-400" />
              <div>
                <span className="text-sm font-medium text-blue-400">
                  Session: {sessionId}
                </span>
                {turnCount > 0 && (
                  <span className="ml-2 text-xs text-slate-400">
                    ({turnCount} turn{turnCount !== 1 ? 's' : ''})
                  </span>
                )}
              </div>
            </div>
            <button
              onClick={handleClearSession}
              className="flex items-center gap-1 px-2 py-1 text-xs text-red-400 hover:text-red-300 hover:bg-red-900/30 rounded transition-colors"
              title="Clear conversation and start fresh"
            >
              <Trash2 className="h-3 w-3" />
              Clear
            </button>
          </div>
        )}

        {/* Settings */}
        <div className="flex items-center gap-4 mb-4 text-sm">
          <label className="flex items-center gap-2 cursor-pointer">
            <input
              type="checkbox"
              checked={useTurtlesim}
              onChange={(e) => setUseTurtlesim(e.target.checked)}
              className="rounded"
            />
            <span className="text-slate-300">Use Turtlesim</span>
          </label>

          <label className="flex items-center gap-2 cursor-pointer">
            <input
              type="checkbox"
              checked={useCompletePipeline}
              onChange={(e) => setUseCompletePipeline(e.target.checked)}
              className="rounded"
            />
            <span className="text-slate-300">Auto-Execute</span>
          </label>
        </div>

        <form onSubmit={handleSubmit} className="space-y-4">
          <div className="flex flex-col space-y-2">
            <label htmlFor="command" className="text-sm text-slate-400">
              Enter a command for the robot or use voice input
            </label>
            <div className="flex">
              <input
                type="text"
                id="command"
                value={command}
                onChange={(e) => setCommand(e.target.value)}
                placeholder="e.g., spin in a circle, move forward 2 meters, stop"
                className="flex-1 p-2 bg-slate-900 border border-slate-700 rounded-l-md focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent text-white placeholder-slate-500"
                disabled={status !== 'idle'}
              />
              <button
                type="button"
                onClick={toggleRecording}
                disabled={status === 'processing' || status === 'executing'}
                className={`p-2 transition-colors ${
                  isRecording
                    ? 'bg-red-600 hover:bg-red-700'
                    : status === 'idle'
                    ? 'bg-slate-700 hover:bg-slate-600'
                    : 'bg-slate-700 opacity-50 cursor-not-allowed'
                } rounded-none`}
                title={isRecording ? 'Stop recording' : 'Start voice recording'}
              >
                {isRecording ? (
                  <Square className="h-5 w-5" />
                ) : (
                  <Mic className="h-5 w-5" />
                )}
              </button>
              <button
                type="submit"
                disabled={!command.trim() || status !== 'idle'}
                className={`p-2 rounded-r-md transition-colors ${
                  command.trim() && status === 'idle'
                    ? 'bg-blue-600 hover:bg-blue-700'
                    : 'bg-blue-600 opacity-50 cursor-not-allowed'
                }`}
                title="Send command"
              >
                <Send className="h-5 w-5" />
              </button>
            </div>
          </div>

          {/* Error Display */}
          {error && (
            <div className="bg-red-900/30 border border-red-500 rounded-md p-3">
              <p className="text-red-300 text-sm">{error}</p>
            </div>
          )}

          {/* Example Commands */}
          <div className="text-sm text-slate-400">
            <h3 className="font-medium mb-1">Example commands:</h3>
            <ul className="list-disc pl-5 space-y-1">
              <li>"spin in a circle"</li>
              <li>"move forward 2 meters"</li>
              <li>"rotate 90 degrees clockwise"</li>
              <li>"stop"</li>
            </ul>
          </div>
        </form>
      </div>

      {/* Command Feedback */}
      {lastResult && <CommandFeedback result={lastResult} />}
    </div>
  );
};
