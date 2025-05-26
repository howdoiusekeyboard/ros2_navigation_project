import React, { useState, useEffect, useRef } from 'react';
import { Mic, Send, History, Play } from 'lucide-react';
import { turtleControlService, type CommandHistoryItem } from '../services/TurtleControlService';
import { llmService } from '../services/LlmService';

// Basic natural-language → velocity extractor. For production this should be
// replaced with a proper LLM call (e.g. OpenAI / local model).
const simpleParser = (text: string) => {
  const lower = text.toLowerCase();

  // Stop command
  if (/stop|halt|pause/.test(lower)) {
    return { action: 'stop' } as const;
  }

  // Match "linear X" and "angular Y" numbers – very naive
  const linMatch = lower.match(/linear\s+([0-9]*\.?[0-9]+)/);
  const angMatch = lower.match(/angular\s+([0-9]*\.?[0-9]+)/);

  const lin = linMatch ? parseFloat(linMatch[1]) : undefined;
  const ang = angMatch ? parseFloat(angMatch[1]) : undefined;

  // If text contains "circle" or "circular" assume start
  if (/circle|circular|spin|rotate/.test(lower)) {
    return { action: 'start', linear: lin ?? 2.0, angular: ang ?? 1.0 } as const;
  }

  return { action: 'unknown' } as const;
};

export const CommandInput: React.FC = () => {
  const [command, setCommand] = useState('');
  const [isRecording, setIsRecording] = useState(false);
  const [isTypingFromVoice, setIsTypingFromVoice] = useState(false);
  const [showHistory, setShowHistory] = useState(false);
  const [commandHistory, setCommandHistory] = useState<CommandHistoryItem[]>([]);
  
  const autoExecuteTimer = useRef<NodeJS.Timeout | null>(null);
  const typingInterval = useRef<NodeJS.Timeout | null>(null);
  const recognitionRef = useRef<any>(null);

  useEffect(() => {
    // Subscribe to command history changes
    const unsubscribe = turtleControlService.onHistoryChange(setCommandHistory);
    return unsubscribe;
  }, []);

  const processCommand = (text: string) => {
    // Clear any pending auto-execution
    if (autoExecuteTimer.current) {
      clearTimeout(autoExecuteTimer.current);
    }

    // First try the lightweight regex parser
    let parsed: any = simpleParser(text);

    // If the simple parser couldn't understand, fall back to the LLM
    if (parsed.action === 'unknown') {
      llmService.parseCircularMotion(text).then((llmParsed) => {
        if (llmParsed) {
          turtleControlService.executeAction(llmParsed, 'command');
          turtleControlService.addToHistory(text, 'success', llmParsed);
        } else {
          turtleControlService.addToHistory(text, 'error');
          console.warn('Unable to parse command');
        }
      });
    } else {
      turtleControlService.executeAction(parsed, 'command');
      turtleControlService.addToHistory(text, 'success', parsed);
    }
  };

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (command.trim()) {
      processCommand(command.trim());
      setCommand('');
    }
  };

  // Enhanced voice recognition with typing animation
  useEffect(() => {
    if (!('webkitSpeechRecognition' in window) && !('SpeechRecognition' in window)) {
      return; // browser unsupported
    }

    const SpeechRecognition = (window as any).SpeechRecognition || (window as any).webkitSpeechRecognition;
    recognitionRef.current = new SpeechRecognition();
    recognitionRef.current.lang = 'en-US';
    recognitionRef.current.continuous = false;
    recognitionRef.current.interimResults = false;

    recognitionRef.current.onresult = (event: any) => {
      const transcript = event.results[0][0].transcript;
      startTypingAnimation(transcript);
    };

    recognitionRef.current.onerror = (event: any) => {
      console.error('Speech recognition error', event);
      setIsRecording(false);
      setIsTypingFromVoice(false);
    };

    recognitionRef.current.onend = () => {
      setIsRecording(false);
    };
  }, []);

  // Typing animation for voice input
  const startTypingAnimation = (text: string) => {
    setIsTypingFromVoice(true);
    setCommand('');
    
    let currentIndex = 0;
    const typeChar = () => {
      if (currentIndex < text.length) {
        setCommand(text.substring(0, currentIndex + 1));
        currentIndex++;
        typingInterval.current = setTimeout(typeChar, 50); // 50ms per character
      } else {
        setIsTypingFromVoice(false);
        // Auto-execute after 3 seconds if no user interaction
        autoExecuteTimer.current = setTimeout(() => {
          processCommand(text);
          setCommand('');
        }, 3000);
      }
    };
    
    typeChar();
  };

  // Handle manual typing (should disable voice features)
  const handleManualTyping = (value: string) => {
    // Clear any ongoing voice typing
    if (typingInterval.current) {
      clearTimeout(typingInterval.current);
      setIsTypingFromVoice(false);
    }
    
    // Clear auto-execute timer
    if (autoExecuteTimer.current) {
      clearTimeout(autoExecuteTimer.current);
    }
    
    // Stop voice recording if user starts typing
    if (isRecording && recognitionRef.current) {
      recognitionRef.current.stop();
    }
    
    setCommand(value);
  };

  const toggleRecording = () => {
    if (!recognitionRef.current || isTypingFromVoice) return;

    if (isRecording) {
      recognitionRef.current.stop();
      setIsRecording(false);
    } else {
      // Clear any existing text before starting voice input
      setCommand('');
      recognitionRef.current.start();
      setIsRecording(true);
    }
  };

  const executeHistoryItem = (item: CommandHistoryItem) => {
    turtleControlService.executeHistoryItem(item);
    setShowHistory(false);
  };

  // Clean-up any running interval when component unmounts
  useEffect(() => {
    return () => {
      if (autoExecuteTimer.current) clearTimeout(autoExecuteTimer.current);
      if (typingInterval.current) clearTimeout(typingInterval.current);
    };
  }, []);

  return (
    <div className="bg-slate-800 rounded-lg p-6">
      <h2 className="text-xl font-bold mb-4">Command Input</h2>
      
      <form onSubmit={handleSubmit} className="space-y-4">
        <div className="flex flex-col space-y-2">
          <div className="flex items-center justify-between">
            <label htmlFor="command" className="text-sm text-slate-400">
              Enter a command for the robot
            </label>
            <button
              type="button"
              onClick={() => setShowHistory(!showHistory)}
              className="text-sm text-slate-400 hover:text-slate-200 flex items-center space-x-1"
            >
              <History className="h-4 w-4" />
              <span>History</span>
            </button>
          </div>
          <div className="flex">
            <input
              type="text"
              id="command"
              value={command}
              onChange={(e) => handleManualTyping(e.target.value)}
              placeholder={isRecording ? "Listening..." : "e.g., Spin in a circle with linear 1 angular 2"}
              className={`flex-1 p-2 bg-slate-900 border border-slate-700 rounded-l-md focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent ${
                isTypingFromVoice ? 'text-green-400' : ''
              }`}
              disabled={isRecording || isTypingFromVoice}
            />
            <button
              type="button"
              onClick={toggleRecording}
              disabled={isTypingFromVoice}
              className={`p-2 ${
                isRecording 
                  ? 'bg-red-600 hover:bg-red-700 animate-pulse' 
                  : isTypingFromVoice
                  ? 'bg-slate-600 cursor-not-allowed'
                  : 'bg-slate-700 hover:bg-slate-600'
              } rounded-none transition-colors`}
            >
              <Mic className="h-5 w-5" />
            </button>
            <button
              type="submit"
              disabled={isTypingFromVoice || !command.trim()}
              className={`p-2 ${
                isTypingFromVoice || !command.trim()
                  ? 'bg-slate-600 cursor-not-allowed'
                  : 'bg-blue-600 hover:bg-blue-700'
              } rounded-r-md transition-colors`}
            >
              <Send className="h-5 w-5" />
            </button>
          </div>
          
          {isTypingFromVoice && (
            <div className="text-sm text-green-400 animate-pulse">
              Voice input detected - auto-executing in 3 seconds...
            </div>
          )}
        </div>
        
        {showHistory && (
          <div className="bg-slate-900 rounded-md p-3 max-h-48 overflow-y-auto">
            <h4 className="text-sm font-medium text-slate-300 mb-2">Command History</h4>
            {commandHistory.length === 0 ? (
              <p className="text-xs text-slate-500">No commands yet</p>
            ) : (
              <div className="space-y-1">
                {commandHistory.slice(0, 10).map((item) => (
                  <div key={item.id} className="flex items-center justify-between text-xs">
                    <span className={`flex-1 ${item.result === 'error' ? 'text-red-400' : 'text-slate-300'}`}>
                      {item.command}
                    </span>
                    <div className="flex items-center space-x-2">
                      <span className="text-slate-500">
                        {item.timestamp.toLocaleTimeString()}
                      </span>
                      {item.parsedAction && (
                        <button
                          onClick={() => executeHistoryItem(item)}
                          className="text-blue-400 hover:text-blue-300"
                          title="Re-execute"
                        >
                          <Play className="h-3 w-3" />
                        </button>
                      )}
                    </div>
                  </div>
                ))}
              </div>
            )}
          </div>
        )}
        
        <div className="text-sm text-slate-400">
          <h3 className="font-medium mb-1">Example commands:</h3>
          <ul className="list-disc pl-5 space-y-1">
            <li>"Spin in a circle"</li>
            <li>"Start circular motion with linear 1.5 angular 2"</li>
            <li>"Stop"</li>
            <li>"Move slowly" or "Go fast"</li>
          </ul>
        </div>
      </form>
    </div>
  );
};