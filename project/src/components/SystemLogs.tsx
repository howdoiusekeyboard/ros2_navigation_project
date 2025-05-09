import React, { useState } from 'react';
import { AlertCircle, Bug, Code, Info } from 'lucide-react';

export const SystemLogs: React.FC = () => {
  const [logLevel, setLogLevel] = useState<'all' | 'info' | 'warning' | 'error'>('all');
  
  const logs = [
    {
      id: 1,
      level: 'info',
      timestamp: "2025-06-18T14:32:40",
      source: "gpt4_command_parser",
      message: "Received command: 'Go to the table, wait for 5 seconds, then return to base'"
    },
    {
      id: 2,
      level: 'info',
      timestamp: "2025-06-18T14:32:41",
      source: "context_memory_node",
      message: "Retrieved 3 relevant context items for command processing"
    },
    {
      id: 3,
      level: 'info',
      timestamp: "2025-06-18T14:32:45",
      source: "robot_command_executor",
      message: "Executing navigation command: target='table'"
    },
    {
      id: 4,
      level: 'warning',
      timestamp: "2025-06-18T14:32:50",
      source: "robot_command_executor",
      message: "Path planning challenged: 2 dynamic obstacles detected"
    },
    {
      id: 5,
      level: 'error',
      timestamp: "2025-06-18T14:55:22",
      source: "perception_node",
      message: "Object detection failed: 'red ball' not in detection database"
    }
  ];
  
  const filteredLogs = logLevel === 'all' 
    ? logs 
    : logs.filter(log => log.level === logLevel);
  
  return (
    <div className="bg-slate-800 rounded-lg p-6 flex-1">
      <div className="flex items-center justify-between mb-4">
        <h2 className="text-xl font-bold">System Logs</h2>
        
        <div className="flex space-x-2">
          <button 
            onClick={() => setLogLevel('all')}
            className={`px-2 py-1 text-xs rounded-md ${
              logLevel === 'all' 
                ? 'bg-slate-600 text-white' 
                : 'bg-slate-700 text-slate-300 hover:bg-slate-600'
            }`}
          >
            All
          </button>
          <button 
            onClick={() => setLogLevel('info')}
            className={`px-2 py-1 text-xs rounded-md ${
              logLevel === 'info' 
                ? 'bg-blue-600 text-white' 
                : 'bg-slate-700 text-slate-300 hover:bg-slate-600'
            }`}
          >
            Info
          </button>
          <button 
            onClick={() => setLogLevel('warning')}
            className={`px-2 py-1 text-xs rounded-md ${
              logLevel === 'warning' 
                ? 'bg-amber-600 text-white' 
                : 'bg-slate-700 text-slate-300 hover:bg-slate-600'
            }`}
          >
            Warning
          </button>
          <button 
            onClick={() => setLogLevel('error')}
            className={`px-2 py-1 text-xs rounded-md ${
              logLevel === 'error' 
                ? 'bg-red-600 text-white' 
                : 'bg-slate-700 text-slate-300 hover:bg-slate-600'
            }`}
          >
            Error
          </button>
        </div>
      </div>
      
      <div className="overflow-y-auto max-h-60 bg-slate-900 rounded-md p-3">
        {filteredLogs.length > 0 ? (
          <div className="space-y-2">
            {filteredLogs.map((log) => (
              <div key={log.id} className="flex items-start space-x-2">
                <div className="mt-0.5">
                  {log.level === 'info' && <Info className="h-4 w-4 text-blue-400" />}
                  {log.level === 'warning' && <AlertCircle className="h-4 w-4 text-amber-400" />}
                  {log.level === 'error' && <Bug className="h-4 w-4 text-red-400" />}
                </div>
                <div>
                  <div className="flex items-center space-x-2">
                    <span className="text-xs text-slate-400">
                      {new Date(log.timestamp).toLocaleTimeString()}
                    </span>
                    <span className="text-xs px-1.5 py-0.5 rounded-sm bg-slate-700 text-slate-300">
                      {log.source}
                    </span>
                  </div>
                  <p className="text-sm mt-0.5 text-slate-300">{log.message}</p>
                </div>
              </div>
            ))}
          </div>
        ) : (
          <p className="text-sm text-slate-500 text-center py-4">No logs match the selected filter</p>
        )}
      </div>
    </div>
  );
};