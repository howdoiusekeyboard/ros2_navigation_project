import React from 'react';
import { CheckCircle2, Clock, XCircle } from 'lucide-react';

export const CommandHistory: React.FC = () => {
  // This data would come from context memory in a real implementation
  const commandHistory = [
    {
      id: 1,
      command: "Go to the table, wait for 5 seconds, then return to base",
      timestamp: "2025-06-18T14:32:45",
      status: "completed",
      duration: "00:47"
    },
    {
      id: 2,
      command: "Find the red ball and bring it to me",
      timestamp: "2025-06-18T14:55:12",
      status: "failed",
      error: "Object 'red ball' not found in environment",
      duration: "00:23"
    },
    {
      id: 3,
      command: "Navigate to kitchen and wait there",
      timestamp: "2025-06-18T15:10:33",
      status: "in_progress",
      progress: 75
    }
  ];
  
  return (
    <div className="bg-slate-800 rounded-lg p-6">
      <h2 className="text-xl font-bold mb-4">Command History</h2>
      
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
            {commandHistory.map((item) => (
              <tr key={item.id} className="border-b border-slate-700">
                <td className="py-3 pr-6 font-mono">
                  {item.command}
                </td>
                <td className="py-3 pr-6 text-slate-400 whitespace-nowrap">
                  {new Date(item.timestamp).toLocaleTimeString()}
                </td>
                <td className="py-3 pr-6">
                  {item.status === "completed" && (
                    <div className="flex items-center text-green-400">
                      <CheckCircle2 className="h-4 w-4 mr-1" />
                      <span>Completed</span>
                    </div>
                  )}
                  {item.status === "failed" && (
                    <div className="flex items-center text-red-400">
                      <XCircle className="h-4 w-4 mr-1" />
                      <span>Failed</span>
                    </div>
                  )}
                  {item.status === "in_progress" && (
                    <div className="flex items-center text-amber-400">
                      <Clock className="h-4 w-4 mr-1" />
                      <span>In Progress ({item.progress}%)</span>
                    </div>
                  )}
                </td>
                <td className="py-3 pr-6 text-slate-400">
                  {item.duration || "-"}
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  );
};