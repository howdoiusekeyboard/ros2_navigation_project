import React, { useState } from 'react';
import { CommandInput } from '../components/CommandInput';
import { RobotStatus } from '../components/RobotStatus';
import { CommandHistory } from '../components/CommandHistory';
import { SystemLogs } from '../components/SystemLogs';
import { SystemStatus } from '../components/SystemStatus';
import { QuickActions } from '../components/QuickActions';
import { ArchitectureDiagram } from '../components/ArchitectureDiagram';
import { SystemPrompt } from '../components/SystemPrompt';
import { Layers, Terminal, MessageSquare } from 'lucide-react';

export const RobotDashboard: React.FC = () => {
  const [activePanel, setActivePanel] = useState<'default' | 'architecture' | 'prompt'>('default');

  const renderPanel = () => {
    switch (activePanel) {
      case 'architecture':
        return <ArchitectureDiagram />;
      case 'prompt':
        return <SystemPrompt />;
      default:
        return (
          <>
            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              <CommandInput />
              <RobotStatus />
            </div>
            <CommandHistory />
            <div className="flex flex-col lg:flex-row gap-6">
              <SystemLogs />
              <SystemStatus />
            </div>
          </>
        );
    }
  };

  return (
    <div className="space-y-6">
      <div className="flex flex-col md:flex-row justify-between items-start md:items-center gap-4">
        <div>
          <h1 className="text-2xl font-bold">Robot Control System</h1>
          <p className="text-slate-400">ROS2 + GPT-4 Integration</p>
        </div>
        
        <div className="flex gap-2">
          <button
            onClick={() => setActivePanel('default')}
            className={`px-4 py-2 rounded-md flex items-center gap-2 ${
              activePanel === 'default' 
                ? 'bg-blue-600 text-white' 
                : 'bg-slate-700 hover:bg-slate-600 text-slate-200'
            }`}
          >
            <Terminal className="h-4 w-4" />
            <span>Dashboard</span>
          </button>
          <button
            onClick={() => setActivePanel('architecture')}
            className={`px-4 py-2 rounded-md flex items-center gap-2 ${
              activePanel === 'architecture' 
                ? 'bg-blue-600 text-white' 
                : 'bg-slate-700 hover:bg-slate-600 text-slate-200'
            }`}
          >
            <Layers className="h-4 w-4" />
            <span>Architecture</span>
          </button>
          <button
            onClick={() => setActivePanel('prompt')}
            className={`px-4 py-2 rounded-md flex items-center gap-2 ${
              activePanel === 'prompt' 
                ? 'bg-blue-600 text-white' 
                : 'bg-slate-700 hover:bg-slate-600 text-slate-200'
            }`}
          >
            <MessageSquare className="h-4 w-4" />
            <span>System Prompt</span>
          </button>
        </div>
      </div>

      <div className="bg-slate-800 rounded-lg p-6">
        {renderPanel()}
      </div>

      <div className="fixed bottom-6 left-6 z-10 w-auto">
        <QuickActions />
      </div>
    </div>
  );
};