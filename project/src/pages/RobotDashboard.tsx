import React, { useState, useEffect } from 'react';
import { CommandInput } from '../components/CommandInput';
import { RobotStatus } from '../components/RobotStatus';
import { TaskHistory } from '../components/TaskHistory';
import { ArchitectureView } from '../components/ArchitectureView';
import { SystemPrompt } from '../components/SystemPrompt';
import { Layers, Terminal, MessageSquare } from 'lucide-react';
import { CircularMotionControl } from '../components/CircularMotionControl';
import rosService from '../services/RosService';
import { CmdVelConsole } from '../components/CmdVelConsole';

export const RobotDashboard: React.FC = () => {
  const [activePanel, setActivePanel] = useState<'default' | 'architecture' | 'prompt'>('default');
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const checkConnection = async () => {
      try {
        const isConnected = await rosService.connect();
        setConnected(isConnected);
      } catch (error) {
        console.error('Failed to connect to ROS:', error);
        setConnected(false);
      }
    };

    checkConnection();

    return () => {
      // Cleanup on component unmount
      if (connected) {
        rosService.disconnect();
      }
    };
  }, []);

  const renderPanel = () => {
    switch (activePanel) {
      case 'architecture':
        return <ArchitectureView />;
      case 'prompt':
        return <SystemPrompt />;
      default:
        return (
          <div className="space-y-6">
            <RobotStatus />
            <TaskHistory />
          </div>
        );
    }
  };

  return (
    <div className="max-w-5xl mx-auto p-6">
      <div className="mb-6 flex space-x-4">
        <button
          className={`px-4 py-2 flex items-center space-x-2 rounded-md transition-colors ${
            activePanel === 'default' ? 'bg-blue-600 text-white' : 'bg-slate-700 text-slate-200 hover:bg-slate-600'
          }`}
          onClick={() => setActivePanel('default')}
        >
          <Terminal size={18} />
          <span>Dashboard</span>
        </button>
        <button
          className={`px-4 py-2 flex items-center space-x-2 rounded-md transition-colors ${
            activePanel === 'architecture' ? 'bg-blue-600 text-white' : 'bg-slate-700 text-slate-200 hover:bg-slate-600'
          }`}
          onClick={() => setActivePanel('architecture')}
        >
          <Layers size={18} />
          <span>Architecture</span>
        </button>
        <button
          className={`px-4 py-2 flex items-center space-x-2 rounded-md transition-colors ${
            activePanel === 'prompt' ? 'bg-blue-600 text-white' : 'bg-slate-700 text-slate-200 hover:bg-slate-600'
          }`}
          onClick={() => setActivePanel('prompt')}
        >
          <MessageSquare size={18} />
          <span>System Prompt</span>
        </button>
      </div>

      {renderPanel()}
    </div>
  );
};