import React, { useState, useEffect } from 'react';
import { CommandInput } from '../components/CommandInput';
import { RobotStatus } from '../components/RobotStatus';
import { TaskHistory } from '../components/TaskHistory';
import { ArchitectureView } from '../components/ArchitectureView';
import { SystemPrompt } from '../components/SystemPrompt';
import { Layers, Terminal, MessageSquare } from 'lucide-react';
import { CircularMotionControl } from '../components/CircularMotionControl';
import rosService from '../services/RosService';

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
            <CommandInput />
            <TaskHistory />
          </div>
        );
    }
  };

  return (
    <div className="max-w-5xl mx-auto p-6">
      <div className="mb-6 flex space-x-4">
        <button
          className={`px-4 py-2 flex items-center space-x-2 rounded-md ${
            activePanel === 'default' ? 'bg-blue-500 text-white' : 'bg-gray-200'
          }`}
          onClick={() => setActivePanel('default')}
        >
          <Terminal size={18} />
          <span>Dashboard</span>
        </button>
        <button
          className={`px-4 py-2 flex items-center space-x-2 rounded-md ${
            activePanel === 'architecture' ? 'bg-blue-500 text-white' : 'bg-gray-200'
          }`}
          onClick={() => setActivePanel('architecture')}
        >
          <Layers size={18} />
          <span>Architecture</span>
        </button>
        <button
          className={`px-4 py-2 flex items-center space-x-2 rounded-md ${
            activePanel === 'prompt' ? 'bg-blue-500 text-white' : 'bg-gray-200'
          }`}
          onClick={() => setActivePanel('prompt')}
        >
          <MessageSquare size={18} />
          <span>System Prompt</span>
        </button>
      </div>

      {renderPanel()}

      {/* ROS Robot Control Section */}
      <div className="mt-8 border-t pt-6">
        <h2 className="text-2xl font-bold mb-6">ROS Robot Control</h2>
        
        <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
          <CircularMotionControl title="Turtle Circular Motion" />
          
          <div className="p-4 bg-white rounded-lg shadow-md">
            <h2 className="text-xl font-bold mb-4">ROS Connection Status</h2>
            <div className="flex items-center space-x-2">
              <div 
                className={`h-4 w-4 rounded-full ${connected ? 'bg-green-500' : 'bg-red-500'}`} 
              />
              <span>{connected ? 'Connected to ROS Bridge' : 'Disconnected from ROS Bridge'}</span>
            </div>
            <div className="mt-4">
              <p className="text-sm text-gray-600">
                To use the control features, make sure you have started the ROS Bridge server with:
              </p>
              <div className="mt-2 bg-gray-100 p-2 rounded text-sm font-mono">
                ros2 launch rosbridge_server rosbridge_websocket_launch.xml
              </div>
            </div>
          </div>
        </div>
        
        <div className="mt-6 p-4 bg-white rounded-lg shadow-md">
          <h2 className="text-xl font-bold mb-4">Getting Started</h2>
          <ol className="list-decimal list-inside space-y-2">
            <li>Start the rosbridge_server in a terminal:
              <pre className="ml-6 mt-1 bg-gray-100 p-2 rounded text-sm font-mono">
                ros2 launch rosbridge_server rosbridge_websocket_launch.xml
              </pre>
            </li>
            <li>Start the turtlesim node:
              <pre className="ml-6 mt-1 bg-gray-100 p-2 rounded text-sm font-mono">
                ros2 run turtlesim turtlesim_node
              </pre>
            </li>
            <li>Use the controls above to move the turtle in a circular motion</li>
          </ol>
        </div>
      </div>
    </div>
  );
};