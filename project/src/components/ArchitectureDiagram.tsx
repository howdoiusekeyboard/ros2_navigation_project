import React from 'react';

export const ArchitectureDiagram: React.FC = () => {
  return (
    <div className="bg-slate-800 rounded-lg p-6">
      <h2 className="text-xl font-bold mb-6">System Architecture</h2>
      
      <div className="overflow-x-auto">
        <div className="min-w-[800px]">
          {/* Architecture diagram using divs and styling */}
          <div className="relative bg-slate-900 p-8 rounded-lg">
            {/* Layers */}
            <div className="flex flex-col gap-6">
              {/* User Interface Layer */}
              <div className="w-full">
                <h3 className="text-center text-sm font-medium text-slate-400 mb-2">User Interface</h3>
                <div className="grid grid-cols-3 gap-4">
                  <div className="bg-blue-900/30 border border-blue-700/50 rounded-md p-3 text-center">
                    <p className="text-sm font-medium text-blue-400">Command Input</p>
                    <p className="text-xs text-slate-400 mt-1">Voice & Text UI</p>
                  </div>
                  <div className="bg-blue-900/30 border border-blue-700/50 rounded-md p-3 text-center">
                    <p className="text-sm font-medium text-blue-400">Status Dashboard</p>
                    <p className="text-xs text-slate-400 mt-1">Robot State Visualization</p>
                  </div>
                  <div className="bg-blue-900/30 border border-blue-700/50 rounded-md p-3 text-center">
                    <p className="text-sm font-medium text-blue-400">Memory Viewer</p>
                    <p className="text-xs text-slate-400 mt-1">Context History</p>
                  </div>
                </div>
              </div>
              
              {/* Processing Layer */}
              <div className="w-full">
                <h3 className="text-center text-sm font-medium text-slate-400 mb-2">Processing Layer</h3>
                <div className="grid grid-cols-4 gap-4">
                  <div className="bg-purple-900/30 border border-purple-700/50 rounded-md p-3 text-center">
                    <p className="text-sm font-medium text-purple-400">Whisper API</p>
                    <p className="text-xs text-slate-400 mt-1">Speech-to-Text</p>
                  </div>
                  <div className="bg-purple-900/30 border border-purple-700/50 rounded-md p-3 text-center">
                    <p className="text-sm font-medium text-purple-400">Supermemory.ai</p>
                    <p className="text-xs text-slate-400 mt-1">Context Management</p>
                  </div>
                  <div className="bg-purple-900/30 border border-purple-700/50 rounded-md p-3 text-center">
                    <p className="text-sm font-medium text-purple-400">GPT-4 API</p>
                    <p className="text-xs text-slate-400 mt-1">Command Parsing</p>
                  </div>
                  <div className="bg-purple-900/30 border border-purple-700/50 rounded-md p-3 text-center">
                    <p className="text-sm font-medium text-purple-400">Command Validator</p>
                    <p className="text-xs text-slate-400 mt-1">Safety Checks</p>
                  </div>
                </div>
              </div>
              
              {/* ROS2 Layer */}
              <div className="w-full">
                <h3 className="text-center text-sm font-medium text-slate-400 mb-2">ROS2 Nodes</h3>
                <div className="grid grid-cols-5 gap-4">
                  <div className="bg-green-900/30 border border-green-700/50 rounded-md p-3 text-center">
                    <p className="text-sm font-medium text-green-400">Command Executor</p>
                    <p className="text-xs text-slate-400 mt-1">Processes Actions</p>
                  </div>
                  <div className="bg-green-900/30 border border-green-700/50 rounded-md p-3 text-center">
                    <p className="text-sm font-medium text-green-400">Navigation</p>
                    <p className="text-xs text-slate-400 mt-1">Nav2 Stack</p>
                  </div>
                  <div className="bg-green-900/30 border border-green-700/50 rounded-md p-3 text-center">
                    <p className="text-sm font-medium text-green-400">Perception</p>
                    <p className="text-xs text-slate-400 mt-1">Object Detection</p>
                  </div>
                  <div className="bg-green-900/30 border border-green-700/50 rounded-md p-3 text-center">
                    <p className="text-sm font-medium text-green-400">Behavior FSM</p>
                    <p className="text-xs text-slate-400 mt-1">Task Management</p>
                  </div>
                  <div className="bg-green-900/30 border border-green-700/50 rounded-md p-3 text-center">
                    <p className="text-sm font-medium text-green-400">Status Monitor</p>
                    <p className="text-xs text-slate-400 mt-1">Diagnostics</p>
                  </div>
                </div>
              </div>
              
              {/* Hardware Layer */}
              <div className="w-full">
                <h3 className="text-center text-sm font-medium text-slate-400 mb-2">Hardware / Simulation</h3>
                <div className="grid grid-cols-4 gap-4">
                  <div className="bg-red-900/30 border border-red-700/50 rounded-md p-3 text-center">
                    <p className="text-sm font-medium text-red-400">Motors & Drivers</p>
                    <p className="text-xs text-slate-400 mt-1">Physical Movement</p>
                  </div>
                  <div className="bg-red-900/30 border border-red-700/50 rounded-md p-3 text-center">
                    <p className="text-sm font-medium text-red-400">Sensors</p>
                    <p className="text-xs text-slate-400 mt-1">LIDAR, Camera, IMU</p>
                  </div>
                  <div className="bg-red-900/30 border border-red-700/50 rounded-md p-3 text-center">
                    <p className="text-sm font-medium text-red-400">Gazebo Simulation</p>
                    <p className="text-xs text-slate-400 mt-1">Virtual Testing</p>
                  </div>
                  <div className="bg-red-900/30 border border-red-700/50 rounded-md p-3 text-center">
                    <p className="text-sm font-medium text-red-400">Microcontrollers</p>
                    <p className="text-xs text-slate-400 mt-1">Low-level Control</p>
                  </div>
                </div>
              </div>
            </div>
            
            {/* Connection lines */}
            <div className="absolute top-0 left-0 w-full h-full pointer-events-none">
              {/* Add connection SVG or divs here */}
            </div>
          </div>
        </div>
      </div>
      
      <div className="mt-8 space-y-6">
        <div>
          <h3 className="text-lg font-semibold mb-2">Data Flow</h3>
          <ol className="list-decimal pl-5 space-y-2 text-slate-300">
            <li>User speaks or types a command (e.g., "Go to the table, wait for 5 seconds, then return to base")</li>
            <li>Whisper API converts speech to text (if voice input was used)</li>
            <li>Command is stored in supermemory.ai and relevant context is retrieved</li>
            <li>GPT-4 parses the command with context into structured JSON format</li>
            <li>Command Validator checks for safety constraints and feasibility</li>
            <li>Parsed command is sent to the ROS2 Command Executor node</li>
            <li>For compound commands, the Behavior FSM manages the sequence execution</li>
            <li>Each sub-command (navigation, wait, etc.) is executed by the appropriate ROS2 nodes</li>
            <li>Status Monitor provides real-time feedback to the UI</li>
            <li>Command completion or errors are logged and stored in context memory</li>
          </ol>
        </div>
        
        <div>
          <h3 className="text-lg font-semibold mb-2">Key Architectural Features</h3>
          <ul className="list-disc pl-5 space-y-2 text-slate-300">
            <li><span className="font-medium text-blue-400">Modular Design:</span> Each component is a separate ROS2 node, allowing for independent development and testing</li>
            <li><span className="font-medium text-blue-400">Context Preservation:</span> supermemory.ai maintains conversation history, entity references, and previous commands</li>
            <li><span className="font-medium text-blue-400">Fallback Mechanisms:</span> Failsafe behaviors are defined for ambiguous or failed commands</li>
            <li><span className="font-medium text-blue-400">Extensibility:</span> New command types can be added without changing the core architecture</li>
            <li><span className="font-medium text-blue-400">Simulation Support:</span> All components work identically with physical robots or Gazebo simulation</li>
          </ul>
        </div>
      </div>
    </div>
  );
};