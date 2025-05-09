import React, { useState } from 'react';
import { Save, RefreshCw } from 'lucide-react';

export const SystemPrompt: React.FC = () => {
  const [prompt, setPrompt] = useState(
    `You are an advanced robot control system that integrates ROS2 with natural language processing capabilities. Your primary functions include:

1. Understanding and executing navigation commands
2. Interpreting sensor data and environmental feedback
3. Managing task sequences and error recovery
4. Maintaining context awareness across interactions
5. Ensuring safe operation within defined parameters

Parse user inputs into structured commands while maintaining awareness of:
- Current robot state and position
- Environmental constraints and obstacles
- Safety parameters and operational limits
- Previous commands and their outcomes
- System capabilities and limitations`
  );

  const handleSave = () => {
    console.log('Saving system prompt:', prompt);
  };

  const handleReset = () => {
    // Reset to default prompt
    setPrompt(`You are an advanced robot control system...`);
  };

  return (
    <div className="space-y-6">
      <div className="flex items-center justify-between">
        <h2 className="text-xl font-bold">System Prompt Configuration</h2>
        <div className="flex gap-2">
          <button
            onClick={handleReset}
            className="flex items-center gap-2 px-4 py-2 bg-slate-700 hover:bg-slate-600 rounded-md transition-colors"
          >
            <RefreshCw className="h-4 w-4" />
            Reset to Default
          </button>
          <button
            onClick={handleSave}
            className="flex items-center gap-2 px-4 py-2 bg-blue-600 hover:bg-blue-700 rounded-md transition-colors"
          >
            <Save className="h-4 w-4" />
            Save Changes
          </button>
        </div>
      </div>

      <div className="space-y-4">
        <p className="text-slate-400">
          Configure the base prompt that defines the system's behavior and capabilities.
          This prompt will be used as context for all command processing.
        </p>

        <textarea
          value={prompt}
          onChange={(e) => setPrompt(e.target.value)}
          className="w-full h-[400px] p-4 bg-slate-900 border border-slate-700 rounded-md text-slate-200 font-mono text-sm resize-none focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent"
          placeholder="Enter system prompt..."
        />

        <div className="flex justify-between text-sm text-slate-400">
          <span>Character count: {prompt.length}</span>
          <span>Recommended: 500-1000 characters</span>
        </div>
      </div>
    </div>
  );
};