import React, { useState } from 'react';
import { Play, Save } from 'lucide-react';

export const CommandControl: React.FC = () => {
  const [model, setModel] = useState('GPT-4');
  const [temperature, setTemperature] = useState(0.7);
  const [systemPrompt, setSystemPrompt] = useState(
    'You are a robot assistant capable of understanding and executing navigation and interaction commands. Parse the user\'s input into a structured JSON format with command type, parameters, and any conditions.'
  );
  
  const handleTestParser = () => {
    console.log('Testing command parser...');
  };
  
  const handleSaveSettings = () => {
    console.log('Saving command control settings...');
  };
  
  return (
    <div className="space-y-6">
      <h1 className="text-2xl font-bold">Command Parser Settings</h1>
      
      <div className="bg-slate-800 rounded-lg p-6 space-y-6">
        <div>
          <label className="block text-sm font-medium text-slate-400 mb-2">
            GPT Model
          </label>
          <select
            value={model}
            onChange={(e) => setModel(e.target.value)}
            className="w-full p-2 bg-slate-900 border border-slate-700 rounded-md text-white focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent"
          >
            <option value="GPT-4">GPT-4</option>
            <option value="GPT-3.5-Turbo">GPT-3.5-Turbo</option>
          </select>
        </div>
        
        <div>
          <label className="block text-sm font-medium text-slate-400 mb-2">
            Temperature
          </label>
          <div className="space-y-2">
            <input
              type="range"
              min="0"
              max="1"
              step="0.1"
              value={temperature}
              onChange={(e) => setTemperature(parseFloat(e.target.value))}
              className="w-full"
            />
            <div className="text-sm text-slate-400">{temperature}</div>
          </div>
        </div>
        
        <div>
          <label className="block text-sm font-medium text-slate-400 mb-2">
            System Prompt
          </label>
          <textarea
            value={systemPrompt}
            onChange={(e) => setSystemPrompt(e.target.value)}
            rows={4}
            className="w-full p-2 bg-slate-900 border border-slate-700 rounded-md text-white focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent"
          />
        </div>
        
        <div className="flex gap-3">
          <button
            onClick={handleTestParser}
            className="flex items-center gap-2 px-4 py-2 bg-blue-600 hover:bg-blue-700 rounded-md transition-colors"
          >
            <Play className="h-4 w-4" />
            Test Parser
          </button>
          <button
            onClick={handleSaveSettings}
            className="flex items-center gap-2 px-4 py-2 bg-green-600 hover:bg-green-700 rounded-md transition-colors"
          >
            <Save className="h-4 w-4" />
            Save Settings
          </button>
        </div>
      </div>
    </div>
  );
};