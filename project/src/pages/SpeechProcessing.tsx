import React, { useState } from 'react';
import { Mic } from 'lucide-react';

export const SpeechProcessing: React.FC = () => {
  const [model, setModel] = useState('Whisper-1');
  const [threshold, setThreshold] = useState(60);
  const [language, setLanguage] = useState('English');
  
  const handleTestMicrophone = () => {
    // Implement microphone testing logic
    console.log('Testing microphone...');
  };
  
  const handleSaveSettings = () => {
    // Implement settings save logic
    console.log('Saving speech processing settings...');
  };
  
  return (
    <div className="space-y-6">
      <h1 className="text-2xl font-bold">Speech Processing</h1>
      
      <div className="bg-slate-800 rounded-lg p-6 space-y-6">
        <div>
          <label className="block text-sm font-medium text-slate-400 mb-2">
            Whisper API Model
          </label>
          <select
            value={model}
            onChange={(e) => setModel(e.target.value)}
            className="w-full p-2 bg-slate-900 border border-slate-700 rounded-md text-white focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent"
          >
            <option value="Whisper-1">Whisper-1</option>
          </select>
        </div>
        
        <div>
          <label className="block text-sm font-medium text-slate-400 mb-2">
            Voice Activity Detection Threshold
          </label>
          <div className="space-y-2">
            <input
              type="range"
              min="0"
              max="100"
              value={threshold}
              onChange={(e) => setThreshold(parseInt(e.target.value))}
              className="w-full"
            />
            <div className="text-sm text-slate-400">{threshold}%</div>
          </div>
        </div>
        
        <div>
          <label className="block text-sm font-medium text-slate-400 mb-2">
            Default Language
          </label>
          <select
            value={language}
            onChange={(e) => setLanguage(e.target.value)}
            className="w-full p-2 bg-slate-900 border border-slate-700 rounded-md text-white focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent"
          >
            <option value="English">English</option>
            <option value="Spanish">Spanish</option>
            <option value="French">French</option>
            <option value="German">German</option>
            <option value="Japanese">Japanese</option>
          </select>
        </div>
        
        <div className="flex gap-3">
          <button
            onClick={handleTestMicrophone}
            className="flex items-center gap-2 px-4 py-2 bg-blue-600 hover:bg-blue-700 rounded-md transition-colors"
          >
            <Mic className="h-4 w-4" />
            Test Microphone
          </button>
          <button
            onClick={handleSaveSettings}
            className="px-4 py-2 bg-green-600 hover:bg-green-700 rounded-md transition-colors"
          >
            Save Settings
          </button>
        </div>
      </div>
    </div>
  );
};