import React, { useState } from 'react';
import { Globe, Link, Save } from 'lucide-react';

export const Settings: React.FC = () => {
  const [openaiKey, setOpenaiKey] = useState('');
  const [apiEndpoint, setApiEndpoint] = useState('https://api.openai.com/v1');
  const [rosUrl, setRosUrl] = useState('ws://localhost:9090');
  const [rosNamespace, setRosNamespace] = useState('/robot1');
  
  const handleTestConnection = () => {
    console.log('Testing API connection...');
  };
  
  const handleSaveApiSettings = () => {
    console.log('Saving API settings...');
  };
  
  const handleConnectRos = () => {
    console.log('Connecting to ROS...');
  };
  
  const handleSaveRosSettings = () => {
    console.log('Saving ROS settings...');
  };
  
  return (
    <div className="space-y-6">
      <h1 className="text-2xl font-bold">Settings</h1>
      
      <div className="bg-slate-800 rounded-lg p-6 space-y-6">
        <h2 className="text-xl font-bold">API Settings</h2>
        
        <div>
          <label className="block text-sm font-medium text-slate-400 mb-2">
            OpenAI API Key
          </label>
          <input
            type="password"
            value={openaiKey}
            onChange={(e) => setOpenaiKey(e.target.value)}
            placeholder="sk-..."
            className="w-full p-2 bg-slate-900 border border-slate-700 rounded-md text-white focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent"
          />
        </div>
        
        <div>
          <label className="block text-sm font-medium text-slate-400 mb-2">
            API Endpoint
          </label>
          <input
            type="text"
            value={apiEndpoint}
            onChange={(e) => setApiEndpoint(e.target.value)}
            className="w-full p-2 bg-slate-900 border border-slate-700 rounded-md text-white focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent"
          />
        </div>
        
        <div className="flex gap-3">
          <button
            onClick={handleTestConnection}
            className="flex items-center gap-2 px-4 py-2 bg-blue-600 hover:bg-blue-700 rounded-md transition-colors"
          >
            <Globe className="h-4 w-4" />
            Test Connection
          </button>
          <button
            onClick={handleSaveApiSettings}
            className="flex items-center gap-2 px-4 py-2 bg-green-600 hover:bg-green-700 rounded-md transition-colors"
          >
            <Save className="h-4 w-4" />
            Save API Settings
          </button>
        </div>
      </div>
      
      <div className="bg-slate-800 rounded-lg p-6 space-y-6">
        <h2 className="text-xl font-bold">ROS2 Settings</h2>
        
        <div>
          <label className="block text-sm font-medium text-slate-400 mb-2">
            ROS Bridge WebSocket URL
          </label>
          <input
            type="text"
            value={rosUrl}
            onChange={(e) => setRosUrl(e.target.value)}
            className="w-full p-2 bg-slate-900 border border-slate-700 rounded-md text-white focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent"
          />
        </div>
        
        <div>
          <label className="block text-sm font-medium text-slate-400 mb-2">
            ROS Namespace
          </label>
          <input
            type="text"
            value={rosNamespace}
            onChange={(e) => setRosNamespace(e.target.value)}
            className="w-full p-2 bg-slate-900 border border-slate-700 rounded-md text-white focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent"
          />
        </div>
        
        <div className="flex gap-3">
          <button
            onClick={handleConnectRos}
            className="flex items-center gap-2 px-4 py-2 bg-blue-600 hover:bg-blue-700 rounded-md transition-colors"
          >
            <Link className="h-4 w-4" />
            Connect to ROS
          </button>
          <button
            onClick={handleSaveRosSettings}
            className="flex items-center gap-2 px-4 py-2 bg-green-600 hover:bg-green-700 rounded-md transition-colors"
          >
            <Save className="h-4 w-4" />
            Save ROS Settings
          </button>
        </div>
      </div>
    </div>
  );
};