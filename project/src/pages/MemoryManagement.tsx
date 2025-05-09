import React, { useState } from 'react';
import { Database, Download, Trash2 } from 'lucide-react';

export const MemoryManagement: React.FC = () => {
  const [memoryType, setMemoryType] = useState('SQLite');
  const [contextLength, setContextLength] = useState(10);
  const [retention, setRetention] = useState(30);
  
  const handleViewDatabase = () => {
    console.log('Opening memory database viewer...');
  };
  
  const handleExportMemory = () => {
    console.log('Exporting memory data...');
  };
  
  const handleClearMemory = () => {
    console.log('Clearing memory...');
  };
  
  return (
    <div className="space-y-6">
      <h1 className="text-2xl font-bold">Memory Management</h1>
      
      <div className="bg-slate-800 rounded-lg p-6 space-y-6">
        <div>
          <label className="block text-sm font-medium text-slate-400 mb-2">
            Memory Type
          </label>
          <select
            value={memoryType}
            onChange={(e) => setMemoryType(e.target.value)}
            className="w-full p-2 bg-slate-900 border border-slate-700 rounded-md text-white focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent"
          >
            <option value="SQLite">SQLite</option>
            <option value="Redis">Redis</option>
            <option value="PostgreSQL">PostgreSQL</option>
          </select>
        </div>
        
        <div>
          <label className="block text-sm font-medium text-slate-400 mb-2">
            Context Length
          </label>
          <input
            type="number"
            value={contextLength}
            onChange={(e) => setContextLength(parseInt(e.target.value))}
            className="w-full p-2 bg-slate-900 border border-slate-700 rounded-md text-white focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent"
          />
        </div>
        
        <div>
          <label className="block text-sm font-medium text-slate-400 mb-2">
            Memory Retention (days)
          </label>
          <input
            type="number"
            value={retention}
            onChange={(e) => setRetention(parseInt(e.target.value))}
            className="w-full p-2 bg-slate-900 border border-slate-700 rounded-md text-white focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent"
          />
        </div>
        
        <div className="flex gap-3">
          <button
            onClick={handleViewDatabase}
            className="flex items-center gap-2 px-4 py-2 bg-blue-600 hover:bg-blue-700 rounded-md transition-colors"
          >
            <Database className="h-4 w-4" />
            View Memory Database
          </button>
          <button
            onClick={handleExportMemory}
            className="flex items-center gap-2 px-4 py-2 bg-green-600 hover:bg-green-700 rounded-md transition-colors"
          >
            <Download className="h-4 w-4" />
            Export Memory
          </button>
          <button
            onClick={handleClearMemory}
            className="flex items-center gap-2 px-4 py-2 bg-red-600 hover:bg-red-700 rounded-md transition-colors"
          >
            <Trash2 className="h-4 w-4" />
            Clear Memory
          </button>
        </div>
      </div>
    </div>
  );
};