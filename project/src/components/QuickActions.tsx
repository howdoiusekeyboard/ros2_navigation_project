import React from 'react';
import { Play, Square, RefreshCw, Trash2, AlertOctagon } from 'lucide-react';

export const QuickActions: React.FC = () => {
  const handleAction = (action: string) => {
    console.log(`Executing action: ${action}`);
  };

  return (
    <div className="bg-slate-800/90 backdrop-blur-sm rounded-lg p-4 shadow-lg border border-slate-700">
      <h2 className="text-lg font-bold mb-3">Quick Actions</h2>
      
      <div className="grid grid-cols-1 sm:grid-cols-2 gap-2">
        <button
          onClick={() => handleAction('initialize')}
          className="flex items-center justify-center gap-2 p-2 bg-blue-600 hover:bg-blue-700 rounded-md transition-colors text-sm"
        >
          <Play className="h-4 w-4" />
          <span>Initialize System</span>
        </button>
        
        <button
          onClick={() => handleAction('reset')}
          className="flex items-center justify-center gap-2 p-2 bg-amber-600 hover:bg-amber-700 rounded-md transition-colors text-sm"
        >
          <RefreshCw className="h-4 w-4" />
          <span>Reset System</span>
        </button>
        
        <button
          onClick={() => handleAction('clearErrors')}
          className="flex items-center justify-center gap-2 p-2 bg-green-600 hover:bg-green-700 rounded-md transition-colors text-sm"
        >
          <Trash2 className="h-4 w-4" />
          <span>Clear Errors</span>
        </button>
        
        <button
          onClick={() => handleAction('emergency')}
          className="flex items-center justify-center gap-2 p-2 bg-red-600 hover:bg-red-700 rounded-md transition-colors text-sm"
        >
          <AlertOctagon className="h-4 w-4" />
          <span>Emergency Stop</span>
        </button>
      </div>
    </div>
  );
};