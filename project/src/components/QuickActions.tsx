import React, { useState } from 'react';
import { Play, Square, RefreshCw, Trash2, AlertOctagon } from 'lucide-react';
import { rosService } from '../services/rosService';
import { backendService } from '../services/backendService';

export const QuickActions: React.FC = () => {
  const [loading, setLoading] = useState(false);

  const handleInitialize = async () => {
    setLoading(true);
    try {
      // Reconnect ROS if disconnected
      if (rosService.getConnectionStatus() !== 'connected') {
        rosService.connect();
      }

      // Check backend health
      const health = await backendService.checkHealth();
      if (health.status !== 'healthy') {
        throw new Error('Backend unhealthy');
      }

      // Clear session for fresh start
      backendService.clearCurrentSession();

      alert('✅ System initialized successfully');
    } catch (error: any) {
      alert(`❌ Initialization failed: ${error.message}`);
    } finally {
      setLoading(false);
    }
  };

  const handleReset = () => {
    if (confirm('⚠️ Reset will clear all history and reload the page. Continue?')) {
      // Clear conversation session
      backendService.clearCurrentSession();

      // Clear any localStorage data
      localStorage.clear();

      // Reload page
      window.location.reload();
    }
  };

  const handleClearErrors = () => {
    // Clear error states from localStorage
    const keysToRemove = Object.keys(localStorage).filter(key =>
      key.includes('error') || key.includes('Error')
    );

    keysToRemove.forEach(key => localStorage.removeItem(key));

    // Dispatch event for other components to clear errors
    window.dispatchEvent(new CustomEvent('clear-errors'));

    alert('✅ Errors cleared');
  };

  const handleEmergencyStop = async () => {
    setLoading(true);
    try {
      // Stop via ROS service
      rosService.emergencyStop();

      // Stop via backend (ensures both turtlesim and Nav2 stop)
      await backendService.stopRobot(true);

      alert('🛑 Emergency stop executed');
    } catch (error: any) {
      console.error('Emergency stop error:', error);
      // Still show success since rosService.emergencyStop() succeeds
      alert('🛑 Emergency stop executed (partial)');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="bg-slate-800/90 backdrop-blur-sm rounded-lg p-4 shadow-lg border border-slate-700">
      <h2 className="text-lg font-bold mb-3">Quick Actions</h2>

      <div className="grid grid-cols-1 sm:grid-cols-2 gap-2">
        <button
          onClick={handleInitialize}
          disabled={loading}
          className="flex items-center justify-center gap-2 p-2 bg-blue-600 hover:bg-blue-700 rounded-md transition-colors text-sm disabled:opacity-50 disabled:cursor-not-allowed"
        >
          <Play className="h-4 w-4" />
          <span>{loading ? 'Initializing...' : 'Initialize System'}</span>
        </button>

        <button
          onClick={handleReset}
          disabled={loading}
          className="flex items-center justify-center gap-2 p-2 bg-amber-600 hover:bg-amber-700 rounded-md transition-colors text-sm disabled:opacity-50 disabled:cursor-not-allowed"
        >
          <RefreshCw className="h-4 w-4" />
          <span>Reset System</span>
        </button>

        <button
          onClick={handleClearErrors}
          disabled={loading}
          className="flex items-center justify-center gap-2 p-2 bg-green-600 hover:bg-green-700 rounded-md transition-colors text-sm disabled:opacity-50 disabled:cursor-not-allowed"
        >
          <Trash2 className="h-4 w-4" />
          <span>Clear Errors</span>
        </button>

        <button
          onClick={handleEmergencyStop}
          disabled={loading}
          className="flex items-center justify-center gap-2 p-2 bg-red-600 hover:bg-red-700 rounded-md transition-colors text-sm disabled:opacity-50 disabled:cursor-not-allowed"
        >
          <AlertOctagon className="h-4 w-4" />
          <span>{loading ? 'Stopping...' : 'Emergency Stop'}</span>
        </button>
      </div>
    </div>
  );
};
