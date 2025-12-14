import React, { useState, useEffect } from 'react';
import { Circle, RefreshCw } from 'lucide-react';
import { rosService } from '../services/rosService';
import { backendService } from '../services/backendService';

interface ServiceStatus {
  name: string;
  status: 'active' | 'inactive' | 'error';
  lastCheck?: Date;
  message?: string;
}

export const SystemStatus: React.FC = () => {
  const [services, setServices] = useState<ServiceStatus[]>([]);
  const [checking, setChecking] = useState(false);

  const checkAllServices = async () => {
    setChecking(true);
    const newServices: ServiceStatus[] = [];

    // 1. ROS Bridge
    const rosStatus = rosService.getConnectionStatus();
    newServices.push({
      name: 'ROS Bridge',
      status: rosStatus === 'connected' ? 'active' : rosStatus === 'error' ? 'error' : 'inactive',
      lastCheck: new Date(),
      message: rosStatus
    });

    // 2. Backend API
    try {
      const health = await backendService.checkHealth();
      newServices.push({
        name: 'Backend API',
        status: health.status === 'healthy' ? 'active' : 'error',
        lastCheck: new Date(),
        message: `Status: ${health.status}`
      });

      // 3. Gemini API (from backend health checks)
      if (health.checks) {
        const geminiStatus = health.checks.gemini || health.checks.ai || 'unknown';
        newServices.push({
          name: 'Gemini API',
          status: geminiStatus === 'connected' ? 'active' : 'inactive',
          lastCheck: new Date(),
          message: geminiStatus
        });
      }
    } catch {
      newServices.push({
        name: 'Backend API',
        status: 'error',
        lastCheck: new Date(),
        message: 'Connection failed'
      });
    }

    // 4. Conversation Memory (always active - localStorage)
    newServices.push({
      name: 'Conversation Memory',
      status: 'active',
      lastCheck: new Date(),
      message: `Session: ${backendService.getCurrentSessionId().substring(0, 12)}...`
    });

    setServices(newServices);
    setChecking(false);
  };

  useEffect(() => {
    checkAllServices();

    // Auto-check every 10 seconds
    const interval = setInterval(checkAllServices, 10000);
    return () => clearInterval(interval);
  }, []);

  return (
    <div className="bg-slate-800 rounded-lg p-6">
      <div className="flex justify-between items-center mb-4">
        <h2 className="text-xl font-bold">System Status</h2>
        <button
          onClick={checkAllServices}
          disabled={checking}
          className="px-3 py-1 bg-blue-600 hover:bg-blue-700 rounded-md text-sm flex items-center gap-2 disabled:opacity-50"
          title="Refresh status"
        >
          <RefreshCw className={`h-4 w-4 ${checking ? 'animate-spin' : ''}`} />
          Refresh
        </button>
      </div>

      <div className="space-y-3">
        {services.map((service) => (
          <div key={service.name} className="flex items-center justify-between p-3 bg-slate-700/50 rounded-md">
            <div className="flex items-center space-x-3">
              <Circle
                className={`h-3 w-3 ${service.status === 'active'
                    ? 'fill-green-500 text-green-500'
                    : service.status === 'error'
                      ? 'fill-red-500 text-red-500'
                      : 'fill-slate-500 text-slate-500'
                  }`}
              />
              <div>
                <p className="font-medium">{service.name}</p>
                {service.message && (
                  <p className="text-xs text-slate-400">{service.message}</p>
                )}
              </div>
            </div>
            <div className={`text-sm font-medium ${service.status === 'active'
                ? 'text-green-400'
                : service.status === 'error'
                  ? 'text-red-400'
                  : 'text-slate-400'
              }`}>
              {service.status === 'active' ? 'Active' :
                service.status === 'error' ? 'Error' : 'Inactive'}
            </div>
          </div>
        ))}
      </div>
    </div>
  );
};
