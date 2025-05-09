import React from 'react';
import { CheckCircle, XCircle, AlertTriangle } from 'lucide-react';

interface ServiceStatus {
  name: string;
  status: 'active' | 'inactive' | 'warning';
}

export const SystemStatus: React.FC = () => {
  const services: ServiceStatus[] = [
    { name: 'Speech Processing', status: 'active' },
    { name: 'Memory Manager', status: 'active' },
    { name: 'Command Parser', status: 'active' },
    { name: 'Command Executor', status: 'active' },
    { name: 'Dialog Manager', status: 'active' },
    { name: 'Vector Memory', status: 'warning' },
    { name: 'Text-to-Speech', status: 'active' },
    { name: 'Robot Integration', status: 'inactive' }
  ];

  const getStatusIcon = (status: string) => {
    switch (status) {
      case 'active':
        return <CheckCircle className="h-4 w-4 text-green-400" />;
      case 'inactive':
        return <XCircle className="h-4 w-4 text-red-400" />;
      case 'warning':
        return <AlertTriangle className="h-4 w-4 text-amber-400" />;
      default:
        return null;
    }
  };

  const getStatusText = (status: string) => {
    switch (status) {
      case 'active':
        return 'Active';
      case 'inactive':
        return 'Inactive';
      case 'warning':
        return 'Warning';
      default:
        return status;
    }
  };

  return (
    <div className="bg-slate-800 rounded-lg p-6 w-1/2">
      <div className="flex items-center justify-between mb-4">
        <h2 className="text-xl font-bold">System Status</h2>
        <button className="px-3 py-1 text-sm bg-blue-600 hover:bg-blue-700 rounded-md transition-colors">
          Refresh
        </button>
      </div>

      <div className="grid gap-4">
        {services.map((service) => (
          <div
            key={service.name}
            className="flex items-center justify-between p-3 bg-slate-700/50 rounded-md"
          >
            <span className="text-sm font-medium">{service.name}</span>
            <div className="flex items-center gap-2">
              {getStatusIcon(service.status)}
              <span className="text-sm">{getStatusText(service.status)}</span>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
};