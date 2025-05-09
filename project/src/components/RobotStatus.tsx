import React from 'react';
import { Battery, Cpu, MapPin, Signal, ThermometerSun } from 'lucide-react';

export const RobotStatus: React.FC = () => {
  // These would come from ROS2 topics in a real implementation
  const status = {
    batteryPercentage: 78,
    location: 'Living Room',
    cpuTemp: 42.5,
    connectionStrength: 95,
    lastCommand: 'Go to the table, wait 5 seconds, return to base',
    navigationStatus: 'Moving to target',
    executionProgress: 1, // 0-2 for the 3 steps in the compound command
  };
  
  return (
    <div className="bg-slate-800 rounded-lg p-6">
      <h2 className="text-xl font-bold mb-4">Robot Status</h2>
      
      <div className="space-y-5">
        <div className="grid grid-cols-2 gap-4">
          <StatusItem 
            icon={<Battery className="h-5 w-5" />}
            label="Battery"
            value={`${status.batteryPercentage}%`}
            colorClass={
              status.batteryPercentage > 50 ? "text-green-400" :
              status.batteryPercentage > 20 ? "text-amber-400" : "text-red-400"
            }
          />
          
          <StatusItem 
            icon={<MapPin className="h-5 w-5" />}
            label="Location"
            value={status.location}
            colorClass="text-blue-400"
          />
          
          <StatusItem 
            icon={<Cpu className="h-5 w-5" />}
            label="CPU Temp"
            value={`${status.cpuTemp}°C`}
            colorClass={
              status.cpuTemp < 50 ? "text-green-400" :
              status.cpuTemp < 75 ? "text-amber-400" : "text-red-400"
            }
          />
          
          <StatusItem 
            icon={<Signal className="h-5 w-5" />}
            label="Connection"
            value={`${status.connectionStrength}%`}
            colorClass={
              status.connectionStrength > 80 ? "text-green-400" :
              status.connectionStrength > 40 ? "text-amber-400" : "text-red-400"
            }
          />
        </div>
        
        <div className="border-t border-slate-700 pt-4">
          <h3 className="text-sm font-medium text-slate-400 mb-2">Current Execution</h3>
          
          {status.lastCommand && (
            <div className="bg-slate-700 p-3 rounded-md mb-3">
              <p className="text-sm font-mono text-slate-300">{status.lastCommand}</p>
              <p className="text-xs text-slate-400 mt-1">{status.navigationStatus}</p>
            </div>
          )}
          
          <div className="relative pt-1">
            <div className="flex mb-2 items-center justify-between">
              <div>
                <span className="text-xs font-semibold inline-block text-blue-500">
                  Step {status.executionProgress + 1} of 3
                </span>
              </div>
              <div className="text-right">
                <span className="text-xs font-semibold inline-block text-blue-500">
                  33%
                </span>
              </div>
            </div>
            <div className="overflow-hidden h-2 mb-4 text-xs flex rounded bg-slate-700">
              <div
                style={{ width: "33%" }}
                className="shadow-none flex flex-col text-center whitespace-nowrap text-white justify-center bg-blue-500"
              ></div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

interface StatusItemProps {
  icon: React.ReactNode;
  label: string;
  value: string;
  colorClass: string;
}

const StatusItem: React.FC<StatusItemProps> = ({ icon, label, value, colorClass }) => {
  return (
    <div className="flex items-center space-x-3">
      <div className={`${colorClass}`}>
        {icon}
      </div>
      <div>
        <p className="text-xs text-slate-400">{label}</p>
        <p className={`text-sm font-medium ${colorClass}`}>{value}</p>
      </div>
    </div>
  );
};