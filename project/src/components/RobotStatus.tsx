import React, { useState, useEffect } from 'react';
import { Battery, Cpu, MapPin, Signal, Navigation } from 'lucide-react';
import { rosService, RobotPose } from '../services/rosService';

interface RobotState {
  pose: RobotPose | null;
  battery: number | null; // Not available in turtlesim
  cpuTemp: number | null; // Not available in turtlesim
  connectionQuality: number;
  lastExplanation: string | null;
}

export const RobotStatus: React.FC = () => {
  const [robotState, setRobotState] = useState<RobotState>({
    pose: null,
    battery: null,
    cpuTemp: null,
    connectionQuality: 0,
    lastExplanation: null
  });

  useEffect(() => {
    // Subscribe to robot pose updates (real-time)
    const poseCallback = (pose: RobotPose) => {
      setRobotState(prev => ({ ...prev, pose }));
    };
    rosService.onPoseChange(poseCallback);

    // Subscribe to connection status changes
    const statusCallback = (status: string) => {
      const quality = rosService.getConnectionStatus() === 'connected' ? 100 : 0;
      setRobotState(prev => ({ ...prev, connectionQuality: quality }));
    };
    rosService.onStatusChange(statusCallback);

    // Subscribe to explanations for current execution context
    const explanationCallback = (explanation: any) => {
      setRobotState(prev => ({
        ...prev,
        lastExplanation: explanation.text || 'Processing...'
      }));
    };
    rosService.onExplanationReceived(explanationCallback);

    // Subscribe to battery state
    rosService.onBattery((battery) => {
      setRobotState(prev => ({ ...prev, battery: Math.round(battery.percentage) }));
    });

    // Set initial connection quality
    const initialQuality = rosService.getConnectionStatus() === 'connected' ? 100 : 0;
    setRobotState(prev => ({ ...prev, connectionQuality: initialQuality }));

    // Cleanup not needed - callbacks persist for app lifetime
  }, []);

  return (
    <div className="bg-slate-800 rounded-lg p-6">
      <h2 className="text-xl font-bold mb-4">Robot Status</h2>

      <div className="space-y-5">
        <div className="grid grid-cols-2 gap-4">
          {/* Battery - Show N/A for turtlesim */}
          <StatusItem
            icon={<Battery className="h-5 w-5" />}
            label="Battery"
            value={robotState.battery !== null ? `${robotState.battery}%` : 'N/A'}
            colorClass={
              robotState.battery !== null
                ? (robotState.battery > 50 ? "text-green-400" :
                   robotState.battery > 20 ? "text-amber-400" : "text-red-400")
                : "text-slate-500"
            }
            subtitle={robotState.battery === null ? "(Turtlesim mode)" : undefined}
          />

          {/* Location - Real pose data */}
          <StatusItem
            icon={<MapPin className="h-5 w-5" />}
            label="Position"
            value={
              robotState.pose
                ? `(${robotState.pose.x.toFixed(2)}, ${robotState.pose.y.toFixed(2)})`
                : 'Waiting...'
            }
            colorClass="text-blue-400"
            subtitle={robotState.pose ? `θ: ${robotState.pose.theta.toFixed(2)} rad` : undefined}
          />

          {/* CPU Temp - Show N/A for turtlesim */}
          <StatusItem
            icon={<Cpu className="h-5 w-5" />}
            label="CPU Temp"
            value={robotState.cpuTemp !== null ? `${robotState.cpuTemp}°C` : 'N/A'}
            colorClass={
              robotState.cpuTemp !== null
                ? (robotState.cpuTemp < 50 ? "text-green-400" :
                   robotState.cpuTemp < 75 ? "text-amber-400" : "text-red-400")
                : "text-slate-500"
            }
            subtitle={robotState.cpuTemp === null ? "(Turtlesim mode)" : undefined}
          />

          {/* Connection - Real rosbridge status */}
          <StatusItem
            icon={<Signal className="h-5 w-5" />}
            label="Connection"
            value={`${robotState.connectionQuality}%`}
            colorClass={
              robotState.connectionQuality > 80 ? "text-green-400" :
              robotState.connectionQuality > 40 ? "text-amber-400" :
              robotState.connectionQuality > 0 ? "text-red-400" : "text-slate-500"
            }
            subtitle={rosService.getConnectionStatus()}
          />
        </div>

        {/* Current Execution / Explanation */}
        {robotState.lastExplanation && (
          <div className="border-t border-slate-700 pt-4">
            <h3 className="text-sm font-medium text-slate-400 mb-2 flex items-center gap-2">
              <Navigation className="h-4 w-4" />
              Latest Navigation Event
            </h3>
            <div className="bg-slate-700 p-3 rounded-md">
              <p className="text-sm text-slate-300">{robotState.lastExplanation}</p>
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

interface StatusItemProps {
  icon: React.ReactNode;
  label: string;
  value: string;
  colorClass: string;
  subtitle?: string;
}

const StatusItem: React.FC<StatusItemProps> = ({ icon, label, value, colorClass, subtitle }) => {
  return (
    <div className="flex items-center space-x-3">
      <div className={`${colorClass}`}>
        {icon}
      </div>
      <div>
        <p className="text-xs text-slate-400">{label}</p>
        <p className={`text-sm font-medium ${colorClass}`}>{value}</p>
        {subtitle && <p className="text-xs text-slate-500">{subtitle}</p>}
      </div>
    </div>
  );
};
