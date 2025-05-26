import React, { useState, useEffect } from 'react';
import { turtleControlService, type TurtleState } from '../services/TurtleControlService';

interface CircularMotionControlProps {
  title?: string;
}

export const CircularMotionControl: React.FC<CircularMotionControlProps> = ({
  title = 'Circular Motion Control'
}) => {
  const [state, setState] = useState<TurtleState>(turtleControlService.getState());

  useEffect(() => {
    // Subscribe to state changes from the centralized service
    const unsubscribe = turtleControlService.onStateChange(setState);
    
    // Cleanup subscription on unmount
    return unsubscribe;
  }, []);

  const handleStart = () => {
    if (!state.connected) {
      turtleControlService.reconnect();
      return;
    }

    turtleControlService.start('manual');
  };

  const handleStop = () => {
    turtleControlService.stop('manual');
  };

  const handleLinearSpeedChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const value = parseFloat(e.target.value);
    turtleControlService.setSpeed(value, state.angularSpeed, 'slider');
  };

  const handleAngularSpeedChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const value = parseFloat(e.target.value);
    turtleControlService.setSpeed(state.linearSpeed, value, 'slider');
  };

  return (
    <div className="panel p-4">
      <h2 className="text-xl font-bold mb-4">{title}</h2>
      
      <div className="mb-4">
        <div className="flex items-center justify-between mb-2">
          <label htmlFor="linearSpeed" className="font-medium">
            Linear Speed: {state.linearSpeed.toFixed(1)}
          </label>
          <span className="text-sm text-gray-500">m/s</span>
        </div>
        <input
          id="linearSpeed"
          type="range"
          min="0"
          max="5"
          step="0.1"
          value={state.linearSpeed}
          onChange={handleLinearSpeedChange}
          className="w-full"
        />
      </div>
      
      <div className="mb-6">
        <div className="flex items-center justify-between mb-2">
          <label htmlFor="angularSpeed" className="font-medium">
            Angular Speed: {state.angularSpeed.toFixed(1)}
          </label>
          <span className="text-sm text-gray-500">rad/s</span>
        </div>
        <input
          id="angularSpeed"
          type="range"
          min="-3"
          max="3"
          step="0.1"
          value={state.angularSpeed}
          onChange={handleAngularSpeedChange}
          className="w-full"
        />
      </div>
      
      <div className="flex justify-between">
        <button
          onClick={handleStart}
          disabled={state.isMoving}
          className={`btn ${
            state.isMoving
              ? 'bg-slate-600 text-slate-400 cursor-not-allowed'
              : 'btn-success'
          }`}
        >
          Start
        </button>
        <button
          onClick={handleStop}
          disabled={!state.isMoving}
          className={`btn ${
            !state.isMoving
              ? 'bg-slate-600 text-slate-400 cursor-not-allowed'
              : 'btn-danger'
          }`}
        >
          Stop
        </button>
      </div>
      
      <div className="mt-4 text-center">
        <span className={`inline-flex items-center ${state.connected ? 'text-green-400' : 'text-red-400'}`}>
          {state.connected ? 'Connected to ROS' : 'Disconnected from ROS'}
          <span className={`ml-2 h-3 w-3 rounded-full ${state.connected ? 'bg-green-400' : 'bg-red-400'}`}></span>
        </span>
      </div>
    </div>
  );
}; 