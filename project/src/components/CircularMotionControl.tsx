import React, { useState, useEffect } from 'react';
import rosService from '../services/RosService';

interface CircularMotionControlProps {
  title?: string;
}

export const CircularMotionControl: React.FC<CircularMotionControlProps> = ({
  title = 'Circular Motion Control'
}) => {
  const [connected, setConnected] = useState(false);
  const [linearSpeed, setLinearSpeed] = useState(2.0);
  const [angularSpeed, setAngularSpeed] = useState(1.0);
  const [isMoving, setIsMoving] = useState(false);

  useEffect(() => {
    // Try to connect to ROS on component mount
    connectToROS();
    
    // Cleanup on unmount
    return () => {
      if (connected) {
        if (isMoving) {
          rosService.stopMotion();
        }
        rosService.disconnect();
      }
    };
  }, []);

  const connectToROS = async () => {
    try {
      const result = await rosService.connect();
      setConnected(result);
    } catch (error) {
      console.error('Failed to connect to ROS:', error);
      setConnected(false);
    }
  };

  const handleStart = () => {
    if (!connected) {
      connectToROS();
      return;
    }

    rosService.setCircularMotion(linearSpeed, angularSpeed, true);
    setIsMoving(true);
  };

  const handleStop = () => {
    if (!connected) return;
    
    rosService.stopMotion();
    setIsMoving(false);
  };

  const handleLinearSpeedChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const value = parseFloat(e.target.value);
    setLinearSpeed(value);
    
    if (isMoving) {
      rosService.setCircularMotion(value, angularSpeed, true);
    }
  };

  const handleAngularSpeedChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const value = parseFloat(e.target.value);
    setAngularSpeed(value);
    
    if (isMoving) {
      rosService.setCircularMotion(linearSpeed, value, true);
    }
  };

  return (
    <div className="p-4 bg-white rounded-lg shadow-md">
      <h2 className="text-xl font-bold mb-4">{title}</h2>
      
      <div className="mb-4">
        <div className="flex items-center justify-between mb-2">
          <label htmlFor="linearSpeed" className="font-medium">
            Linear Speed: {linearSpeed.toFixed(1)}
          </label>
          <span className="text-sm text-gray-500">m/s</span>
        </div>
        <input
          id="linearSpeed"
          type="range"
          min="0"
          max="5"
          step="0.1"
          value={linearSpeed}
          onChange={handleLinearSpeedChange}
          className="w-full"
        />
      </div>
      
      <div className="mb-6">
        <div className="flex items-center justify-between mb-2">
          <label htmlFor="angularSpeed" className="font-medium">
            Angular Speed: {angularSpeed.toFixed(1)}
          </label>
          <span className="text-sm text-gray-500">rad/s</span>
        </div>
        <input
          id="angularSpeed"
          type="range"
          min="-3"
          max="3"
          step="0.1"
          value={angularSpeed}
          onChange={handleAngularSpeedChange}
          className="w-full"
        />
      </div>
      
      <div className="flex justify-between">
        <button
          onClick={handleStart}
          disabled={isMoving}
          className={`px-4 py-2 rounded-md ${
            isMoving
              ? 'bg-gray-300 cursor-not-allowed'
              : 'bg-green-500 hover:bg-green-600 text-white'
          }`}
        >
          Start
        </button>
        <button
          onClick={handleStop}
          disabled={!isMoving}
          className={`px-4 py-2 rounded-md ${
            !isMoving
              ? 'bg-gray-300 cursor-not-allowed'
              : 'bg-red-500 hover:bg-red-600 text-white'
          }`}
        >
          Stop
        </button>
      </div>
      
      <div className="mt-4 text-center">
        <span className={`inline-flex items-center ${connected ? 'text-green-500' : 'text-red-500'}`}>
          {connected ? 'Connected to ROS' : 'Disconnected from ROS'}
          <span className={`ml-2 h-3 w-3 rounded-full ${connected ? 'bg-green-500' : 'bg-red-500'}`}></span>
        </span>
      </div>
    </div>
  );
}; 