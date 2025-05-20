import React from 'react';

export const ArchitectureView: React.FC = () => {
  return (
    <div className="bg-white p-4 rounded-lg shadow">
      <h2 className="text-lg font-semibold mb-4">System Architecture</h2>
      
      <div className="border border-gray-300 rounded-lg p-4">
        <div className="grid grid-cols-1 md:grid-cols-3 gap-4 mb-4">
          <div className="bg-blue-100 p-3 rounded-lg text-center">
            <h3 className="font-medium text-blue-800">Web Frontend</h3>
            <p className="text-sm text-blue-600">React Application</p>
          </div>
          <div className="bg-green-100 p-3 rounded-lg text-center">
            <h3 className="font-medium text-green-800">ROS Bridge</h3>
            <p className="text-sm text-green-600">WebSocket Interface</p>
          </div>
          <div className="bg-purple-100 p-3 rounded-lg text-center">
            <h3 className="font-medium text-purple-800">ROS2 System</h3>
            <p className="text-sm text-purple-600">Robot Control Nodes</p>
          </div>
        </div>
        
        <div className="flex justify-center">
          <div className="w-24 h-1 bg-gray-300"></div>
        </div>
        
        <div className="grid grid-cols-1 md:grid-cols-2 gap-4 mt-4">
          <div className="border border-gray-200 rounded-lg p-3">
            <h3 className="font-medium mb-2">Circular Motion Package</h3>
            <ul className="text-sm space-y-1">
              <li>• Control circular movements</li>
              <li>• Linear and angular velocity</li>
              <li>• Custom motion patterns</li>
            </ul>
          </div>
          <div className="border border-gray-200 rounded-lg p-3">
            <h3 className="font-medium mb-2">Turtlesim Package</h3>
            <ul className="text-sm space-y-1">
              <li>• Turtle simulation</li>
              <li>• Visual feedback</li>
              <li>• Testing environment</li>
            </ul>
          </div>
        </div>
      </div>
      
      <div className="mt-4 text-sm text-gray-500">
        <p>Architecture shows the connection between the web frontend and ROS2 system through the ROS Bridge.</p>
      </div>
    </div>
  );
}; 