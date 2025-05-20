import React from 'react';

export const TaskHistory: React.FC = () => {
  // Sample task history data
  const tasks = [
    { id: 1, name: 'Navigation started', status: 'completed', timestamp: '10:23 AM' },
    { id: 2, name: 'Circular motion activated', status: 'completed', timestamp: '10:25 AM' },
    { id: 3, name: 'Path planning', status: 'in-progress', timestamp: '10:28 AM' },
  ];

  return (
    <div className="bg-white p-4 rounded-lg shadow">
      <h2 className="text-lg font-semibold mb-4">Task History</h2>
      <div className="space-y-2">
        {tasks.map((task) => (
          <div key={task.id} className="flex justify-between items-center p-2 bg-gray-50 rounded">
            <div>
              <span className="font-medium">{task.name}</span>
              <span className="ml-2 text-xs px-2 py-1 rounded-full 
                {task.status === 'completed' ? 'bg-green-100 text-green-800' : 
                task.status === 'in-progress' ? 'bg-blue-100 text-blue-800' : 'bg-gray-100'}">
                {task.status}
              </span>
            </div>
            <span className="text-gray-500 text-sm">{task.timestamp}</span>
          </div>
        ))}
      </div>
    </div>
  );
}; 