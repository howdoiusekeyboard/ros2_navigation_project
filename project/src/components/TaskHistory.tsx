import React from 'react';

export const TaskHistory: React.FC = () => {
  // Sample task history data
  const tasks = [
    { id: 1, name: 'Navigation started', status: 'completed', timestamp: '10:23 AM' },
    { id: 2, name: 'Circular motion activated', status: 'completed', timestamp: '10:25 AM' },
    { id: 3, name: 'Path planning', status: 'in-progress', timestamp: '10:28 AM' },
  ];

  const getStatusClass = (status: string) => {
    switch (status) {
      case 'completed':
        return 'bg-green-900 text-green-300 border border-green-700';
      case 'in-progress':
        return 'bg-blue-900 text-blue-300 border border-blue-700';
      default:
        return 'bg-slate-800 text-slate-300';
    }
  };

  return (
    <div className="panel p-4">
      <h2 className="text-lg font-semibold mb-4 text-blue-300">Task History</h2>
      <div className="space-y-2">
        {tasks.map((task) => (
          <div key={task.id} className="flex justify-between items-center p-2 bg-slate-700 rounded border border-slate-600">
            <div>
              <span className="font-medium">{task.name}</span>
              <span className={`ml-2 text-xs px-2 py-1 rounded-full font-semibold ${getStatusClass(task.status)}`}>
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