import React, { useEffect, useState } from 'react';
import rosService from '../services/RosService';

interface TwistMsg {
  linear: { x: number; y: number; z: number };
  angular: { x: number; y: number; z: number };
}

export const CmdVelConsole: React.FC = () => {
  const [messages, setMessages] = useState<TwistMsg[]>([]);

  useEffect(() => {
    if (!rosService.isConnected()) return;

    rosService.subscribe('/turtle1/cmd_vel', 'geometry_msgs/msg/Twist', (msg: TwistMsg) => {
      setMessages((prev) => {
        const next = [msg, ...prev];
        return next.slice(0, 20); // keep last 20
      });
    });

    return () => rosService.unsubscribe('/turtle1/cmd_vel');
  }, [rosService.isConnected()]);

  return (
    <div className="panel p-4 h-60 overflow-auto relative">
      <h2 className="text-xl font-bold mb-2 text-blue-300">cmd_vel stream</h2>
      <div className="absolute top-4 right-4 flex items-center">
        <div className="h-2 w-2 rounded-full bg-green-400 mr-2 animate-pulse"></div>
        <span className="text-xs text-green-400">live</span>
      </div>
      <div className="mt-4 h-44 overflow-auto bg-slate-900 rounded-md p-3">
        {messages.map((m, idx) => (
          <div key={idx} className="font-mono text-xs mb-1 flex">
            <span className="text-slate-500 mr-2">[{idx.toString().padStart(2, '0')}]</span>
            <span className="text-green-300">v=(</span>
            <span className="text-blue-300 mx-1">{m.linear.x.toFixed(2)}, {m.linear.y.toFixed(2)}, {m.linear.z.toFixed(2)}</span>
            <span className="text-green-300">)</span>
            <span className="text-slate-400 mx-2">|</span>
            <span className="text-green-300">ω=(</span>
            <span className="text-amber-300 mx-1">{m.angular.x.toFixed(2)}, {m.angular.y.toFixed(2)}, {m.angular.z.toFixed(2)}</span>
            <span className="text-green-300">)</span>
          </div>
        ))}
      </div>
      {messages.length === 0 && (
        <div className="flex items-center justify-center h-44 text-slate-500 text-sm">
          No messages yet. Start the turtle to see cmd_vel data.
        </div>
      )}
    </div>
  );
}; 