import React, { useEffect, useState } from 'react';
import { CircularMotionControl } from '../components/CircularMotionControl';
import { CmdVelConsole } from '../components/CmdVelConsole';
import { CommandInput } from '../components/CommandInput';
import { llmService } from '../services/LlmService';

export const Services: React.FC = () => {
  const [llmStatus, setLlmStatus] = useState<'checking' | 'ready' | 'error'>('checking');
  const [llmMessage, setLlmMessage] = useState('');

  useEffect(() => {
    // Robust check: try a dummy LLM call
    let cancelled = false;
    setLlmStatus('checking');
    setLlmMessage('Verifying LLM API service...');
    llmService.parseCircularMotion('Spin in a circle').then((result) => {
      if (cancelled) return;
      if (result && result.action === 'start') {
        setLlmStatus('ready');
        setLlmMessage('LLM API is ready and responding.');
      } else {
        setLlmStatus('error');
        setLlmMessage('LLM API did not return a valid response.');
      }
    }).catch((e) => {
      if (cancelled) return;
      setLlmStatus('error');
      setLlmMessage('LLM API check failed: ' + (e?.message || e));
    });
    return () => { cancelled = true; };
  }, []);

  return (
    <div className="space-y-8">
      <div className="flex items-center gap-3 p-4 rounded bg-slate-800">
        <span className={`h-3 w-3 rounded-full ${llmStatus === 'ready' ? 'bg-green-400' : llmStatus === 'checking' ? 'bg-yellow-400 animate-pulse' : 'bg-red-400'}`}></span>
        <span className={`text-sm ${llmStatus === 'ready' ? 'text-green-300' : llmStatus === 'checking' ? 'text-yellow-300' : 'text-red-300'}`}>{llmMessage}</span>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 xl:grid-cols-3 gap-6">
        <CircularMotionControl title="Turtle Circular Motion" />
        <div className="panel p-4">
          <h2 className="text-xl font-bold mb-4 text-blue-300">ROS Connection Status</h2>
          {/* You can add a ROS connection status component here if needed */}
          <div className="mt-4">
            <p className="text-sm text-slate-300">
              To use the control features, make sure you have started the ROS Bridge server with:
            </p>
            <div className="mt-2 bg-slate-700 p-2 rounded text-sm font-mono text-slate-300">
              ros2 launch rosbridge_server rosbridge_websocket_launch.xml
            </div>
          </div>
        </div>
        <CmdVelConsole />
      </div>

      <div>
        <CommandInput />
      </div>
    </div>
  );
}; 