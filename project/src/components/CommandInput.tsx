import React, { useState } from 'react';
import { Mic, Send } from 'lucide-react';

export const CommandInput: React.FC = () => {
  const [command, setCommand] = useState('');
  const [isRecording, setIsRecording] = useState(false);
  
  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (command.trim()) {
      console.log('Command submitted:', command);
      // Here you would send the command to the processing pipeline
      setCommand('');
    }
  };
  
  const toggleRecording = () => {
    setIsRecording(!isRecording);
    // Here you would integrate with Whisper API for speech-to-text
  };
  
  return (
    <div className="bg-slate-800 rounded-lg p-6">
      <h2 className="text-xl font-bold mb-4">Command Input</h2>
      
      <form onSubmit={handleSubmit} className="space-y-4">
        <div className="flex flex-col space-y-2">
          <label htmlFor="command" className="text-sm text-slate-400">
            Enter a command for the robot
          </label>
          <div className="flex">
            <input
              type="text"
              id="command"
              value={command}
              onChange={(e) => setCommand(e.target.value)}
              placeholder="e.g., Go to the table, wait for 5 seconds, then return to base"
              className="flex-1 p-2 bg-slate-900 border border-slate-700 rounded-l-md focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent"
            />
            <button
              type="button"
              onClick={toggleRecording}
              className={`p-2 ${
                isRecording 
                  ? 'bg-red-600 hover:bg-red-700' 
                  : 'bg-slate-700 hover:bg-slate-600'
              } rounded-none transition-colors`}
            >
              <Mic className="h-5 w-5" />
            </button>
            <button
              type="submit"
              className="p-2 bg-blue-600 hover:bg-blue-700 rounded-r-md transition-colors"
            >
              <Send className="h-5 w-5" />
            </button>
          </div>
        </div>
        
        <div className="text-sm text-slate-400">
          <h3 className="font-medium mb-1">Example commands:</h3>
          <ul className="list-disc pl-5 space-y-1">
            <li>"Go to the kitchen"</li>
            <li>"Move forward 2 meters, then turn left"</li>
            <li>"Find the red ball and bring it to me"</li>
            <li>"Navigate to charging station"</li>
          </ul>
        </div>
      </form>
    </div>
  );
};