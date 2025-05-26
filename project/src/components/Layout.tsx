import React, { ReactNode } from 'react';
import { BrainCircuit, Menu, Mic, Navigation, Settings, X, Circle, Globe } from 'lucide-react';
import { useState } from 'react';
import { turtleControlService } from '../services/TurtleControlService';

interface LayoutProps {
  children: ReactNode;
  onPageChange: (page: string) => void;
  currentPage: string;
}

export const Layout: React.FC<LayoutProps> = ({ children, onPageChange, currentPage }) => {
  const [sidebarOpen, setSidebarOpen] = useState(false);
  const [systemStatus, setSystemStatus] = useState<'online' | 'offline' | 'error'>('online');
  
  const navItems = [
    { id: 'dashboard', label: 'Dashboard', icon: Navigation },
    { id: 'services', label: 'Services', icon: Globe },
    { id: 'speech', label: 'Speech Processing', icon: Mic },
    { id: 'memory', label: 'Memory Management', icon: BrainCircuit },
    { id: 'command', label: 'Command Control', icon: BrainCircuit },
    { id: 'settings', label: 'Settings', icon: Settings }
  ];
  
  const getStatusColor = () => {
    switch (systemStatus) {
      case 'online':
        return 'text-green-400';
      case 'error':
        return 'text-red-400';
      case 'offline':
        return 'text-slate-400';
    }
  };
  
  const getConnectButtonText = () => {
    switch (systemStatus) {
      case 'online':
        return 'Disconnect';
      case 'error':
        return 'Retry Connection';
      case 'offline':
        return 'Connect';
    }
  };
  
  return (
    <div className="min-h-screen bg-slate-900 text-white flex">
      {/* Mobile sidebar backdrop */}
      {sidebarOpen && (
        <div 
          className="fixed inset-0 bg-black/50 z-10 lg:hidden"
          onClick={() => setSidebarOpen(false)}
        />
      )}
      
      {/* Sidebar */}
      <aside className={`${sidebarOpen ? 'translate-x-0' : '-translate-x-full'} fixed top-0 left-0 z-20 h-full w-64 bg-slate-800 transition-transform duration-300 ease-in-out lg:translate-x-0 lg:static`}>
        <div className="p-4 flex items-center justify-between border-b border-slate-700">
          <div className="flex items-center gap-2">
            <BrainCircuit className="h-6 w-6 text-blue-400" />
            <h1 className="text-xl font-bold">RoboMind</h1>
          </div>
          <button
            className="lg:hidden text-slate-400 hover:text-white"
            onClick={() => setSidebarOpen(false)}
          >
            <X className="h-5 w-5" />
          </button>
        </div>
        
        <nav className="p-4">
          <ul className="space-y-2">
            {navItems.map((item) => (
              <li key={item.id}>
                <button
                  onClick={() => {
                    onPageChange(item.id);
                    setSidebarOpen(false);
                  }}
                  className={`w-full flex items-center gap-3 p-2 rounded-md ${
                    currentPage === item.id
                      ? 'bg-slate-700 text-white'
                      : 'text-slate-300 hover:bg-slate-700 hover:text-white'
                  } transition-colors`}
                >
                  <item.icon className="h-5 w-5" />
                  <span>{item.label}</span>
                </button>
              </li>
            ))}
          </ul>
        </nav>
      </aside>
      
      {/* Main content */}
      <main className="flex-1 max-h-screen overflow-auto">
        {/* Header */}
        <header className="bg-slate-800 p-4 flex items-center justify-between sticky top-0 z-10">
          <button
            className="lg:hidden text-slate-400 hover:text-white"
            onClick={() => setSidebarOpen(true)}
          >
            <Menu className="h-6 w-6" />
          </button>
          
          <div className="flex items-center gap-4 ml-auto">
            <div className="flex items-center gap-2">
              <Circle className={`h-3 w-3 ${getStatusColor()}`} fill="currentColor" />
              <span className="text-sm font-medium">System {systemStatus.charAt(0).toUpperCase() + systemStatus.slice(1)}</span>
            </div>
            <button
              onClick={() => {
                if (window.confirm('Are you sure you want to shutdown the application?')) {
                  try {
                    turtleControlService.destroy();
                  } catch (e) {}
                  if (window.close) window.close();
                  window.location.href = 'about:blank';
                }
              }}
              className="px-3 py-1.5 rounded-md bg-red-700 hover:bg-red-800 transition-colors font-bold"
              title="Shutdown the application"
            >
              Shutdown
            </button>
          </div>
        </header>
        
        {/* Page content */}
        <div className="p-6">
          {children}
        </div>
      </main>
    </div>
  );
};