import React from 'react';
import { Layout } from './components/Layout';
import { RobotDashboard } from './pages/RobotDashboard';
import { SpeechProcessing } from './pages/SpeechProcessing';
import { MemoryManagement } from './pages/MemoryManagement';
import { CommandControl } from './pages/CommandControl';
import { Settings } from './pages/Settings';
import { useState } from 'react';

function App() {
  const [currentPage, setCurrentPage] = useState('dashboard');
  
  const renderPage = () => {
    switch (currentPage) {
      case 'dashboard':
        return <RobotDashboard />;
      case 'speech':
        return <SpeechProcessing />;
      case 'memory':
        return <MemoryManagement />;
      case 'command':
        return <CommandControl />;
      case 'settings':
        return <Settings />;
      default:
        return <RobotDashboard />;
    }
  };
  
  return (
    <Layout onPageChange={setCurrentPage} currentPage={currentPage}>
      {renderPage()}
    </Layout>
  );
}

export default App;