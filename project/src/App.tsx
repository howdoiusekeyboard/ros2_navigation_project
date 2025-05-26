import React, { useState } from 'react';
import { Layout } from './components/Layout';
import { RobotDashboard } from './pages/RobotDashboard';
import { Services } from './pages/Services';
import { SpeechProcessing } from './pages/SpeechProcessing';
import { MemoryManagement } from './pages/MemoryManagement';
import { CommandControl } from './pages/CommandControl';
import { Settings } from './pages/Settings';

export const App: React.FC = () => {
  const [page, setPage] = useState('dashboard');

  let content: React.ReactNode = null;
  switch (page) {
    case 'dashboard':
      content = <RobotDashboard />;
      break;
    case 'services':
      content = <Services />;
      break;
    case 'speech':
      content = <SpeechProcessing />;
      break;
    case 'memory':
      content = <MemoryManagement />;
      break;
    case 'command':
      content = <CommandControl />;
      break;
    case 'settings':
      content = <Settings />;
      break;
    default:
      content = <RobotDashboard />;
  }

  return (
    <Layout onPageChange={setPage} currentPage={page}>
      {content}
    </Layout>
  );
};

export default App;