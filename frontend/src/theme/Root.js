import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { AuthProvider } from '@site/src/hooks/useAuth';
import ChatWidget from '@site/src/components/ChatWidget';

// Root wraps the whole site and never unmounts, so the chat keeps its conversation across page changes.
export default function Root({children}) {
  return (
    <AuthProvider>
      {children}
      <BrowserOnly>{() => <ChatWidget />}</BrowserOnly>
    </AuthProvider>
  );
}
