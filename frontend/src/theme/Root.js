import React, {useEffect} from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { AuthProvider } from '@site/src/hooks/useAuth';
import ChatWidget from '@site/src/components/ChatWidget';
import ProgressSync from '@site/src/components/ProgressSync';
import {loadUrduFont} from '@site/src/utils/urduFont';

// Root wraps the whole site and never unmounts, so the chat keeps its conversation across page changes.
export default function Root({children}) {
  useEffect(() => {
    if (document.documentElement.lang === 'ur') loadUrduFont(); // the Urdu edition reads in Nastaliq
  }, []);
  return (
    <AuthProvider>
      {children}
      <ProgressSync />
      <BrowserOnly>{() => <ChatWidget />}</BrowserOnly>
    </AuthProvider>
  );
}
