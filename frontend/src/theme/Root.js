import React from 'react';
import { AuthProvider } from '@site/src/hooks/useAuth'; // Ensure this matches your hook export

// The Root component wraps your entire Docusaurus site
export default function Root({children}) {
  return (
    <AuthProvider>
      {children}
    </AuthProvider>
  );
}