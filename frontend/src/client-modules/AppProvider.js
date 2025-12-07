import React from 'react';
import { AuthProvider } from '@site/src/hooks/useAuth';

// This client module wraps the entire app with the AuthProvider
// allowing navbar items to access the authentication context
export default function AppProvider({ children }) {
  return <AuthProvider>{children}</AuthProvider>;
}