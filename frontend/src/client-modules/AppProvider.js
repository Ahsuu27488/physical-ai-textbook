import React, { useEffect } from 'react';
import { AuthProvider } from '@site/src/hooks/useAuth';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// This client module wraps the entire app with the AuthProvider
// allowing navbar items to access the authentication context
export default function AppProvider({ children }) {
  const { siteConfig } = useDocusaurusContext();

  useEffect(() => {
    // Inject backend URL into global scope for API clients
    if (typeof window !== 'undefined') {
      window.__BACKEND_URL__ = siteConfig.customFields.backendUrl;
    }
  }, [siteConfig]);

  return <AuthProvider>{children}</AuthProvider>;
}