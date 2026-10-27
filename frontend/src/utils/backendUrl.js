import siteConfig from '@generated/docusaurus.config';

// Set at build time from BACKEND_URL (see docusaurus.config.js customFields). Read straight from the
// generated config so it is correct on the very first render; the old window.__BACKEND_URL__ global was
// never set, so every build silently used the fallback.
export const getBackendUrl = () =>
  (siteConfig.customFields?.backendUrl || 'https://ahsuu27488-physical-ai-textbook-backend.hf.space')
    .replace(/\/+$/, '');
