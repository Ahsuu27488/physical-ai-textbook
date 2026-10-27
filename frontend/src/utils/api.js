// API client for chat endpoints. The base URL is read per call, not frozen at import time.
import { getBackendUrl } from './backendUrl';

const apiBase = () => `${getBackendUrl()}/api`;

export class ApiError extends Error {
  constructor(message, status) {
    super(message);
    this.status = status;
  }
}

async function post(path, body, signal) {
  const response = await fetch(`${apiBase()}${path}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
    signal,
  });
  if (!response.ok) {
    let detail = '';
    try {
      detail = (await response.json()).detail || '';
    } catch {
      // not JSON
    }
    throw new ApiError(detail || `HTTP ${response.status}`, response.status);
  }
  return response.json();
}

export const chatApi = {
  // Send a message to the chat API. `signal` lets the caller cancel (the Stop button).
  sendMessage: (message, conversationId = null, { signal } = {}) =>
    post('/chat', { message, conversation_id: conversationId }, signal),

  // Search documents in the knowledge base
  searchDocuments: (query, topK = 5) => post('/search', { query, top_k: topK }),
};
