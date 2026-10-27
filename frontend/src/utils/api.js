// API client for chat endpoints. The base URL is read per call, not frozen at import time.
import { getBackendUrl } from './backendUrl';

const apiBase = () => `${getBackendUrl()}/api`;

export const chatApi = {
  // Send a message to the chat API
  sendMessage: async (message, conversationId = null) => {
    try {
      const response = await fetch(`${apiBase()}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message,
          conversation_id: conversationId,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error sending message:', error);
      throw error;
    }
  },

  // Search documents in the knowledge base
  searchDocuments: async (query, topK = 5) => {
    try {
      const response = await fetch(`${apiBase()}/search`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query,
          top_k: topK,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error searching documents:', error);
      throw error;
    }
  },
};