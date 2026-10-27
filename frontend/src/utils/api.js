// API client for the course assistant. The base URL is read per call, not frozen at import time.
import { getBackendUrl } from './backendUrl';

const apiBase = () => `${getBackendUrl()}/api`;

export class ApiError extends Error {
  constructor(message, status) {
    super(message);
    this.status = status;
  }
}

async function errorFrom(response) {
  let detail = '';
  try {
    detail = (await response.json()).detail || '';
  } catch {
    // not JSON
  }
  return new ApiError(typeof detail === 'string' && detail ? detail : `HTTP ${response.status}`, response.status);
}

async function post(path, body, signal) {
  const response = await fetch(`${apiBase()}${path}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
    signal,
  });
  if (!response.ok) throw await errorFrom(response);
  return response.json();
}

// Older backends returned sources as file paths; newer ones return {n, title, section, url}.
function normalizeSources(sources = []) {
  return sources
    .map((s, i) => {
      if (s && typeof s === 'object') return { n: s.n ?? i + 1, title: s.title, section: s.section, url: s.url };
      const m = String(s).replace(/\\/g, '/').match(/docs\/(.+?)\.mdx?$/);
      if (!m) return null;
      const slug = m[1].split('/').map((part) => part.replace(/^\d+-/, '')).join('/').replace(/(^|\/)index$/, '');
      return { n: i + 1, title: slug.split('/').pop()?.replace(/-/g, ' ') || 'Course', url: `/docs/${slug}` };
    })
    .filter(Boolean);
}

/**
 * Ask Rivet and stream the answer.
 * Calls onMeta({conversationId, sources}) once, then onToken(text) for each piece of the answer.
 * Falls back to the non-streaming endpoint when the backend has no /chat/stream yet.
 */
export async function streamChat({ message, history, page, conversationId, signal, onMeta, onToken }) {
  const body = { message, history, page, conversation_id: conversationId };
  const response = await fetch(`${apiBase()}/chat/stream`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json', Accept: 'text/event-stream' },
    body: JSON.stringify(body),
    signal,
  });

  if (response.status === 404 || response.status === 405) {
    const res = await post('/chat', body, signal);
    onMeta?.({ conversationId: res.conversation_id, sources: normalizeSources(res.sources) });
    onToken?.(res.response || '');
    return;
  }
  if (!response.ok || !response.body) throw await errorFrom(response);

  const reader = response.body.getReader();
  const decoder = new TextDecoder();
  let buffer = '';
  for (;;) {
    const { value, done } = await reader.read();
    if (done) break;
    buffer += decoder.decode(value, { stream: true });
    let cut;
    while ((cut = buffer.indexOf('\n\n')) !== -1) {
      const raw = buffer.slice(0, cut);
      buffer = buffer.slice(cut + 2);
      let event = 'message';
      let data = '';
      for (const line of raw.split('\n')) {
        if (line.startsWith('event:')) event = line.slice(6).trim();
        else if (line.startsWith('data:')) data += line.slice(5).trim();
      }
      const payload = data ? JSON.parse(data) : {};
      if (event === 'meta') onMeta?.({ conversationId: payload.conversation_id, sources: normalizeSources(payload.sources) });
      else if (event === 'token') onToken?.(payload.text || '');
      else if (event === 'error') throw new ApiError(payload.message || 'The assistant failed.', payload.status || 500);
      else if (event === 'done') return;
    }
  }
}

export const chatApi = {
  searchDocuments: (query, topK = 5) => post('/search', { query, top_k: topK }),
};
