import React, {useCallback, useEffect, useId, useRef, useState} from 'react';
import Link from '@docusaurus/Link';
import ReactMarkdown from 'react-markdown';
import {chatApi} from '@site/src/utils/api';
import {OPEN_EVENT} from './events';
import styles from './styles.module.css';

const SUGGESTIONS = [
  'What is ROS 2, and why do robots need middleware?',
  'How does a digital twin help before using real hardware?',
  'What does NVIDIA Isaac Sim add over Gazebo?',
  'What is a vision-language-action model?',
];

// "../frontend/docs/module-1/lesson-2-x.md" -> {to: "/docs/module-1/lesson-2-x", label: "Lesson 2 x"}
function toSource(path) {
  const m = String(path).replace(/\\/g, '/').match(/docs\/(.+?)\.mdx?$/);
  if (!m) return null;
  const slug = m[1].replace(/(^|\/)index$/, '');
  const last = slug.split('/').pop() || 'Start here';
  const label = last === 'intro' ? slug.split('/')[0].replace(/-/g, ' ') : last.replace(/^lesson-\d+-/, '').replace(/-/g, ' ');
  return {to: `/docs/${slug}`, label: label.charAt(0).toUpperCase() + label.slice(1)};
}

function uniqueSources(paths = []) {
  const seen = new Set();
  return paths
    .map(toSource)
    .filter((s) => s && !seen.has(s.to) && seen.add(s.to))
    .slice(0, 3);
}

function errorText(err) {
  if (err?.status === 503) return 'The assistant isn’t switched on at the server right now.';
  if (err?.status === 429) return 'Too many questions at once. Wait a moment, then retry.';
  if (typeof navigator !== 'undefined' && !navigator.onLine) return 'You’re offline. Reconnect, then retry.';
  return 'Couldn’t reach the assistant.';
}

const Icon = {
  chat: (
    <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      <path d="M4 5.5A2.5 2.5 0 0 1 6.5 3h11A2.5 2.5 0 0 1 20 5.5v8a2.5 2.5 0 0 1-2.5 2.5H10l-4.5 4v-4h0A1.5 1.5 0 0 1 4 14.5z" />
      <path d="M8.5 8.5h7M8.5 11.5h4" />
    </svg>
  ),
  close: (
    <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" aria-hidden="true">
      <path d="M6 6l12 12M18 6L6 18" />
    </svg>
  ),
  reset: (
    <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      <path d="M4 12a8 8 0 1 0 2.4-5.7M4 4v4h4" />
    </svg>
  ),
  send: (
    <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      <path d="M12 19V5M5 12l7-7 7 7" />
    </svg>
  ),
  stop: (
    <svg width="14" height="14" viewBox="0 0 24 24" aria-hidden="true">
      <rect x="5" y="5" width="14" height="14" rx="2.5" fill="currentColor" />
    </svg>
  ),
};

export default function ChatWidget() {
  const [open, setOpen] = useState(false);
  const [closing, setClosing] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [pending, setPending] = useState(false);
  const [error, setError] = useState(null);
  const [unseen, setUnseen] = useState(false);
  const conversationId = useRef(null);
  const abortRef = useRef(null);
  const lastQuestion = useRef('');
  const listRef = useRef(null);
  const inputRef = useRef(null);
  const launcherRef = useRef(null);
  const titleId = useId();

  const nearBottom = () => {
    const el = listRef.current;
    return !el || el.scrollHeight - el.scrollTop - el.clientHeight < 80;
  };
  const scrollToEnd = () => {
    const el = listRef.current;
    if (el) el.scrollTop = el.scrollHeight;
    setUnseen(false);
  };

  const close = useCallback(() => {
    setClosing(true);
    const reduced = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    setTimeout(() => {
      setOpen(false);
      setClosing(false);
      launcherRef.current?.focus();
    }, reduced ? 0 : 160);
  }, []);

  const ask = useCallback(async (text) => {
    const question = text.trim();
    if (!question) return;
    lastQuestion.current = question;
    setError(null);
    setMessages((m) => [...m, {id: Date.now(), role: 'user', text: question}]);
    setInput('');
    setPending(true);
    requestAnimationFrame(scrollToEnd);
    const controller = new AbortController();
    abortRef.current = controller;
    try {
      const res = await chatApi.sendMessage(question, conversationId.current, {signal: controller.signal});
      conversationId.current = res.conversation_id || conversationId.current;
      const stick = nearBottom();
      setMessages((m) => [...m, {id: Date.now() + 1, role: 'bot', text: res.response || '', sources: uniqueSources(res.sources)}]);
      if (stick) requestAnimationFrame(scrollToEnd);
      else setUnseen(true);
    } catch (err) {
      if (err.name !== 'AbortError') setError(errorText(err));
    } finally {
      setPending(false);
      abortRef.current = null;
    }
  }, []);

  // open from anywhere: openAssistant(prompt?)
  useEffect(() => {
    const onOpen = (e) => {
      setOpen(true);
      const prompt = e.detail?.prompt;
      if (prompt) ask(prompt);
    };
    window.addEventListener(OPEN_EVENT, onOpen);
    return () => window.removeEventListener(OPEN_EVENT, onOpen);
  }, [ask]);

  useEffect(() => {
    if (!open) return undefined;
    inputRef.current?.focus({preventScroll: true});
    const onKey = (e) => {
      if (e.key === 'Escape') close();
    };
    document.addEventListener('keydown', onKey);
    // full-screen sheet on phones: keep the page behind it still
    const phone = window.matchMedia('(max-width: 639px)').matches;
    if (phone) document.documentElement.style.overflow = 'hidden';
    return () => {
      document.removeEventListener('keydown', onKey);
      document.documentElement.style.overflow = '';
    };
  }, [open, close]);

  const onKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey && !e.nativeEvent.isComposing) {
      e.preventDefault();
      if (!pending) ask(input);
    }
  };

  const reset = () => {
    abortRef.current?.abort();
    conversationId.current = null;
    setMessages([]);
    setError(null);
    setInput('');
    inputRef.current?.focus();
  };

  return (
    <div className={styles.root}>
      {open && (
        <section
          className={styles.panel}
          data-closing={closing || undefined}
          role="dialog"
          aria-modal="false"
          aria-labelledby={titleId}>
          <header className={styles.header}>
            <span className={styles.avatar} aria-hidden="true" />
            <div className={styles.headText}>
              <h2 id={titleId} className={styles.title}>Textbook assistant</h2>
              <p className={styles.status}>Answers from the chapters, with sources</p>
            </div>
            {messages.length > 0 && (
              <button type="button" className="pa-icon-button" onClick={reset} aria-label="New chat" title="New chat">
                {Icon.reset}
              </button>
            )}
            <button type="button" className="pa-icon-button" onClick={close} aria-label="Close chat" title="Close (Esc)">
              {Icon.close}
            </button>
          </header>

          <div ref={listRef} className={styles.log} role="log" aria-live="polite" aria-relevant="additions">
            {messages.length === 0 && !pending && (
              <div className={styles.empty}>
                <p className={styles.emptyTitle}>Ask anything in the book.</p>
                <p className={styles.emptyText}>Answers link back to the lessons they came from.</p>
                <div className={styles.suggestions}>
                  {SUGGESTIONS.map((s) => (
                    <button key={s} type="button" className={styles.suggestion} onClick={() => ask(s)}>
                      {s}
                    </button>
                  ))}
                </div>
              </div>
            )}

            {messages.map((m) =>
              m.role === 'user' ? (
                <p key={m.id} className={styles.user}>{m.text}</p>
              ) : (
                <div key={m.id} className={styles.bot}>
                  <div className={styles.markdown}>
                    <ReactMarkdown>{m.text}</ReactMarkdown>
                  </div>
                  {m.sources?.length > 0 && (
                    <ul className={styles.sources} aria-label="Sources">
                      {m.sources.map((s) => (
                        <li key={s.to}>
                          <Link to={s.to} className={styles.source} onClick={() => window.matchMedia('(max-width: 639px)').matches && close()}>
                            {s.label}
                          </Link>
                        </li>
                      ))}
                    </ul>
                  )}
                </div>
              ),
            )}

            {pending && (
              <div className={styles.thinking} aria-label="Assistant is answering">
                <span /><span /><span />
              </div>
            )}

            {error && (
              <div className={styles.error} role="alert">
                <p>{error}</p>
                <button type="button" className={styles.retry} onClick={() => {
                  setMessages((m) => (m[m.length - 1]?.role === 'user' ? m.slice(0, -1) : m));
                  ask(lastQuestion.current);
                }}>
                  Retry
                </button>
              </div>
            )}
          </div>

          {unseen && (
            <button type="button" className={styles.newPill} onClick={scrollToEnd}>
              New answer
            </button>
          )}

          <form
            className={styles.composer}
            onSubmit={(e) => {
              e.preventDefault();
              if (!pending) ask(input);
            }}>
            <label htmlFor={`${titleId}-input`} className={styles.srOnly}>
              Your question
            </label>
            <textarea
              id={`${titleId}-input`}
              ref={inputRef}
              name="question"
              autoComplete="off"
              className={styles.input}
              value={input}
              rows={1}
              maxLength={2000}
              placeholder="Ask about any lesson…"
              onChange={(e) => setInput(e.target.value)}
              onKeyDown={onKeyDown}
            />
            {pending ? (
              <button type="button" className={styles.send} onClick={() => abortRef.current?.abort()} aria-label="Stop answering">
                {Icon.stop}
              </button>
            ) : (
              <button type="submit" className={styles.send} disabled={!input.trim()} aria-label="Send message">
                {Icon.send}
              </button>
            )}
          </form>
        </section>
      )}

      <button
        ref={launcherRef}
        type="button"
        className={styles.launcher}
        data-hidden={open || undefined}
        aria-label="Open chat"
        aria-expanded={open}
        onClick={() => setOpen(true)}>
        {Icon.chat}
        <span className={styles.launcherLabel}>Ask the book</span>
      </button>
    </div>
  );
}
