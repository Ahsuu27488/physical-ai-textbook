import React, {useCallback, useEffect, useId, useMemo, useRef, useState} from 'react';
import {createPortal} from 'react-dom';
import Link from '@docusaurus/Link';
import {useLocation} from '@docusaurus/router';
import ReactMarkdown from 'react-markdown';
import remarkMath from 'remark-math';
import rehypeKatex from 'rehype-katex';
import {streamChat} from '@site/src/utils/api';
import {OPEN_EVENT} from './events';
import styles from './styles.module.css';

export const ASSISTANT = 'Rivet';
const STORE = 'proprio:rivet';
const HISTORY_TURNS = 8;

const GENERAL = [
  'Where should a complete beginner start?',
  'What is ROS 2, and why do robots need middleware?',
  'How do humanoid robots keep their balance?',
  'What is a vision-language-action model?',
];
const ON_LESSON = [
  'Summarize this lesson in 5 bullet points',
  'Quiz me on this lesson',
  'Explain the hardest idea on this page simply',
  'What should I know before this lesson?',
];

function errorText(err) {
  if (err?.status === 503) return `${ASSISTANT} isn’t switched on at the server right now.`;
  if (err?.status === 429) return 'You’re asking faster than the server allows. Wait a few seconds, then retry.';
  if (typeof navigator !== 'undefined' && !navigator.onLine) return 'You’re offline. Reconnect, then retry.';
  return `Couldn’t reach ${ASSISTANT}.`;
}

// The page the learner is reading, so answers can lean on it.
function usePageContext() {
  const {pathname} = useLocation();
  const [title, setTitle] = useState('');
  useEffect(() => {
    const t = setTimeout(() => {
      const h1 = document.querySelector('article h1, main h1');
      setTitle(h1?.textContent?.trim() || document.title.split('|')[0].trim());
    }, 50);
    return () => clearTimeout(t);
  }, [pathname]);
  const isLesson = /\/docs\/.+/.test(pathname);
  return {path: pathname, title, isLesson};
}

function loadStore() {
  try {
    return JSON.parse(sessionStorage.getItem(STORE) || 'null') || {messages: [], conversationId: null};
  } catch {
    return {messages: [], conversationId: null};
  }
}

const Icon = {
  close: (
    <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" aria-hidden="true">
      <path d="M6 6l12 12M18 6L6 18" />
    </svg>
  ),
  reset: (
    <svg width="17" height="17" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      <path d="M12 5v14M5 12h14" />
    </svg>
  ),
  expand: (
    <svg width="17" height="17" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      <path d="M14 4h6v6M10 20H4v-6M20 4l-7 7M4 20l7-7" />
    </svg>
  ),
  shrink: (
    <svg width="17" height="17" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      <path d="M4 14h6v6M20 10h-6V4M14 10l7-7M10 14l-7 7" />
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
  copy: (
    <svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      <rect x="9" y="9" width="11" height="11" rx="2.5" />
      <path d="M5 15V6.5A2.5 2.5 0 0 1 7.5 4H15" />
    </svg>
  ),
  down: (
    <svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      <path d="M12 5v14M6 13l6 6 6-6" />
    </svg>
  ),
};

function CopyButton({text, label = 'Copy'}) {
  const [done, setDone] = useState(false);
  return (
    <button
      type="button"
      className={styles.miniButton}
      onClick={async () => {
        try {
          await navigator.clipboard.writeText(text);
          setDone(true);
          setTimeout(() => setDone(false), 1400);
        } catch {
          // clipboard blocked
        }
      }}>
      {Icon.copy}
      {done ? 'Copied' : label}
    </button>
  );
}

// Answer text with [1]-style citations turned into chips that open the cited section.
function Answer({message, onNavigate}) {
  const byN = useMemo(() => new Map((message.sources || []).map((s) => [String(s.n), s])), [message.sources]);
  const text = useMemo(
    () => (message.text || '').replace(/\[(\d{1,2})\](?!\()/g, (m, n) => (byN.has(n) ? `[${n}](#rivet-cite-${n})` : m)),
    [message.text, byN],
  );
  const components = useMemo(
    () => ({
      a({href = '', children}) {
        const cite = href.match(/^#rivet-cite-(\d+)$/);
        if (cite && byN.has(cite[1])) {
          const src = byN.get(cite[1]);
          return (
            <Link to={src.url} className={styles.cite} title={`${src.title}${src.section ? ` · ${src.section}` : ''}`} onClick={onNavigate}>
              {cite[1]}
            </Link>
          );
        }
        if (href.startsWith('/')) {
          return (
            <Link to={href} onClick={onNavigate}>
              {children}
            </Link>
          );
        }
        return (
          <a href={href} target="_blank" rel="noopener noreferrer">
            {children}
          </a>
        );
      },
      pre({children}) {
        const code = React.Children.toArray(children)[0];
        const raw = String(code?.props?.children ?? '').replace(/\n$/, '');
        return (
          <div className={styles.codeBlock}>
            <pre>{children}</pre>
            <CopyButton text={raw} />
          </div>
        );
      },
    }),
    [byN, onNavigate],
  );
  return (
    <div className={styles.markdown}>
      <ReactMarkdown remarkPlugins={[remarkMath]} rehypePlugins={[rehypeKatex]} components={components}>
        {text}
      </ReactMarkdown>
      {message.status === 'streaming' && <span className={styles.caret} aria-hidden="true" />}
    </div>
  );
}

// "Ask Rivet" bubble for text selected inside a lesson.
function SelectionAsk({onAsk}) {
  const [spot, setSpot] = useState(null);
  useEffect(() => {
    const update = () => {
      const sel = window.getSelection();
      const text = sel?.toString().trim() || '';
      if (!sel || sel.rangeCount === 0 || text.length < 3 || text.length > 700) return setSpot(null);
      const node = sel.anchorNode?.nodeType === 1 ? sel.anchorNode : sel.anchorNode?.parentElement;
      if (!node?.closest?.('article .markdown')) return setSpot(null);
      const r = sel.getRangeAt(0).getBoundingClientRect();
      return setSpot({text, x: Math.min(Math.max(r.left + r.width / 2, 70), window.innerWidth - 70), y: Math.max(r.top - 10, 72)});
    };
    const hide = () => setSpot(null);
    const onUp = () => setTimeout(update, 0);
    document.addEventListener('mouseup', onUp);
    document.addEventListener('keyup', onUp);
    window.addEventListener('scroll', hide, {passive: true});
    return () => {
      document.removeEventListener('mouseup', onUp);
      document.removeEventListener('keyup', onUp);
      window.removeEventListener('scroll', hide);
    };
  }, []);
  if (!spot) return null;
  return createPortal(
    <button
      type="button"
      className={styles.selectionAsk}
      style={{left: spot.x, top: spot.y}}
      onMouseDown={(e) => e.preventDefault()}
      onClick={() => {
        onAsk(spot.text);
        setSpot(null);
        window.getSelection()?.removeAllRanges();
      }}>
      <span className={styles.selectionDot} aria-hidden="true" />
      Ask {ASSISTANT}
    </button>,
    document.body,
  );
}

export default function ChatWidget() {
  const page = usePageContext();
  const [open, setOpen] = useState(false);
  const [closing, setClosing] = useState(false);
  const [wide, setWide] = useState(false);
  const [{messages, conversationId}, setStore] = useState(loadStore);
  const [input, setInput] = useState('');
  const [pending, setPending] = useState(false);
  const [error, setError] = useState(null);
  const [atBottom, setAtBottom] = useState(true);
  const abortRef = useRef(null);
  const lastQuestion = useRef('');
  const listRef = useRef(null);
  const inputRef = useRef(null);
  const launcherRef = useRef(null);
  const stickRef = useRef(true);
  const titleId = useId();

  useEffect(() => {
    try {
      sessionStorage.setItem(STORE, JSON.stringify({messages: messages.filter((m) => m.status !== 'streaming'), conversationId}));
    } catch {
      // storage blocked: the conversation lasts until reload
    }
  }, [messages, conversationId]);

  const scrollToEnd = useCallback((smooth) => {
    const el = listRef.current;
    if (el) el.scrollTo({top: el.scrollHeight, behavior: smooth ? 'smooth' : 'auto'});
  }, []);

  // keep the newest text in view while streaming, unless the learner scrolled up to read
  useEffect(() => {
    if (stickRef.current) scrollToEnd(false);
  }, [messages, pending, scrollToEnd]);

  const onScroll = () => {
    const el = listRef.current;
    const bottom = el.scrollHeight - el.scrollTop - el.clientHeight < 48;
    stickRef.current = bottom;
    setAtBottom(bottom);
  };

  const close = useCallback(() => {
    abortRef.current?.abort();
    setClosing(true);
    const reduced = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    setTimeout(() => {
      setOpen(false);
      setClosing(false);
      launcherRef.current?.focus();
    }, reduced ? 0 : 160);
  }, []);

  const ask = useCallback(
    async (text) => {
      const question = text.trim();
      if (!question || abortRef.current) return;
      lastQuestion.current = question;
      setError(null);
      setInput('');
      stickRef.current = true;
      const history = messages
        .filter((m) => m.text && m.status !== 'error')
        .slice(-HISTORY_TURNS)
        .map((m) => ({role: m.role, content: m.text}));
      const answerId = `a${Date.now()}`;
      setStore((s) => ({
        ...s,
        messages: [
          ...s.messages,
          {id: `u${Date.now()}`, role: 'user', text: question},
          {id: answerId, role: 'assistant', text: '', sources: [], status: 'streaming'},
        ],
      }));
      setPending(true);
      const patch = (fn) => setStore((s) => ({...s, messages: s.messages.map((m) => (m.id === answerId ? fn(m) : m))}));
      const controller = new AbortController();
      abortRef.current = controller;
      try {
        await streamChat({
          message: question,
          history,
          page: page.isLesson ? {title: page.title, path: page.path} : undefined,
          conversationId,
          signal: controller.signal,
          onMeta: ({conversationId: id, sources}) => {
            if (id) setStore((s) => ({...s, conversationId: id}));
            patch((m) => ({...m, sources}));
          },
          onToken: (t) => patch((m) => ({...m, text: m.text + t})),
        });
        patch((m) => ({...m, status: 'done'}));
      } catch (err) {
        if (err.name === 'AbortError') patch((m) => ({...m, status: 'done', text: m.text || '_Stopped._'}));
        else {
          setError(errorText(err));
          setStore((s) => ({...s, messages: s.messages.filter((m) => m.id !== answerId || m.text)}));
          patch((m) => ({...m, status: 'done'}));
        }
      } finally {
        abortRef.current = null;
        setPending(false);
      }
    },
    [messages, conversationId, page.isLesson, page.title, page.path],
  );

  // open from anywhere: openAssistant(prompt?)
  const askRef = useRef(ask);
  askRef.current = ask;
  useEffect(() => {
    const onOpen = (e) => {
      setOpen(true);
      const prompt = e.detail?.prompt;
      if (prompt) setTimeout(() => askRef.current(prompt), 0);
    };
    window.addEventListener(OPEN_EVENT, onOpen);
    return () => window.removeEventListener(OPEN_EVENT, onOpen);
  }, []);

  useEffect(() => {
    if (!open) return undefined;
    inputRef.current?.focus({preventScroll: true});
    requestAnimationFrame(() => scrollToEnd(false));
    const onKey = (e) => {
      if (e.key === 'Escape') close();
    };
    document.addEventListener('keydown', onKey);
    const phone = window.matchMedia('(max-width: 639px)').matches;
    if (phone) document.documentElement.style.overflow = 'hidden';
    return () => {
      document.removeEventListener('keydown', onKey);
      document.documentElement.style.overflow = '';
    };
  }, [open, close, scrollToEnd]);

  const onKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey && !e.nativeEvent.isComposing) {
      e.preventDefault();
      if (!pending) ask(input);
    }
  };

  const reset = () => {
    abortRef.current?.abort();
    setStore({messages: [], conversationId: null});
    setError(null);
    setInput('');
    inputRef.current?.focus();
  };

  const navigateFromChat = useCallback(() => {
    if (window.matchMedia('(max-width: 639px)').matches) close();
  }, [close]);

  const suggestions = page.isLesson ? ON_LESSON : GENERAL;

  return (
    <div className={styles.root}>
      <SelectionAsk
        onAsk={(text) => {
          setOpen(true);
          setTimeout(() => askRef.current(`Explain this passage from “${page.title}”:\n\n> ${text.replace(/\n+/g, ' ')}`), 0);
        }}
      />

      {open && (
        <section
          className={styles.panel}
          data-closing={closing || undefined}
          data-wide={wide || undefined}
          data-lenis-prevent
          role="dialog"
          aria-modal="false"
          aria-labelledby={titleId}>
          <header className={styles.header}>
            <span className={styles.avatar} aria-hidden="true" />
            <div className={styles.headText}>
              <h2 id={titleId} className={styles.title}>
                {ASSISTANT}
              </h2>
              <p className={styles.status}>
                <span className={styles.statusDot} aria-hidden="true" />
                {page.isLesson && page.title ? `Reading along: ${page.title}` : 'Your Proprio course tutor'}
              </p>
            </div>
            {messages.length > 0 && (
              <button type="button" className="pa-icon-button" onClick={reset} aria-label="New chat" title="New chat">
                {Icon.reset}
              </button>
            )}
            <button
              type="button"
              className={`pa-icon-button ${styles.wideToggle}`}
              onClick={() => setWide((w) => !w)}
              aria-label={wide ? 'Make chat smaller' : 'Make chat larger'}
              aria-pressed={wide}
              title={wide ? 'Smaller' : 'Larger'}>
              {wide ? Icon.shrink : Icon.expand}
            </button>
            <button type="button" className="pa-icon-button" onClick={close} aria-label="Close chat" title="Close (Esc)">
              {Icon.close}
            </button>
          </header>

          <div ref={listRef} className={styles.log} role="log" aria-live="polite" aria-relevant="additions" onScroll={onScroll} data-lenis-prevent>
            {messages.length === 0 && (
              <div className={styles.empty}>
                <span className={styles.emptyMark} aria-hidden="true" />
                <p className={styles.emptyTitle}>Hi, I’m {ASSISTANT}.</p>
                <p className={styles.emptyText}>
                  I’ve read every lesson in Proprio. Ask me anything, select text in a lesson to ask about it, or try one
                  of these:
                </p>
                <div className={styles.suggestions}>
                  {suggestions.map((s) => (
                    <button key={s} type="button" className={styles.suggestion} onClick={() => ask(s)}>
                      {s}
                    </button>
                  ))}
                </div>
              </div>
            )}

            {messages.map((m) =>
              m.role === 'user' ? (
                <div key={m.id} className={styles.userRow}>
                  <p className={styles.user}>{m.text}</p>
                </div>
              ) : (
                <div key={m.id} className={styles.bot}>
                  {m.text ? (
                    <Answer message={m} onNavigate={navigateFromChat} />
                  ) : (
                    <div className={styles.thinking} aria-label={`${ASSISTANT} is thinking`}>
                      <span />
                      <span />
                      <span />
                    </div>
                  )}
                  {m.status !== 'streaming' && m.text && (
                    <div className={styles.answerFoot}>
                      {m.sources?.length > 0 && (
                        <ul className={styles.sources} aria-label="Sources">
                          {m.sources.slice(0, 4).map((s) => (
                            <li key={`${s.url}-${s.n}`}>
                              <Link to={s.url} className={styles.source} onClick={navigateFromChat}>
                                <span className={styles.sourceN}>{s.n}</span>
                                {s.section && s.section !== s.title ? `${s.title} · ${s.section}` : s.title}
                              </Link>
                            </li>
                          ))}
                        </ul>
                      )}
                      <CopyButton text={m.text} label="Copy answer" />
                    </div>
                  )}
                </div>
              ),
            )}

            {error && (
              <div className={styles.error} role="alert">
                <p>{error}</p>
                <button
                  type="button"
                  className={styles.retry}
                  onClick={() => {
                    setStore((s) => {
                      const last = s.messages[s.messages.length - 1];
                      return last?.role === 'user' ? {...s, messages: s.messages.slice(0, -1)} : s;
                    });
                    setTimeout(() => askRef.current(lastQuestion.current), 0);
                  }}>
                  Retry
                </button>
              </div>
            )}
          </div>

          {!atBottom && (
            <button
              type="button"
              className={styles.jump}
              onClick={() => {
                stickRef.current = true;
                scrollToEnd(true);
              }}
              aria-label="Jump to latest">
              {Icon.down}
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
              maxLength={4000}
              placeholder={page.isLesson ? 'Ask about this lesson…' : `Ask ${ASSISTANT} anything…`}
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
          <p className={styles.disclaimer}>{ASSISTANT} can be wrong. Check important details in the lessons.</p>
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
        <span className={styles.launcherMark} aria-hidden="true" />
        <span className={styles.launcherLabel}>Ask {ASSISTANT}</span>
      </button>
    </div>
  );
}
