import React, {Suspense, lazy, useCallback, useEffect, useId, useLayoutEffect, useMemo, useRef, useState} from 'react';
import {createPortal} from 'react-dom';
import Link from '@docusaurus/Link';
import {useLocation} from '@docusaurus/router';
import {openSignIn, useAuth} from '@site/src/hooks/useAuth';
import {streamChat} from '@site/src/utils/api';
import {OPEN_EVENT} from './events';
import styles from './styles.module.css';

const Markdown = lazy(() => import('./Markdown')); // react-markdown + GFM + KaTeX: only once there is an answer

export const ASSISTANT = 'Rivet';
const STORE = 'proprio:rivet'; // pre-rename key, kept on purpose
const HISTORY_TURNS = 8;
const MAX_INPUT = 4000;
const PHONE = '(max-width: 639px)';

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
  return `Couldn’t reach ${ASSISTANT}. Check your connection, then retry.`;
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

const isMac = () => typeof navigator !== 'undefined' && /Mac|iPhone|iPad/.test(navigator.platform || navigator.userAgent);

// Rivet's mark: a sensor head with three "eyes". While Rivet works, the eyes scan in turn.
export function RivetMark({size = 20, busy = false, className}) {
  return (
    <svg className={className} width={size} height={size} viewBox="0 0 24 24" fill="none" aria-hidden="true" data-busy={busy || undefined}>
      <rect x="2.75" y="2.75" width="18.5" height="18.5" rx="6" stroke="currentColor" strokeWidth="1.6" />
      <circle className={styles.eye} cx="12" cy="8.6" r="2" fill="var(--rivet-eye, currentColor)" />
      <circle className={styles.eye} cx="8" cy="14.6" r="1.6" fill="currentColor" />
      <circle className={styles.eye} cx="16" cy="14.6" r="1.6" fill="currentColor" />
    </svg>
  );
}

const svg = (path, size = 16, extra = {}) => (
  <svg width={size} height={size} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true" {...extra}>
    {path}
  </svg>
);
const Icon = {
  close: svg(<path d="M6 6l12 12M18 6L6 18" />, 18),
  plus: svg(<path d="M12 5v14M5 12h14" />, 17),
  expand: svg(<path d="M14 4h6v6M10 20H4v-6M20 4l-7 7M4 20l7-7" />, 16),
  shrink: svg(<path d="M4 14h6v6M20 10h-6V4M14 10l7-7M10 14l-7 7" />, 16),
  send: svg(<path d="M12 19V5M5 12l7-7 7 7" />, 18, {strokeWidth: 2.3}),
  stop: (
    <svg width="12" height="12" viewBox="0 0 24 24" aria-hidden="true">
      <rect x="3" y="3" width="18" height="18" rx="4" fill="currentColor" />
    </svg>
  ),
  copy: svg(
    <>
      <rect x="9" y="9" width="11" height="11" rx="2.5" />
      <path d="M5 15V6.5A2.5 2.5 0 0 1 7.5 4H15" />
    </>,
    15,
  ),
  check: svg(<path d="M5 12.5l4.5 4.5L19 7.5" />, 15, {strokeWidth: 2.4}),
  redo: svg(
    <>
      <path d="M20 11a8 8 0 1 0-2.3 5.7" />
      <path d="M20 4v7h-7" />
    </>,
    15,
  ),
  down: svg(<path d="M12 5v14M6 13l6 6 6-6" />, 15, {strokeWidth: 2.2}),
  arrow: svg(<path d="M7 17L17 7M9 7h8v8" />, 15),
  alert: svg(
    <>
      <circle cx="12" cy="12" r="9" />
      <path d="M12 7.5v5.5M12 16.5v.01" />
    </>,
    18,
  ),
  doc: svg(
    <>
      <path d="M7 3h7l5 5v13H7z" />
      <path d="M14 3v5h5M10 13h6M10 17h4" />
    </>,
    14,
  ),
};

function CopyButton({text, label = 'Copy', className}) {
  const [done, setDone] = useState(false);
  useEffect(() => {
    if (!done) return undefined;
    const t = setTimeout(() => setDone(false), 1500);
    return () => clearTimeout(t);
  }, [done]);
  return (
    <button
      type="button"
      className={className || styles.action}
      data-done={done || undefined}
      onClick={async () => {
        try {
          await navigator.clipboard.writeText(text);
          setDone(true);
        } catch {
          // clipboard blocked: the text stays selectable
        }
      }}>
      {done ? Icon.check : Icon.copy}
      <span aria-live="polite">{done ? 'Copied' : label}</span>
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
          const label = `${src.title}${src.section && src.section !== src.title ? ` · ${src.section}` : ''}`;
          return (
            <Link to={src.url} className={styles.cite} data-n={cite[1]} title={label} aria-label={`Source ${cite[1]}: ${label}`} onClick={onNavigate}>
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
        const lang = /language-([\w+-]+)/.exec(code?.props?.className || '')?.[1];
        return (
          <div className={styles.codeBlock}>
            <div className={styles.codeHead}>
              <span>{lang || 'code'}</span>
              <CopyButton text={raw} className={styles.codeCopy} />
            </div>
            <pre>{children}</pre>
          </div>
        );
      },
      table({children}) {
        return (
          <div className={styles.tableWrap} tabIndex={0} role="region" aria-label="Table">
            <table>{children}</table>
          </div>
        );
      },
    }),
    [byN, onNavigate],
  );
  return (
    <div className={styles.markdown} data-streaming={message.status === 'streaming' || undefined}>
      <Suspense fallback={<p className={styles.plain}>{message.text}</p>}>
        <Markdown components={components}>{text}</Markdown>
      </Suspense>
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
      const below = r.top < 130; // no room above (under the navbar): show it under the selection
      return setSpot({
        text,
        below,
        x: Math.min(Math.max(r.left + r.width / 2, 80), window.innerWidth - 80),
        y: below ? r.bottom + 10 : r.top - 10,
      });
    };
    const hide = () => setSpot(null);
    const onUp = () => setTimeout(update, 0);
    document.addEventListener('mouseup', onUp);
    document.addEventListener('keyup', onUp);
    document.addEventListener('touchend', onUp);
    window.addEventListener('scroll', hide, {passive: true});
    return () => {
      document.removeEventListener('mouseup', onUp);
      document.removeEventListener('keyup', onUp);
      document.removeEventListener('touchend', onUp);
      window.removeEventListener('scroll', hide);
    };
  }, []);
  if (!spot) return null;
  return createPortal(
    <button
      type="button"
      className={styles.selectionAsk}
      data-below={spot.below || undefined}
      style={{left: spot.x, top: spot.y}}
      onMouseDown={(e) => e.preventDefault()}
      onClick={() => {
        onAsk(spot.text);
        setSpot(null);
        window.getSelection()?.removeAllRanges();
      }}>
      <RivetMark size={16} className={styles.selectionMark} />
      Ask {ASSISTANT}
    </button>,
    document.body,
  );
}

function Sources({sources, onNavigate}) {
  if (!sources?.length) return null;
  return (
    <div className={styles.sources}>
      <p className={styles.sourcesLabel}>
        {Icon.doc}
        {sources.length === 1 ? '1 source' : `${Math.min(sources.length, 4)} sources`}
      </p>
      <ol className={styles.sourceList}>
        {sources.slice(0, 4).map((s) => (
          <li key={`${s.url}-${s.n}`}>
            <Link to={s.url} className={styles.source} data-n={s.n} onClick={onNavigate}>
              <span className={styles.sourceN}>{s.n}</span>
              <span className={styles.sourceText}>
                <span className={styles.sourceTitle}>{s.title}</span>
                {s.section && s.section !== s.title && <span className={styles.sourceSection}>{s.section}</span>}
              </span>
              <span className={styles.sourceGo}>{Icon.arrow}</span>
            </Link>
          </li>
        ))}
      </ol>
    </div>
  );
}

export default function ChatWidget() {
  const page = usePageContext();
  const {user, loading: authLoading, getToken, logout} = useAuth();
  const signedIn = Boolean(user);
  const showWall = !authLoading && !signedIn; // Rivet answers signed-in learners only
  const [waiting, setWaiting] = useState(''); // a question asked while signed out, sent after sign-in
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
  const panelRef = useRef(null);
  const listRef = useRef(null);
  const inputRef = useRef(null);
  const launcherRef = useRef(null);
  const stickRef = useRef(true);
  const returnFocus = useRef(false);
  const titleId = useId();
  const shortcut = useMemo(() => (isMac() ? '⌘ /' : 'Ctrl /'), []);

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
  }, [messages, pending, error, scrollToEnd]);

  const onScroll = () => {
    const el = listRef.current;
    const bottom = el.scrollHeight - el.scrollTop - el.clientHeight < 48;
    stickRef.current = bottom;
    setAtBottom(bottom);
  };

  // the composer grows with its text; field-sizing does it natively, this covers Firefox and older Safari
  useLayoutEffect(() => {
    const el = inputRef.current;
    if (!el || (typeof CSS !== 'undefined' && CSS.supports?.('field-sizing', 'content'))) return;
    el.style.height = 'auto';
    el.style.height = `${Math.min(el.scrollHeight, 168)}px`;
  }, [input, open]);

  const close = useCallback(() => {
    abortRef.current?.abort();
    setClosing(true);
    const reduced = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    setTimeout(() => {
      returnFocus.current = true;
      setOpen(false);
      setClosing(false);
    }, reduced ? 0 : 170);
  }, []);

  // focus goes back to the launcher after it renders visible again (a hidden button can't take focus)
  useEffect(() => {
    if (!open && returnFocus.current) {
      returnFocus.current = false;
      launcherRef.current?.focus();
    }
  }, [open]);

  const ask = useCallback(
    async (text) => {
      const question = text.trim();
      if (!question || abortRef.current) return;
      if (!signedIn) {
        setWaiting(question);
        setOpen(true);
        return;
      }
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
          token: await getToken(),
          signal: controller.signal,
          onMeta: ({conversationId: id, sources}) => {
            if (id) setStore((s) => ({...s, conversationId: id}));
            patch((m) => ({...m, sources}));
          },
          onToken: (t) => patch((m) => ({...m, text: m.text + t})),
        });
        patch((m) => ({...m, status: 'done'}));
      } catch (err) {
        if (err.name === 'AbortError') patch((m) => ({...m, status: 'done', stopped: true, text: m.text || '_Stopped._'}));
        else if (err.status === 401) {
          // the session expired: keep the question, show the sign-in wall
          setStore((s) => ({...s, messages: s.messages.slice(0, -2)}));
          setWaiting(question);
          logout();
        } else {
          setError(errorText(err));
          setStore((s) => ({...s, messages: s.messages.filter((m) => m.id !== answerId || m.text)}));
          patch((m) => ({...m, status: 'done'}));
        }
      } finally {
        abortRef.current = null;
        setPending(false);
      }
    },
    [messages, conversationId, page.isLesson, page.title, page.path, signedIn, getToken, logout],
  );

  // after signing in, send the question that was waiting
  useEffect(() => {
    if (signedIn && open && waiting) {
      setWaiting('');
      setTimeout(() => askRef.current(waiting), 0);
    }
  }, [signedIn, open, waiting]);

  // a different learner (or nobody) on this tab must not see the previous conversation
  const userKey = user?.id ?? user?.email;
  const lastUser = useRef(userKey);
  useEffect(() => {
    if (authLoading) return;
    if (lastUser.current && lastUser.current !== userKey) {
      abortRef.current?.abort();
      setStore({messages: [], conversationId: null});
      setError(null);
    }
    lastUser.current = userKey;
  }, [userKey, authLoading]);

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

  // Ctrl/⌘ + / opens and closes Rivet from any page
  const openRef = useRef(open);
  openRef.current = open;
  useEffect(() => {
    const onKey = (e) => {
      if (e.key === '/' && (e.metaKey || e.ctrlKey) && !e.altKey) {
        e.preventDefault();
        if (openRef.current) close();
        else setOpen(true);
      }
    };
    document.addEventListener('keydown', onKey);
    return () => document.removeEventListener('keydown', onKey);
  }, [close]);

  useEffect(() => {
    if (!open) return undefined;
    inputRef.current?.focus({preventScroll: true});
    requestAnimationFrame(() => scrollToEnd(false));
    const onKey = (e) => {
      if (e.key === 'Escape' && !e.defaultPrevented) close();
    };
    document.addEventListener('keydown', onKey);
    const phone = window.matchMedia(PHONE).matches;
    // phones: the sheet follows the visual viewport, so the composer stays above the on-screen keyboard
    const vv = window.visualViewport;
    const fit = () => {
      const el = panelRef.current;
      if (!el || !vv) return;
      el.style.setProperty('--vv-height', `${vv.height}px`);
      el.style.setProperty('--vv-top', `${vv.offsetTop}px`);
    };
    if (phone) {
      document.documentElement.style.overflow = 'hidden';
      fit();
      vv?.addEventListener('resize', fit);
      vv?.addEventListener('scroll', fit);
    }
    return () => {
      document.removeEventListener('keydown', onKey);
      document.documentElement.style.overflow = '';
      vv?.removeEventListener('resize', fit);
      vv?.removeEventListener('scroll', fit);
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

  // ask the last question again: drop the last answer (and its question) first
  const rerun = (question) => {
    setError(null);
    setStore((s) => {
      const i = s.messages.map((m) => m.role).lastIndexOf('user');
      return i === -1 ? s : {...s, messages: s.messages.slice(0, i)};
    });
    setTimeout(() => askRef.current(question), 0);
  };

  const navigateFromChat = useCallback(() => {
    if (window.matchMedia(PHONE).matches) close();
  }, [close]);

  const suggestions = page.isLesson ? ON_LESSON : GENERAL;
  const lastAnswerId = [...messages].reverse().find((m) => m.role === 'assistant')?.id;
  const nearLimit = input.length > MAX_INPUT - 400;

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
          ref={panelRef}
          className={styles.panel}
          data-closing={closing || undefined}
          data-wide={wide || undefined}
          data-lenis-prevent
          role="dialog"
          aria-modal="false"
          aria-labelledby={titleId}>
          <header className={styles.header}>
            <span className={styles.avatar}>
              <RivetMark size={22} busy={pending} />
            </span>
            <div className={styles.headText}>
              <h2 id={titleId} className={styles.title}>
                {ASSISTANT}
              </h2>
              <p className={styles.status}>
                <span className={styles.statusDot} data-busy={pending || undefined} aria-hidden="true" />
                {pending ? 'Answering…' : page.isLesson && page.title ? `Reading along · ${page.title}` : 'Your Kinemata course tutor'}
              </p>
            </div>
            {messages.length > 0 && (
              <button type="button" className={styles.headButton} onClick={reset} aria-label="New chat" title="New chat">
                {Icon.plus}
              </button>
            )}
            <button
              type="button"
              className={`${styles.headButton} ${styles.wideToggle}`}
              onClick={() => setWide((w) => !w)}
              aria-label={wide ? 'Make chat smaller' : 'Make chat larger'}
              aria-pressed={wide}
              title={wide ? 'Smaller' : 'Larger'}>
              {wide ? Icon.shrink : Icon.expand}
            </button>
            <button type="button" className={styles.headButton} onClick={close} aria-label="Close chat" title={`Close (Esc · ${shortcut})`}>
              {Icon.close}
            </button>
          </header>

          <div
            ref={listRef}
            className={styles.log}
            role="log"
            aria-live="polite"
            aria-relevant="additions"
            aria-busy={pending}
            onScroll={onScroll}
            data-lenis-prevent>
            {showWall && (
              <div className={styles.empty}>
                <span className={styles.emptyMark}>
                  <RivetMark size={30} />
                </span>
                <p className={styles.emptyTitle}>
                  Hi, I’m {ASSISTANT}. <em>Sign in to start.</em>
                </p>
                <p className={styles.emptyText}>
                  {ASSISTANT} is free with a Kinemata account. Signing in also keeps your progress and quiz scores on every
                  device.
                </p>
                {waiting && (
                  <p className={styles.context}>
                    {Icon.doc}
                    <span>
                      I’ll answer <strong>“{waiting.length > 90 ? `${waiting.slice(0, 89)}…` : waiting}”</strong> as soon as
                      you’re in.
                    </span>
                  </p>
                )}
                <div className={styles.wallActions}>
                  <button type="button" className="button button--primary" onClick={() => openSignIn('register')}>
                    Create free account
                  </button>
                  <button type="button" className="button button--secondary" onClick={() => openSignIn('login')}>
                    Sign in
                  </button>
                </div>
              </div>
            )}

            {!showWall && messages.length === 0 && (
              <div className={styles.empty}>
                <span className={styles.emptyMark}>
                  <RivetMark size={30} />
                </span>
                <p className={styles.emptyTitle}>
                  Hi, I’m {ASSISTANT}. <em>Ask me anything.</em>
                </p>
                <p className={styles.emptyText}>
                  I’ve read every lesson in Kinemata, and I link each answer to the sections I used.
                </p>
                {page.isLesson && page.title && (
                  <p className={styles.context}>
                    {Icon.doc}
                    <span>
                      Reading along with <strong>{page.title}</strong>
                    </span>
                  </p>
                )}
                <p className={styles.suggestLabel}>Try asking</p>
                <ul className={styles.suggestions}>
                  {suggestions.map((s) => (
                    <li key={s}>
                      <button type="button" className={styles.suggestion} onClick={() => ask(s)}>
                        <span>{s}</span>
                        {Icon.arrow}
                      </button>
                    </li>
                  ))}
                </ul>
                <p className={styles.tip}>Tip: select any text in a lesson, then press “Ask {ASSISTANT}”.</p>
              </div>
            )}

            {!showWall && messages.map((m) =>
              m.role === 'user' ? (
                <div key={m.id} className={styles.userRow}>
                  <p className={styles.user}>{m.text}</p>
                </div>
              ) : (
                <article key={m.id} className={styles.bot} aria-label={`${ASSISTANT}’s answer`}>
                  <p className={styles.botLabel}>
                    <RivetMark size={16} busy={m.status === 'streaming'} className={styles.botMark} />
                    {ASSISTANT}
                  </p>
                  {m.text ? (
                    <Answer message={m} onNavigate={navigateFromChat} />
                  ) : (
                    <p className={styles.thinking} role="status">
                      <span className={styles.shimmer}>Searching the lessons…</span>
                    </p>
                  )}
                  {m.status !== 'streaming' && m.text && (
                    <>
                      <Sources sources={m.sources} onNavigate={navigateFromChat} />
                      <div className={styles.actions}>
                        <CopyButton text={m.text} label="Copy" />
                        {m.id === lastAnswerId && !pending && lastQuestion.current && (
                          <button type="button" className={styles.action} onClick={() => rerun(lastQuestion.current)}>
                            {Icon.redo}
                            <span>{m.stopped ? 'Answer again' : 'Regenerate'}</span>
                          </button>
                        )}
                      </div>
                    </>
                  )}
                </article>
              ),
            )}

            {error && (
              <div className={styles.error} role="alert">
                <span className={styles.errorIcon}>{Icon.alert}</span>
                <p>{error}</p>
                <button
                  type="button"
                  className={styles.retry}
                  onClick={() => {
                    setError(null);
                    setStore((s) => {
                      const last = s.messages[s.messages.length - 1];
                      return last?.role === 'user' ? {...s, messages: s.messages.slice(0, -1)} : s;
                    });
                    setTimeout(() => askRef.current(lastQuestion.current), 0);
                  }}>
                  {Icon.redo}
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
              }}>
              {Icon.down}
              Latest
            </button>
          )}

          {!showWall && (
            <div className={styles.dock}>
              <form
                className={styles.composer}
                data-busy={pending || undefined}
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
                  maxLength={MAX_INPUT}
                  enterKeyHint="send"
                  placeholder={page.isLesson ? 'Ask about this lesson…' : `Ask ${ASSISTANT} anything…`}
                  onChange={(e) => setInput(e.target.value)}
                  onKeyDown={onKeyDown}
                />
                {pending ? (
                  <button type="button" className={styles.stop} onClick={() => abortRef.current?.abort()} aria-label="Stop answering" title="Stop">
                    {Icon.stop}
                  </button>
                ) : (
                  <button type="submit" className={styles.send} disabled={!input.trim()} aria-label="Send message" title="Send (Enter)">
                    {Icon.send}
                  </button>
                )}
              </form>
              <p className={styles.footnote}>
                {nearLimit ? (
                  <span className={styles.count} data-full={input.length >= MAX_INPUT || undefined}>
                    {input.length.toLocaleString()} / {MAX_INPUT.toLocaleString()}
                  </span>
                ) : (
                  <span className={styles.disclaimer}>{ASSISTANT} can be wrong. Check the lessons it links.</span>
                )}
                <span className={styles.keys}>
                  <kbd className="pa-kbd">Enter</kbd> send · <kbd className="pa-kbd">Shift</kbd>+<kbd className="pa-kbd">Enter</kbd> new line
                </span>
              </p>
            </div>
          )}
        </section>
      )}

      <button
        ref={launcherRef}
        type="button"
        className={styles.launcher}
        data-hidden={open || undefined}
        aria-label={`Ask ${ASSISTANT}, the course tutor`}
        aria-expanded={open}
        aria-keyshortcuts={isMac() ? 'Meta+/' : 'Control+/'}
        onClick={() => setOpen(true)}>
        <RivetMark size={22} className={styles.launcherMark} />
        <span className={styles.launcherLabel}>Ask {ASSISTANT}</span>
        <kbd className={styles.launcherKey}>{shortcut}</kbd>
      </button>
    </div>
  );
}
