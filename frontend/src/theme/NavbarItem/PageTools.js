import React, {Suspense, lazy, useCallback, useEffect, useRef, useState} from 'react';
import {createPortal} from 'react-dom';
import clsx from 'clsx';
import {useLocation} from '@docusaurus/router';
import {useNavbarMobileSidebar} from '@docusaurus/theme-common/internal';
import {openSignIn, useAuth} from '@site/src/hooks/useAuth';
import {useDialog} from '@site/src/hooks/useDialog';
import {loadUrduFont} from '@site/src/utils/urduFont';
import styles from './PageTools.module.css';

const ReactMarkdown = lazy(() => import('react-markdown')); // only needed once a result is shown

// [code, English name, name in that language, right-to-left]; same list as backend/app/services/translation.py
const LANGUAGES = [
  ['ur', 'Urdu', 'اردو', true],
  ['hi', 'Hindi', 'हिन्दी'],
  ['bn', 'Bengali', 'বাংলা'],
  ['ar', 'Arabic', 'العربية', true],
  ['fa', 'Persian', 'فارسی', true],
  ['tr', 'Turkish', 'Türkçe'],
  ['zh', 'Chinese', '简体中文'],
  ['ja', 'Japanese', '日本語'],
  ['ko', 'Korean', '한국어'],
  ['id', 'Indonesian', 'Indonesia'],
  ['vi', 'Vietnamese', 'Tiếng Việt'],
  ['ru', 'Russian', 'Русский'],
  ['es', 'Spanish', 'Español'],
  ['pt', 'Portuguese', 'Português'],
  ['fr', 'French', 'Français'],
  ['de', 'German', 'Deutsch'],
  ['it', 'Italian', 'Italiano'],
];
const isRtl = (code) => Boolean(LANGUAGES.find(([c]) => c === code)?.[3]);
const LANG_KEY = 'kinemata:translate-lang';
const MAX_CHARS = 12000;

// The lesson's own text: title + paragraphs and list items of the article, in reading order.
function readPageText() {
  const root = document.querySelector('article .markdown:not([data-inplace])') || document.querySelector('main');
  if (!root) return {text: '', truncated: false, title: ''};
  const parts = [];
  const title = document.querySelector('article h1, main h1')?.textContent.trim() || '';
  if (title) parts.push(title);
  // light Markdown, so the translation keeps headings, lists and code blocks (code is never translated)
  root.querySelectorAll('h2, h3, p, li, pre').forEach((el) => {
    if (el.closest('pre') && el.tagName !== 'PRE') return;
    const t = el.textContent.trim();
    if (!t) return;
    if (el.tagName === 'PRE') parts.push(['```', t, '```'].join('\n'));
    else if (el.tagName === 'H2') parts.push(`## ${t}`);
    else if (el.tagName === 'H3') parts.push(`### ${t}`);
    else if (el.tagName === 'LI') parts.push(`- ${t}`);
    else parts.push(t);
  });
  const all = parts.join('\n\n');
  return {text: all.slice(0, MAX_CHARS), truncated: all.length > MAX_CHARS, title};
}

async function readError(res) {
  try {
    return (await res.json()).detail || `Request failed (${res.status})`;
  } catch {
    return `Request failed (${res.status})`;
  }
}

const svg = (path, size = 16) => (
  <svg width={size} height={size} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
    {path}
  </svg>
);
const Icon = {
  tools: svg(
    <>
      <path d="M4 7h10M18 7h2M4 17h4M12 17h8" />
      <circle cx="16" cy="7" r="2" />
      <circle cx="10" cy="17" r="2" />
    </>,
  ),
  chevron: svg(<path d="M6 9l6 6 6-6" />, 14),
  translate: svg(
    <>
      <path d="M4 5h9M8.5 3v2M6 5c.6 3.2 2.8 5.8 6 7M11 5c-.8 3.7-3.4 6.6-7 8" />
      <path d="M13 21l4-9 4 9M14.5 18h5" />
    </>,
  ),
  person: svg(
    <>
      <circle cx="12" cy="8" r="3.5" />
      <path d="M5 20c.8-3.6 3.6-5.5 7-5.5s6.2 1.9 7 5.5" />
    </>,
  ),
  copy: svg(
    <>
      <rect x="9" y="9" width="11" height="11" rx="2.5" />
      <path d="M5 15V6.5A2.5 2.5 0 0 1 7.5 4H15" />
    </>,
  ),
  check: svg(<path d="M5 12.5l4.5 4.5L19 7.5" />),
  close: svg(<path d="M6 6l12 12M18 6L6 18" />),
  alert: svg(
    <>
      <circle cx="12" cy="12" r="9" />
      <path d="M12 7.5v5.5M12 16.5v.01" />
    </>,
    18,
  ),
  info: svg(
    <>
      <circle cx="12" cy="12" r="9" />
      <path d="M12 11v5.5M12 7.5v.01" />
    </>,
  ),
};

function ResultDialog({result, onClose, onRetry}) {
  const ref = useDialog(onClose);
  const [copied, setCopied] = useState(false);
  const lang = LANGUAGES.find(([c]) => c === result.lang);
  const title = result.error
    ? result.kind === 'translate'
      ? 'Couldn’t translate this page'
      : 'Couldn’t personalize this page'
    : result.kind === 'translate'
      ? `${lang?.[1] || 'Translated'} translation`
      : 'Personalized for you';

  const copy = async () => {
    try {
      await navigator.clipboard.writeText(result.text);
      setCopied(true);
      setTimeout(() => setCopied(false), 1600);
    } catch {
      // clipboard blocked: the text stays selectable
    }
  };

  return (
    <div className="pa-overlay" data-lenis-prevent onPointerDown={(e) => e.target === e.currentTarget && onClose()}>
      <div ref={ref} className="pa-dialog pa-dialog--wide" data-lenis-prevent role="dialog" aria-modal="true" aria-labelledby="pa-tools-result" tabIndex={-1}>
        <div className="pa-dialog__head">
          <span className={clsx('pa-dialog__icon', result.error && 'pa-dialog__icon--danger')}>
            {result.error ? Icon.alert : result.kind === 'translate' ? Icon.translate : Icon.person}
          </span>
          <div className="pa-dialog__heading">
            <h2 id="pa-tools-result" className="pa-dialog__title">
              {title}
            </h2>
            {result.page && <p className="pa-dialog__sub">{result.page}</p>}
          </div>
          <button type="button" className="pa-icon-button" onClick={onClose} aria-label="Close">
            {Icon.close}
          </button>
        </div>
        {result.error ? (
          <>
            <p className="pa-note pa-note--error">
              {Icon.alert}
              <span>{result.error}</span>
            </p>
            <div className={styles.resultActions}>
              <button type="button" className="button button--primary" onClick={onRetry} data-autofocus>
                Try again
              </button>
              <button type="button" className="button button--secondary" onClick={onClose}>
                Close
              </button>
            </div>
          </>
        ) : (
          <>
            {result.truncated && (
              <p className={clsx('pa-note', styles.partial)}>
                {Icon.info}
                <span>This covers the first {MAX_CHARS.toLocaleString()} characters of the page.</span>
              </p>
            )}
            <div
              className={clsx(styles.result, result.lang === 'ur' && 'translate-button-urdu')}
              lang={result.lang || undefined}
              dir={isRtl(result.lang) ? 'rtl' : undefined}
              tabIndex={0}>
              <Suspense fallback={<p style={{whiteSpace: 'pre-wrap'}}>{result.text}</p>}>
                <ReactMarkdown>{result.text}</ReactMarkdown>
              </Suspense>
            </div>
            <div className={styles.resultActions}>
              <button type="button" className="button button--primary" onClick={copy} data-autofocus>
                {copied ? Icon.check : Icon.copy}
                <span aria-live="polite">{copied ? 'Copied' : 'Copy text'}</span>
              </button>
              <button type="button" className="button button--secondary" onClick={onClose}>
                Close
              </button>
            </div>
          </>
        )}
      </div>
    </div>
  );
}

// A successful result replaces the lesson text on the page, with a bar to flip back to the original. The original
// article is only hidden (its quizzes and progress stay intact) and comes back on close or navigation.
const ARTICLE = 'article .markdown:not([data-inplace])'; // the lesson itself, never the shown result

function InPlace({result, onClose}) {
  const [host, setHost] = useState(null);
  const [showing, setShowing] = useState('result'); // 'result' | 'original'
  const [copied, setCopied] = useState(false);
  const originalRef = useRef(null);
  const lang = LANGUAGES.find(([c]) => c === result.lang);
  const label = result.kind === 'translate' ? `${lang?.[1] || 'Translated'} translation` : 'Personalized for you';

  useEffect(() => {
    const original = document.querySelector(ARTICLE);
    if (!original) return undefined;
    originalRef.current = original;
    const el = document.createElement('div');
    el.className = styles.inplaceHost;
    original.parentNode.insertBefore(el, original);
    setHost(el);
    el.scrollIntoView({block: 'start'});
    return () => {
      original.style.display = '';
      el.remove();
    };
  }, []);

  useEffect(() => {
    if (originalRef.current) originalRef.current.style.display = showing === 'result' ? 'none' : '';
  }, [showing, host]);

  const copy = async () => {
    try {
      await navigator.clipboard.writeText(result.text);
      setCopied(true);
      setTimeout(() => setCopied(false), 1600);
    } catch {
      // clipboard blocked: the text stays selectable
    }
  };

  if (!host) return null;
  return createPortal(
    <>
      <div className={styles.inplaceBar} role="region" aria-label={label}>
        <span className={styles.inplaceLabel}>
          {result.kind === 'translate' ? Icon.translate : Icon.person}
          <span>{label}</span>
        </span>
        <div className={clsx('pa-segmented', styles.inplaceToggle)} role="radiogroup" aria-label="Show">
          <button type="button" role="radio" aria-checked={showing === 'original'} onClick={() => setShowing('original')}>
            Original
          </button>
          <button type="button" role="radio" aria-checked={showing === 'result'} onClick={() => setShowing('result')}>
            {result.kind === 'translate' ? lang?.[2] || lang?.[1] || 'Translated' : 'For you'}
          </button>
        </div>
        <span className={styles.inplaceActions}>
          <button type="button" className="pa-icon-button" onClick={copy} aria-label={copied ? 'Copied' : 'Copy text'} title="Copy text">
            {copied ? Icon.check : Icon.copy}
          </button>
          <button type="button" className="pa-icon-button" onClick={onClose} aria-label="Close and show the original" title="Close">
            {Icon.close}
          </button>
        </span>
      </div>
      {showing === 'result' && (
        <div
          data-inplace
          className={clsx('markdown', styles.inplaceText, result.lang === 'ur' && 'translate-button-urdu')}
          lang={result.lang || undefined}
          dir={isRtl(result.lang) ? 'rtl' : undefined}>
          {result.truncated && (
            <p className={clsx('pa-note', styles.partial)}>
              {Icon.info}
              <span>This covers the first {MAX_CHARS.toLocaleString()} characters of the page.</span>
            </p>
          )}
          <Suspense fallback={<p style={{whiteSpace: 'pre-wrap'}}>{result.text}</p>}>
            <ReactMarkdown>{result.text}</ReactMarkdown>
          </Suspense>
        </div>
      )}
    </>,
    host,
  );
}

export default function PageTools({mobile}) {
  const {isAuthenticated, authFetch} = useAuth();
  const location = useLocation();
  const [menuOpen, setMenuOpen] = useState(false);
  const [lang, setLangState] = useState('ur');
  useEffect(() => {
    try {
      const saved = localStorage.getItem(LANG_KEY);
      if (LANGUAGES.some(([c]) => c === saved)) setLangState(saved);
    } catch {
      // storage blocked: default language
    }
  }, []);
  const setLang = useCallback((code) => {
    setLangState(code);
    try {
      localStorage.setItem(LANG_KEY, code);
    } catch {
      // storage blocked
    }
  }, []);
  const [busy, setBusy] = useState(null); // 'translate' | 'personalize' | null
  const [result, setResult] = useState(null); // {kind, text, lang, page, truncated} | {kind, error}
  const wrapRef = useRef(null);
  const triggerRef = useRef(null);
  const langRefs = useRef([]);

  useEffect(() => {
    setMenuOpen(false);
    setResult(null); // a translation belongs to the page it was made from
  }, [location.pathname]);

  // on phones the Tools panel sits in the slide-out menu: close it so the result on the page is visible
  const mobileSidebar = useNavbarMobileSidebar();
  const shownInPlace = Boolean(result?.inPlace && !result.error);
  useEffect(() => {
    if (shownInPlace && mobileSidebar.shown) mobileSidebar.toggle();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [shownInPlace]);

  // close the menu on outside click / Esc
  useEffect(() => {
    if (!menuOpen) return undefined;
    const onDown = (e) => wrapRef.current && !wrapRef.current.contains(e.target) && setMenuOpen(false);
    const onKey = (e) => {
      if (e.key === 'Escape') {
        setMenuOpen(false);
        triggerRef.current?.focus();
      }
    };
    document.addEventListener('pointerdown', onDown);
    document.addEventListener('keydown', onKey);
    return () => {
      document.removeEventListener('pointerdown', onDown);
      document.removeEventListener('keydown', onKey);
    };
  }, [menuOpen]);

  const translate = useCallback(async () => {
    const {text: content, truncated, title} = readPageText();
    if (!content) return setResult({kind: 'translate', error: 'There is no text on this page to translate.'});
    if (!isAuthenticated) {
      setMenuOpen(false);
      openSignIn('login');
      return;
    }
    setBusy('translate');
    try {
      const res = await authFetch('/api/v1/translation/translate', {
        method: 'POST',
        body: JSON.stringify({content, target_language: lang}),
      });
      if (!res.ok) throw new Error(await readError(res));
      const data = await res.json();
      if (lang === 'ur') loadUrduFont();
      setResult({kind: 'translate', text: data.translated_content, lang, page: title, truncated, inPlace: Boolean(document.querySelector(ARTICLE))});
    } catch (err) {
      const msg = String(err.message || '');
      setResult({kind: 'translate', error: /failed to fetch|network/i.test(msg) ? 'Couldn’t reach the server. Check your connection and try again.' : msg || 'Translation failed.'});
    } finally {
      setBusy(null);
      setMenuOpen(false);
    }
  }, [isAuthenticated, authFetch, lang]);

  const personalize = useCallback(async () => {
    const {text: content, truncated, title} = readPageText();
    if (!content) return setResult({kind: 'personalize', error: 'There is no text on this page to personalize.'});
    setBusy('personalize');
    try {
      const res = await authFetch('/api/v1/personalization/rewrite', {
        method: 'POST',
        body: JSON.stringify({content, page_title: title}),
      });
      if (!res.ok) throw new Error(await readError(res));
      const data = await res.json();
      setResult({kind: 'personalize', text: data.personalized_content, page: title, truncated, inPlace: Boolean(document.querySelector(ARTICLE))});
    } catch (err) {
      const msg = String(err.message || '');
      setResult({kind: 'personalize', error: /failed to fetch|network/i.test(msg) ? 'Couldn’t reach the server. Check your connection and try again.' : msg || 'Personalization failed.'});
    } finally {
      setBusy(null);
      setMenuOpen(false);
    }
  }, [authFetch]);

  // radiogroup keyboard: arrows move and select, like native radios
  const onLangKey = (e, i) => {
    const step = {ArrowRight: 1, ArrowDown: 1, ArrowLeft: -1, ArrowUp: -1}[e.key];
    if (!step) return;
    e.preventDefault();
    const next = (i + step + LANGUAGES.length) % LANGUAGES.length;
    setLang(LANGUAGES[next][0]);
    langRefs.current[next]?.focus();
  };

  const panel = (
    <div className={styles.panel}>
      <section className={styles.section} aria-labelledby="pa-tools-lang">
        <p className={styles.sectionHead} id="pa-tools-lang">
          {Icon.translate}
          Translate this page
        </p>
        <div className={styles.langs} role="radiogroup" aria-labelledby="pa-tools-lang">
          {LANGUAGES.map(([code, name, native], i) => (
            <button
              key={code}
              ref={(n) => (langRefs.current[i] = n)}
              type="button"
              role="radio"
              aria-checked={lang === code}
              tabIndex={lang === code ? 0 : -1}
              className={styles.lang}
              onClick={() => setLang(code)}
              onKeyDown={(e) => onLangKey(e, i)}>
              <span className={styles.native} lang={code} dir={isRtl(code) ? 'rtl' : undefined}>
                {native}
              </span>
              <span className={styles.english}>{name}</span>
            </button>
          ))}
        </div>
        <button type="button" className={clsx('button button--primary', styles.action)} onClick={translate} disabled={!!busy} aria-busy={busy === 'translate'}>
          {!isAuthenticated
            ? 'Sign in to translate'
            : busy === 'translate'
              ? 'Translating…'
              : `Translate to ${LANGUAGES.find(([c]) => c === lang)[1]}`}
        </button>
      </section>
      <section className={clsx(styles.section, styles.personal)} aria-labelledby="pa-tools-personal">
        <p className={styles.sectionHead} id="pa-tools-personal">
          {Icon.person}
          Personalize
        </p>
        <p className={styles.hint}>Rivet re-explains this page for your software and hardware level (set them in your profile).</p>
        {isAuthenticated ? (
          <button type="button" className={clsx('button button--secondary', styles.action)} onClick={personalize} disabled={!!busy} aria-busy={busy === 'personalize'}>
            {busy === 'personalize' ? 'Personalizing…' : 'Personalize for me'}
          </button>
        ) : (
          <button
            type="button"
            className={clsx('button button--secondary', styles.action)}
            onClick={() => {
              setMenuOpen(false);
              openSignIn('login');
            }}>
            Sign in to personalize
          </button>
        )}
      </section>
    </div>
  );

  return (
    <>
      {mobile ? (
        <li className={clsx('menu__list-item', styles.mobile)}>
          <p className={styles.mobileTitle}>Page tools</p>
          {panel}
        </li>
      ) : (
        <div ref={wrapRef} className={clsx('navbar__item', styles.wrap)}>
          <button
            ref={triggerRef}
            type="button"
            className={clsx('navbar__link', styles.trigger)}
            aria-expanded={menuOpen}
            aria-haspopup="dialog"
            aria-controls={menuOpen ? 'pa-tools-menu' : undefined}
            onClick={() => setMenuOpen((o) => !o)}>
            {Icon.tools}
            <span>Tools</span>
            <span className={styles.chevron}>{Icon.chevron}</span>
          </button>
          {menuOpen && (
            <div id="pa-tools-menu" className={clsx('pa-popover', styles.menu)} role="dialog" aria-label="Page tools">
              {panel}
            </div>
          )}
        </div>
      )}

      {/* portal: the glass navbar's backdrop-filter would otherwise trap position:fixed inside it */}
      {result && !result.error && result.inPlace && (
        <InPlace key={`${result.kind}:${result.lang}:${result.page}`} result={result} onClose={() => setResult(null)} />
      )}
      {result &&
        (result.error || !result.inPlace) &&
        createPortal(
          <ResultDialog
            result={result}
            onClose={() => setResult(null)}
            onRetry={() => {
              const kind = result.kind;
              setResult(null);
              (kind === 'translate' ? translate : personalize)();
            }}
          />,
          document.body,
        )}
    </>
  );
}
