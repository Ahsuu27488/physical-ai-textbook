import React, {useCallback, useEffect, useRef, useState} from 'react';
import {createPortal} from 'react-dom';
import clsx from 'clsx';
import {useLocation} from '@docusaurus/router';
import ReactMarkdown from 'react-markdown';
import {useAuth} from '@site/src/hooks/useAuth';
import {getBackendUrl} from '@site/src/utils/backendUrl';
import {loadUrduFont} from '@site/src/utils/urduFont';
import styles from './PageTools.module.css';

const LANGUAGES = [
  ['ur', 'Urdu'],
  ['es', 'Spanish'],
  ['fr', 'French'],
  ['de', 'German'],
];
const MAX_CHARS = 2000; // the API takes the text in the query string

// The lesson's own text: title + paragraphs and list items of the article, in reading order.
function readPageText() {
  const root = document.querySelector('article .markdown') || document.querySelector('main');
  if (!root) return '';
  const parts = [];
  const title = document.querySelector('article h1, main h1');
  if (title) parts.push(title.textContent.trim());
  root.querySelectorAll('h2, h3, p, li').forEach((el) => {
    const t = el.textContent.trim();
    if (t) parts.push(t);
  });
  return parts.join('\n\n').slice(0, MAX_CHARS);
}

async function readError(res) {
  try {
    return (await res.json()).detail || `Request failed (${res.status})`;
  } catch {
    return `Request failed (${res.status})`;
  }
}

const ToolsIcon = (
  <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" aria-hidden="true">
    <path d="M4 7h10M18 7h2M4 17h4M12 17h8" />
    <circle cx="16" cy="7" r="2" />
    <circle cx="10" cy="17" r="2" />
  </svg>
);

export default function PageTools({mobile}) {
  const {isAuthenticated, getToken} = useAuth();
  const location = useLocation();
  const [menuOpen, setMenuOpen] = useState(false);
  const [lang, setLang] = useState('ur');
  const [busy, setBusy] = useState(null); // 'translate' | 'personalize' | null
  const [result, setResult] = useState(null); // {kind, text, lang} | {kind, error}
  const [copied, setCopied] = useState(false);
  const wrapRef = useRef(null);
  const triggerRef = useRef(null);

  useEffect(() => setMenuOpen(false), [location.pathname]);

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

  useEffect(() => {
    if (!result) return undefined;
    const onKey = (e) => e.key === 'Escape' && setResult(null);
    document.addEventListener('keydown', onKey);
    return () => document.removeEventListener('keydown', onKey);
  }, [result]);

  const translate = useCallback(async () => {
    const content = readPageText();
    if (!content) return setResult({kind: 'translate', error: 'There is no text on this page to translate.'});
    setBusy('translate');
    try {
      const token = isAuthenticated ? await getToken() : null;
      let res;
      if (token) {
        const url = new URL(`${getBackendUrl()}/api/v1/translation/translate-chapter`);
        url.searchParams.set('content', content);
        url.searchParams.set('target_language', lang);
        res = await fetch(url, {method: 'POST', headers: {Authorization: `Bearer ${token}`}});
      } else if (lang === 'ur') {
        const form = new FormData();
        form.append('content', content);
        res = await fetch(`${getBackendUrl()}/api/v1/translation/translate-to-urdu`, {method: 'POST', body: form});
      } else {
        const url = new URL(`${getBackendUrl()}/api/v1/translation/translate-chapter`);
        url.searchParams.set('content', content);
        url.searchParams.set('target_language', lang);
        res = await fetch(url, {method: 'POST'});
      }
      if (!res.ok) throw new Error(await readError(res));
      const data = await res.json();
      if (lang === 'ur') loadUrduFont();
      setResult({kind: 'translate', text: data.translated_content, lang});
    } catch (err) {
      setResult({kind: 'translate', error: err.message || 'Translation failed.'});
    } finally {
      setBusy(null);
      setMenuOpen(false);
    }
  }, [isAuthenticated, getToken, lang]);

  const personalize = useCallback(async () => {
    const content = readPageText();
    if (!content) return setResult({kind: 'personalize', error: 'There is no text on this page to personalize.'});
    setBusy('personalize');
    try {
      const token = await getToken();
      if (!token) throw new Error('Sign in to personalize pages.');
      const chapter = location.pathname.replace(/\//g, '-').replace(/^-|-$/g, '') || 'home';
      const url = new URL(`${getBackendUrl()}/api/v1/personalization/apply/${chapter}`);
      url.searchParams.set('content', content);
      const res = await fetch(url, {method: 'POST', headers: {Authorization: `Bearer ${token}`}});
      if (!res.ok) throw new Error(await readError(res));
      const data = await res.json();
      setResult({kind: 'personalize', text: data.personalized_content});
    } catch (err) {
      setResult({kind: 'personalize', error: err.message || 'Personalization failed.'});
    } finally {
      setBusy(null);
      setMenuOpen(false);
    }
  }, [getToken, location.pathname]);

  const copy = async () => {
    try {
      await navigator.clipboard.writeText(result.text);
      setCopied(true);
      setTimeout(() => setCopied(false), 1600);
    } catch {
      // clipboard blocked: leave the text selectable
    }
  };

  const panel = (
    <div className={styles.panel}>
      <p className={styles.groupLabel} id="pa-tools-lang">Translate this page to</p>
      <div className={styles.langs} role="radiogroup" aria-labelledby="pa-tools-lang">
        {LANGUAGES.map(([code, name]) => (
          <button
            key={code}
            type="button"
            role="radio"
            aria-checked={lang === code}
            className={clsx(styles.lang, lang === code && styles.langOn)}
            onClick={() => setLang(code)}>
            {name}
          </button>
        ))}
      </div>
      <button type="button" className={clsx('button button--primary', styles.action)} onClick={translate} disabled={!!busy} aria-busy={busy === 'translate'}>
        {busy === 'translate' ? 'Translating…' : 'Translate page'}
      </button>
      <hr className={styles.rule} />
      <button
        type="button"
        className={clsx('button button--secondary', styles.action)}
        onClick={personalize}
        disabled={!!busy || !isAuthenticated}
        aria-busy={busy === 'personalize'}>
        {busy === 'personalize' ? 'Personalizing…' : 'Personalize for me'}
      </button>
      {!isAuthenticated && <p className={styles.hint}>Sign in to rewrite pages for your background.</p>}
    </div>
  );

  const resultTitle =
    result?.kind === 'translate'
      ? `${LANGUAGES.find(([c]) => c === (result.lang || lang))?.[1] || 'Translated'} translation`
      : 'Personalized for you';

  return (
    <>
      {mobile ? (
        <li className="menu__list-item">
          <p className={clsx('menu__link', styles.mobileTitle)}>Page tools</p>
          {panel}
        </li>
      ) : (
        <div ref={wrapRef} className={clsx('navbar__item', styles.wrap)}>
          <button
            ref={triggerRef}
            type="button"
            className={clsx('navbar__link', styles.trigger)}
            aria-expanded={menuOpen}
            aria-haspopup="true"
            onClick={() => setMenuOpen((o) => !o)}>
            {ToolsIcon}
            <span>Tools</span>
          </button>
          {menuOpen && <div className={styles.menu}>{panel}</div>}
        </div>
      )}

      {/* portal: the glass navbar's backdrop-filter would otherwise trap position:fixed inside it */}
      {result && createPortal(
        <div className="pa-overlay" onPointerDown={(e) => e.target === e.currentTarget && setResult(null)}>
          <div className="pa-dialog pa-dialog--wide" role="dialog" aria-modal="true" aria-labelledby="pa-tools-result">
            <div className="pa-dialog__head">
              <h2 id="pa-tools-result" className="pa-dialog__title">
                {result.error ? 'Something went wrong' : resultTitle}
              </h2>
              <button type="button" className="pa-icon-button" onClick={() => setResult(null)} aria-label="Close" autoFocus>
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" aria-hidden="true"><path d="M6 6l12 12M18 6L6 18" /></svg>
              </button>
            </div>
            {result.error ? (
              <p className={styles.error}>{result.error}</p>
            ) : (
              <>
                <div className={clsx(styles.result, result.lang === 'ur' && 'translate-button-urdu')} lang={result.lang || undefined}>
                  <ReactMarkdown>{result.text}</ReactMarkdown>
                </div>
                <div className={styles.resultActions}>
                  <button type="button" className="button button--primary" onClick={copy}>
                    {copied ? 'Copied' : 'Copy text'}
                  </button>
                  <button type="button" className="button button--secondary" onClick={() => setResult(null)}>
                    Close
                  </button>
                </div>
              </>
            )}
          </div>
        </div>,
        document.body,
      )}
    </>
  );
}
