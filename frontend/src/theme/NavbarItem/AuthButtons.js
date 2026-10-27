import React, {useEffect, useRef, useState} from 'react';
import {createPortal} from 'react-dom';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import {OPEN_SIGN_IN, useAuth} from '@site/src/hooks/useAuth';
import styles from './AuthButtons.module.css';

const LEVELS = [
  ['', 'Choose one'],
  ['beginner', 'Beginner'],
  ['intermediate', 'Intermediate'],
  ['advanced', 'Advanced'],
];

function Field({id, label, error, children}) {
  return (
    <div className={styles.field}>
      <label htmlFor={id}>{label}</label>
      {children}
      {error && (
        <p id={`${id}-error`} className={styles.fieldError}>
          {error}
        </p>
      )}
    </div>
  );
}

function PasswordInput({id, value, onChange, autoComplete}) {
  const [shown, setShown] = useState(false);
  return (
    <div className={styles.password}>
      <input id={id} name="password" type={shown ? 'text' : 'password'} value={value} onChange={onChange} autoComplete={autoComplete} minLength={8} required />
      <button type="button" className={styles.reveal} aria-pressed={shown} aria-label={shown ? 'Hide password' : 'Show password'} onClick={() => setShown((s) => !s)}>
        {shown ? 'Hide' : 'Show'}
      </button>
    </div>
  );
}

function AuthDialog({initialMode, onClose}) {
  const {login, register} = useAuth();
  const [mode, setMode] = useState(initialMode);
  const [pending, setPending] = useState(false);
  const [error, setError] = useState('');
  const [notice, setNotice] = useState('');
  const [form, setForm] = useState({name: '', email: '', password: '', software_background: '', hardware_background: ''});
  const firstField = useRef(null);
  const set = (key) => (e) => setForm((f) => ({...f, [key]: e.target.value}));

  useEffect(() => {
    firstField.current?.focus();
  }, [mode]);
  useEffect(() => {
    const onKey = (e) => e.key === 'Escape' && onClose();
    document.addEventListener('keydown', onKey);
    return () => document.removeEventListener('keydown', onKey);
  }, [onClose]);

  const submit = async (e) => {
    e.preventDefault();
    setError('');
    setPending(true);
    try {
      if (mode === 'login') {
        const res = await login(form.email, form.password);
        if (!res.success) throw new Error(res.error);
        onClose();
      } else {
        const res = await register(form);
        if (!res.success) throw new Error(res.error);
        setMode('login');
        setNotice('Account created. Sign in to continue.');
      }
    } catch (err) {
      const msg = String(err.message || '');
      setError(/failed to fetch|network/i.test(msg) ? 'Couldn’t reach the server. Check your connection and try again.' : msg || 'Something went wrong.');
    } finally {
      setPending(false);
    }
  };

  const isLogin = mode === 'login';
  return (
    <div className="pa-overlay" onPointerDown={(e) => e.target === e.currentTarget && onClose()}>
      <div className="pa-dialog" role="dialog" aria-modal="true" aria-labelledby="pa-auth-title">
        <div className="pa-dialog__head">
          <h2 id="pa-auth-title" className="pa-dialog__title">
            {isLogin ? 'Welcome back' : 'Create your account'}
          </h2>
          <button type="button" className="pa-icon-button" onClick={onClose} aria-label="Close">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" aria-hidden="true"><path d="M6 6l12 12M18 6L6 18" /></svg>
          </button>
        </div>

        <div className={styles.tabs} role="tablist">
          {[
            ['login', 'Sign in'],
            ['register', 'Create account'],
          ].map(([key, label]) => (
            <button
              key={key}
              type="button"
              role="tab"
              aria-selected={mode === key}
              className={clsx(styles.tab, mode === key && styles.tabOn)}
              onClick={() => {
                setMode(key);
                setError('');
                setNotice('');
              }}>
              {label}
            </button>
          ))}
        </div>

        {notice && <p className={styles.notice}>{notice}</p>}
        {error && (
          <p className={styles.error} role="alert">
            {error}
          </p>
        )}

        <form className={styles.form} onSubmit={submit} noValidate={false}>
          {!isLogin && (
            <Field id="pa-name" label="Name">
              <input ref={firstField} id="pa-name" name="name" type="text" value={form.name} onChange={set('name')} autoComplete="name" required />
            </Field>
          )}
          <Field id="pa-email" label="Email">
            <input ref={isLogin ? firstField : undefined} id="pa-email" name="email" type="email" value={form.email} onChange={set('email')} autoComplete="email" inputMode="email" spellCheck={false} required />
          </Field>
          <Field id="pa-password" label="Password">
            <PasswordInput id="pa-password" value={form.password} onChange={set('password')} autoComplete={isLogin ? 'current-password' : 'new-password'} />
          </Field>
          {!isLogin && (
            <div className={styles.row}>
              <Field id="pa-sw" label="Software background">
                <select id="pa-sw" name="software_background" value={form.software_background} onChange={set('software_background')}>
                  {LEVELS.map(([v, l]) => (
                    <option key={v} value={v}>{l}</option>
                  ))}
                </select>
              </Field>
              <Field id="pa-hw" label="Hardware background">
                <select id="pa-hw" name="hardware_background" value={form.hardware_background} onChange={set('hardware_background')}>
                  {LEVELS.map(([v, l]) => (
                    <option key={v} value={v}>{l}</option>
                  ))}
                </select>
              </Field>
            </div>
          )}
          {!isLogin && <p className={styles.hint}>Your background lets Personalize pitch lessons at your level.</p>}
          <button type="submit" className={clsx('button button--primary', styles.submit)} disabled={pending} aria-busy={pending}>
            {pending ? (isLogin ? 'Signing in…' : 'Creating account…') : isLogin ? 'Sign in' : 'Create account'}
          </button>
        </form>
      </div>
    </div>
  );
}

export default function AuthButtons({mobile}) {
  const {user, isAuthenticated, loading, logout} = useAuth();
  const [dialog, setDialog] = useState(null); // 'login' | 'register' | null
  const [menu, setMenu] = useState(false);
  const name = user?.name || user?.email?.split('@')[0] || 'Account';
  const menuRef = useRef(null);

  // any page can ask for the sign-in dialog (openSignIn() in hooks/useAuth)
  useEffect(() => {
    const onOpen = (e) => setDialog(e.detail?.mode || 'login');
    window.addEventListener(OPEN_SIGN_IN, onOpen);
    return () => window.removeEventListener(OPEN_SIGN_IN, onOpen);
  }, []);

  // close the account menu on outside click / Esc
  useEffect(() => {
    if (!menu) return undefined;
    const onDown = (e) => menuRef.current && !menuRef.current.contains(e.target) && setMenu(false);
    const onKey = (e) => e.key === 'Escape' && setMenu(false);
    document.addEventListener('pointerdown', onDown);
    document.addEventListener('keydown', onKey);
    return () => {
      document.removeEventListener('pointerdown', onDown);
      document.removeEventListener('keydown', onKey);
    };
  }, [menu]);

  const content = loading ? (
    <span className={styles.placeholder} aria-hidden="true" />
  ) : isAuthenticated ? (
    <div className={styles.account} ref={menuRef}>
      <button type="button" className={styles.accountButton} aria-expanded={menu} onClick={() => setMenu((m) => !m)}>
        <span className={styles.avatar} aria-hidden="true">{name.charAt(0).toUpperCase()}</span>
        <span className={styles.accountName}>{name}</span>
      </button>
      {menu && (
        <div className={styles.accountMenu}>
          <Link className={styles.menuItem} to="/profile" onClick={() => setMenu(false)}>
            Your profile &amp; progress
          </Link>
          <button
            type="button"
            className={styles.menuItem}
            onClick={() => {
              setMenu(false);
              logout();
            }}>
            Sign out
          </button>
        </div>
      )}
    </div>
  ) : (
    <button type="button" className={clsx('button button--secondary button--sm', styles.signIn)} onClick={() => setDialog('login')}>
      Sign in
    </button>
  );

  return (
    <>
      {mobile ? <li className={clsx('menu__list-item', styles.mobile)}>{content}</li> : <div className="navbar__item">{content}</div>}
      {/* portal: the glass navbar's backdrop-filter would otherwise trap position:fixed inside it */}
      {dialog && createPortal(<AuthDialog initialMode={dialog} onClose={() => setDialog(null)} />, document.body)}
    </>
  );
}
