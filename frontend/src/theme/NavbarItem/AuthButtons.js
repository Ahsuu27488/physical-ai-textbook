import React, {useEffect, useRef, useState} from 'react';
import {createPortal} from 'react-dom';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import {OPEN_SIGN_IN, useAuth} from '@site/src/hooks/useAuth';
import {useDialog} from '@site/src/hooks/useDialog';
import {useProgress} from '@site/src/utils/progress';
import {getBackendUrl} from '@site/src/utils/backendUrl';
import styles from './AuthButtons.module.css';

const LEVELS = [
  ['', 'Choose one'],
  ['beginner', 'Beginner'],
  ['intermediate', 'Intermediate'],
  ['advanced', 'Advanced'],
];
const EMAIL = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;

const svg = (path, size = 16) => (
  <svg width={size} height={size} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
    {path}
  </svg>
);
const Icon = {
  close: svg(<path d="M6 6l12 12M18 6L6 18" />),
  eye: svg(
    <>
      <path d="M2.5 12S6 5.5 12 5.5 21.5 12 21.5 12 18 18.5 12 18.5 2.5 12 2.5 12z" />
      <circle cx="12" cy="12" r="3" />
    </>,
    18,
  ),
  eyeOff: svg(
    <>
      <path d="M10.6 5.6A9.8 9.8 0 0 1 12 5.5C18 5.5 21.5 12 21.5 12a17 17 0 0 1-2.6 3.4M6.5 7.1C3.9 8.8 2.5 12 2.5 12S6 18.5 12 18.5c1.7 0 3.2-.5 4.5-1.2" />
      <path d="M9.9 9.9a3 3 0 0 0 4.2 4.2M3 3l18 18" />
    </>,
    18,
  ),
  alert: svg(
    <>
      <circle cx="12" cy="12" r="9" />
      <path d="M12 7.5v5.5M12 16.5v.01" />
    </>,
  ),
  ok: svg(
    <>
      <circle cx="12" cy="12" r="9" />
      <path d="M8 12.5l2.8 2.8L16.5 9.5" />
    </>,
  ),
  check: svg(<path d="M5 12.5l4.5 4.5L19 7.5" />, 14),
  dot: svg(<circle cx="12" cy="12" r="3" fill="currentColor" stroke="none" />, 14),
  chevron: svg(<path d="M6 9l6 6 6-6" />, 14),
  profile: svg(
    <>
      <circle cx="12" cy="8" r="3.5" />
      <path d="M5 20c.8-3.6 3.6-5.5 7-5.5s6.2 1.9 7 5.5" />
    </>,
  ),
  play: svg(<path d="M8 5.5v13l10.5-6.5z" />),
  out: svg(
    <>
      <path d="M15 4h3a2 2 0 0 1 2 2v12a2 2 0 0 1-2 2h-3" />
      <path d="M10 16l-4-4 4-4M6 12h10" />
    </>,
  ),
};

function Field({id, label, error, hint, aside, children}) {
  return (
    <div className="pa-field">
      <label htmlFor={id}>
        <span>{label}</span>
        {aside}
      </label>
      {children}
      {error ? (
        <p id={`${id}-error`} className="pa-field__error">
          {Icon.alert}
          {error}
        </p>
      ) : (
        hint
      )}
    </div>
  );
}

function PasswordInput({id, value, onChange, autoComplete, describedBy}) {
  const [shown, setShown] = useState(false);
  const [caps, setCaps] = useState(false);
  const onKey = (e) => setCaps(e.getModifierState?.('CapsLock') || false);
  return (
    <>
      <div className={styles.password}>
        <input
          id={id}
          className="pa-input"
          name="password"
          type={shown ? 'text' : 'password'}
          value={value}
          onChange={onChange}
          onKeyUp={onKey}
          onKeyDown={onKey}
          onBlur={() => setCaps(false)}
          autoComplete={autoComplete}
          minLength={8}
          required
          aria-describedby={describedBy}
        />
        <button type="button" className={styles.reveal} aria-pressed={shown} aria-label={shown ? 'Hide password' : 'Show password'} title={shown ? 'Hide password' : 'Show password'} onClick={() => setShown((s) => !s)}>
          {shown ? Icon.eyeOff : Icon.eye}
        </button>
      </div>
      {caps && <p className={styles.caps}>Caps Lock is on</p>}
    </>
  );
}

function AuthDialog({initialMode, onClose}) {
  const {login, register} = useAuth();
  const logo = useBaseUrl('/img/logo.svg');
  const [mode, setMode] = useState(initialMode);
  const [pending, setPending] = useState(false);
  const [error, setError] = useState('');
  const [notice, setNotice] = useState('');
  const [shake, setShake] = useState(0);
  const [touched, setTouched] = useState({});
  const [form, setForm] = useState({name: '', email: '', password: '', software_background: '', hardware_background: ''});
  const dialogRef = useDialog(onClose);
  const firstField = useRef(null);
  const set = (key) => (e) => setForm((f) => ({...f, [key]: e.target.value}));
  const isLogin = mode === 'login';
  const isForgot = mode === 'forgot';
  const [emailOn, setEmailOn] = useState(false); // the server can send reset emails (MAIL_API_KEY set)

  useEffect(() => {
    fetch(`${getBackendUrl()}/health`)
      .then((r) => (r.ok ? r.json() : {}))
      .then((h) => setEmailOn(Boolean(h.email)))
      .catch(() => {});
  }, []);

  useEffect(() => {
    firstField.current?.focus();
  }, [mode]);

  const emailError = touched.email && form.email && !EMAIL.test(form.email) ? 'Enter an email like name@example.com.' : '';
  const longEnough = form.password.length >= 8;

  const submit = async (e) => {
    e.preventDefault();
    setTouched({email: true, password: true});
    if (isForgot) {
      if (!EMAIL.test(form.email)) return setShake((n) => n + 1);
      setError('');
      setNotice('');
      setPending(true);
      try {
        const r = await fetch(`${getBackendUrl()}/api/v1/auth/forgot-password`, {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({email: form.email.trim()}),
        });
        const data = await r.json().catch(() => ({}));
        if (!r.ok) throw new Error(typeof data.detail === 'string' ? data.detail : 'Couldn’t send the email. Try again.');
        setNotice(data.message);
      } catch (err) {
        const msg = String(err.message || '');
        setError(/failed to fetch|network/i.test(msg) ? 'Couldn’t reach the server. Check your connection and try again.' : msg);
        setShake((n) => n + 1);
      } finally {
        setPending(false);
      }
      return undefined;
    }
    if (!EMAIL.test(form.email) || (!isLogin && (!longEnough || !form.name.trim()))) {
      setShake((n) => n + 1);
      return;
    }
    setError('');
    setPending(true);
    try {
      if (isLogin) {
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
      setShake((n) => n + 1);
    } finally {
      setPending(false);
    }
  };

  const switchTo = (key) => {
    setMode(key);
    setError('');
    setNotice('');
    setTouched({});
  };

  return (
    <div className="pa-overlay" onPointerDown={(e) => e.target === e.currentTarget && onClose()}>
      <div ref={dialogRef} className={clsx('pa-dialog', styles.dialog)} role="dialog" aria-modal="true" aria-labelledby="pa-auth-title" aria-describedby="pa-auth-sub" tabIndex={-1}>
        <button type="button" className={clsx('pa-icon-button', styles.close)} onClick={onClose} aria-label="Close">
          {Icon.close}
        </button>
        <div className={styles.brand}>
          <img src={logo} alt="" width="40" height="40" />
          <h2 id="pa-auth-title" className={styles.title}>
            {isForgot ? 'Reset your password' : isLogin ? 'Welcome back' : 'Create your account'}
          </h2>
          <p id="pa-auth-sub" className={styles.sub}>
            {isForgot
              ? 'Enter your account’s email and we’ll send you a link to choose a new password.'
              : isLogin
                ? 'Sign in to sync your progress on every device.'
                : 'Free. Keeps your progress and quiz scores on every device.'}
          </p>
        </div>

        {!isForgot && (
        <div className={clsx('pa-segmented', styles.tabs)} role="tablist" aria-label="Account">
          {[
            ['login', 'Sign in'],
            ['register', 'Create account'],
          ].map(([key, label]) => (
            <button key={key} type="button" role="tab" aria-selected={mode === key} aria-controls="pa-auth-form" onClick={() => switchTo(key)}>
              {label}
            </button>
          ))}
        </div>
        )}

        <div key={shake} className={clsx(styles.messages, shake > 0 && 'pa-shake')}>
          {notice && (
            <p className="pa-note pa-note--ok" role="status">
              {Icon.ok}
              <span>{notice}</span>
            </p>
          )}
          {error && (
            <p className="pa-note pa-note--error" role="alert">
              {Icon.alert}
              <span>{error}</span>
            </p>
          )}
        </div>

        <form id="pa-auth-form" className={styles.form} onSubmit={submit} noValidate>
          {!isLogin && (
            <Field id="pa-name" label="Name" error={touched.password && !form.name.trim() ? 'Tell us what to call you.' : ''}>
              <input ref={firstField} id="pa-name" className="pa-input" name="name" type="text" value={form.name} onChange={set('name')} autoComplete="name" required />
            </Field>
          )}
          <Field id="pa-email" label="Email" error={emailError}>
            <input
              ref={isLogin ? firstField : undefined}
              id="pa-email"
              className="pa-input"
              name="email"
              type="email"
              value={form.email}
              onChange={set('email')}
              onBlur={() => setTouched((t) => ({...t, email: true}))}
              autoComplete="email"
              inputMode="email"
              spellCheck={false}
              required
              aria-invalid={!!emailError}
              aria-describedby={emailError ? 'pa-email-error' : undefined}
            />
          </Field>
          {!isForgot && (
          <Field
            id="pa-password"
            label="Password"
            hint={
              isLogin ? (
                emailOn && (
                  <button type="button" className={styles.forgot} onClick={() => switchTo('forgot')}>
                    Forgot password?
                  </button>
                )
              ) : (
                <p className={styles.rule} data-ok={longEnough || undefined} id="pa-password-rule">
                  {longEnough ? Icon.check : Icon.dot}
                  At least 8 characters
                </p>
              )
            }>
            <PasswordInput
              id="pa-password"
              value={form.password}
              onChange={set('password')}
              autoComplete={isLogin ? 'current-password' : 'new-password'}
              describedBy={!isLogin ? 'pa-password-rule' : undefined}
            />
          </Field>
          )}
          {!isLogin && (
            <fieldset className={styles.background}>
              <legend>Your background (optional)</legend>
              <div className={styles.row}>
                <Field id="pa-sw" label="Software">
                  <select id="pa-sw" className="pa-select" name="software_background" value={form.software_background} onChange={set('software_background')}>
                    {LEVELS.map(([v, l]) => (
                      <option key={v} value={v}>
                        {l}
                      </option>
                    ))}
                  </select>
                </Field>
                <Field id="pa-hw" label="Hardware">
                  <select id="pa-hw" className="pa-select" name="hardware_background" value={form.hardware_background} onChange={set('hardware_background')}>
                    {LEVELS.map(([v, l]) => (
                      <option key={v} value={v}>
                        {l}
                      </option>
                    ))}
                  </select>
                </Field>
              </div>
              <p className="pa-field__hint">Personalize and Rivet use it to pitch explanations at your level.</p>
            </fieldset>
          )}
          <button type="submit" className={clsx('button button--primary button--lg', styles.submit)} disabled={pending} aria-busy={pending}>
            {isForgot
              ? pending
                ? 'Sending…'
                : 'Send reset link'
              : pending
                ? isLogin
                  ? 'Signing in…'
                  : 'Creating account…'
                : isLogin
                  ? 'Sign in'
                  : 'Create account'}
          </button>
        </form>
        {isForgot ? (
          <p className={styles.switch}>
            Remembered it?{' '}
            <button type="button" onClick={() => switchTo('login')}>
              Back to sign in
            </button>
          </p>
        ) : (
        <p className={styles.switch}>
          {isLogin ? 'New to Kinemata?' : 'Already have an account?'}{' '}
          <button type="button" onClick={() => switchTo(isLogin ? 'register' : 'login')}>
            {isLogin ? 'Create an account' : 'Sign in'}
          </button>
        </p>
        )}
      </div>
    </div>
  );
}

function AccountMenu({user, name, onSignOut}) {
  const [open, setOpen] = useState(false);
  const {last} = useProgress();
  const wrap = useRef(null);
  const button = useRef(null);
  const items = () => [...(wrap.current?.querySelectorAll('[role="menuitem"]') || [])];

  useEffect(() => {
    if (!open) return undefined;
    items()[0]?.focus();
    const onDown = (e) => wrap.current && !wrap.current.contains(e.target) && setOpen(false);
    const onKey = (e) => {
      if (e.key === 'Escape') {
        setOpen(false);
        button.current?.focus();
      }
    };
    document.addEventListener('pointerdown', onDown);
    document.addEventListener('keydown', onKey);
    return () => {
      document.removeEventListener('pointerdown', onDown);
      document.removeEventListener('keydown', onKey);
    };
  }, [open]);

  // arrow keys move through the menu, like a native menu
  const onMenuKey = (e) => {
    const list = items();
    const i = list.indexOf(document.activeElement);
    const next = {ArrowDown: i + 1, ArrowUp: i - 1, Home: 0, End: list.length - 1}[e.key];
    if (next === undefined) return;
    e.preventDefault();
    list[(next + list.length) % list.length]?.focus();
  };

  return (
    <div className={styles.account} ref={wrap}>
      <button
        ref={button}
        type="button"
        className={styles.accountButton}
        aria-haspopup="menu"
        aria-expanded={open}
        aria-label={`Account: ${name}`}
        onClick={() => setOpen((m) => !m)}>
        <span className={styles.avatar} aria-hidden="true">
          {name.charAt(0).toUpperCase()}
        </span>
        <span className={styles.accountName}>{name}</span>
        <span className={styles.chevron}>{Icon.chevron}</span>
      </button>
      {open && (
        <div className={clsx('pa-popover', styles.accountMenu)} role="menu" aria-label="Account" onKeyDown={onMenuKey}>
          <div className={styles.who}>
            <span className={clsx(styles.avatar, styles.avatarLg)} aria-hidden="true">
              {name.charAt(0).toUpperCase()}
            </span>
            <span className={styles.whoText}>
              <strong>{name}</strong>
              {user?.email && <span>{user.email}</span>}
            </span>
          </div>
          <hr className="pa-menu__sep" />
          {last && (
            <Link className="pa-menu__item" role="menuitem" to={last.path} onClick={() => setOpen(false)}>
              {Icon.play}
              <span className={styles.itemText}>
                Continue
                <span>{last.title}</span>
              </span>
            </Link>
          )}
          <Link className="pa-menu__item" role="menuitem" to="/profile" onClick={() => setOpen(false)}>
            {Icon.profile}
            Profile &amp; progress
          </Link>
          <hr className="pa-menu__sep" />
          <button
            type="button"
            role="menuitem"
            className="pa-menu__item"
            onClick={() => {
              setOpen(false);
              onSignOut();
            }}>
            {Icon.out}
            Sign out
          </button>
        </div>
      )}
    </div>
  );
}

export default function AuthButtons({mobile}) {
  const {user, isAuthenticated, loading, logout} = useAuth();
  const [dialog, setDialog] = useState(null); // 'login' | 'register' | null
  const name = user?.name || user?.email?.split('@')[0] || 'Account';

  // any page can ask for the sign-in dialog (openSignIn() in hooks/useAuth). Only the navbar copy listens: on
  // phones the menu copy is mounted too, and both answering opened two dialogs on top of each other.
  useEffect(() => {
    if (mobile) return undefined;
    const onOpen = (e) => setDialog(e.detail?.mode || 'login');
    window.addEventListener(OPEN_SIGN_IN, onOpen);
    return () => window.removeEventListener(OPEN_SIGN_IN, onOpen);
  }, [mobile]);

  let content;
  if (loading) content = <span className={styles.placeholder} aria-hidden="true" />;
  else if (isAuthenticated && mobile)
    content = (
      <div className={styles.mobileAccount}>
        <span className={styles.avatar} aria-hidden="true">
          {name.charAt(0).toUpperCase()}
        </span>
        <span className={styles.whoText}>
          <strong>{name}</strong>
          {user?.email && <span>{user.email}</span>}
        </span>
        <Link className="button button--secondary button--sm" to="/profile">
          Profile
        </Link>
        <button type="button" className="button button--ghost button--sm" onClick={logout}>
          Sign out
        </button>
      </div>
    );
  else if (isAuthenticated) content = <AccountMenu user={user} name={name} onSignOut={logout} />;
  else
    content = (
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
