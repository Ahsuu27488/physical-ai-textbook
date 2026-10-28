import React, {useEffect, useState} from 'react';
import Head from '@docusaurus/Head';
import Layout from '@theme/Layout';
import {openSignIn} from '@site/src/hooks/useAuth';
import {getBackendUrl} from '@site/src/utils/backendUrl';
import styles from './reset-password.module.css';

// The page the password-reset email links to: /reset-password?token=…
export default function ResetPassword() {
  const [token, setToken] = useState(null); // read after mount: the static build has no query string
  const [password, setPassword] = useState('');
  const [repeat, setRepeat] = useState('');
  const [shown, setShown] = useState(false);
  const [touched, setTouched] = useState(false);
  const [pending, setPending] = useState(false);
  const [error, setError] = useState('');
  const [done, setDone] = useState(false);

  useEffect(() => {
    setToken(new URLSearchParams(window.location.search).get('token') || '');
  }, []);

  const tooShort = password.length < 8;
  const mismatch = repeat && repeat !== password;

  const submit = async (e) => {
    e.preventDefault();
    setTouched(true);
    if (tooShort || repeat !== password) return;
    setPending(true);
    setError('');
    try {
      const r = await fetch(`${getBackendUrl()}/api/v1/auth/reset-password`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({token, new_password: password}),
      });
      const data = await r.json().catch(() => ({}));
      if (!r.ok) throw new Error(typeof data.detail === 'string' ? data.detail : 'Couldn’t change the password. Try again.');
      setDone(true);
      window.history.replaceState(null, '', window.location.pathname); // the link is spent; keep it out of history
    } catch (err) {
      const msg = String(err.message || '');
      setError(/failed to fetch|network/i.test(msg) ? 'Couldn’t reach the server. Check your connection and try again.' : msg);
    } finally {
      setPending(false);
    }
  };

  return (
    <Layout title="Reset your password" description="Choose a new password for your Kinemata account.">
      <Head>
        <meta name="robots" content="noindex" />
      </Head>
      <main className={styles.main}>
        <div className={styles.card}>
          <h1 className={styles.title}>{done ? 'Password changed' : 'Choose a new password'}</h1>
          {done ? (
            <>
              <p className={styles.sub}>You can sign in with your new password now.</p>
              <button type="button" className="button button--primary button--lg" onClick={() => openSignIn('login')}>
                Sign in
              </button>
            </>
          ) : token === '' ? (
            <>
              <p className="pa-note pa-note--error" role="alert">
                <span>This page needs the link from the reset email. Ask for a new one from “Forgot password?” in the sign-in window.</span>
              </p>
              <button type="button" className="button button--secondary" onClick={() => openSignIn('login')}>
                Open sign-in
              </button>
            </>
          ) : (
            <form className={styles.form} onSubmit={submit} noValidate>
              <p className={styles.sub}>At least 8 characters. The link in your email works once, for 30 minutes.</p>
              {error && (
                <p className="pa-note pa-note--error" role="alert">
                  <span>{error}</span>
                </p>
              )}
              <div className="pa-field">
                <label htmlFor="pa-new-password">New password</label>
                <input
                  id="pa-new-password"
                  className="pa-input"
                  type={shown ? 'text' : 'password'}
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  autoComplete="new-password"
                  minLength={8}
                  required
                  aria-invalid={touched && tooShort}
                  aria-describedby="pa-new-password-hint"
                />
                <p id="pa-new-password-hint" className={touched && tooShort ? 'pa-field__error' : 'pa-field__hint'}>
                  At least 8 characters
                </p>
              </div>
              <div className="pa-field">
                <label htmlFor="pa-repeat-password">Repeat it</label>
                <input
                  id="pa-repeat-password"
                  className="pa-input"
                  type={shown ? 'text' : 'password'}
                  value={repeat}
                  onChange={(e) => setRepeat(e.target.value)}
                  autoComplete="new-password"
                  required
                  aria-invalid={Boolean((touched || repeat) && mismatch)}
                  aria-describedby={mismatch ? 'pa-repeat-error' : undefined}
                />
                {mismatch && (
                  <p id="pa-repeat-error" className="pa-field__error">
                    The two passwords don’t match.
                  </p>
                )}
              </div>
              <label className={styles.show}>
                <input type="checkbox" checked={shown} onChange={(e) => setShown(e.target.checked)} /> Show passwords
              </label>
              <button type="submit" className="button button--primary button--lg" disabled={pending || token === null} aria-busy={pending}>
                {pending ? 'Saving…' : 'Save new password'}
              </button>
            </form>
          )}
        </div>
      </main>
    </Layout>
  );
}
