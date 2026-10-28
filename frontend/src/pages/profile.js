import React, {useEffect, useMemo, useState} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Layout from '@theme/Layout';
import {useAllDocsData} from '@docusaurus/plugin-content-docs/client';
import {openSignIn, useAuth} from '@site/src/hooks/useAuth';
import {resetProgress, useProgress} from '@site/src/utils/progress';
import styles from './profile.module.css';

const MODULES = [
  ['start-here', 'Start here'],
  ['foundations', 'Foundations'],
  ['robot-fundamentals', 'Robot fundamentals'],
  ['ros2', 'ROS 2'],
  ['simulation', 'Simulation'],
  ['perception', 'Perception'],
  ['motion', 'Motion'],
  ['robot-learning', 'Robot learning'],
  ['humanoids', 'Humanoids'],
  ['foundation-models', 'Foundation models'],
  ['capstone-and-career', 'Capstone & career'],
];
const LEVELS = [
  ['', 'Not set'],
  ['beginner', 'Beginner'],
  ['intermediate', 'Intermediate'],
  ['advanced', 'Advanced'],
];

// Lessons per module, from the docs plugin (module overviews and the course home don't count).
function useCourseLessons() {
  const all = useAllDocsData();
  return useMemo(() => {
    const docs = Object.values(all)[0]?.versions?.[0]?.docs || [];
    const byModule = new Map(MODULES.map(([slug]) => [slug, []]));
    for (const doc of docs) {
      if (/(^|\/)index$/.test(doc.id)) continue;
      const slug = doc.id.split('/')[0];
      if (byModule.has(slug)) byModule.get(slug).push(doc);
    }
    return byModule;
  }, [all]);
}

function Stat({value, label}) {
  return (
    <div className={styles.stat}>
      <dd>{value}</dd>
      <dt>{label}</dt>
    </div>
  );
}

function ProgressOverview({progress, lessons}) {
  const total = [...lessons.values()].reduce((n, l) => n + l.length, 0);
  const done = [...lessons.values()].flat().filter((d) => progress.completed.has(d.id)).length;
  const quizzes = Object.values(progress.quizzes);
  const avg = quizzes.length ? Math.round((quizzes.reduce((s, q) => s + q.best / q.total, 0) / quizzes.length) * 100) : null;
  const pct = total ? Math.round((done / total) * 100) : 0;

  return (
    <>
      <dl className={styles.stats}>
        <Stat value={`${pct}%`} label="of the course" />
        <Stat value={`${done}/${total}`} label="lessons completed" />
        <Stat value={quizzes.length} label="quizzes taken" />
        <Stat value={avg === null ? '–' : `${avg}%`} label="average quiz score" />
      </dl>

      {progress.last && (
        <Link to={progress.last.path} className={styles.resume}>
          <span className={styles.resumeLabel}>Continue where you left off</span>
          <span className={styles.resumeTitle}>{progress.last.title}</span>
          <span className={styles.resumeArrow} aria-hidden="true">
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.2" strokeLinecap="round" strokeLinejoin="round">
              <path d="M5 12h14M13 6l6 6-6 6" />
            </svg>
          </span>
        </Link>
      )}

      <section className={styles.card} aria-labelledby="modules-h">
        <h2 id="modules-h" className={styles.cardTitle}>
          Progress by module
        </h2>
        <ul className={styles.modules}>
          {MODULES.map(([slug, label], i) => {
            const list = lessons.get(slug) || [];
            const n = list.filter((d) => progress.completed.has(d.id)).length;
            const next = list.find((d) => !progress.completed.has(d.id));
            return (
              <li key={slug} className={styles.module}>
                <span className={styles.moduleN}>{String(i).padStart(2, '0')}</span>
                <Link to={`/docs/${slug}`} className={styles.moduleName}>
                  {label}
                </Link>
                <span className={styles.bar} role="progressbar" aria-valuemin={0} aria-valuemax={list.length} aria-valuenow={n} aria-label={`${label}: ${n} of ${list.length} lessons`}>
                  <span style={{width: `${list.length ? (n / list.length) * 100 : 0}%`}} />
                </span>
                <span className={styles.moduleCount}>
                  {n}/{list.length}
                </span>
                {next && n > 0 && (
                  <Link to={next.path} className={styles.moduleNext}>
                    Next lesson
                  </Link>
                )}
              </li>
            );
          })}
        </ul>
      </section>
    </>
  );
}

function ProfileForm({user, updateProfile}) {
  const [form, setForm] = useState({name: '', software_background: '', hardware_background: ''});
  const [state, setState] = useState({pending: false, message: '', error: ''});
  useEffect(() => {
    setForm({name: user.name || '', software_background: user.software_background || '', hardware_background: user.hardware_background || ''});
  }, [user]);
  const set = (k) => (e) => setForm((f) => ({...f, [k]: e.target.value}));
  const submit = async (e) => {
    e.preventDefault();
    setState({pending: true, message: '', error: ''});
    const res = await updateProfile(form);
    setState({pending: false, message: res.success ? 'Saved.' : '', error: res.success ? '' : res.error});
  };
  return (
    <form className={clsx(styles.card, styles.form)} onSubmit={submit} aria-labelledby="profile-h">
      <h2 id="profile-h" className={styles.cardTitle}>
        Your profile
      </h2>
      <p className={styles.hint}>Your background tells Personalize and Rivet how to pitch explanations.</p>
      <label className={styles.field}>
        <span>Name</span>
        <input className="pa-input" name="name" autoComplete="name" value={form.name} onChange={set('name')} maxLength={80} />
      </label>
      <div className={styles.row}>
        <label className={styles.field}>
          <span>Software background</span>
          <select className="pa-select" name="software_background" value={form.software_background} onChange={set('software_background')}>
            {LEVELS.map(([v, l]) => (
              <option key={v} value={v}>
                {l}
              </option>
            ))}
          </select>
        </label>
        <label className={styles.field}>
          <span>Hardware background</span>
          <select className="pa-select" name="hardware_background" value={form.hardware_background} onChange={set('hardware_background')}>
            {LEVELS.map(([v, l]) => (
              <option key={v} value={v}>
                {l}
              </option>
            ))}
          </select>
        </label>
      </div>
      <div className={styles.actions}>
        <button type="submit" className="button button--primary" disabled={state.pending} aria-busy={state.pending}>
          {state.pending ? 'Saving…' : 'Save profile'}
        </button>
        <span className={state.error ? styles.error : styles.ok} role="status">
          {state.error || state.message}
        </span>
      </div>
    </form>
  );
}

function PasswordForm({changePassword}) {
  const [current, setCurrent] = useState('');
  const [next, setNext] = useState('');
  const [state, setState] = useState({pending: false, message: '', error: ''});
  const submit = async (e) => {
    e.preventDefault();
    if (next.length < 8) return setState({pending: false, message: '', error: 'Use at least 8 characters.'});
    setState({pending: true, message: '', error: ''});
    const res = await changePassword(current, next);
    if (res.success) {
      setCurrent('');
      setNext('');
    }
    return setState({pending: false, message: res.success ? 'Password changed.' : '', error: res.success ? '' : res.error});
  };
  return (
    <form className={clsx(styles.card, styles.form)} onSubmit={submit} aria-labelledby="password-h">
      <h2 id="password-h" className={styles.cardTitle}>
        Password
      </h2>
      <label className={styles.field}>
        <span>Current password</span>
        <input className="pa-input" type="password" name="current-password" autoComplete="current-password" value={current} onChange={(e) => setCurrent(e.target.value)} required />
      </label>
      <label className={styles.field}>
        <span>New password</span>
        <input className="pa-input" type="password" name="new-password" autoComplete="new-password" minLength={8} value={next} onChange={(e) => setNext(e.target.value)} required />
      </label>
      <div className={styles.actions}>
        <button type="submit" className="button button--secondary" disabled={state.pending} aria-busy={state.pending}>
          {state.pending ? 'Changing…' : 'Change password'}
        </button>
        <span className={state.error ? styles.error : styles.ok} role="status">
          {state.error || state.message}
        </span>
      </div>
    </form>
  );
}

function DangerZone({signedIn, deleteAccount}) {
  const [confirming, setConfirming] = useState(false);
  const [resetting, setResetting] = useState(false);
  const [password, setPassword] = useState('');
  const [state, setState] = useState({pending: false, error: ''});
  return (
    <section className={clsx(styles.card, styles.danger)} aria-labelledby="danger-h">
      <h2 id="danger-h" className={styles.cardTitle}>
        Reset or delete
      </h2>
      <div className={styles.dangerRow}>
        <p>
          {resetting
            ? 'Clear every completed lesson and quiz score in this browser? Progress saved to your account comes back when you sign in.'
            : 'Clear the lessons and quiz scores saved in this browser.'}
        </p>
        {resetting ? (
          <div className={styles.actions}>
            <button
              type="button"
              className="button button--danger"
              onClick={() => {
                resetProgress();
                setResetting(false);
              }}>
              Clear progress
            </button>
            <button type="button" className="button button--secondary" onClick={() => setResetting(false)}>
              Cancel
            </button>
          </div>
        ) : (
          <button type="button" className="button button--secondary" onClick={() => setResetting(true)}>
            Reset progress
          </button>
        )}
      </div>
      {signedIn && (
        <div className={styles.dangerRow}>
          <p>Delete your account, profile and synced progress for good.</p>
          {!confirming ? (
            <button type="button" className="button button--danger" onClick={() => setConfirming(true)}>
              Delete account
            </button>
          ) : (
            <form
              className={styles.confirm}
              onSubmit={async (e) => {
                e.preventDefault();
                setState({pending: true, error: ''});
                const res = await deleteAccount(password);
                setState({pending: false, error: res.success ? '' : res.error});
              }}>
              <label className={styles.field}>
                <span>Type your password to confirm</span>
                <input className="pa-input" type="password" name="password" autoComplete="current-password" value={password} onChange={(e) => setPassword(e.target.value)} required />
              </label>
              <div className={styles.actions}>
                <button type="submit" className="button button--danger" disabled={state.pending} aria-busy={state.pending}>
                  {state.pending ? 'Deleting…' : 'Delete for good'}
                </button>
                <button type="button" className="button button--secondary" onClick={() => setConfirming(false)}>
                  Keep my account
                </button>
              </div>
              {state.error && (
                <p className={styles.error} role="alert">
                  {state.error}
                </p>
              )}
            </form>
          )}
        </div>
      )}
    </section>
  );
}

export default function Profile() {
  const {user, isAuthenticated, loading, updateProfile, changePassword, deleteAccount, logout} = useAuth();
  const progress = useProgress();
  const lessons = useCourseLessons();
  const name = user?.name || user?.email?.split('@')[0];
  const since = user?.created_at ? new Date(user.created_at).toLocaleDateString(undefined, {year: 'numeric', month: 'long'}) : null;

  return (
    <Layout title="Your profile" description="Your Kinemata profile, learning progress and account settings.">
      <main className={styles.page}>
        <header className={styles.head}>
          <span className={styles.avatar} aria-hidden="true">
            {(name || 'P').charAt(0).toUpperCase()}
          </span>
          <div>
            <h1 className={styles.title}>{isAuthenticated ? name : 'Your progress'}</h1>
            <p className={styles.sub}>
              {loading ? 'Loading…' : isAuthenticated ? `${user.email}${since ? ` · learning since ${since}` : ''}` : 'Saved in this browser. Sign in to keep it on every device.'}
            </p>
          </div>
          {!loading &&
            (isAuthenticated ? (
              <button type="button" className={clsx('button button--secondary', styles.headAction)} onClick={logout}>
                Sign out
              </button>
            ) : (
              <div className={styles.headAction}>
                <button type="button" className="button button--primary" onClick={() => openSignIn('login')}>
                  Sign in to sync
                </button>
                <button type="button" className="button button--secondary" onClick={() => openSignIn('register')}>
                  Create account
                </button>
              </div>
            ))}
        </header>

        <ProgressOverview progress={progress} lessons={lessons} />

        {isAuthenticated && (
          <div className={styles.grid}>
            <ProfileForm user={user} updateProfile={updateProfile} />
            <PasswordForm changePassword={changePassword} />
          </div>
        )}
        <DangerZone signedIn={isAuthenticated} deleteAccount={deleteAccount} />
      </main>
    </Layout>
  );
}
