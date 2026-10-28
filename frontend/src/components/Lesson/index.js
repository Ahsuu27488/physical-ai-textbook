// Lesson building blocks, available in every .mdx file without imports (see theme/MDXComponents.js).
// The authoring contract (props and order) is in docs/plans/2026-10-27-proprio-course.md.
import React, {useId, useMemo, useRef, useState} from 'react';
import clsx from 'clsx';
import {useLocation} from '@docusaurus/router';
import {setQuizScore} from '@site/src/utils/progress';
import styles from './styles.module.css';

const LEVELS = ['Beginner', 'Intermediate', 'Advanced', 'Expert'];

const Icon = {
  clock: (
    <svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" aria-hidden="true">
      <circle cx="12" cy="12" r="9" />
      <path d="M12 7v5l3 2" />
    </svg>
  ),
  check: (
    <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.4" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      <path d="M5 12.5l4.5 4.5L19 7.5" />
    </svg>
  ),
  cross: (
    <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.4" strokeLinecap="round" aria-hidden="true">
      <path d="M6 6l12 12M18 6L6 18" />
    </svg>
  ),
  spark: (
    <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.9" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      <path d="M12 3v4M12 17v4M3 12h4M17 12h4M6.3 6.3l2.5 2.5M15.2 15.2l2.5 2.5M6.3 17.7l2.5-2.5M15.2 8.8l2.5-2.5" />
    </svg>
  ),
  target: (
    <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.9" aria-hidden="true">
      <circle cx="12" cy="12" r="8.5" />
      <circle cx="12" cy="12" r="4.5" />
      <circle cx="12" cy="12" r="1" fill="currentColor" />
    </svg>
  ),
  wrench: (
    <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.9" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      <path d="M14.5 6.5a4 4 0 0 0 5 5l-8.8 8.8a2.1 2.1 0 0 1-3-3z" />
      <path d="M14.5 6.5L17 4l3 3-2.5 2.5" />
    </svg>
  ),
};

export function LessonMeta({level = 'Beginner', time, prereqs = []}) {
  const rank = Math.max(0, LEVELS.indexOf(level));
  return (
    <div className={styles.meta}>
      <span className={styles.metaChip} title={`Level: ${level}`}>
        <span className={styles.meter} aria-hidden="true">
          {LEVELS.map((l, i) => (
            <span key={l} data-on={i <= rank} />
          ))}
        </span>
        {level}
      </span>
      {time && (
        <span className={styles.metaChip}>
          {Icon.clock}
          {time}
        </span>
      )}
      {prereqs.length > 0 && (
        <span className={styles.prereqs}>
          <span className={styles.prereqLabel}>Before this:</span>
          {prereqs.join(' · ')}
        </span>
      )}
    </div>
  );
}

export function Objectives({children}) {
  return (
    <section className={clsx(styles.card, styles.objectives)} aria-label="Learning objectives">
      <p className={styles.cardTitle}>
        {Icon.target}
        You will be able to
      </p>
      {children}
    </section>
  );
}

export function KeyIdea({title, children}) {
  return (
    <aside className={styles.keyIdea}>
      <p className={styles.cardTitle}>
        {Icon.spark}
        {title || 'Key idea'}
      </p>
      <div className={styles.body}>{children}</div>
    </aside>
  );
}

export function Term({def, children}) {
  const id = useId();
  const ref = useRef(null);
  const [edge, setEdge] = useState(null); // keeps the tooltip inside the viewport near the screen edges
  const place = () => {
    const r = ref.current?.getBoundingClientRect();
    if (!r) return;
    const half = Math.min(320, window.innerWidth * 0.8) / 2;
    const mid = r.left + r.width / 2;
    setEdge(mid - half < 12 ? 'left' : mid + half > window.innerWidth - 12 ? 'right' : null);
  };
  return (
    <span ref={ref} className={styles.term} tabIndex={0} aria-describedby={id} data-edge={edge || undefined} onMouseEnter={place} onFocus={place}>
      <dfn>{children}</dfn>
      <span id={id} role="tooltip" className={styles.tip}>
        {def}
      </span>
    </span>
  );
}

export function Steps({children}) {
  return <div className={styles.steps}>{children}</div>;
}

export function Exercise({title, level, children}) {
  return (
    <section className={clsx(styles.card, styles.exercise)}>
      <p className={styles.exerciseHead}>
        {Icon.wrench}
        <span className={styles.exerciseTitle}>{title}</span>
        {level && <span className={styles.levelChip}>{level}</span>}
      </p>
      <div className={styles.body}>{children}</div>
    </section>
  );
}

export function Solution({children}) {
  return (
    <details className={styles.solution}>
      <summary>
        <span className={styles.show}>Show solution</span>
        <span className={styles.hide}>Hide solution</span>
        <svg className={styles.chevron} width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
          <path d="M6 9l6 6 6-6" />
        </svg>
      </summary>
      <div className={styles.body}>{children}</div>
    </details>
  );
}

export function Takeaways({children}) {
  return (
    <section className={clsx(styles.card, styles.takeaways)} aria-label="Key takeaways">
      <p className={styles.cardTitle}>
        {Icon.check}
        Takeaways
      </p>
      {children}
    </section>
  );
}

function QuizQuestion({index, question, picked, onPick}) {
  const name = useId();
  const answered = picked !== undefined;
  return (
    <fieldset className={styles.question}>
      <legend>
        <span className={styles.qNumber}>{index + 1}</span>
        {question.q}
      </legend>
      <div className={styles.options}>
        {question.options.map((opt, i) => {
          const state = !answered ? undefined : i === question.answer ? 'right' : i === picked ? 'wrong' : undefined;
          return (
            <label key={i} className={styles.option} data-state={state} data-locked={answered || undefined}>
              <input type="radio" name={name} value={i} checked={picked === i} disabled={answered} onChange={() => onPick(i)} />
              <span className={styles.key} aria-hidden="true">
                {state === 'right' ? Icon.check : state === 'wrong' ? Icon.cross : String.fromCharCode(65 + i)}
              </span>
              <span className={styles.optionText}>{opt}</span>
            </label>
          );
        })}
      </div>
      {answered && (
        <p className={styles.explain} data-right={picked === question.answer} role="status">
          <strong>{picked === question.answer ? 'Correct. ' : 'Not quite. '}</strong>
          {question.explain}
        </p>
      )}
    </fieldset>
  );
}

export function Quiz({questions = []}) {
  const {pathname} = useLocation();
  const [picked, setPicked] = useState({});
  const answered = Object.keys(picked).length;
  const score = useMemo(() => questions.filter((q, i) => picked[i] === q.answer).length, [picked, questions]);
  const done = answered === questions.length && questions.length > 0;

  const pick = (i, choice) => {
    const next = {...picked, [i]: choice};
    setPicked(next);
    if (Object.keys(next).length === questions.length) {
      setQuizScore(pathname, questions.filter((q, k) => next[k] === q.answer).length, questions.length);
    }
  };

  return (
    <section className={styles.quiz} aria-label="Quiz">
      {questions.map((q, i) => (
        <QuizQuestion key={i} index={i} question={q} picked={picked[i]} onPick={(c) => pick(i, c)} />
      ))}
      <div className={styles.quizFoot} data-done={done || undefined}>
        <div className={styles.scoreBox}>
          <span className={styles.score} aria-live="polite">
            {done ? `You scored ${score} of ${questions.length}` : `${answered} of ${questions.length} answered`}
          </span>
          {done && <span className={styles.verdict}>{score === questions.length ? 'Perfect. On to the next lesson.' : score >= questions.length / 2 ? 'Good. Reread the ones you missed.' : 'Worth another pass through the lesson.'}</span>}
          <span className={styles.scoreBar} aria-hidden="true">
            <span style={{width: `${questions.length ? ((done ? score : answered) / questions.length) * 100 : 0}%`}} />
          </span>
        </div>
        {answered > 0 && (
          <button type="button" className="button button--secondary button--sm" onClick={() => setPicked({})}>
            Try again
          </button>
        )}
      </div>
    </section>
  );
}
