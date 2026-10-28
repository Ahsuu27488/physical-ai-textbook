import React, {useMemo} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Footer from '@theme-original/DocItem/Footer';
import {useAllDocsData, useDoc} from '@docusaurus/plugin-content-docs/client';
import {setCompleted, useCompleted} from '@site/src/utils/progress';
import styles from './styles.module.css';

// Number of lessons in the course (module overviews and the course home don't count).
function useLessonCount() {
  const all = useAllDocsData();
  return useMemo(() => {
    const docs = Object.values(all)[0]?.versions?.[0]?.docs || [];
    return docs.filter((d) => !/(^|\/)index$/.test(d.id)).length;
  }, [all]);
}

// Adds "Mark lesson complete" under every lesson (not on module overviews or the course home).
export default function FooterWrapper(props) {
  const {metadata} = useDoc();
  const completed = useCompleted();
  const total = useLessonCount();
  const isLesson = !/(^|\/)index$/.test(metadata.id);
  const done = completed.has(metadata.id);
  const pct = total ? Math.min(100, Math.round((completed.size / total) * 100)) : 0;
  const next = metadata.next;

  return (
    <>
      {isLesson && (
        <section className={styles.complete} data-done={done} aria-label="Lesson progress">
          <span className={styles.mark} aria-hidden="true">
            <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.6" strokeLinecap="round" strokeLinejoin="round">
              <path className={styles.tick} d="M5 12.5l4.5 4.5L19 7.5" pathLength="1" />
            </svg>
          </span>
          <div className={styles.copy}>
            <p className={styles.title}>{done ? 'Lesson complete' : 'Finished the lab and the quiz?'}</p>
            <p className={styles.text} aria-live="polite">
              {done
                ? `${completed.size} of ${total} lessons done (${pct}% of the course). Ticks show in the sidebar.`
                : 'Mark it done to track your progress through the course.'}
            </p>
            {done && (
              <span className={styles.bar} aria-hidden="true">
                <span style={{width: `${pct}%`}} />
              </span>
            )}
          </div>
          <div className={styles.actions}>
            {done ? (
              <>
                {next && (
                  <Link className="button button--primary" to={next.permalink}>
                    Next: {next.title.length > 30 ? `${next.title.slice(0, 28)}…` : next.title}
                  </Link>
                )}
                <button type="button" className={clsx('button', next ? 'button--ghost' : 'button--secondary')} aria-pressed onClick={() => setCompleted(metadata.id, false)}>
                  Mark as not done
                </button>
              </>
            ) : (
              <button type="button" className="button button--primary" aria-pressed={false} onClick={() => setCompleted(metadata.id, true)}>
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.6" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
                  <path d="M5 12.5l4.5 4.5L19 7.5" />
                </svg>
                Mark lesson complete
              </button>
            )}
          </div>
        </section>
      )}
      <Footer {...props} />
    </>
  );
}
