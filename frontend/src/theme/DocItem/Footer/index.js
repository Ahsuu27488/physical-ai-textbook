import React from 'react';
import Footer from '@theme-original/DocItem/Footer';
import {useDoc} from '@docusaurus/plugin-content-docs/client';
import {setCompleted, useCompleted} from '@site/src/utils/progress';
import styles from './styles.module.css';

// Adds "Mark lesson complete" under every lesson (not on module overviews or the course home).
export default function FooterWrapper(props) {
  const {metadata} = useDoc();
  const completed = useCompleted();
  const isLesson = !/(^|\/)index$/.test(metadata.id);
  const done = completed.has(metadata.id);

  return (
    <>
      {isLesson && (
        <div className={styles.complete} data-done={done}>
          <div>
            <p className={styles.title}>{done ? 'Lesson complete' : 'Finished the lab and the quiz?'}</p>
            <p className={styles.text}>
              {done ? `${completed.size} lesson${completed.size === 1 ? '' : 's'} done so far. Ticks show in the sidebar.` : 'Mark it done to track your progress through the course.'}
            </p>
          </div>
          <button
            type="button"
            className={done ? 'button button--secondary' : 'button button--primary'}
            aria-pressed={done}
            onClick={() => setCompleted(metadata.id, !done)}>
            {done ? 'Mark as not done' : 'Mark lesson complete'}
          </button>
        </div>
      )}
      <Footer {...props} />
    </>
  );
}
