import React from 'react';
import Link from '@docusaurus/Link';
import {openAssistant} from '@site/src/components/ChatWidget/events';
import styles from './styles.module.css';

export default function NotFoundContent() {
  return (
    <main className={styles.wrap}>
      <p className={styles.code}>404 · no return signal</p>
      <h1 className={styles.title}>This page isn’t in the course.</h1>
      <p className={styles.text}>The link may be old, or the lesson moved. Start from the contents, or ask Rivet where a topic lives.</p>
      <div className={styles.actions}>
        <Link className="button button--primary button--lg" to="/docs/">
          Open the contents
        </Link>
        <button type="button" className="button button--secondary button--lg" onClick={() => openAssistant()}>
          Ask Rivet
        </button>
      </div>
    </main>
  );
}
