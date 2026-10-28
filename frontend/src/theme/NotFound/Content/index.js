import React from 'react';
import Link from '@docusaurus/Link';
import {openAssistant} from '@site/src/components/ChatWidget/events';
import styles from './styles.module.css';

const PLACES = [
  ['Start here', '/docs/start-here', 'How the course works and your workstation'],
  ['Robot fundamentals', '/docs/robot-fundamentals', 'Frames, kinematics, dynamics, PID'],
  ['ROS 2', '/docs/ros2', 'Nodes, topics, TF2, URDF, launch'],
  ['Humanoids', '/docs/humanoids', 'Balance, walking, whole-body control'],
];

export default function NotFoundContent() {
  return (
    <main className={styles.wrap}>
      <div className={styles.scope} aria-hidden="true">
        <span className={styles.ring} />
        <span className={styles.ring} />
        <span className={styles.ring} />
        <span className={styles.sweep} />
        <span className={styles.code}>404</span>
      </div>
      <p className={styles.kicker}>No return signal</p>
      <h1 className={styles.title}>This page isn’t in the course.</h1>
      <p className={styles.text}>
        The link may be old, or the lesson moved. Start from the contents, search with{' '}
        <kbd className="pa-kbd">Ctrl</kbd> <kbd className="pa-kbd">K</kbd>, or ask Rivet where a topic lives.
      </p>
      <div className={styles.actions}>
        <Link className="button button--primary button--lg" to="/docs/">
          Open the contents
        </Link>
        <button type="button" className="button button--secondary button--lg" onClick={() => openAssistant()}>
          Ask Rivet
        </button>
      </div>
      <nav className={styles.places} aria-label="Popular modules">
        {PLACES.map(([title, to, sub]) => (
          <Link key={to} to={to} className={styles.place}>
            <span className={styles.placeTitle}>{title}</span>
            <span className={styles.placeSub}>{sub}</span>
          </Link>
        ))}
      </nav>
    </main>
  );
}
