import React from 'react';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import {useThemeConfig} from '@docusaurus/theme-common';
import {openAssistant} from '@site/src/components/ChatWidget/events';
import styles from './styles.module.css';

// Site footer: brand block, link columns from themeConfig.footer.links, and a giant wordmark.
export default function Footer() {
  const {footer} = useThemeConfig();
  const logo = useBaseUrl('/img/logo.svg');
  if (!footer) return null;

  return (
    <footer className={styles.footer}>
      <div className={styles.inner}>
        <div className={styles.brand}>
          <Link to="/" className={styles.brandLink} aria-label="Proprio home">
            <img src={logo} alt="" width="36" height="36" />
            <span>Proprio</span>
          </Link>
          <p className={styles.pitch}>
            The open course in Physical AI and humanoid robotics. From your first ROS 2 node to robots that act on
            spoken instructions.
          </p>
          <div className={styles.actions}>
            <Link className="button button--primary" to="/docs/">
              Start the course
            </Link>
            <button type="button" className="button button--secondary" onClick={() => openAssistant()}>
              Ask Rivet
            </button>
          </div>
        </div>

        <nav className={styles.columns} aria-label="Footer">
          {footer.links.map((col) => (
            <div key={col.title} className={styles.column}>
              <p className={styles.colTitle}>{col.title}</p>
              <ul>
                {col.items.map((item) => (
                  <li key={item.label}>
                    {item.href ? (
                      <a href={item.href} target="_blank" rel="noopener noreferrer">
                        {item.label}
                        <svg className={styles.ext} width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
                          <path d="M7 17L17 7M9 7h8v8" />
                        </svg>
                        <span className="sr-only"> (opens in a new tab)</span>
                      </a>
                    ) : (
                      <Link to={item.to}>{item.label}</Link>
                    )}
                  </li>
                ))}
              </ul>
            </div>
          ))}
        </nav>
      </div>

      <div className={styles.bottom}>
        <span>{footer.copyright}</span>
        <span>Open source · lessons, labs and code free to use</span>
      </div>
      <p className={styles.wordmark} aria-hidden="true">
        Proprio
      </p>
    </footer>
  );
}
