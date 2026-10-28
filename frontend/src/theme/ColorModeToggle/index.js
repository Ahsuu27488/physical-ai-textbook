// Theme switch: one tap flips light ↔ dark. The sun folds into a moon, and the new theme spreads out
// from the button as a circle (View Transitions; an instant swap where unsupported or with reduced motion).
// Until the learner picks one, the site follows the device setting (respectPrefersColorScheme).
import React, {useCallback, useRef} from 'react';
import {flushSync} from 'react-dom';
import clsx from 'clsx';
import useIsBrowser from '@docusaurus/useIsBrowser';
import {useColorMode} from '@docusaurus/theme-common';
import styles from './styles.module.css';

const RAYS = [0, 45, 90, 135, 180, 225, 270, 315];

function SunMoon() {
  return (
    <svg className={styles.icon} viewBox="0 0 24 24" width="20" height="20" aria-hidden="true">
      <mask id="pa-moon-cut">
        <rect width="24" height="24" fill="#fff" />
        <circle className={styles.cut} cx="24" cy="4" r="7" fill="#000" />
      </mask>
      <circle className={styles.disc} cx="12" cy="12" r="5" fill="currentColor" mask="url(#pa-moon-cut)" />
      <g className={styles.rays} stroke="currentColor" strokeWidth="2" strokeLinecap="round">
        {RAYS.map((deg) => (
          <line key={deg} x1="12" y1="2.5" x2="12" y2="4.5" transform={`rotate(${deg} 12 12)`} />
        ))}
      </g>
    </svg>
  );
}

function spread(button, apply) {
  const reduced = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  if (!document.startViewTransition || reduced || !button) {
    apply();
    return;
  }
  const rect = button.getBoundingClientRect();
  const x = rect.left + rect.width / 2;
  const y = rect.top + rect.height / 2;
  const radius = Math.hypot(Math.max(x, innerWidth - x), Math.max(y, innerHeight - y));
  document.documentElement.dataset.themeSwitching = '';
  const transition = document.startViewTransition(() => flushSync(apply));
  transition.ready
    .then(() =>
      document.documentElement.animate(
        {clipPath: [`circle(0px at ${x}px ${y}px)`, `circle(${radius}px at ${x}px ${y}px)`]},
        {duration: 520, easing: 'cubic-bezier(0.22, 1, 0.36, 1)', pseudoElement: '::view-transition-new(root)'},
      ),
    )
    .catch(() => {});
  transition.finished.finally(() => delete document.documentElement.dataset.themeSwitching);
}

function ColorModeToggle({className, buttonClassName, onChange}) {
  const isBrowser = useIsBrowser();
  const {colorMode} = useColorMode();
  const buttonRef = useRef(null);
  const next = colorMode === 'dark' ? 'light' : 'dark';
  const flip = useCallback(() => spread(buttonRef.current, () => onChange(next)), [onChange, next]);

  return (
    <div className={clsx(styles.toggle, className)}>
      <button
        ref={buttonRef}
        type="button"
        className={clsx('clean-btn', 'pa-theme-toggle', styles.button, buttonClassName)}
        onClick={flip}
        disabled={!isBrowser}
        role="switch"
        aria-checked={colorMode === 'dark'}
        aria-label="Dark mode"
        title={`Switch to ${next} mode`}>
        <SunMoon />
      </button>
    </div>
  );
}

export default React.memo(ColorModeToggle);
