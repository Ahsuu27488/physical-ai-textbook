import React, {useEffect, useRef, useState} from 'react';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './HeroScene.module.css';

// The headline and CTAs never wait for this. Everyone first sees a pre-rendered poster of the figure
// (design/make_poster.py); desktop-class devices then swap in the live WebGL scene once it has drawn its
// first frame. Phones, slow devices and reduced-motion users keep the poster and never download three.js.
export default function HeroScene() {
  const ref = useRef(null);
  const [ready, setReady] = useState(false);
  const poster = useBaseUrl('/img/hero-humanoid.webp');

  useEffect(() => {
    const el = ref.current;
    if (!el) return undefined;
    const forceLive = new URLSearchParams(window.location.search).has('poster'); // used to render the poster
    const reducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    const slow =
      window.innerWidth < 768 || (navigator.hardwareConcurrency || 8) <= 4 || (navigator.deviceMemory || 8) <= 4;
    if ((reducedMotion || slow) && !forceLive) return undefined;

    let dispose;
    let cancelled = false;
    const accent = getComputedStyle(el).getPropertyValue('--pa-hero-accent').trim() || '#c6f432';
    const start = () =>
      import('./humanoidScene')
        .then(({createHumanoidScene}) => {
          if (cancelled) return;
          dispose = createHumanoidScene(el, {reducedMotion, accent, onFirstFrame: () => setReady(true)});
        })
        .catch(() => {}); // no WebGL: the poster stays

    const idle = window.requestIdleCallback || ((cb) => setTimeout(cb, 200));
    const handle = idle(start, {timeout: 2500});
    return () => {
      cancelled = true;
      (window.cancelIdleCallback || clearTimeout)(handle);
      dispose?.();
    };
  }, []);

  return (
    <div className={styles.stage} aria-hidden="true">
      <div className={styles.glow} />
      <img className={styles.poster} src={poster} alt="" width="560" height="653" decoding="async" fetchPriority="high" data-hidden={ready} />
      <div ref={ref} className={styles.canvas} data-ready={ready} />
    </div>
  );
}
