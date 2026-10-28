import React, {useEffect, useRef, useState} from 'react';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './HeroScene.module.css';

// The headline and CTAs never wait for this. Every device with motion allowed plays the live scene, whose intro
// builds the figure from nothing (phones and mid-range machines get a lighter cloud, D-023). Reduced motion, data
// saver and very weak devices keep the pre-rendered poster (design/make_poster.py) and never download three.js;
// if WebGL fails, the poster comes back.
export default function HeroScene() {
  const ref = useRef(null);
  const [ready, setReady] = useState(false);
  const [fallback, setFallback] = useState(false); // show the poster on desktop too (no live scene)
  const poster = useBaseUrl('/img/hero-humanoid.webp');

  useEffect(() => {
    const el = ref.current;
    if (!el) return undefined;
    const forceLive = new URLSearchParams(window.location.search).has('poster'); // used to render the poster
    const reducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    const cores = navigator.hardwareConcurrency || 8;
    const memory = navigator.deviceMemory || 8;
    const weak = cores <= 2 || memory <= 2 || navigator.connection?.saveData;
    const lowPower = window.innerWidth < 768 || cores <= 4 || memory <= 4;
    if ((reducedMotion || weak) && !forceLive) {
      setFallback(true);
      return undefined;
    }

    let dispose;
    let cancelled = false;
    const accent = getComputedStyle(el).getPropertyValue('--pa-hero-accent').trim() || '#c6f432';
    const start = () =>
      import('./humanoidScene')
        .then(({createHumanoidScene}) => {
          if (cancelled) return;
          dispose = createHumanoidScene(el, {reducedMotion, lowPower, accent, onFirstFrame: () => setReady(true)});
        })
        .catch(() => setFallback(true)); // no WebGL: show the poster

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
      <img className={styles.poster} src={poster} alt="" width="560" height="653" decoding="async" fetchPriority="high" data-fallback={fallback} />
      <div ref={ref} className={styles.canvas} data-ready={ready} />
    </div>
  );
}
