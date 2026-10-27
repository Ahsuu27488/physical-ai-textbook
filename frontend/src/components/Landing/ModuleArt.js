import React from 'react';
import styles from './ModuleArt.module.css';

// Small line drawings, one per module. Neutral strokes; the accent marks the "signal" in each.
const ART = [
  // 01 ROS 2: nodes on a graph, one message travelling along a topic
  <svg viewBox="0 0 240 140" key="ros">
    <path className={styles.line} d="M40 70 L110 34 L190 52 M110 34 L120 106 L190 52 M40 70 L120 106" />
    <path className={styles.signal} d="M40 70 L110 34 L190 52" pathLength="100" />
    <circle className={styles.node} cx="40" cy="70" r="9" />
    <circle className={styles.node} cx="110" cy="34" r="9" />
    <circle className={styles.nodeOn} cx="190" cy="52" r="9" />
    <circle className={styles.node} cx="120" cy="106" r="9" />
    <text className={styles.label} x="28" y="96">/camera</text>
    <text className={styles.label} x="160" y="80">/planner</text>
  </svg>,
  // 02 Digital twin: a perspective floor, a robot block and its dashed twin
  <svg viewBox="0 0 240 140" key="twin">
    {[0, 1, 2, 3, 4].map((i) => (
      <path key={`h${i}`} className={styles.grid} d={`M${30 - i * 6} ${84 + i * 12} L${210 + i * 6} ${84 + i * 12}`} />
    ))}
    {[0, 1, 2, 3, 4, 5, 6].map((i) => (
      <path key={`v${i}`} className={styles.grid} d={`M${60 + i * 20} 84 L${12 + i * 36} 132`} />
    ))}
    <path className={styles.line} d="M70 84 V50 L86 40 H116 V74 L100 84 Z M70 50 H100 L116 40 M100 50 V84" />
    <path className={styles.twin} d="M140 84 V50 L156 40 H186 V74 L170 84 Z M140 50 H170 L186 40 M170 50 V84" />
    <path className={styles.signal} d="M120 62 H136" pathLength="100" />
  </svg>,
  // 03 Isaac perception: a camera, its frustum, and a detection box
  <svg viewBox="0 0 240 140" key="isaac">
    <rect className={styles.line} x="22" y="60" width="26" height="20" rx="4" />
    <path className={styles.grid} d="M48 64 L206 22 M48 76 L206 122" />
    <rect className={styles.line} x="120" y="54" width="44" height="44" rx="6" />
    <rect className={styles.detect} x="112" y="46" width="60" height="60" rx="4" />
    <rect className={styles.tag} x="112" y="32" width="58" height="12" rx="3" />
    <text className={styles.tagText} x="116" y="41">box 0.97</text>
  </svg>,
  // 04 VLA: an instruction becomes a path to a gripper
  <svg viewBox="0 0 240 140" key="vla">
    <rect className={styles.line} x="18" y="24" width="88" height="44" rx="12" />
    <path className={styles.line} d="M36 68 L30 82 L50 68" />
    <path className={styles.grid} d="M32 40 H90 M32 52 H72" />
    <path className={styles.signal} d="M106 52 C150 52 150 104 186 104" pathLength="100" />
    <path className={styles.line} d="M186 92 V116 M186 92 L204 84 V96 M186 116 L204 124 V112" />
    <circle className={styles.nodeOn} cx="214" cy="104" r="6" />
  </svg>,
];

export function ModuleArt({index}) {
  return (
    <div className={styles.art} aria-hidden="true">
      {ART[index]}
    </div>
  );
}
