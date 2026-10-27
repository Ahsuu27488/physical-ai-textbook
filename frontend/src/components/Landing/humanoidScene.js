// LiDAR-style point-cloud humanoid for the landing hero.
// Plain three.js (WebGLRenderer + one ShaderMaterial on THREE.Points): one draw call for the body,
// one for the floor. Loaded with a dynamic import so none of this is in the main bundle.
import * as THREE from 'three';

// Body parts in metres (figure is ~1.8 m tall, feet at y = 0). Capsules are [a, b, radius],
// ellipsoids are [centre, radii]. `w` scales how many points a part gets.
const CAPSULES = [
  // neck, spine
  [[0, 1.41, 0], [0, 1.5, 0], 0.05, 0.6],
  // arms: shoulder -> elbow -> wrist. The right forearm reaches forward, as if about to pick something up.
  [[-0.23, 1.35, 0], [-0.29, 1.07, -0.02], 0.048, 1],
  [[-0.29, 1.07, -0.02], [-0.31, 0.81, 0.02], 0.04, 1],
  [[0.23, 1.35, 0], [0.29, 1.08, 0.02], 0.048, 1],
  [[0.29, 1.08, 0.02], [0.27, 0.98, 0.27], 0.04, 1],
  // legs: hip -> knee -> ankle
  [[-0.1, 0.84, 0], [-0.115, 0.47, 0.01], 0.072, 1.4],
  [[-0.115, 0.47, 0.01], [-0.115, 0.09, -0.01], 0.056, 1.2],
  [[0.1, 0.84, 0], [0.12, 0.47, 0.02], 0.072, 1.4],
  [[0.12, 0.47, 0.02], [0.12, 0.09, 0], 0.056, 1.2],
];
const ELLIPSOIDS = [
  [[0, 1.63, 0], [0.105, 0.13, 0.118], 1.3], // head
  [[0, 1.25, 0], [0.205, 0.19, 0.125], 2.4], // chest
  [[0, 1.01, 0], [0.15, 0.13, 0.1], 1.2], // abdomen
  [[0, 0.87, 0], [0.175, 0.09, 0.115], 1.1], // pelvis
  [[-0.23, 1.36, 0], [0.065, 0.065, 0.065], 0.4], // shoulder joints
  [[0.23, 1.36, 0], [0.065, 0.065, 0.065], 0.4],
  [[-0.315, 0.73, 0.02], [0.03, 0.065, 0.045], 0.35], // hands
  [[0.27, 0.97, 0.33], [0.035, 0.045, 0.065], 0.35],
  [[-0.115, 0.04, 0.05], [0.05, 0.035, 0.11], 0.45], // feet
  [[0.12, 0.04, 0.05], [0.05, 0.035, 0.11], 0.45],
];
// Bright accent rings at the joints and a visor band, so it reads as a machine, not a mannequin.
const RINGS = [
  [[-0.29, 1.07, -0.02], 0.055], [[0.29, 1.08, 0.02], 0.055], // elbows
  [[-0.115, 0.47, 0.01], 0.075], [[0.12, 0.47, 0.02], 0.075], // knees
  [[0, 1.12, 0], 0.17], [[0, 1.44, 0], 0.07], // waist line, collar
];

const VERTEX = /* glsl */ `
  attribute float aSeed;
  attribute float aKind; // 0 body, 1 accent detail, 2 floor
  uniform float uTime;
  uniform float uScanY;
  uniform float uIntro;
  uniform float uSize;
  uniform float uPixelRatio;
  varying float vGlow;
  varying float vKind;
  varying float vFade;

  float hash(float n) { return fract(sin(n) * 43758.5453123); }

  void main() {
    vec3 p = position;
    // Assemble: every point flies in from a scattered position, staggered by its seed.
    float t = clamp((uIntro - aSeed * 0.45) / 0.55, 0.0, 1.0);
    t = 1.0 - pow(1.0 - t, 3.0);
    vec3 scatter = vec3(hash(aSeed * 91.7) - 0.5, hash(aSeed * 13.1) - 0.2, hash(aSeed * 47.3) - 0.5) * 2.4;
    p = mix(p + scatter, p, t);
    // idle "breathing" so it never looks frozen
    p.y += sin(uTime * 1.1 + p.y * 2.0) * 0.004 * step(aKind, 1.5);

    float d = p.y - uScanY;
    vGlow = exp(-d * d * 90.0) * step(aKind, 1.5);
    vKind = aKind;
    vFade = aKind > 1.5 ? 1.0 - smoothstep(0.4, 1.35, length(p.xz)) : t;

    vec4 mv = modelViewMatrix * vec4(p, 1.0);
    gl_Position = projectionMatrix * mv;
    float size = uSize * (aKind > 1.5 ? 0.7 : 1.0) * (1.0 + vGlow * 1.6 + aKind * step(aKind, 1.5) * 0.4);
    gl_PointSize = size * uPixelRatio / -mv.z;
  }
`;

const FRAGMENT = /* glsl */ `
  uniform vec3 uBase;
  uniform vec3 uAccent;
  varying float vGlow;
  varying float vKind;
  varying float vFade;

  void main() {
    float r = length(gl_PointCoord - 0.5);
    if (r > 0.5) discard;
    float a = smoothstep(0.5, 0.1, r);
    float accent = clamp(vGlow + (vKind > 0.5 && vKind < 1.5 ? 0.85 : 0.0), 0.0, 1.0);
    vec3 col = mix(uBase, uAccent, accent);
    float alpha = a * vFade * (vKind > 1.5 ? 0.4 : 0.72 + accent * 0.28);
    gl_FragColor = vec4(col * alpha, alpha);
  }
`;

function randomOnCapsule(out, a, b, r, rand) {
  const ax = new THREE.Vector3(...b).sub(new THREE.Vector3(...a));
  const len = ax.length();
  ax.normalize();
  const u = new THREE.Vector3(1, 0, 0);
  if (Math.abs(ax.dot(u)) > 0.9) u.set(0, 0, 1);
  u.cross(ax).normalize();
  const v = new THREE.Vector3().crossVectors(ax, u);
  const t = rand() * len;
  const th = rand() * Math.PI * 2;
  out.set(...a).addScaledVector(ax, t).addScaledVector(u, Math.cos(th) * r).addScaledVector(v, Math.sin(th) * r);
  return out;
}

function randomOnEllipsoid(out, c, radii, rand) {
  // uniform direction, then scaled: dense enough at these sizes
  const z = rand() * 2 - 1;
  const th = rand() * Math.PI * 2;
  const s = Math.sqrt(1 - z * z);
  out.set(c[0] + radii[0] * s * Math.cos(th), c[1] + radii[1] * z, c[2] + radii[2] * s * Math.sin(th));
  return out;
}

function mulberry32(seed) {
  return () => {
    seed |= 0;
    seed = (seed + 0x6d2b79f5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function buildGeometry(count) {
  const rand = mulberry32(7);
  const floorCount = Math.round(count * 0.18);
  const ringCount = Math.round(count * 0.07);
  const bodyCount = count - floorCount - ringCount;
  const positions = new Float32Array(count * 3);
  const seeds = new Float32Array(count);
  const kinds = new Float32Array(count);
  const tmp = new THREE.Vector3();

  const parts = [
    ...CAPSULES.map(([a, b, r, w]) => ({w: w * r * new THREE.Vector3(...a).distanceTo(new THREE.Vector3(...b)) * 40, fn: () => randomOnCapsule(tmp, a, b, r, rand)})),
    ...ELLIPSOIDS.map(([c, radii, w]) => ({w: w * 0.12, fn: () => randomOnEllipsoid(tmp, c, radii, rand)})),
  ];
  const total = parts.reduce((s, p) => s + p.w, 0);
  let i = 0;
  for (const part of parts) {
    const n = Math.round((part.w / total) * bodyCount);
    for (let k = 0; k < n && i < bodyCount; k++, i++) {
      part.fn().toArray(positions, i * 3);
      kinds[i] = 0;
    }
  }
  for (; i < bodyCount + ringCount; i++) {
    const [c, r] = RINGS[i % RINGS.length];
    const th = rand() * Math.PI * 2;
    // a visor band on the head for every 4th accent point
    if (i % 4 === 0) {
      const a = (rand() - 0.5) * 1.6;
      tmp.set(Math.sin(a) * 0.112, 1.645 + (rand() - 0.5) * 0.03, Math.cos(a) * 0.122);
    } else {
      tmp.set(c[0] + Math.cos(th) * r, c[1], c[2] + Math.sin(th) * r * 0.8);
    }
    tmp.toArray(positions, i * 3);
    kinds[i] = 1;
  }
  for (; i < count; i++) {
    // floor: concentric scan rings, like a lidar sweep hitting the ground
    const ring = Math.floor(rand() * 9) + 1;
    const th = rand() * Math.PI * 2;
    const r = ring * 0.14 + (rand() - 0.5) * 0.01;
    tmp.set(Math.cos(th) * r, 0, Math.sin(th) * r);
    tmp.toArray(positions, i * 3);
    kinds[i] = 2;
  }
  for (let j = 0; j < count; j++) seeds[j] = rand();

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute('aSeed', new THREE.BufferAttribute(seeds, 1));
  geometry.setAttribute('aKind', new THREE.BufferAttribute(kinds, 1));
  geometry.computeBoundingSphere();
  return geometry;
}

/**
 * Mounts the scene into `container` and returns a cleanup function.
 * options: { reducedMotion, lowPower, accent, base, onFirstFrame }
 */
export function createHumanoidScene(container, options = {}) {
  const {reducedMotion = false, lowPower = false, accent = '#c6f432', base = '#b4c0d4', onFirstFrame} = options;

  const renderer = new THREE.WebGLRenderer({antialias: false, alpha: true, powerPreference: 'high-performance'});
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, lowPower ? 1.25 : 2));
  renderer.setClearColor(0x000000, 0);
  renderer.domElement.setAttribute('aria-hidden', 'true');
  container.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(30, 1, 0.1, 20);
  camera.position.set(0, 1.0, 3.75);
  camera.lookAt(0, 0.9, 0);

  const geometry = buildGeometry(lowPower ? 7000 : 16000);
  const material = new THREE.ShaderMaterial({
    vertexShader: VERTEX,
    fragmentShader: FRAGMENT,
    transparent: true,
    depthWrite: false,
    blending: THREE.AdditiveBlending,
    uniforms: {
      uTime: {value: 0},
      uScanY: {value: 0.9},
      uIntro: {value: reducedMotion ? 2 : 0},
      uSize: {value: lowPower ? 16 : 13},
      uPixelRatio: {value: renderer.getPixelRatio()},
      uBase: {value: new THREE.Color(base)},
      uAccent: {value: new THREE.Color(accent)},
    },
  });
  const figure = new THREE.Points(geometry, material);
  scene.add(figure);

  // pointer: the figure turns toward the cursor (fine pointers only)
  const target = {x: 0, y: 0};
  const fine = window.matchMedia('(hover: hover) and (pointer: fine)').matches;
  const onPointer = (e) => {
    target.x = (e.clientX / window.innerWidth) * 2 - 1;
    target.y = (e.clientY / window.innerHeight) * 2 - 1;
  };
  if (fine && !reducedMotion) window.addEventListener('pointermove', onPointer, {passive: true});

  const resize = () => {
    const {clientWidth: w, clientHeight: h} = container;
    if (!w || !h) return;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  };
  const ro = new ResizeObserver(resize);
  ro.observe(container);
  resize();

  const timer = new THREE.Timer();
  timer.connect(document);
  let first = true;
  const render = () => {
    timer.update();
    const t = timer.getElapsed();
    const u = material.uniforms;
    u.uTime.value = t;
    if (!reducedMotion) {
      u.uIntro.value = Math.min(u.uIntro.value + timer.getDelta() * 0.55, 2);
      // scan beam sweeps head -> feet and back
      u.uScanY.value = 0.95 + Math.sin(t * 0.7) * 0.95;
      const idle = fine ? 0 : Math.sin(t * 0.35) * 0.35;
      figure.rotation.y += (target.x * 0.55 + idle - figure.rotation.y) * 0.05;
      figure.rotation.x += (target.y * 0.06 - figure.rotation.x) * 0.05;
    }
    renderer.render(scene, camera);
    if (first) {
      first = false;
      onFirstFrame?.();
    }
  };

  // Only animate while visible on screen and the tab is in front.
  let onScreen = true;
  const sync = () => renderer.setAnimationLoop(onScreen && !document.hidden && !reducedMotion ? render : null);
  const io = new IntersectionObserver(([entry]) => {
    onScreen = entry.isIntersecting;
    sync();
  });
  io.observe(container);
  document.addEventListener('visibilitychange', sync);
  render(); // first frame right away (and the only one with reduced motion)
  sync();

  return () => {
    renderer.setAnimationLoop(null);
    io.disconnect();
    ro.disconnect();
    timer.dispose();
    document.removeEventListener('visibilitychange', sync);
    window.removeEventListener('pointermove', onPointer);
    geometry.dispose();
    material.dispose();
    renderer.dispose();
    renderer.domElement.remove();
  };
}
