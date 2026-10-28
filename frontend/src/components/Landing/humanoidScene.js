// LiDAR-style point-cloud humanoid for the landing hero.
// Plain three.js (WebGLRenderer + ShaderMaterial on THREE.Points, plus one LineSegments skeleton).
// Each point carries its surface normal, so the figure is lit (key light + rim) instead of flat dots.
// Loaded with a dynamic import, so none of this is in the main bundle.
import * as THREE from 'three';

// Body parts in metres (figure ~1.8 m tall, feet at y = 0). Capsules: [a, b, radius, weight];
// ellipsoids: [centre, radii, weight].
const CAPSULES = [
  [[0, 1.41, 0], [0, 1.5, 0], 0.05, 0.6], // neck
  [[-0.23, 1.35, 0], [-0.29, 1.07, -0.02], 0.048, 1], // left upper arm
  [[-0.29, 1.07, -0.02], [-0.31, 0.81, 0.02], 0.04, 1], // left forearm
  [[0.23, 1.35, 0], [0.29, 1.08, 0.02], 0.048, 1], // right upper arm
  [[0.29, 1.08, 0.02], [0.27, 0.98, 0.27], 0.04, 1], // right forearm, reaching forward
  [[-0.1, 0.84, 0], [-0.115, 0.47, 0.01], 0.072, 1.4], // thighs and shins
  [[-0.115, 0.47, 0.01], [-0.115, 0.09, -0.01], 0.056, 1.2],
  [[0.1, 0.84, 0], [0.12, 0.47, 0.02], 0.072, 1.4],
  [[0.12, 0.47, 0.02], [0.12, 0.09, 0], 0.056, 1.2],
];
const ELLIPSOIDS = [
  [[0, 1.63, 0], [0.105, 0.13, 0.118], 1.4], // head
  [[0, 1.25, 0], [0.205, 0.19, 0.125], 2.6], // chest
  [[0, 1.01, 0], [0.15, 0.13, 0.1], 1.2], // abdomen
  [[0, 0.87, 0], [0.175, 0.09, 0.115], 1.1], // pelvis
  [[-0.23, 1.36, 0], [0.065, 0.065, 0.065], 0.45], // shoulders
  [[0.23, 1.36, 0], [0.065, 0.065, 0.065], 0.45],
  [[-0.315, 0.73, 0.02], [0.03, 0.065, 0.045], 0.35], // hands
  [[0.27, 0.97, 0.33], [0.035, 0.045, 0.065], 0.35],
  [[-0.115, 0.04, 0.05], [0.05, 0.035, 0.11], 0.45], // feet
  [[0.12, 0.04, 0.05], [0.05, 0.035, 0.11], 0.45],
];
// Accent rings at the joints, so it reads as a machine.
const RINGS = [
  [[-0.29, 1.07, -0.02], 0.055], [[0.29, 1.08, 0.02], 0.055], // elbows
  [[-0.115, 0.47, 0.01], 0.075], [[0.12, 0.47, 0.02], 0.075], // knees
  [[0, 1.12, 0], 0.17], [[0, 1.44, 0], 0.07], // waist, collar
];
// Skeleton drawn inside the body: joint-to-joint segments.
const BONES = [
  [[0, 1.6, 0], [0, 1.44, 0]], [[0, 1.44, 0], [0, 0.88, 0]],
  [[-0.23, 1.36, 0], [0.23, 1.36, 0]], [[-0.13, 0.86, 0], [0.13, 0.86, 0]],
  [[-0.23, 1.36, 0], [-0.29, 1.07, -0.02]], [[-0.29, 1.07, -0.02], [-0.31, 0.79, 0.02]],
  [[0.23, 1.36, 0], [0.29, 1.08, 0.02]], [[0.29, 1.08, 0.02], [0.27, 0.98, 0.3]],
  [[-0.1, 0.86, 0], [-0.115, 0.47, 0.01]], [[-0.115, 0.47, 0.01], [-0.115, 0.08, 0]],
  [[0.1, 0.86, 0], [0.12, 0.47, 0.02]], [[0.12, 0.47, 0.02], [0.12, 0.08, 0]],
];

const HEIGHT = 1.9;
const INTRO_SECONDS = 3.2;

const VERTEX = /* glsl */ `
  attribute vec3 aNormal;
  attribute float aSeed;
  attribute float aKind; // 0 body, 1 accent detail, 2 floor, 3 scan ring
  uniform float uTime;
  uniform float uScanY;
  uniform float uIntro;   // 0 → 1 over the intro
  uniform float uSize;
  uniform float uPixelRatio;
  uniform vec2 uMouse;    // pointer in NDC
  uniform float uAspect;
  varying float vGlow;
  varying float vKind;
  varying float vAlpha;
  varying float vLight;

  float hash(float n) { return fract(sin(n) * 43758.5453123); }
  float easeOut(float x) { return 1.0 - pow(1.0 - clamp(x, 0.0, 1.0), 3.0); }

  void main() {
    vec3 p = position;
    float h = clamp(p.y / ${HEIGHT.toFixed(2)}, 0.0, 1.0);
    vKind = aKind;

    // Intro: the body is built bottom-up; each point rises out of drifting dust to its place.
    float local = clamp((uIntro * 1.4 - h * 0.9 - aSeed * 0.3) / 0.3, 0.0, 1.0);
    float arrive = easeOut(local);
    if (aKind < 1.5) {
      vec3 dust = vec3(hash(aSeed * 91.7) - 0.5, -0.35 - hash(aSeed * 13.1) * 0.3, hash(aSeed * 47.3) - 0.5) * 0.9;
      p = mix(p + dust, p, arrive);
      p.y += sin(uTime * 1.1 + p.y * 2.0) * 0.004 * arrive; // idle breathing
    } else if (aKind > 2.5) {
      p.y = uScanY; // the scan ring rides the scan plane
      float a = aSeed * 6.2831 + uTime * 0.6;
      float r = 0.42 + hash(aSeed * 7.7) * 0.36;
      p.x = cos(a) * r;
      p.z = sin(a) * r;
    }

    // Lighting from the per-point normal (view space): a soft key light plus a bright rim on the silhouette.
    vec3 n = normalize(normalMatrix * aNormal);
    float key = max(dot(n, normalize(vec3(-0.4, 0.6, 0.7))), 0.0);
    float rim = pow(1.0 - abs(n.z), 2.2);
    vLight = aKind < 0.5 ? 0.42 + key * 0.6 + rim * 0.95 : 1.0;

    float d = p.y - uScanY;
    vGlow = aKind < 1.5 ? exp(-d * d * 140.0) : 0.0;

    vec4 mv = modelViewMatrix * vec4(p, 1.0);
    vec4 clip = projectionMatrix * mv;

    // Points near the cursor ripple outward along their normal.
    if (aKind < 1.5) {
      vec2 ndc = clip.xy / clip.w;
      float md = distance(vec2(ndc.x * uAspect, ndc.y), vec2(uMouse.x * uAspect, uMouse.y));
      float push = (1.0 - smoothstep(0.0, 0.22, md)) * (0.035 + 0.015 * sin(uTime * 6.0 + aSeed * 30.0));
      mv = modelViewMatrix * vec4(p + aNormal * push * arrive, 1.0);
      clip = projectionMatrix * mv;
    }
    gl_Position = clip;

    float twinkle = 0.85 + 0.15 * sin(uTime * 2.3 + aSeed * 61.0);
    float depth = smoothstep(5.2, 3.0, -mv.z); // farther points dimmer
    if (aKind > 2.5) {
      vAlpha = 0.5 * step(0.001, uIntro);
    } else if (aKind > 1.5) {
      float r = length(position.xz);
      float wave = 0.5 + 0.5 * sin(r * 9.0 - uTime * 2.2); // radar pulse on the floor rings
      vAlpha = (1.0 - smoothstep(0.35, 1.3, r)) * (0.25 + wave * 0.45) * clamp(uIntro * 3.0, 0.0, 1.0);
    } else {
      vAlpha = arrive * twinkle * (0.7 + depth * 0.3);
    }

    float pop = 1.0 + (1.0 - arrive) * 1.6 * step(0.01, arrive);
    float size = uSize * (aKind > 1.5 ? 0.75 : 1.0) * (1.0 + vGlow * 1.4 + step(0.5, aKind) * step(aKind, 1.5) * 0.35) * pop;
    gl_PointSize = size * uPixelRatio / -mv.z;
  }
`;

const FRAGMENT = /* glsl */ `
  uniform vec3 uBase;
  uniform vec3 uRim;
  uniform vec3 uAccent;
  varying float vGlow;
  varying float vKind;
  varying float vAlpha;
  varying float vLight;

  void main() {
    vec2 c = gl_PointCoord - 0.5;
    float r2 = dot(c, c);
    if (r2 > 0.25) discard;
    float core = exp(-r2 * 38.0);  // crisp bright centre
    float halo = exp(-r2 * 8.0) * 0.45;
    float accent = clamp(vGlow + (vKind > 0.5 && vKind < 1.5 ? 0.9 : 0.0) + (vKind > 2.5 ? 1.0 : 0.0), 0.0, 1.0);
    vec3 col = mix(mix(uBase, uRim, clamp(vLight - 0.8, 0.0, 1.0)) * vLight, uAccent * 1.25, accent);
    float a = (core + halo) * vAlpha;
    gl_FragColor = vec4(col * a, a);
  }
`;

const tmpA = new THREE.Vector3();
const tmpB = new THREE.Vector3();
const axis = new THREE.Vector3();
const u = new THREE.Vector3();
const v = new THREE.Vector3();

function sampleCapsule(pos, nrm, a, b, r, rand) {
  tmpA.set(...a);
  tmpB.set(...b);
  axis.subVectors(tmpB, tmpA);
  const len = axis.length();
  axis.normalize();
  u.set(1, 0, 0);
  if (Math.abs(axis.dot(u)) > 0.9) u.set(0, 0, 1);
  u.cross(axis).normalize();
  v.crossVectors(axis, u);
  const t = rand() * len;
  const th = rand() * Math.PI * 2;
  nrm.copy(u).multiplyScalar(Math.cos(th)).addScaledVector(v, Math.sin(th));
  pos.copy(tmpA).addScaledVector(axis, t).addScaledVector(nrm, r);
}

function sampleEllipsoid(pos, nrm, c, radii, rand) {
  const z = rand() * 2 - 1;
  const th = rand() * Math.PI * 2;
  const s = Math.sqrt(1 - z * z);
  const x = s * Math.cos(th);
  const y = z;
  const w = s * Math.sin(th);
  pos.set(c[0] + radii[0] * x, c[1] + radii[1] * y, c[2] + radii[2] * w);
  nrm.set(x / radii[0], y / radii[1], w / radii[2]).normalize();
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

function buildPoints(count) {
  const rand = mulberry32(7);
  const floorCount = Math.round(count * 0.12);
  const ringCount = Math.round(count * 0.05);
  const scanCount = Math.round(count * 0.03);
  const bodyCount = count - floorCount - ringCount - scanCount;
  const positions = new Float32Array(count * 3);
  const normals = new Float32Array(count * 3);
  const seeds = new Float32Array(count);
  const kinds = new Float32Array(count);
  const pos = new THREE.Vector3();
  const nrm = new THREE.Vector3();

  const parts = [
    ...CAPSULES.map(([a, b, r, w]) => ({w: w * r * Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]) * 40, fn: () => sampleCapsule(pos, nrm, a, b, r, rand)})),
    ...ELLIPSOIDS.map(([c, radii, w]) => ({w: w * 0.12, fn: () => sampleEllipsoid(pos, nrm, c, radii, rand)})),
  ];
  const total = parts.reduce((sum, p) => sum + p.w, 0);
  let i = 0;
  const put = (kind) => {
    pos.toArray(positions, i * 3);
    nrm.toArray(normals, i * 3);
    kinds[i] = kind;
    i++;
  };
  for (const part of parts) {
    const n = Math.round((part.w / total) * bodyCount);
    for (let k = 0; k < n && i < bodyCount; k++) {
      part.fn();
      put(0);
    }
  }
  while (i < bodyCount + ringCount) {
    if (i % 4 === 0) {
      // visor band across the face
      const a = (rand() - 0.5) * 1.6;
      pos.set(Math.sin(a) * 0.112, 1.645 + (rand() - 0.5) * 0.03, Math.cos(a) * 0.122);
      nrm.set(Math.sin(a), 0, Math.cos(a));
    } else {
      const [c, r] = RINGS[i % RINGS.length];
      const th = rand() * Math.PI * 2;
      pos.set(c[0] + Math.cos(th) * r, c[1], c[2] + Math.sin(th) * r * 0.8);
      nrm.set(Math.cos(th), 0, Math.sin(th));
    }
    put(1);
  }
  while (i < bodyCount + ringCount + floorCount) {
    const ring = Math.floor(rand() * 9) + 1;
    const th = rand() * Math.PI * 2;
    const r = ring * 0.14 + (rand() - 0.5) * 0.008;
    pos.set(Math.cos(th) * r, 0, Math.sin(th) * r);
    nrm.set(0, 1, 0);
    put(2);
  }
  while (i < count) {
    pos.set(0, 0, 0); // placed by the shader on the scan plane
    nrm.set(0, 1, 0);
    put(3);
  }
  for (let j = 0; j < count; j++) seeds[j] = rand();

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute('aNormal', new THREE.BufferAttribute(normals, 3));
  geometry.setAttribute('aSeed', new THREE.BufferAttribute(seeds, 1));
  geometry.setAttribute('aKind', new THREE.BufferAttribute(kinds, 1));
  geometry.boundingSphere = new THREE.Sphere(new THREE.Vector3(0, 0.95, 0), 1.6);
  return geometry;
}

function buildSkeleton(accent) {
  const pts = new Float32Array(BONES.length * 6);
  BONES.forEach(([a, b], k) => {
    pts.set(a, k * 6);
    pts.set(b, k * 6 + 3);
  });
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(pts, 3));
  const material = new THREE.LineBasicMaterial({color: accent, transparent: true, opacity: 0, blending: THREE.AdditiveBlending, depthWrite: false});
  return new THREE.LineSegments(geometry, material);
}

const easeInOut = (x) => (x < 0.5 ? 4 * x * x * x : 1 - Math.pow(-2 * x + 2, 3) / 2);

/**
 * Mounts the scene into `container` and returns a cleanup function.
 * options: { reducedMotion, lowPower, accent, base, onFirstFrame }
 */
export function createHumanoidScene(container, options = {}) {
  const {reducedMotion = false, lowPower = false, accent = '#c6f432', base = '#c9d6ee', onFirstFrame} = options;

  const renderer = new THREE.WebGLRenderer({antialias: false, alpha: true, powerPreference: 'high-performance'});
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, lowPower ? 1.25 : 2));
  renderer.setClearColor(0x000000, 0);
  renderer.domElement.setAttribute('aria-hidden', 'true');
  container.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(30, 1, 0.1, 20);
  const camFrom = new THREE.Vector3(0, 1.25, 4.6);
  const camTo = new THREE.Vector3(0, 1.0, 3.75);
  const lookAt = new THREE.Vector3(0, 0.9, 0);
  camera.position.copy(reducedMotion ? camTo : camFrom);
  camera.lookAt(lookAt);

  const figure = new THREE.Group();
  figure.rotation.y = reducedMotion ? 0 : -0.55;
  scene.add(figure);

  const geometry = buildPoints(lowPower ? 16000 : 46000);
  const material = new THREE.ShaderMaterial({
    vertexShader: VERTEX,
    fragmentShader: FRAGMENT,
    transparent: true,
    depthWrite: false,
    blending: THREE.AdditiveBlending,
    uniforms: {
      uTime: {value: 0},
      uScanY: {value: 0.95},
      uIntro: {value: reducedMotion ? 1 : 0},
      uSize: {value: lowPower ? 12 : 9.5},
      uPixelRatio: {value: renderer.getPixelRatio()},
      uMouse: {value: new THREE.Vector2(9, 9)},
      uAspect: {value: 1},
      uBase: {value: new THREE.Color(base)},
      uRim: {value: new THREE.Color('#ffffff')},
      uAccent: {value: new THREE.Color(accent)},
    },
  });
  const cloud = new THREE.Points(geometry, material);
  figure.add(cloud);
  const skeleton = buildSkeleton(accent);
  figure.add(skeleton);

  // pointer: the figure turns toward the cursor, and nearby points ripple (fine pointers only)
  const target = {x: 0, y: 0};
  const mouse = material.uniforms.uMouse.value;
  const fine = window.matchMedia('(hover: hover) and (pointer: fine)').matches;
  const onPointer = (e) => {
    target.x = (e.clientX / window.innerWidth) * 2 - 1;
    target.y = (e.clientY / window.innerHeight) * 2 - 1;
    const r = renderer.domElement.getBoundingClientRect();
    mouse.set(((e.clientX - r.left) / r.width) * 2 - 1, -(((e.clientY - r.top) / r.height) * 2 - 1));
  };
  const onLeave = () => mouse.set(9, 9);
  if (fine && !reducedMotion) {
    window.addEventListener('pointermove', onPointer, {passive: true});
    document.addEventListener('pointerleave', onLeave);
  }

  const resize = () => {
    const {clientWidth: w, clientHeight: h} = container;
    if (!w || !h) return;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    material.uniforms.uAspect.value = w / h;
  };
  const ro = new ResizeObserver(resize);
  ro.observe(container);
  resize();

  const timer = new THREE.Timer();
  timer.connect(document);
  let first = true;
  let t = 0; // scene time: advances only while rendering, so the intro waits until the figure is on screen
  const u = material.uniforms;
  const render = () => {
    timer.update();
    t += Math.min(timer.getDelta(), 1 / 20);
    u.uTime.value = t;
    if (!reducedMotion) {
      const intro = Math.min(t / INTRO_SECONDS, 1);
      u.uIntro.value = intro;
      const e = easeInOut(intro);
      camera.position.lerpVectors(camFrom, camTo, e);
      camera.lookAt(lookAt);
      // the scan line leads the build, then keeps sweeping head → feet → head
      const front = Math.min(Math.max((intro * 1.4 - 0.15) / 0.9, 0), 1.05) * HEIGHT;
      u.uScanY.value = intro < 1 ? front : 0.96 + 0.98 * Math.cos((t - INTRO_SECONDS) * 0.7);
      skeleton.material.opacity = Math.max(0, (intro - 0.55) / 0.45) * 0.28;
      const idle = fine ? 0 : Math.sin(t * 0.35) * 0.35;
      const yaw = intro < 1 ? -0.55 * (1 - e) : target.x * 0.55 + idle;
      figure.rotation.y += (yaw - figure.rotation.y) * (intro < 1 ? 0.2 : 0.05);
      figure.rotation.x += (target.y * 0.06 * e - figure.rotation.x) * 0.05;
    } else {
      skeleton.material.opacity = 0.28;
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
    document.removeEventListener('pointerleave', onLeave);
    geometry.dispose();
    material.dispose();
    skeleton.geometry.dispose();
    skeleton.material.dispose();
    renderer.dispose();
    renderer.domElement.remove();
  };
}
