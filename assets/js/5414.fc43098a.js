"use strict";(self.webpackChunkmy_website_1=self.webpackChunkmy_website_1||[]).push([["5414"],{1157(e,t,a){a.d(t,{createHumanoidScene:()=>f});var i=a(4922),o=a(9437);let n=[[[0,1.41,0],[0,1.5,0],.05,.6],[[-.23,1.35,0],[-.29,1.07,-.02],.048,1],[[-.29,1.07,-.02],[-.31,.81,.02],.04,1],[[.23,1.35,0],[.29,1.08,.02],.048,1],[[.29,1.08,.02],[.27,.98,.27],.04,1],[[-.1,.84,0],[-.115,.47,.01],.072,1.4],[[-.115,.47,.01],[-.115,.09,-.01],.056,1.2],[[.1,.84,0],[.12,.47,.02],.072,1.4],[[.12,.47,.02],[.12,.09,0],.056,1.2]],r=[[[0,1.63,0],[.105,.13,.118],1.4],[[0,1.25,0],[.205,.19,.125],2.6],[[0,1.01,0],[.15,.13,.1],1.2],[[0,.87,0],[.175,.09,.115],1.1],[[-.23,1.36,0],[.065,.065,.065],.45],[[.23,1.36,0],[.065,.065,.065],.45],[[-.315,.73,.02],[.03,.065,.045],.35],[[.27,.97,.33],[.035,.045,.065],.35],[[-.115,.04,.05],[.05,.035,.11],.45],[[.12,.04,.05],[.05,.035,.11],.45]],l=[[[-.29,1.07,-.02],.055],[[.29,1.08,.02],.055],[[-.115,.47,.01],.075],[[.12,.47,.02],.075],[[0,1.12,0],.17],[[0,1.44,0],.07]],s=[[[0,1.6,0],[0,1.44,0]],[[0,1.44,0],[0,.88,0]],[[-.23,1.36,0],[.23,1.36,0]],[[-.13,.86,0],[.13,.86,0]],[[-.23,1.36,0],[-.29,1.07,-.02]],[[-.29,1.07,-.02],[-.31,.79,.02]],[[.23,1.36,0],[.29,1.08,.02]],[[.29,1.08,.02],[.27,.98,.3]],[[-.1,.86,0],[-.115,.47,.01]],[[-.115,.47,.01],[-.115,.08,0]],[[.1,.86,0],[.12,.47,.02]],[[.12,.47,.02],[.12,.08,0]]],c=`
  attribute vec3 aNormal;
  attribute float aSeed;
  attribute float aKind; // 0 body, 1 accent detail, 2 floor, 3 scan ring
  uniform float uTime;
  uniform float uScanY;
  uniform float uIntro;   // 0 \u{2192} 1 over the intro
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
    float h = clamp(p.y / 1.90, 0.0, 1.0);
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
`,d=`
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
`,u=new i.Pq0,p=new i.Pq0,h=new i.Pq0,m=new i.Pq0,v=new i.Pq0;function f(e,t={}){let a,w,M,{reducedMotion:g=!1,lowPower:y=!1,accent:x="#c6f432",base:b="#c9d6ee",onFirstFrame:S}=t,A=new o.JeP({antialias:!1,alpha:!0,powerPreference:"high-performance"});A.setPixelRatio(Math.min(window.devicePixelRatio,y?1.25:2)),A.setClearColor(0,0),A.domElement.setAttribute("aria-hidden","true"),e.appendChild(A.domElement);let P=new i.Z58,K=new i.ubm(30,1,.1,20),z=new i.Pq0(0,1.25,4.6),L=new i.Pq0(0,1,3.75),I=new i.Pq0(0,.9,0);K.position.copy(g?L:z),K.lookAt(I);let T=new i.YJl;T.rotation.y=g?0:-.55,P.add(T);let k=function(e){var t;let a=(t=7,()=>{t|=0;let e=Math.imul((t=t+0x6d2b79f5|0)^t>>>15,1|t);return(((e=e+Math.imul(e^e>>>7,61|e)^e)^e>>>14)>>>0)/0x100000000}),o=Math.round(.12*e),s=Math.round(.05*e),c=Math.round(.03*e),d=e-o-s-c,f=new Float32Array(3*e),w=new Float32Array(3*e),M=new Float32Array(e),g=new Float32Array(e),y=new i.Pq0,x=new i.Pq0,b=[...n.map(([e,t,i,o])=>({w:o*i*Math.hypot(t[0]-e[0],t[1]-e[1],t[2]-e[2])*40,fn:()=>{let o,n,r;return u.set(...e),p.set(...t),h.subVectors(p,u),o=h.length(),h.normalize(),m.set(1,0,0),Math.abs(h.dot(m))>.9&&m.set(0,0,1),m.cross(h).normalize(),v.crossVectors(h,m),n=a()*o,r=a()*Math.PI*2,void(x.copy(m).multiplyScalar(Math.cos(r)).addScaledVector(v,Math.sin(r)),y.copy(u).addScaledVector(h,n).addScaledVector(x,i))}})),...r.map(([e,t,i])=>({w:.12*i,fn:()=>{let i,o,n,r,l;return i=2*a()-1,o=a()*Math.PI*2,r=(n=Math.sqrt(1-i*i))*Math.cos(o),l=n*Math.sin(o),void(y.set(e[0]+t[0]*r,e[1]+t[1]*i,e[2]+t[2]*l),x.set(r/t[0],i/t[1],l/t[2]).normalize())}}))],S=b.reduce((e,t)=>e+t.w,0),A=0,P=e=>{y.toArray(f,3*A),x.toArray(w,3*A),g[A]=e,A++};for(let e of b){let t=Math.round(e.w/S*d);for(let a=0;a<t&&A<d;a++)e.fn(),P(0)}for(;A<d+s;){if(A%4==0){let e=(a()-.5)*1.6;y.set(.112*Math.sin(e),1.645+(a()-.5)*.03,.122*Math.cos(e)),x.set(Math.sin(e),0,Math.cos(e))}else{let[e,t]=l[A%l.length],i=a()*Math.PI*2;y.set(e[0]+Math.cos(i)*t,e[1],e[2]+Math.sin(i)*t*.8),x.set(Math.cos(i),0,Math.sin(i))}P(1)}for(;A<d+s+o;){let e=Math.floor(9*a())+1,t=a()*Math.PI*2,i=.14*e+(a()-.5)*.008;y.set(Math.cos(t)*i,0,Math.sin(t)*i),x.set(0,1,0),P(2)}for(;A<e;)y.set(0,0,0),x.set(0,1,0),P(3);for(let t=0;t<e;t++)M[t]=a();let K=new i.LoY;return K.setAttribute("position",new i.THS(f,3)),K.setAttribute("aNormal",new i.THS(w,3)),K.setAttribute("aSeed",new i.THS(M,1)),K.setAttribute("aKind",new i.THS(g,1)),K.boundingSphere=new i.iyt(new i.Pq0(0,.95,0),1.6),K}(y?16e3:46e3),q=new i.BKk({vertexShader:c,fragmentShader:d,transparent:!0,depthWrite:!1,blending:2,uniforms:{uTime:{value:0},uScanY:{value:.95},uIntro:{value:+!!g},uSize:{value:y?12:9.5},uPixelRatio:{value:A.getPixelRatio()},uMouse:{value:new i.I9Y(9,9)},uAspect:{value:1},uBase:{value:new i.Q1f(b)},uRim:{value:new i.Q1f("#ffffff")},uAccent:{value:new i.Q1f(x)}}}),E=new i.ONl(k,q);T.add(E);let R=(a=new Float32Array(6*s.length),s.forEach(([e,t],i)=>{a.set(e,6*i),a.set(t,6*i+3)}),(w=new i.LoY).setAttribute("position",new i.THS(a,3)),M=new i.mrM({color:x,transparent:!0,opacity:0,blending:2,depthWrite:!1}),new i.DXC(w,M));T.add(R);let Y={x:0,y:0},C=q.uniforms.uMouse.value,H=window.matchMedia("(hover: hover) and (pointer: fine)").matches,V=e=>{Y.x=e.clientX/window.innerWidth*2-1,Y.y=e.clientY/window.innerHeight*2-1;let t=A.domElement.getBoundingClientRect();C.set((e.clientX-t.left)/t.width*2-1,-((e.clientY-t.top)/t.height*2-1))},_=()=>C.set(9,9);H&&!g&&(window.addEventListener("pointermove",V,{passive:!0}),document.addEventListener("pointerleave",_));let F=()=>{let{clientWidth:t,clientHeight:a}=e;t&&a&&(A.setSize(t,a,!1),K.aspect=t/a,K.updateProjectionMatrix(),q.uniforms.uAspect.value=t/a)},G=new ResizeObserver(F);G.observe(e),F();let N=new i.M4G;N.connect(document);let B=!0,O=q.uniforms,W=()=>{N.update();let e=N.getElapsed();if(O.uTime.value=e,g)R.material.opacity=.28;else{let t=Math.min(e/3.2,1);O.uIntro.value=t;let a=t<.5?4*t*t*t:1-Math.pow(-2*t+2,3)/2;K.position.lerpVectors(z,L,a),K.lookAt(I);let i=1.9*Math.min(Math.max((1.4*t-.15)/.9,0),1.05);O.uScanY.value=t<1?i:.96+.98*Math.cos((e-3.2)*.7),R.material.opacity=.28*Math.max(0,(t-.55)/.45);let o=H?0:.35*Math.sin(.35*e),n=t<1?-.55*(1-a):.55*Y.x+o;T.rotation.y+=(n-T.rotation.y)*(t<1?.2:.05),T.rotation.x+=(.06*Y.y*a-T.rotation.x)*.05}A.render(P,K),B&&(B=!1,S?.())},j=!0,Q=()=>A.setAnimationLoop(!j||document.hidden||g?null:W),X=new IntersectionObserver(([e])=>{j=e.isIntersecting,Q()});return X.observe(e),document.addEventListener("visibilitychange",Q),W(),Q(),()=>{A.setAnimationLoop(null),X.disconnect(),G.disconnect(),N.dispose(),document.removeEventListener("visibilitychange",Q),window.removeEventListener("pointermove",V),document.removeEventListener("pointerleave",_),k.dispose(),q.dispose(),R.geometry.dispose(),R.material.dispose(),A.dispose(),A.domElement.remove()}}}}]);