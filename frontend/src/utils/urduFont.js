// Noto Nastaliq Urdu is large, and only the Urdu sample and Urdu translations need it, so it is
// fetched on demand instead of blocking every page.
const HREF = 'https://fonts.googleapis.com/css2?family=Noto+Nastaliq+Urdu:wght@400;600&display=swap';

export function loadUrduFont() {
  if (typeof document === 'undefined' || document.querySelector(`link[href="${HREF}"]`)) return;
  const link = document.createElement('link');
  link.rel = 'stylesheet';
  link.href = HREF;
  document.head.appendChild(link);
}
