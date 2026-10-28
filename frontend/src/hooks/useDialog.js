import {useEffect, useRef} from 'react';

const FOCUSABLE = 'a[href], button:not([disabled]), input:not([disabled]), select:not([disabled]), textarea:not([disabled]), [tabindex]:not([tabindex="-1"])';

// Modal behaviour for the shared .pa-dialog: Esc closes, Tab stays inside, the page behind doesn't scroll,
// focus goes back to whatever opened the dialog, and on phones the sheet stays above the on-screen keyboard.
// Returns a ref for the dialog element.
export function useDialog(onClose) {
  const ref = useRef(null);
  const closeRef = useRef(onClose);
  closeRef.current = onClose;

  useEffect(() => {
    const opener = document.activeElement;
    const el = ref.current;
    if (el && !el.contains(document.activeElement)) {
      (el.querySelector('[autofocus], [data-autofocus]') || el.querySelector(FOCUSABLE) || el).focus({preventScroll: true});
    }
    const onKey = (e) => {
      if (e.key === 'Escape') {
        e.preventDefault(); // keeps Rivet (which also listens for Esc) open underneath
        closeRef.current();
        return;
      }
      if (e.key !== 'Tab' || !ref.current) return;
      const items = [...ref.current.querySelectorAll(FOCUSABLE)].filter((n) => n.offsetParent !== null);
      if (!items.length) return;
      const first = items[0];
      const last = items[items.length - 1];
      if (e.shiftKey && document.activeElement === first) {
        e.preventDefault();
        last.focus();
      } else if (!e.shiftKey && document.activeElement === last) {
        e.preventDefault();
        first.focus();
      }
    };
    window.addEventListener('keydown', onKey, true); // capture: runs before Rivet's own Esc handler

    // The keyboard shrinks only the visual viewport (iOS Safari, Android Chrome): position:fixed and dvh keep the
    // full height, so a bottom sheet ended up under the keyboard. The overlay follows the visible area instead
    // (same approach as Rivet's panel), and the field being typed in is scrolled into view inside the sheet.
    const overlay = el?.closest('.pa-overlay');
    const vv = window.visualViewport;
    const fit = () => {
      if (!overlay || !vv) return;
      overlay.style.setProperty('--vv-height', `${vv.height}px`);
      overlay.style.setProperty('--vv-top', `${vv.offsetTop}px`);
      const field = document.activeElement;
      if (field && el.contains(field) && field.matches('input, textarea, select')) {
        requestAnimationFrame(() => field.scrollIntoView({block: 'nearest'}));
      }
    };
    fit();
    vv?.addEventListener('resize', fit);
    vv?.addEventListener('scroll', fit);
    const {overflow} = document.documentElement.style;
    document.documentElement.style.overflow = 'hidden';
    return () => {
      window.removeEventListener('keydown', onKey, true);
      vv?.removeEventListener('resize', fit);
      vv?.removeEventListener('scroll', fit);
      document.documentElement.style.overflow = overflow;
      if (opener && opener.isConnected && typeof opener.focus === 'function') opener.focus({preventScroll: true});
    };
  }, []);

  return ref;
}
