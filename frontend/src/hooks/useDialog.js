import {useEffect, useRef} from 'react';

const FOCUSABLE = 'a[href], button:not([disabled]), input:not([disabled]), select:not([disabled]), textarea:not([disabled]), [tabindex]:not([tabindex="-1"])';

// Modal behaviour for the shared .pa-dialog: Esc closes, Tab stays inside, the page behind doesn't scroll,
// and focus goes back to whatever opened the dialog. Returns a ref for the dialog element.
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
    const {overflow} = document.documentElement.style;
    document.documentElement.style.overflow = 'hidden';
    return () => {
      window.removeEventListener('keydown', onKey, true);
      document.documentElement.style.overflow = overflow;
      if (opener && opener.isConnected && typeof opener.focus === 'function') opener.focus({preventScroll: true});
    };
  }, []);

  return ref;
}
