import {useEffect} from 'react';
import {useLocation} from '@docusaurus/router';
import {useAuth} from '@site/src/hooks/useAuth';
import {UNCOMPLETE_EVENT, mergeRemote, setLastLesson, snapshot, subscribe} from '@site/src/utils/progress';

// Keeps learning progress in the account when signed in (GET → merge → PUT, then PUT on every change),
// and remembers the last lesson read for "Continue where you left off". Renders nothing.
export default function ProgressSync() {
  const {isAuthenticated, authFetch} = useAuth();
  const {pathname} = useLocation();

  useEffect(() => {
    if (!/\/docs\/.+/.test(pathname)) return undefined;
    const t = setTimeout(() => {
      const title = document.querySelector('article h1')?.textContent?.trim();
      if (title) setLastLesson(pathname, title);
    }, 400);
    return () => clearTimeout(t);
  }, [pathname]);

  useEffect(() => {
    if (!isAuthenticated) return undefined;
    let stopped = false;
    let timer;
    let stopSync = () => {};
    // un-completing a lesson must be removed on the server too (PUT only adds)
    const onUndo = (e) =>
      authFetch(`/api/v1/progress/completed/${encodeURI(e.detail.docId)}`, {method: 'DELETE'}).catch(() => {});
    window.addEventListener(UNCOMPLETE_EVENT, onUndo);
    const push = () =>
      authFetch('/api/v1/progress', {method: 'PUT', body: JSON.stringify(snapshot())}).catch(() => {});

    (async () => {
      try {
        const res = await authFetch('/api/v1/progress');
        if (!res.ok) return; // older backend without progress sync: stay local-only
        mergeRemote(await res.json());
        await push();
      } catch {
        return;
      }
      if (stopped) return;
      const unsubscribe = subscribe(() => {
        clearTimeout(timer);
        timer = setTimeout(push, 1200);
      });
      stopSync = unsubscribe;
    })();

    return () => {
      stopped = true;
      clearTimeout(timer);
      stopSync();
      window.removeEventListener(UNCOMPLETE_EVENT, onUndo);
    };
  }, [isAuthenticated, authFetch]);

  return null;
}
