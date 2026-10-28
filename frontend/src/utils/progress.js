// Learning progress: completed lessons, best quiz scores and the last lesson read.
// Kept in this browser (localStorage) and, when signed in, synced to the account by <ProgressSync />.
// A tiny external store, so the lesson footer, sidebar ticks, quizzes and profile stay in sync.
import {useSyncExternalStore} from 'react';

const KEY = 'proprio:progress';
const LEGACY_COMPLETED = 'proprio:completed';
const listeners = new Set();
const EMPTY = {completed: new Set(), quizzes: {}, last: null};
let cache = null;

function read() {
  if (cache) return cache;
  try {
    const raw = JSON.parse(localStorage.getItem(KEY) || 'null');
    const legacy = JSON.parse(localStorage.getItem(LEGACY_COMPLETED) || '[]');
    cache = {
      completed: new Set([...(raw?.completed || []), ...legacy]),
      quizzes: raw?.quizzes || {},
      last: raw?.last || null,
    };
  } catch {
    cache = {...EMPTY, completed: new Set()};
  }
  return cache;
}

function write(next) {
  cache = next;
  try {
    localStorage.setItem(KEY, JSON.stringify({completed: [...next.completed], quizzes: next.quizzes, last: next.last}));
    localStorage.removeItem(LEGACY_COMPLETED);
  } catch {
    // storage blocked: progress lasts for this visit only
  }
  listeners.forEach((l) => l());
}

export const UNCOMPLETE_EVENT = 'pa:lesson-uncompleted';

export function setCompleted(docId, done) {
  const s = read();
  const completed = new Set(s.completed);
  if (done) completed.add(docId);
  else {
    completed.delete(docId);
    window.dispatchEvent(new CustomEvent(UNCOMPLETE_EVENT, {detail: {docId}}));
  }
  write({...s, completed});
}

export function setQuizScore(path, score, total) {
  const s = read();
  const prev = s.quizzes[path];
  if (prev && prev.best >= score) return;
  write({...s, quizzes: {...s.quizzes, [path]: {best: score, total}}});
}

export function setLastLesson(path, title) {
  const s = read();
  if (s.last?.path === path && s.last?.title === title) return;
  write({...s, last: {path, title, at: Date.now()}});
}

export function resetProgress() {
  write({completed: new Set(), quizzes: {}, last: read().last});
}

// Merge account progress into this browser: union of lessons, best of each quiz.
export function mergeRemote(remote) {
  const s = read();
  const completed = new Set([...s.completed, ...(remote?.completed || [])]);
  const quizzes = {...s.quizzes};
  for (const [path, q] of Object.entries(remote?.quizzes || {})) {
    if (!quizzes[path] || quizzes[path].best < q.best) quizzes[path] = {best: q.best, total: q.total};
  }
  write({...s, completed, quizzes});
  return {completed: [...completed], quizzes};
}

export function snapshot() {
  const s = read();
  return {completed: [...s.completed], quizzes: s.quizzes};
}

export function subscribe(l) {
  listeners.add(l);
  const onStorage = (e) => {
    if (e.key === KEY) {
      cache = null;
      l();
    }
  };
  window.addEventListener('storage', onStorage);
  return () => {
    listeners.delete(l);
    window.removeEventListener('storage', onStorage);
  };
}

export function useProgress() {
  return useSyncExternalStore(subscribe, read, () => EMPTY);
}

export function useCompleted() {
  return useProgress().completed;
}
