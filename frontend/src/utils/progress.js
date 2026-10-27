// Lesson progress, kept in this browser (localStorage). A tiny store so the lesson footer and the sidebar
// ticks stay in sync without a context provider.
import {useSyncExternalStore} from 'react';

const KEY = 'proprio:completed';
const listeners = new Set();
let cache = null;

function read() {
  if (cache) return cache;
  try {
    cache = new Set(JSON.parse(localStorage.getItem(KEY) || '[]'));
  } catch {
    cache = new Set();
  }
  return cache;
}

function write(next) {
  cache = next;
  try {
    localStorage.setItem(KEY, JSON.stringify([...next]));
  } catch {
    // storage blocked: progress lasts for this visit only
  }
  listeners.forEach((l) => l());
}

export function setCompleted(docId, done) {
  const next = new Set(read());
  if (done) next.add(docId);
  else next.delete(docId);
  write(next);
}

const subscribe = (l) => {
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
};
const EMPTY = new Set();

export function useCompleted() {
  return useSyncExternalStore(subscribe, read, () => EMPTY);
}
