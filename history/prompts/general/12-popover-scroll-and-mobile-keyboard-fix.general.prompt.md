---
id: 12
title: Contain popover scroll leaking and fix mobile keyboard viewport gap
stage: general
date: 2026-10-30
surface: agent
model: claude-opus-5-5
feature: none
branch: fix/popover-scroll-mobile-keyboard
user: Muhammad Ahsan
command: (chat)
labels: ["frontend", "ux", "lenis", "popover", "mobile-keyboard", "scroll-containment", "chat-widget"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - frontend/src/components/ChatWidget/{index.js,styles.module.css}
  - frontend/src/theme/NavbarItem/{AuthButtons.js,PageTools.js}
  - frontend/src/css/custom.css
  - docs/{STATUS.md,CHANGELOG.md}
tests:
  - Frontend npm run build (en + ur static files generated)
  - check-mdx.mjs docs (64/64 clean)
  - Chromium DevTools MCP live verification of PageTools scroll containment and Lenis delta (0)
  - Mobile viewport emulation (390x844) verifying backdrop overlay and body scroll lock
---

## Prompt

1. Fix popover/dropdown scroll leaking on desktop landing page: when scrolling inside the Tools menu "Translate this page" (`.langs`) or account dropdown, the background landing page scrolled via Lenis smooth-scroll.
2. Fix mobile phone Rivet chat virtual keyboard viewport gap: when input is focused and the virtual keyboard opens, the bottom disclaimer detached from the keyboard, revealing the background landing page ("Start reading" / "Ask Rivet" hero buttons) through the gap.

## Response Summary

1. Isolated popover scroll events:
   - Added `data-lenis-prevent` and `wheel` stopPropagation to `#pa-tools-menu`, `.panel`, and `.langs` in `frontend/src/theme/NavbarItem/PageTools.js`.
   - Added `data-lenis-prevent` and `wheel` stopPropagation to `AccountMenu` in `frontend/src/theme/NavbarItem/AuthButtons.js`.
   - Added `overscroll-behavior: contain` to `.pa-popover`, `[class*='searchBar_'] [class*='dropdownMenu_']`, `.navbar-sidebar`, and `.dropdown__menu` in `frontend/src/css/custom.css`.
2. Fixed mobile keyboard viewport gap:
   - Added a fixed full-viewport backdrop overlay (`<div className={styles.backdrop} data-lenis-prevent aria-hidden="true" />`) behind the chat panel on mobile (`@media (max-width: 639px)`), styled with `background: var(--pa-bg)` and `z-index: 999`.
   - Anchored `.panel` with `bottom: auto; z-index: 1000;`.
   - Locked `document.body.style.overflow = 'hidden'` alongside `documentElement.style.overflow` and called `window.__lenis?.stop()` / `window.__lenis?.start()` during mobile chat open/close.
   - Added active input `field.scrollIntoView({block: 'nearest'})` inside visual viewport `fit()`.
