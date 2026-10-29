---
id: 10
title: Modal scroll lock and chat widget UX improvements
stage: general
date: 2026-10-30
surface: agent
model: claude-opus-5-5
feature: none
branch: fix/window-scroll-chat-ux
user: Muhammad Ahsan
command: (chat)
labels: ["frontend", "ux", "lenis", "modal", "chat-widget", "scroll-containment"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - frontend/src/components/ChatWidget/{index.js,styles.module.css}
 - frontend/src/hooks/useDialog.js
 - frontend/src/pages/index.js
 - frontend/src/theme/NavbarItem/{AuthButtons.js,PageTools.js}
 - frontend/src/css/custom.css
 - docs/{STATUS.md,CHANGELOG.md}
tests:
 - Frontend npm run build (en + ur passed)
 - Chromium DevTools MCP live verification on localhost:3123
---

## Prompt

Fix 4 UX bugs on physical-ai-textbook:
1. Dialog & chat scroll leaking to background landing page.
2. Slow chat latency.
3. Chat stream aborting/cancelling on window close or outside navigation.
4. Outside click not minimizing chat window, and close button killing stream.

## Response Summary

1. Locked documentElement and body overflow and halted Lenis smooth scroll while modals/dialogs are mounted.
2. Added data-lenis-prevent and overscroll-behavior: contain to modals and chat panel.
3. Added wheel stopPropagation on chat panel to prevent scroll event leaking.
4. Removed abortRef.abort() from chat widget close callback, allowing ongoing streams to complete in background.
5. Added outside-click pointerdown listener on chat panel to dismiss without aborting stream.
