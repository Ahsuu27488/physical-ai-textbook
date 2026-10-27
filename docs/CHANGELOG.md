# Changelog

> Newest first. Each entry: date, commit (or "uncommitted"), what changed, and for fixes the root cause.
> This is the history; STATUS.md holds only the current truth.

## 2026-10-28 · feat/proprio-pro-course

- All 52 lessons written (modules 00–10) and validated; course home sidebar label "Course overview".
- Backend RAG v2 (D-016): `app/config.py`, `app/rag/` (chunking, embeddings, store, ingest, prompts, service),
  `/api/chat/stream` (SSE), page-aware retrieval, rate limiting, Rivet system prompt; removed `app/etl/` and
  `app/vector_db.py`; dropped markdown/bs4/tiktoken. 70 tests.
- Accounts and progress (D-017): `/api/v1/progress`, change password, delete account, richer `/me` + profile;
  frontend profile page (`/profile`), progress sync, "Continue" on the landing hero, profile link in the account
  menu, `openSignIn()`. Fixed: any network error used to delete the saved login token.
- Local site search (D-018). `backend/.env.example` (names only).
- Qdrant: free cluster `proprio-course` created (AWS us-east-1); not indexed yet.

## 2026-10-28 · feat/proprio-pro-course (WIP)

- Rebrand to **Proprio**; course assistant named **Rivet**. New SVG logo, favicons, app icons, social card,
  manifest, metadata (favicon/manifest links moved from `metadata` to `headTags`, where they work).
- Old 13-page textbook removed; new 11-module, 52-lesson course (written by Sonnet agents from the plan) with
  lesson components, KaTeX math, Mermaid diagrams, quizzes, exercises and per-lesson progress.
- Navbar is a floating glass pill; footer swizzled into a brand footer with a giant wordmark.
- Rivet chat v2: streaming answers (with fallback to `/api/chat`), conversation kept per tab, page-aware
  suggestions, select text in a lesson to ask, citation chips, copy buttons, larger mode, themed scrollbar.
  Fixed: the message list never scrolled (flex child without `min-height: 0`; Lenis also ate wheel events).
- Hero v2: 46k lit points (normals, key + rim light), skeleton lines, scan ring, floor radar pulse, cursor
  ripple, and a bottom-up build intro led by the scan line.

## 2026-10-27 · feat/immersive-3d-redesign

- New visual system (D-011): OKLCH tokens, one accent, Geist / Geist Mono / Instrument Serif (self-hosted, D-013),
  glass navbar, restyled docs (sidebar, TOC, code, pagination, reading progress bar), real footer.
- Landing rebuilt: hero with a live point-cloud humanoid and scan beam (three.js, lazy; poster fallback, D-012),
  module bento with line art and real lesson links, 13-week timeline, tools section, closing band. Fixes the old
  hero (title nearly invisible in dark mode) and the 375 px horizontal overflow.
- Chat widget rewritten (D-014): grows from the launcher, full-screen sheet on phones, clickable suggestions,
  Markdown answers, source chips linking to lessons, inline errors with Retry, Stop, Esc + focus return.
- Sign-in dialog and Tools menu rewritten: labelled fields with autocomplete, show/hide password, loading states,
  inline errors (no `alert()`), Esc to close; dialogs portal to `<body>` because the glass navbar's backdrop filter
  trapped `position: fixed` children (sign-in dialog was clipped, mobile menu was see-through).
- 404 page; Prism comment colours raised to 4.5:1; FastAPI 422 details shown as text, not `[object Object]`.
- Removed dead code: `TranslateButton`, `PersonalizeButton`, old `ChatWidget`, `HomepageFeatures`, Layout swizzle,
  sample SVGs. Navbar logo uses a 96 px copy (the 500 px `logo.png` was 276 KB).
- `design/` scripts for screenshots, poster rendering and before/after comparisons.

## 2026-10-27 · fix/broken-config-and-deps

- `BACKEND_URL` now works. Root cause: `client-modules/AppProvider.js` was a client module whose default export
  never renders, so `window.__BACKEND_URL__` was never set, and `utils/api.js` froze its URL at import time.
  Replaced five copies of `getBackendUrl` with `src/utils/backendUrl.js` reading `@generated/docusaurus.config`.
- API starts without `OPENAI_API_KEY` (D-009). Root cause: `OpenAI()` built at import in `api/chat.py` and
  `services/translation.py`.
- Tables are created on startup. Root cause: `database.py` declared a second, empty `Base`; nothing called
  `create_all` with the real one, so registration failed on a fresh database.
- `translate-to-urdu` reads `content` as a form field (the UI sends FormData; it was a 422).
- `translate-chapter` accepts anonymous callers (`OAuth2PasswordBearer(auto_error=False)`; it was a 401).
- Translation errors return 500 instead of crashing on `status.HTTP_500_INTERNAL_ERROR` (no such constant).
- `requirements.txt` trimmed to what is imported; `pip install` works again (D-008).
- Removed unused `better-auth` (critical advisory); Docusaurus 3.9.2 → 3.10.2 + `@docusaurus/faster` (D-007);
  `npm audit` 66 → 22, all build tooling.
- GitHub Pages build sets `BASE_URL=/physical-ai-textbook/` (assets 404'd under `/`); `npm ci` instead of `npm install`.
- Footer rewritten (the `/docs/intro` link broke every page); sample blog, markdown-page and duplicate
  `HomepageFeatures.jsx` removed (D-010); `backend/test.db` untracked and ignored.
- Added `backend/tests/` (5 API tests) and a `backend` job in `ci.yml`.

## 2026-10-27 · chore/adopt-shipshape

- Project memory and workflow set up with shipshape (D-001): docs/, AGENTS.md, GEMINI.md, CLAUDE.md imports,
  .shipshape/, .githooks/, CI (`ci.yml`), PR template. No runtime change.
- Decisions D-002..D-006 recovered from the git history.

## 2025-12-06 → 2025-12-? · earlier history (summary)

- Spec-Kit template, textbook content and both apps added (b647b34); GitHub Pages workflow fixed in several
  steps; backend moved Railway → Hugging Face Space with auto-deploy; frontend moved to Vercel;
  `BACKEND_URL` build setting added (9fda10f).
