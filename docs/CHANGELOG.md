# Changelog

> Newest first. Each entry: date, commit (or "uncommitted"), what changed, and for fixes the root cause.
> This is the history; STATUS.md holds only the current truth.

## 2026-10-28 · feat/autoindex

- PR #2 merged and deployed: `/health` on the Space reports postgres, reachable (the existing `DATABASE_URL` works).
- The Space indexes the lessons itself on startup (D-025): `app/rag/autoindex.py`, lessons shipped as `course_docs`
  by `deploy-hf.yml` (which now also runs on lesson changes); `/api/documents/health` shows `index` state and says
  `empty` instead of `unhealthy` before the first index. 83 backend tests.

## 2026-10-28 · feat/kinemata-mobile

- Renamed Proprio → Kinemata everywhere (site, lessons and their package names, Rivet's prompt, backend settings,
  collection `kinemata_course`); new "K" robot-arm logo, favicons, app icons and social card (D-024).
- Phone header: Docusaurus pinned the search box over the theme switch (`position: absolute` on the search container);
  now search is a round button next to the switch, and tapping it opens the field across the whole navbar pill with
  full-width results (the autocomplete's inline `position: relative` had to be overridden). GitHub moved out of the
  phone header into the menu as a labelled row (it was a lone icon there).
- Phones play the live 3D hero with the build-up intro, starting when it scrolls into view (D-023).
- Backend also reads the Space's existing secret names: `DATABASE_URL` (was ignored, so the live API ran on a
  throwaway SQLite file) and `JWT_SECRET_KEY` (was ignored, so tokens used a random key per restart). `/health` now
  reports the database kind and whether it answers (never the URL). Space `QDRANT_URL` set to the new cluster.
- `design/mobile_chrome.py`, `design/mobile_search.py` check the phone header, menu, hero and search.

## 2026-10-28 · main (deployed)

- PR #1 merged (62bbbf1): Vercel, GitHub Pages and the HF Space deployed. `design/live_smoke.py` checks a deployed
  site without signing in.

## 2026-10-28 · feat/proprio-pro-course (free Gemini, sign-in wall)

- Merged the UI pass (`claude/recursing-agnesi-7f51f8`, fast-forward).
- Rivet moved to Google Gemini's free tier (D-020): `GEMINI_API_KEY`, chat `gemini-3.8-flash` with a flash-lite
  fallback on 429, embeddings `gemini-embedding-2` (3072-d, task prefixes), 429 backoff for indexing, refuses a
  collection of the wrong vector size. Translation uses the same chat model. Re-index needed.
- Rivet needs an account (D-021): chat, stream and search return 401 without a Bearer token; limits per account
  (20/min, 150/day) replace the X-Forwarded-For IP limiter. The chat window shows a sign-in wall and sends the
  waiting question after sign-in; an expired token brings the wall back; another user's sign-in clears the chat.
- Theme switch redesigned (D-022): two-state, sun → moon morph, circular reveal from the button; replaces the
  three-step cycle with the unclear "system" icon. `design/theme_toggle.py` captures it.
- Security: an unset `SECRET_KEY` fell back to a fixed string from this public repo, so anyone could sign a token for
  any account; it is now a random key per process (with a warning). Sign-ins now last 7 days by default
  (`ACCESS_TOKEN_EXPIRE_MINUTES`, was 30 min), since Rivet needs one.
- `backend/*.db` ignored; `design/rivet_wall.py` checks the wall end to end. 77 backend tests.

## 2026-10-28 · claude/recursing-agnesi-7f51f8 (UI pass over every component)

- Rivet chat v3: Rivet mark (eyes scan while answering), calmer tinted user bubbles, labelled answers, GFM Markdown
  (tables now render; before they showed as raw `|` pipes because react-markdown had no GFM plugin), code blocks with a
  language bar and copy, citation chips that light up their source row, source rows with title + section, Copy and
  Regenerate, "Searching the lessons…" shimmer, caret inline at the end of the text, error card with Retry, "Latest"
  pill, composer with JS auto-grow fallback, length counter, key hints while typing, Stop with a turning ring,
  Ctrl/⌘ + / to open and close, phone sheet follows the on-screen keyboard (visualViewport), launcher hides under the
  mobile menu. Fixed: focus never returned to the launcher after Esc (it was still `visibility: hidden`).
- Navbar: scroll-lifted pill, styled search box and results (the active result was solid lime), Tools menu redesigned
  (native language names, arrow keys, "Sign in to personalize" instead of a dead disabled button, partial-translation
  note, Retry on errors), sign-in/create-account dialog (logo, segmented tabs, eye toggle, Caps Lock hint, live
  password rule, email check on blur, shake on error), account menu (who, Continue, Profile, Sign out; arrow keys),
  mobile menu sheet. Dialogs: focus trap + bottom sheet on phones (`useDialog`).
- Docs: sidebar active bar, module progress lines, ringed completion ticks, TOC rail with "On this page",
  breadcrumbs, prev/next cards, tinted admonitions with sentence-case titles, framed tables with row hover, code title
  bar and copy buttons, segmented Tabs, Mermaid on a dot grid, back-to-top moved above Rivet's launcher.
- Lesson components: numbered objectives, ticked takeaways, glow Key idea, edge-aware Term tooltip with arrow, Steps
  rail, Solution disclosure (Show/Hide + chevron), quiz options with letter keys and a score bar; lesson-complete card
  with drawn check, course %, and "Next lesson".
- 404 with a radar scope and module links; footer link underline and SVG external icon; profile uses the shared
  inputs and an inline confirm for Reset progress; landing example chat matches Rivet v3.
- Fixed: 22 px horizontal scroll at 320 px (hidden Term tooltip and a non-shrinking Takeaways item) and long inline
  formulas widening two kinematics/dynamics lessons; the landing closing band sat flush on the footer (bottom padding
  lost when its wordmark moved to the footer) and its title was near-invisible in light theme (`--ifm-heading-color`
  resolved at :root).
- Perf: Markdown renderers lazy-loaded (D-019): main.js 285 → 173 KB gz; site CSS 28.5 → 36.8 KB gz.
- `design/shoot.py` rewritten: every surface and state at 375/768/1440 + light + 320/1920, backend mocked in the page;
  `design/compare.py` pairs every shot; `design/sheet.py` contact sheets; `design/ui-inventory.md` checklist.
- Added `remark-gfm` as a direct dependency (was already installed through Docusaurus).

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
