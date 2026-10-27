# Changelog

> Newest first. Each entry: date, commit (or "uncommitted"), what changed, and for fixes the root cause.
> This is the history; STATUS.md holds only the current truth.

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
