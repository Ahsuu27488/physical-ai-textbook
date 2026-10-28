# Project Status

> Current truth only: what works now, what doesn't, what's next. History goes in CHANGELOG.md, reasons in
> DECISIONS.md. Updated in the same commit as every code change (the commit-msg hook checks). Keep it short:
> it is loaded into every agent session.

**Last updated:** 2026-10-28 · **Code at:** feat/kinemata-mobile (on main)
**In one line:** Live as Proprio; rename to Kinemata + phone header/search/hero fixes on a branch (PR); Rivet waits for keys.

## What works (verified)

- Frontend: `npm run build` passes with the full course, no broken links (2026-10-28); `check-mdx.mjs docs` → 64/64 ok (both re-run after the UI pass).
- Course: 11 modules, 52 lessons + overviews; Python examples in modules 00–09 were run by the writers (see
  CHANGELOG); ROS 2 / Gazebo / Isaac snippets were not run (no ROS install).
- Backend: `python -m pytest -q` → 78 passed (2026-10-28, no network/secrets); ingest dry-run → 64 files, 611 chunks.
- Rivet sign-in wall (2026-10-28, dev site + local backend, Chromium): signed out shows the wall, no composer; sign up →
  sign in → the waiting question is sent with the token and reaches the API (503 there only because no key locally).
- Theme switch (2026-10-28, Chromium): flips light ↔ dark with the circular reveal, choice kept after reload.
- Seen in the dev site (2026-10-28): lesson components, Rivet reading along on a lesson, profile page, search box.
- UI pass over every component (2026-10-28): before/after shots of 140+ states at 320–1920 px, dark + light, from a
  production build with the backend mocked; no horizontal scroll at 320/375/768 on any of the 64 docs pages; keyboard
  pass (Rivet, dialogs, Tools, account menu) and reduced motion checked in Chromium; console clean.

## In progress

- feat/kinemata-mobile: rename to Kinemata (D-024), phone header and search, live hero on phones (D-023). PR open,
  not merged; checked in Chromium at 393 px (design/mobile_chrome.py, mobile_search.py), build + 79 tests pass.
- Waiting on the owner for the Space secrets (see Next 1).

## Live state

- Merged to main 2026-10-28 (PR #1, 62bbbf1); `design/live_smoke.py` passed on the Vercel site (landing, lesson,
  Rivet wall, theme switch, desktop + phone, no console errors or failed requests).
- Frontend: https://physical-ai-textbook-puce-two.vercel.app (Vercel) and https://ahsuu27488.github.io/physical-ai-textbook/
- Backend: HF Space `Ahsuu27488/physical-ai-textbook-backend`. Its secrets (names, 2026-10-28): ADDITIONAL_CORS_ORIGIN,
  DATABASE_URL, FRONTEND_URL, JWT_SECRET_KEY, OPENAI_API_KEY (unused now), QDRANT_API_KEY (old cluster's),
  QDRANT_URL (set to the new cluster 2026-10-28). Missing: GEMINI_API_KEY. Until PR #2 deploys, the API ignores
  DATABASE_URL/JWT_SECRET_KEY (SQLite, sign-ins reset on restart). Neon not verified (no Neon login here).
- Qdrant: free cluster `proprio-course` (AWS us-east-1), empty until `run_etl.py` runs.

## Known gaps

1. Qdrant collection `proprio_course` not indexed yet (needs `QDRANT_URL`, `QDRANT_API_KEY`, `GEMINI_API_KEY`);
   Rivet has never been run against the real Gemini API (model ids from the pricing page, 2026-10-28).
2. `npm audit`: advisories in build/dev tooling under Docusaurus; nothing ships to the browser.
3. ROS 2 / Gazebo / Isaac / OpenCV / Open3D lesson code is unexecuted; needs a run on Ubuntu 24.04 + Jazzy.
4. Rivet's limits are per account but in memory, per process (reset on restart); chat history is not stored
   server-side. Without `SECRET_KEY` the API uses a random key per start (sign-ins end on restart): set it on the Space.
5. No password reset by email (no mail provider); no account email verification.
6. Mobile LCP above 2.5 s on Lighthouse slow-4G (Docusaurus main bundle).
7. Redesign not checked on a real phone or Safari/Firefox (the UI pass was checked in Chromium only; Firefox uses
   the composer auto-grow fallback and has no CSS scroll timelines, so the navbar lift and reading bar don't animate).
8. `railway.json`, `Procfile`, README still describe the old project and Railway.

## Next

1. Owner: add Space secret `GEMINI_API_KEY`, replace `QDRANT_API_KEY` with a new key for `proprio-course`, delete
   `OPENAI_API_KEY`; same names in `backend/.env`. Then `run_etl.py`, and check `/health` shows postgres reachable.
2. After the keys: index, then ask Rivet a few questions on the live site with a real account.
3. Merge the Kinemata PR after a look at its Vercel preview on a phone; check Safari and Firefox.
4. Rewrite README for Proprio; run the ROS 2 lesson code on a real Jazzy install.

## How to run and check

- Run: `cd frontend && npm start` (site :3000), `cd backend && uvicorn app.main:app --reload` (API :8000)
- Check: `(cd frontend && npm run build) && (cd backend && python -m pytest -q)` (both pass 2026-10-28)
