# Project Status

> Current truth only: what works now, what doesn't, what's next. History goes in CHANGELOG.md, reasons in
> DECISIONS.md. Updated in the same commit as every code change (the commit-msg hook checks). Keep it short:
> it is loaded into every agent session.

**Last updated:** 2026-10-28 · **Code at:** feat/proprio-pro-course (UI pass merged)
**In one line:** Course, UI pass and Rivet on free Gemini (sign-in only) built and tested; not live yet, not indexed yet.

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

- feat/proprio-pro-course: course + UI pass (merged from `claude/recursing-agnesi-7f51f8`) + Gemini + sign-in wall.
  Not pushed. Gemini calls not yet tried with a real key.

## Live state

- Frontend: Vercel (production `url`) and GitHub Pages (`deploy.yml` on main, now built with `BASE_URL`).
- Backend: HF Space `Ahsuu27488/physical-ai-textbook-backend`, auto-deployed by `deploy-hf.yml`.
- Old Qdrant cluster no longer exists. New free cluster `proprio-course` (AWS us-east-1) created 2026-10-28,
  empty until `run_etl.py` runs with real keys. The live Space still runs the old backend until main is merged.

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

1. Owner: put `GEMINI_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY` in `backend/.env` and the HF Space secrets; then run
   `run_etl.py` and ask Rivet a few questions.
2. Deploy: push, PR to main, merge (Vercel + Pages + HF Space); check the live site.
3. Decide on the name (Proprio clashes with a surgical-navigation company); check on a real phone, Safari, Firefox.
4. Rewrite README for Proprio; run the ROS 2 lesson code on a real Jazzy install.

## How to run and check

- Run: `cd frontend && npm start` (site :3000), `cd backend && uvicorn app.main:app --reload` (API :8000)
- Check: `(cd frontend && npm run build) && (cd backend && python -m pytest -q)` (both pass 2026-10-28)
