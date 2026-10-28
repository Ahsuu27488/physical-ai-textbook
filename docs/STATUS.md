# Project Status

> Current truth only: what works now, what doesn't, what's next. History goes in CHANGELOG.md, reasons in
> DECISIONS.md. Updated in the same commit as every code change (the commit-msg hook checks). Keep it short:
> it is loaded into every agent session.

**Last updated:** 2026-10-28 · **Code at:** feat/coldstart-canonical-cleanup
**In one line:** Kinemata live; Rivet indexed (611 chunks) on free Gemini, sign-in only; Urdu translation job started.

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

- feat/coldstart-canonical-cleanup: Rivet "Waking up…" state + keep-alive ping, Pages → Vercel redirects (D-028), cleanup.
- Urdu lessons: `backend/scripts/translate_course.py --locale ur` running (streams, 3 at a time; owner's
  GEMINI_API_KEY is set in the system env for one-off jobs); output in `frontend/i18n/ur`, to commit when done.

## Live state

- Merged to main 2026-10-28 (PR #1 62bbbf1, PR #2 e5337ed: Kinemata); `design/live_smoke.py` passed on the Vercel site (landing, lesson,
  Rivet wall, theme switch, desktop + phone, no console errors or failed requests).
- Frontend: https://physical-ai-textbook-puce-two.vercel.app (Vercel) and https://ahsuu27488.github.io/physical-ai-textbook/
- Backend: HF Space `Ahsuu27488/physical-ai-textbook-backend`: `/health` → postgres reachable (2026-10-28). Secrets:
  ADDITIONAL_CORS_ORIGIN, DATABASE_URL, FRONTEND_URL, JWT_SECRET_KEY, GEMINI_API_KEY, QDRANT_API_KEY, QDRANT_URL
  (owner set the Gemini key and removed OPENAI_API_KEY; QDRANT_URL points at the new cluster).
- Qdrant: free cluster `proprio-course` (AWS us-east-1), empty until `run_etl.py` runs.

## Known gaps

1. Collection `kinemata_course` not indexed until the autoindex PR deploys;
   Rivet has never been run against the real Gemini API (model ids from the pricing page, 2026-10-28).
2. `npm audit`: advisories in build/dev tooling under Docusaurus; nothing ships to the browser.
3. ROS 2 / Gazebo / Isaac / OpenCV / Open3D lesson code is unexecuted; needs a run on Ubuntu 24.04 + Jazzy.
4. Rivet's limits are per account but in memory, per process (reset on restart); chat history is not stored
   server-side. Without `SECRET_KEY` the API uses a random key per start (sign-ins end on restart): set it on the Space.
5. No password reset by email (no mail provider); no account email verification.
6. Mobile LCP above 2.5 s on Lighthouse slow-4G (Docusaurus main bundle).
7. Redesign not checked on a real phone or Safari/Firefox (the UI pass was checked in Chromium only; Firefox uses
   the composer auto-grow fallback and has no CSS scroll timelines, so the navbar lift and reading bar don't animate).

## Next (owner's to-do list, 2026-10-28; item "run ROS 2 code on Ubuntu" deferred)

1. Owner: test Rivet live with a real account (answers, citations, a follow-up).
2. Finish and commit the Urdu edition; translate the Urdu UI strings (script without `--skip-json`).
5. Password reset by email (needs a mail provider, e.g. Resend free tier).
6. Translate / Personalize in place on the page with an original ↔ translated toggle, instead of a dialog.
7. Chat history saved to the account (continue on another device).
8. Thumbs up/down on Rivet answers.
9. Re-run Lighthouse on phones now that phones load the 3D hero; check battery/heat on a mid-range phone.
10. Check Safari and Firefox.
12. Rate limits are in memory (reset on restart): move to the database if traffic grows.

## How to run and check

- Run: `cd frontend && npm start` (site :3000), `cd backend && uvicorn app.main:app --reload` (API :8000)
- Check: `(cd frontend && npm run build) && (cd backend && python -m pytest -q)` (both pass 2026-10-28)
