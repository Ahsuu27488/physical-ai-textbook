# Project Status

> Current truth only: what works now, what doesn't, what's next. History goes in CHANGELOG.md, reasons in
> DECISIONS.md. Updated in the same commit as every code change (the commit-msg hook checks). Keep it short:
> it is loaded into every agent session.

**Last updated:** 2026-10-28 · **Code at:** claude/recursing-agnesi-7f51f8 (UI pass, on top of feat/proprio-pro-course)
**In one line:** Proprio course complete (52 lessons), RAG v2 + accounts tested; Qdrant cluster created, not indexed yet.

## What works (verified)

- Frontend: `npm run build` passes with the full course, no broken links (2026-10-28); `check-mdx.mjs docs` → 64/64 ok (both re-run after the UI pass).
- Course: 11 modules, 52 lessons + overviews; Python examples in modules 00–09 were run by the writers (see
  CHANGELOG); ROS 2 / Gazebo / Isaac snippets were not run (no ROS install).
- Backend: `python -m pytest -q` → 70 passed (2026-10-28, no network/secrets); ingest dry-run → 63 files, 595 chunks.
- Seen in the dev site (2026-10-28): lesson components, Rivet reading along on a lesson, profile page, search box.
- UI pass over every component (2026-10-28): before/after shots of 140+ states at 320–1920 px, dark + light, from a
  production build with the backend mocked; no horizontal scroll at 320/375/768 on any of the 64 docs pages; keyboard
  pass (Rivet, dialogs, Tools, account menu) and reduced motion checked in Chromium; console clean.

## In progress

- feat/proprio-pro-course (this branch, stacked on feat/immersive-3d-redesign) - not pushed.
- Full UI/UX pass over every component: done on branch `claude/recursing-agnesi-7f51f8` (not pushed); to merge into
  feat/proprio-pro-course. Inventory and checklist: `design/ui-inventory.md`.

## Live state

- Frontend: Vercel (production `url`) and GitHub Pages (`deploy.yml` on main, now built with `BASE_URL`).
- Backend: HF Space `Ahsuu27488/physical-ai-textbook-backend`, auto-deployed by `deploy-hf.yml`.
- Old Qdrant cluster no longer exists. New free cluster `proprio-course` (AWS us-east-1) created 2026-10-28,
  empty until `run_etl.py` runs with real keys. The live Space still runs the old backend until main is merged.

## Known gaps

1. Qdrant collection `proprio_course` not indexed yet (needs `QDRANT_URL`, `QDRANT_API_KEY`, `OPENAI_API_KEY`).
2. `npm audit`: advisories in build/dev tooling under Docusaurus; nothing ships to the browser.
3. ROS 2 / Gazebo / Isaac / OpenCV / Open3D lesson code is unexecuted; needs a run on Ubuntu 24.04 + Jazzy.
4. Rate limiter is per process and keys on X-Forwarded-For; chat history is not stored server-side.
5. No password reset by email (no mail provider); no account email verification.
6. Mobile LCP above 2.5 s on Lighthouse slow-4G (Docusaurus main bundle).
7. Redesign not checked on a real phone or Safari/Firefox (the UI pass was checked in Chromium only; Firefox uses
   the composer auto-grow fallback and has no CSS scroll timelines, so the navbar lift and reading bar don't animate).
8. `railway.json`, `Procfile`, README still describe the old project and Railway.

## Next

1. Owner: put the Qdrant + OpenAI keys in `backend/.env` and the HF Space secrets; then run `run_etl.py`.
2. Merge `claude/recursing-agnesi-7f51f8` (UI pass) into feat/proprio-pro-course; check it on a real phone.
3. Owner: review, push and merge the stacked branches (adopt → fixes → redesign → proprio).
4. Rewrite README for Proprio; run the ROS 2 lesson code on a real Jazzy install.

## How to run and check

- Run: `cd frontend && npm start` (site :3000), `cd backend && uvicorn app.main:app --reload` (API :8000)
- Check: `(cd frontend && npm run build) && (cd backend && python -m pytest -q)` (both pass 2026-10-28)
