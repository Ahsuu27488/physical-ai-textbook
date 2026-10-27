# Project Status

> Current truth only: what works now, what doesn't, what's next. History goes in CHANGELOG.md, reasons in
> DECISIONS.md. Updated in the same commit as every code change (the commit-msg hook checks). Keep it short:
> it is loaded into every agent session.

**Last updated:** 2026-10-28 · **Code at:** feat/proprio-pro-course (WIP, on feat/immersive-3d-redesign)
**In one line:** Rebranding to Proprio with a new 11-module course; content, backend RAG v2 and a full UI pass in progress.

## What works (verified)

- Frontend build: `cd frontend && npm ci && npm run build` succeeds, no broken links (2026-10-27, Node 24).
- `BACKEND_URL` reaches the bundle; `BASE_URL=/physical-ai-textbook/` produces correct Pages asset paths
  (checked in the built `index.html`, 2026-10-27).
- Backend: `cd backend && python -m pytest -q` → 5 passed (2026-10-27, Python 3.12): starts with no secrets,
  creates tables, register → token → me, AI endpoints 503 without a key, form-data Urdu translation, anonymous
  translate-chapter.
- Redesign (2026-10-27, `design/*.py` against the dev site): no horizontal scroll at 375/768/1440; light and dark;
  mobile menu; reduced motion shows the poster and end-state reveals; keyboard: chat opens into its input, Esc closes
  and returns focus to the launcher.
- Lighthouse mobile, gzip-served build, median of 3 (2026-10-27): landing perf 83–87, a11y 100, LCP ~3.6 s, CLS 0;
  lesson perf 89–90, LCP ~3.25 s, CLS 0.014 (before the redesign: landing 87 / 3.46 s, lesson 85 / 3.74 s).
- Live (before these branches): site 200 on Vercel; HF Space `/health` healthy.

## In progress

- chore/adopt-shipshape - shipshape setup (docs, hooks, CI). Not pushed.
- fix/broken-config-and-deps - the fixes listed in CHANGELOG. Not pushed.
- feat/immersive-3d-redesign - new landing, 3D hero, docs theme, chat, sign-in, Tools menu, 404. Not pushed.
- feat/proprio-pro-course - plan: `docs/plans/2026-10-27-proprio-course.md`. Done: brand (name, logo, icons,
  social card, metadata), floating pill navbar, new footer, lesson components (`src/components/Lesson`, KaTeX,
  Mermaid), lesson progress ticks, Rivet chat v2 UI (streaming client, page-aware, select-to-ask), hero v2.
  In progress: lessons for modules 04–09 (writer agents), backend RAG v2 (uncommitted), Qdrant cluster.
- A separate session does the full UI/UX pass over every component (its own worktree and branch).

## Live state

- Frontend: Vercel (production `url`) and GitHub Pages (`deploy.yml` on main, now built with `BASE_URL`).
- Backend: HF Space `Ahsuu27488/physical-ai-textbook-backend`, auto-deployed by `deploy-hf.yml`.
- `GET /api/documents/health` → `unhealthy: Connection reset by peer`: the Qdrant cluster is gone or asleep,
  so `/api/chat` answers without textbook context.

## Known gaps

1. Qdrant unreachable from the live backend; needs a new cluster + `run_etl.py` (owner: account + secrets).
2. `npm audit`: 22 advisories left, all in build/dev-server tooling under Docusaurus (serialize-javascript,
   tinypool, uuid via sockjs); fixes need breaking upstream releases. Nothing ships to the browser.
3. `backend/test_auth.py` is an old print-script (not collected; `pytest.ini` points at `tests/`).
4. `QdrantManager._generate_embedding` returns a zero vector on OpenAI errors, so search silently returns junk.
5. `/api/chat` keeps no history: `conversation_id` is echoed, nothing is stored.
6. `railway.json`, `Procfile` and README badges still point at the old Railway deployment.
7. Mobile LCP is above the 2.5 s design budget on Lighthouse's slow-4G profile (~3.3–3.6 s); `main.js` is 173 KB gz
   (Docusaurus + React 19), unchanged by the redesign.
8. Chat answers are not streamed (the API returns whole answers); the widget shows a typing indicator and Stop.
9. Redesign not checked on a real phone or Safari/Firefox (Chromium only).

## Next

1. Owner: push the three branches, open the PRs, review and merge in order (adopt → fixes → redesign).
2. Check the redesign on a real phone and in Safari after the Vercel preview deploys.
3. Owner: create a new Qdrant cluster, set `QDRANT_URL`/`QDRANT_API_KEY` on the HF Space, run `run_etl.py`.

## How to run and check

- Run: `cd frontend && npm start` (site :3000), `cd backend && uvicorn app.main:app --reload` (API :8000)
- Check: `(cd frontend && npm run build) && (cd backend && python -m pytest -q)` (both pass 2026-10-27)
