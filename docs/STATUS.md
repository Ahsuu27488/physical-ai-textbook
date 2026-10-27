# Project Status

> Current truth only: what works now, what doesn't, what's next. History goes in CHANGELOG.md, reasons in
> DECISIONS.md. Updated in the same commit as every code change (the commit-msg hook checks). Keep it short:
> it is loaded into every agent session.

**Last updated:** 2026-10-27 · **Code at:** fix/broken-config-and-deps (on chore/adopt-shipshape)
**In one line:** Site and API build and pass tests locally; the live chatbot still lacks Qdrant (owner action).

## What works (verified)

- Frontend build: `cd frontend && npm ci && npm run build` succeeds, no broken links (2026-10-27, Node 24).
- `BACKEND_URL` reaches the bundle; `BASE_URL=/physical-ai-textbook/` produces correct Pages asset paths
  (checked in the built `index.html`, 2026-10-27).
- Backend: `cd backend && python -m pytest -q` → 5 passed (2026-10-27, Python 3.12): starts with no secrets,
  creates tables, register → token → me, AI endpoints 503 without a key, form-data Urdu translation, anonymous
  translate-chapter.
- Live (before this branch): site 200 on Vercel; HF Space `/health` healthy.

## In progress

- chore/adopt-shipshape - shipshape setup (docs, hooks, CI). Not pushed.
- fix/broken-config-and-deps - the fixes listed in CHANGELOG. Not pushed.

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

## Next

1. Redesign the landing page and chat widget (3D, design-craft) on its own branch.
2. Owner: push the branches, open the PRs, review and merge in order (adopt → fixes → redesign).
3. Owner: create a new Qdrant cluster, set `QDRANT_URL`/`QDRANT_API_KEY` on the HF Space, run `run_etl.py`.

## How to run and check

- Run: `cd frontend && npm start` (site :3000), `cd backend && uvicorn app.main:app --reload` (API :8000)
- Check: `(cd frontend && npm run build) && (cd backend && python -m pytest -q)` (both pass 2026-10-27)
