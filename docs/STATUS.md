# Project Status

> Current truth only: what works now, what doesn't, what's next. History goes in CHANGELOG.md, reasons in
> DECISIONS.md. Updated in the same commit as every code change (the commit-msg hook checks). Keep it short:
> it is loaded into every agent session.

**Last updated:** 2026-10-27 · **Code at:** 9fda10f + chore/adopt-shipshape
**In one line:** Textbook site builds and is live; the chatbot is down because the backend can't reach Qdrant.

## What works (verified)

- Frontend production build: `cd frontend && npm ci && npm run build` succeeds (2026-10-27, Node 24).
- Live site: https://physical-ai-textbook-puce-two.vercel.app returns 200 (2026-10-27).
- Live API: `GET /health` on the HF Space returns `{"status":"healthy"}` (2026-10-27).

## In progress

- chore/adopt-shipshape - this setup (docs, hooks, CI). No runtime change.

## Live state

- Frontend: Vercel (production `url` in `docusaurus.config.js`) and GitHub Pages (`deploy.yml` on main).
- Backend: HF Space `Ahsuu27488/physical-ai-textbook-backend`, auto-deployed by `deploy-hf.yml`.
- `GET /api/documents/health` → `unhealthy: Connection reset by peer`: the Qdrant cluster is gone or asleep,
  so `/api/chat` answers without textbook context.

## Known gaps

1. Qdrant unreachable from the live backend (see Live state); needs a new cluster + `run_etl.py`.
2. `BACKEND_URL` never takes effect: `client-modules/AppProvider.js` default export never renders, and
   `utils/api.js` computes its base URL once at import.
3. Backend crashes at import without `OPENAI_API_KEY` (`OpenAI()` built at module load in `api/chat.py`,
   `services/translation.py`).
4. SQL tables are never created at startup (`database.py` and `models.py` each declare their own `Base`).
5. `/api/v1/translation/translate-to-urdu` reads `content` as a query param; the UI sends form data → 422.
6. `translate-chapter`'s "optional" auth uses an `OAuth2PasswordBearer` that rejects missing tokens.
7. Footer links to `/docs/intro`, which doesn't exist (broken link on every page); footer/blog still Docusaurus boilerplate.
8. `npm audit`: 66 vulnerabilities reported (2026-10-27). `backend/test.db` is committed.
9. No automated tests run in CI (`backend/test_auth.py` is a print-script, not pytest).

## Next

1. Fix gaps 2-9 on a branch with tests.
2. Redesign the landing page and chat widget (3D, design-craft).
3. Owner: create a new Qdrant cluster, set `QDRANT_URL`/`QDRANT_API_KEY` on the HF Space, run `run_etl.py`.

## How to run and check

- Run: `cd frontend && npm start` (site), `cd backend && uvicorn app.main:app --reload` (API)
- Check: `cd frontend && npm run build` (passes 2026-10-27). No test suite yet.
