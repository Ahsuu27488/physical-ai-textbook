# Decision Log

> Each entry: what was decided, **why**, and where it lives in the code. Numbering is permanent: never
> renumber or delete an entry. When a decision changes, add a new entry and mark the old heading
> *(superseded by D-NNN)*. A decision here beats anything that contradicts it elsewhere.

**D-001 Project memory and workflow set up with shipshape.** (2026-10-27)
Docs in `docs/` (STATUS, DECISIONS, ARCHITECTURE, CHANGELOG) are the project's memory and are loaded at every
session start; code commits must update STATUS and CHANGELOG (`.githooks/commit-msg`); work happens on branches
and lands through pull requests; the default branch is protected by `.githooks/pre-push`.
*Why:* AI coding sessions start with no memory; without written state and reasons, settled decisions get
re-made and working code gets broken. Pushing main also deploys (Vercel, Pages, HF Space), so main must only
move through reviewed PRs. *Where:* `AGENTS.md`, `.shipshape/`, `.githooks/`, `docs/`.

**D-002 Docusaurus 3 site for the textbook, separate FastAPI service for AI features.** (2025-12-07, recovered)
Lessons are Markdown in `frontend/docs/`; chat (RAG), auth, personalization and translation are a Python API.
*Why:* from `specs/001-physical-ai-textbook` and commit b647b34: static docs site for content, Python for the
OpenAI/Qdrant pipeline. *Where:* `frontend/`, `backend/`.

**D-003 Backend hosted on a Hugging Face Space, frontend on Vercel.** (2025-12, recovered)
`deploy-hf.yml` pushes `backend/` to `Ahsuu27488/physical-ai-textbook-backend` on every main push touching
`backend/**`; the site's production `url` is the Vercel domain. Railway (railway.json, Procfile) was the earlier
host. *Why:* from commits 01b0946 (HF auto-deploy) and ac61216 (Vercel base URL); reason for leaving Railway not
recorded. *Where:* `.github/workflows/deploy-hf.yml`, `frontend/docusaurus.config.js`.

**D-004 Backend URL is a build-time setting, not hard-coded.** (2025-12, recovered)
`BACKEND_URL` at build time → `customFields.backendUrl`; the HF Space URL is the fallback.
*Why:* commit 9fda10f; the URL had been changed by hand in four files during host moves (2b77905, 8c0358f).
*Where:* `frontend/docusaurus.config.js`.

**D-005 Qdrant connects lazily; the API starts without it.** (2025-12, recovered)
`QdrantManager(skip_init=True)`; endpoints that need it degrade instead of crashing the process.
*Why:* commit a4c4816: startup crashed when Qdrant was unreachable. *Where:* `backend/app/vector_db.py`.

**D-006 Broken links warn instead of failing the build.** (2025-12-07, recovered)
`onBrokenLinks: 'warn'`, `onBrokenAnchors: 'warn'`. *Why:* commit a4d13ff, to let CI deploy despite minor link
issues. *Where:* `frontend/docusaurus.config.js`.

**D-007 Docusaurus 3.10 with the Rspack bundler (`@docusaurus/faster`).** (2026-10-27)
Upgraded from 3.9.2; `future.v4: true` turns the faster bundler on, which needs the `@docusaurus/faster` package.
*Why:* `npm audit fix` moved webpack past what 3.9.2's progress bar accepts (build failed with "Progress Plugin
... options object"); 3.10 fixes that and drops 44 advisories. Builds take ~14 s. *Where:* `frontend/package.json`.

**D-008 `backend/requirements.txt` lists only what the code imports.** (2026-10-27)
Removed unstructured, pandas, numpy, PyYAML, betterproto (beta), alembic, asyncpg, python-jose, requests.
*Why:* `unstructured` made `pip install` fail with ResolutionImpossible on Python 3.12, none of these were imported,
and they slowed the HF Space image. Add a package back only together with the code that imports it.
*Where:* `backend/requirements.txt`; test-only deps in `backend/requirements-dev.txt`.

**D-009 OpenAI is optional at startup; missing key → 503 on AI endpoints only.** (2026-10-27)
One lazy client in `app/llm.py`. *Why:* building `OpenAI()` at import time crashed the whole API (auth and
/health too) when the key was unset. Extends D-005 (same rule for Qdrant). *Where:* `backend/app/llm.py`.

**D-010 Blog turned off.** (2026-10-27)
`blog: false`; the 2019-2021 Docusaurus sample posts were deleted. *Why:* they were template content shown as the
project's own. Turn it back on with real posts. *Where:* `frontend/docusaurus.config.js`.

<!-- Template for the next entry:

**D-00N <Decision in one line>.** (YYYY-MM-DD)
<What exactly was decided.>
*Why:* <the problem, the alternatives considered, why this one.> *Where:* <files / functions / settings>.
-->
