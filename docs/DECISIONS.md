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

**D-011 Visual direction: dark "sensor lab", one accent, LiDAR humanoid as the signature.** (2026-10-27)
Dark-first OKLCH tokens (`--pa-*` in `custom.css`) with one accent (sensor lime) in both themes; Geist + Geist Mono
with Instrument Serif italic for accent words; glass only on the navbar and chat composer; docs stay calm (no
smooth scroll, a CSS scroll-timeline progress bar). The landing hero and closing band are always dark.
*Why:* the request was an immersive 3D redesign; the owner's design-craft taste profile asks for dark premium,
one signature effect and no generic AI look. A point cloud being scanned matches the subject (sensors, simulation).
*Where:* `src/css/custom.css`, `src/pages/index.*`, `src/components/Landing/`.

**D-012 3D is progressive: poster first, live WebGL only on desktop-class devices.** (2026-10-27)
Plain three.js (no R3F) in a lazy chunk; phones (<768 px), ≤4 cores/≤4 GB and reduced motion keep a pre-rendered
WebP (`static/img/hero-humanoid.webp`, from `design/make_poster.py`) and never download three.js. DPR capped at 2,
loop paused off-screen and in hidden tabs. *Why:* design-craft budget (LCP is HTML, static fallback on mobile);
Lighthouse showed the hero image is the mobile LCP element. Re-run `make_poster.py` after editing the scene.
*Where:* `src/components/Landing/HeroScene.js`, `humanoidScene.js`.

**D-013 Fonts self-hosted, Latin only; Urdu font on demand.** (2026-10-27)
*Why:* Google Fonts stylesheets were render-blocking on every page (Lighthouse), and Fontsource's full CSS made
the bundler inline the extra subsets as base64 (site CSS 16 → 59 KB gz). Now 22 KB gz. *Where:* `src/fonts/`,
`@font-face` in `custom.css`, `src/utils/urduFont.js`.

**D-014 The chat widget lives in `theme/Root`, not `theme/Layout`.** (2026-10-27)
*Why:* Layout remounts on every navigation, which wiped the conversation. *Where:* `src/theme/Root.js`.

**D-015 Rebrand to Proprio; the assistant is Rivet; a new 11-module course replaces the textbook.** (2026-10-28)
Plan and syllabus: `docs/plans/2026-10-27-proprio-course.md`. Lessons follow one format (meta, objectives,
teaching, lab, exercises, quiz, takeaways) using the components in `src/components/Lesson`. Content was written by
cheaper-model agents from that plan and validated with `scripts/check-mdx.mjs`.
*Why:* owner request: a full beginner-to-expert course instead of 13 thin pages, under a new name. The old content
is in git history. *Where:* `frontend/docs/`, `frontend/src/components/Lesson/`, `frontend/docusaurus.config.js`.

**D-016 RAG v2: heading-aware chunks with lesson URLs, text-embedding-3-small, SSE streaming.** (2026-10-28)
Chunks per `##`/`###` section with page title, section and `/docs/...#anchor`; deterministic ids so re-indexing is
idempotent; stale points removed; retrieval favours the page the learner is on; answers stream over SSE with a
non-streaming fallback. *Why:* the old ETL embedded raw file text with ada-002, linked sources as file paths, and
returned zero vectors on errors. *Where:* `backend/app/rag/`, `backend/app/api/chat.py`, `frontend/src/utils/api.js`.

**D-017 Progress lives in the browser first and syncs to the account when signed in.** (2026-10-28)
`proprio:progress` in localStorage; `ProgressSync` merges with `/api/v1/progress` (union of lessons, best quiz score).
*Why:* learners should not need an account to track progress, and should not lose it when they sign in.
*Where:* `frontend/src/utils/progress.js`, `frontend/src/components/ProgressSync.js`, `backend/app/api/v1/progress.py`.

**D-018 Site search is the offline local-search plugin.** (2026-10-28)
`@easyops-cn/docusaurus-search-local` builds the index at build time. *Why:* no external service or account
(Algolia DocSearch needs an application); works on Vercel and Pages. Search works only on built sites.
*Where:* `frontend/docusaurus.config.js`.

**D-019 One shared control system; Markdown renderers load lazily.** (2026-10-28)
Buttons, inputs, selects, segmented controls, popover menus, notes, kbd and dialogs are global classes in
`custom.css` (`.button--ghost/--danger`, `.pa-*`), used by auth, Tools, profile and Rivet; dialogs get their
behaviour from `hooks/useDialog.js` (bottom sheet on phones). Rivet's and the Tools dialog's Markdown renderer
(react-markdown, remark-gfm, KaTeX plugins) are `React.lazy` chunks. Keeps D-011 (tokens, one accent, glass only on
the floating layer). *Why:* each component had its own copy of input/menu styles that had drifted apart, dialogs had
no focus trap, and the Markdown stack sat in the main bundle of every page (main.js 285 → 173 KB gz).
*Where:* `src/css/custom.css`, `src/hooks/useDialog.js`, `src/components/ChatWidget/Markdown.js`, `src/theme/NavbarItem/`.

<!-- Template for the next entry:

**D-00N <Decision in one line>.** (YYYY-MM-DD)
<What exactly was decided.>
*Why:* <the problem, the alternatives considered, why this one.> *Where:* <files / functions / settings>.
-->
