# Architecture (as built)

> The map of what actually exists. Update it whenever modules, data, interfaces, commands or flows change.
> Only real paths and names: every one here must exist in the code.

## Runtime

```
 Browser ── static site (Docusaurus build) ─────────── Vercel  (also GitHub Pages via deploy.yml)
    │
    │  fetch  {backendUrl}/api/...        backendUrl = docusaurus.config.js customFields.backendUrl
    ▼                                                 (env BACKEND_URL at build time, else the HF Space URL)
 FastAPI (uvicorn app.main:app) ──────────────────── Hugging Face Space (Docker, deploy-hf.yml)
    ├── OpenAI  chat (gpt-4o-mini) + embeddings (text-embedding-ada-002, 1536-d)
    ├── Qdrant  collection QDRANT_COLLECTION_NAME (default physical_ai_docs)
    └── SQL     NEON_DATABASE_URL (Postgres) or sqlite:///./test.db fallback
```

`railway.json`, `Procfile` and the README badges still describe an older Railway deployment.

## Code map

| Path | Responsibility |
|---|---|
| `frontend/docs/` | The textbook: Markdown lessons for modules 1-4 and weekly intros |
| `frontend/sidebars.js` | `textbookSidebar` order of lessons |
| `frontend/docusaurus.config.js` | Site config, navbar custom items, `customFields.backendUrl` |
| `frontend/src/pages/index.js` | Landing page |
| `frontend/src/components/HomepageFeatures/` | Landing feature cards |
| `frontend/src/components/ChatWidget.js` | Floating RAG chat window, rendered on every page by `theme/Layout` |
| `frontend/src/theme/Root.js` | Wraps the app in `AuthProvider` |
| `frontend/src/theme/Layout/index.js` | Wraps the original Layout and adds `ChatWidget` (browser only) |
| `frontend/src/theme/NavbarItem/` | `custom-AuthButtons` (sign in/up modal) and `custom-TranslatePersonalizeButtons` ("Tools" menu) |
| `frontend/src/hooks/useAuth.js` | Auth context: JWT in `localStorage.auth_token`, login/register/logout |
| `frontend/src/utils/api.js` | `chatApi.sendMessage`, `chatApi.searchDocuments` |
| `frontend/src/utils/backendUrl.js` | `getBackendUrl()`: the one place the API base URL comes from (`@generated/docusaurus.config`) |
| `backend/app/main.py` | FastAPI app, lifespan (`create_tables()`), CORS, router mounting, `/` and `/health` |
| `backend/app/llm.py` | `get_openai_client()`: lazy shared OpenAI client; 503 when `OPENAI_API_KEY` is unset |
| `backend/app/api/chat.py` | `/api/chat`, `/api/search`, `/api/documents/health` |
| `backend/app/api/v1/auth.py` | Register, token, login, me, profile |
| `backend/app/api/v1/personalization.py` | Per-chapter preferences and `apply` |
| `backend/app/api/v1/translation.py` | Translate endpoints (cached) |
| `backend/app/vector_db.py` | `QdrantManager`: lazy client, embeddings, search, upsert |
| `backend/app/etl/data_ingestion.py` | `ETLProcessor`: read `frontend/docs`, chunk, embed, load to Qdrant |
| `backend/app/database/` | SQLAlchemy engine/session (`database.py`) and tables (`models.py`) |
| `backend/app/auth/` | Password hashing + JWT (`utils.py`), user lookup services |
| `backend/app/services/` | `TranslationService` (OpenAI + cache), `PersonalizationService` |
| `backend/tests/` | pytest API tests: fresh SQLite DB, no secrets (`conftest.py` sets the env) |

## Data

SQL tables (`backend/app/database/models.py`): `users` (id, email, hashed_password, software_background,
hardware_background), `personalization_preferences` (user_id, chapter_id, preferences JSON),
`translation_cache` (content hash → translated text). No migrations: `create_tables()` runs `create_all` on
startup (adds missing tables, never alters existing ones). `database.Base` is `models.Base`.

Qdrant points: payload `title`, `content`, `source_path`, `chunk_index`, `total_chunks`, `created_at`.

## Interfaces

| Method | Path | Auth |
|---|---|---|
| POST | `/api/chat` `{message, conversation_id}` → `{response, conversation_id, sources[]}` | none |
| POST | `/api/search` `{query, top_k}` | none |
| GET | `/api/documents/health` | none |
| POST | `/api/v1/auth/register`, `/api/v1/auth/token` (form), `/api/v1/auth/login` (JSON) | none |
| GET/PUT | `/api/v1/auth/me`, `/api/v1/auth/profile` | Bearer |
| GET/POST/DELETE | `/api/v1/personalization/preference[s]...`, POST `/apply/{chapter_id}` | Bearer |
| POST | `/api/v1/translation/translate`, `/translate-chapter`, `/translate-to-urdu`; GET `/supported-languages` | mixed |

## How a chat request flows

1. `ChatWidget.handleSendMessage` → `chatApi.sendMessage` (`utils/api.js`) → POST `/api/chat`.
2. `chat.py:chat_endpoint` → `QdrantManager.generate_query_embedding` (OpenAI embeddings).
3. `QdrantManager.search_documents` → top-k chunks → joined into the system prompt.
4. OpenAI chat completion → `ChatResponse` with `sources` (chunk `source_path`s) → rendered in the widget.

## Configuration

Backend env vars (names only): `OPENAI_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY`, `QDRANT_COLLECTION_NAME`,
`NEON_DATABASE_URL`, `SECRET_KEY`, `ACCESS_TOKEN_EXPIRE_MINUTES`, `FRONTEND_URL`, `ADDITIONAL_CORS_ORIGIN`,
`BETTERAUTH_URL`, `PORT`. Frontend build-time: `BACKEND_URL`, `SITE_URL`, `BASE_URL` (`/physical-ai-textbook/` for GitHub Pages). CI secrets: `HF_TOKEN`, `HF_USERNAME`.

## Commands

```
cd frontend && npm ci && npm start          # dev site on :3000
cd frontend && npm run build                # production build (Docusaurus 3.10 + Rspack via @docusaurus/faster)
cd backend && pip install -r requirements-dev.txt && python -m pytest -q   # API tests
cd backend && pip install -r requirements.txt && uvicorn app.main:app --reload --port 8000
cd backend && python run_etl.py             # re-embed lessons into Qdrant (live, costs OpenAI credits)
```
