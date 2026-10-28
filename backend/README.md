# Proprio backend (FastAPI)

API for the Proprio course: **Rivet**, the course assistant (retrieval-augmented chat over the lessons), plus accounts,
learning-progress sync, personalization and translation.

## Rivet pipeline

```
frontend/docs/*.mdx --(app/rag/chunking.py)--> chunks --(embeddings.py)--> vectors --(store.py)--> Qdrant
question --> embed --> Qdrant top-k (+ current page) --> prompts.py --> OpenAI chat --> answer + numbered sources
```

| Module | Job |
|---|---|
| `app/config.py` | `Settings` (env vars / `.env`), `get_settings()` |
| `app/rag/chunking.py` | MDX lesson -> plain text -> section chunks with url, anchor, module |
| `app/rag/embeddings.py` | Batched OpenAI embeddings (raises on error) |
| `app/rag/store.py` | Qdrant collection, upsert, stale cleanup, `query_points` search, health |
| `app/rag/ingest.py` | `python -m app.rag.ingest` |
| `app/rag/prompts.py` | Rivet's system prompt and message builder |
| `app/rag/service.py` | `answer()` / `answer_stream()` and retrieval |
| `app/api/chat.py` | `/api/chat`, `/api/chat/stream` (SSE), `/api/search`, `/api/documents/health` |
| `app/ratelimit.py` | In-memory per-account limits (per minute, per day) for the chat endpoints |

## Setup

```bash
python -m venv .venv && .venv/Scripts/activate     # Linux/Mac: source .venv/bin/activate
pip install -r requirements.txt -r requirements-dev.txt
# create backend/.env with the variable names listed below (never commit it)
```

## Commands

```bash
uvicorn app.main:app --reload --port 8000            # run the API
python -m pytest -q                                  # tests: no network, no secrets

python -m app.rag.ingest --docs ../frontend/docs --dry-run   # chunk only, print stats (no keys needed)
python -m app.rag.ingest --docs ../frontend/docs             # embed + upsert + remove stale points (live, uses the Gemini free quota)
python run_etl.py                                            # same as the live command, docs path resolved for you
```

## Environment variables (names only)

`GEMINI_API_KEY`, `CHAT_MODEL`, `CHAT_FALLBACK_MODEL`, `CHAT_REASONING_EFFORT`, `EMBED_MODEL`, `EMBED_DIM`,
`LLM_BASE_URL`, `LLM_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY`, `QDRANT_COLLECTION_NAME`, `RAG_TOP_K`,
`RAG_SCORE_THRESHOLD`, `SITE_URL`, `FRONTEND_URL`, `ADDITIONAL_CORS_ORIGIN` (comma-separated list allowed),
`CHAT_RATE_LIMIT_PER_MIN`, `CHAT_DAILY_LIMIT_PER_USER`, `NEON_DATABASE_URL`, `SECRET_KEY`, `ACCESS_TOKEN_EXPIRE_MINUTES`, `BETTERAUTH_URL`, `PORT`.

`http://localhost:3000` and `http://localhost:3123` are always allowed by CORS.
Defaults run on Google Gemini's free tier (D-020). Changing `EMBED_MODEL`/`EMBED_DIM` needs a new collection
(`QDRANT_COLLECTION_NAME`) and a re-index; indexing refuses a collection of another vector size.

## API

| Method | Path | Notes |
|---|---|---|
| GET | `/health` | liveness |
| POST | `/api/chat` | `{message, conversation_id?, history?, page?}` -> `{response, conversation_id, sources[{n,title,section,url,score}]}` |
| POST | `/api/chat/stream` | SSE: `meta` -> `token`* -> `done`, or `error` |
| POST | `/api/search` | `{query, top_k}` -> `{query, results[{n,title,section,url,score,text}]}` |
| GET | `/api/documents/health` | collection, points count, embed model |
| POST | `/api/v1/auth/register`, `/token`, `/login` | accounts |
| GET / PUT | `/api/v1/auth/me`, `/profile` | profile: name, software/hardware background (`beginner`, `intermediate`, `advanced`, or empty) |
| POST | `/api/v1/auth/change-password` | `{current_password, new_password}` (min 8) |
| DELETE | `/api/v1/auth/account` | `{password}`; also deletes progress and preferences |
| GET / PUT | `/api/v1/progress/` | `{completed[], quizzes{page_path:{best,total}}}`; PUT merges (union / max) |
| DELETE | `/api/v1/progress/completed/{doc_id}` | un-complete one lesson |
| | `/api/v1/personalization/...`, `/api/v1/translation/...` | unchanged |

Chat and search need a signed-in account (Bearer token, 401 otherwise). Without `GEMINI_API_KEY` they return 503. Without Qdrant, Rivet still answers from general knowledge with
`sources: []`. Each account gets `CHAT_RATE_LIMIT_PER_MIN` questions a minute and `CHAT_DAILY_LIMIT_PER_USER` a day (429 above that).
