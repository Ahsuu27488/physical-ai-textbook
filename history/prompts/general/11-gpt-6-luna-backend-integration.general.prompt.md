---
id: 11
title: Integrate OpenAI GPT-6 Luna as primary chat engine
stage: general
date: 2026-10-30
surface: agent
model: gpt-6-luna
feature: chat-engine
branch: feat/gpt-6-luna-backend
user: Muhammad Ahsan
command: (chat)
labels: ["backend", "openai", "gpt-6-luna", "rag", "embeddings", "streaming"]
links:
  spec: null
  ticket: null
  adr: D-034
  pr: null
files:
 - backend/app/config.py
 - backend/app/llm.py
 - backend/app/rag/embeddings.py
 - backend/app/rag/autoindex.py
 - backend/.env.example
 - backend/tests/conftest.py
 - backend/tests/test_chat_api.py
 - docs/STATUS.md
 - docs/CHANGELOG.md
 - docs/DECISIONS.md
tests:
 - Pytest suite: 103 passed (0 failed)
 - End-to-end Python streaming verification: 34 tokens streamed in 1.63s (time to first token: 1.29s)
---

## Prompt

Integrate OpenAI's GPT-6 Luna (`gpt-6-luna`) as the primary chat engine for Kinemata textbook backend, replacing the rate-limited Gemini free-tier chat models while keeping Gemini embeddings (`gemini-embedding-2`, 3072-d) for Qdrant retrieval.
Handle `max_completion_tokens` parameter compatibility for `gpt-6-luna` and fallback chain `gpt-5.6-luna,gpt-5-mini`.

## Response Summary

1. Updated `backend/app/config.py`: Added `openai_api_key`, automatic OpenAI base URL resolution (`https://api.openai.com/v1`) when `OPENAI_API_KEY` is present, default `chat_model="gpt-6-luna"`, fallback chain `"gpt-5.6-luna,gpt-5-mini"`, and `chat_reasoning_effort="none"`.
2. Updated `backend/app/llm.py`: Added `_prepare_kwargs` mapping `max_tokens` to `max_completion_tokens` for `gpt-6*`, `gpt-5*`, `o1*`, `o3*`, `o4*` models, and updated the 503 detail message.
3. Updated `backend/app/rag/embeddings.py`: Ensured native Gemini batch embeddings use `gemini_api_key` regardless of `llm_base_url` pointing to OpenAI, protecting the 3072-d Qdrant collection.
4. Updated `backend/app/rag/autoindex.py`: Applied `_prepare_kwargs` to health check completion calls.
5. Updated `backend/.env.example` and test fixtures in `backend/tests/conftest.py` and `backend/tests/test_chat_api.py`.
6. Verified 103 unit tests pass and completed end-to-end streaming live test with 1.29s time-to-first-token.
