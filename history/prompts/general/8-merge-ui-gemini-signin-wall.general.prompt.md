---
id: 8
title: Merge UI pass, free Gemini, Rivet sign-in wall
stage: general
date: 2026-10-28
surface: agent
model: claude-opus-5-5
feature: none
branch: feat/proprio-pro-course
user: Muhammad Ahsan
command: (chat)
labels: ["chatbot", "gemini", "auth", "deploy", "branding"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - backend/app/{config.py,llm.py,ratelimit.py,api/chat.py,services/translation.py}
 - backend/app/rag/{embeddings.py,service.py,store.py,ingest.py}
 - backend/tests/{conftest.py,test_api.py,test_chat_api.py,test_ingest.py,test_retrieval.py,test_account_progress.py}
 - backend/{.env.example,README.md,README_HF.md,run_etl.py}
 - frontend/src/components/ChatWidget/{index.js,styles.module.css}
 - frontend/src/utils/api.js
 - design/rivet_wall.py
 - docs/{STATUS,DECISIONS,ARCHITECTURE,CHANGELOG}.md
tests:
 - backend: python -m pytest -q (77 passed)
 - frontend: npm run build (success); node scripts/check-mdx.mjs docs (ok)
 - design/rivet_wall.py against the dev site + local backend (wall, sign-up, question sent after sign-in)
---

## Prompt

yes merge, and give me the live link to the working site, make sure its working while live on internet, then start working on switch rivet to the free model. And my question, should we rename proprio to a new name because you have told me there is already something on internet with the same name. and make sure the rivet asks for a logged in session or a wall of users signed in or signed up.

(mid-turn) the theme mode switch doesnt look and feel good

## Response snapshot

Fast-forwarded the UI pass branch into feat/proprio-pro-course. Moved Rivet to Gemini's free tier through the
OpenAI-compatible API (gemini-3.8-flash with a flash-lite fallback on 429, gemini-embedding-2 at 3072-d with task
prefixes, backoff for indexing), D-020. Made chat/stream/search require an account with per-account minute and day
limits, plus a sign-in wall in the chat window that sends the waiting question after sign-in, D-021. Checked the wall
end to end in the browser. Theme switch redesign and the live deploy follow in the same session.

## Outcome

- ✅ Impact: Rivet costs nothing to run and only signed-in learners can spend the free quota.
- 🧪 Tests: 77 backend tests; build and MDX check pass; wall checked in Chromium.
- 📁 Files: see list above.
- 🔁 Next prompts: owner adds GEMINI/Qdrant keys and SECRET_KEY to the Space; index; deploy; pick a name.
- 🧠 Reflection: real Gemini calls are untested until a key exists; model ids come from the pricing page.

## Evaluation notes (flywheel)

- Failure modes observed: an AliasChoices env alias let a blank LLM_API_KEY hide GEMINI_API_KEY; fixed with two fields.
- Next experiment: run one live question against Gemini as soon as the key is in backend/.env.
