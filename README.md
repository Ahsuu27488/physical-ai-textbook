<div align="center">

<img src="frontend/static/img/logo.svg" alt="" width="72" height="72" />

# Kinemata

**A free course in robotics and Physical AI, from first principles to humanoids, with Rivet, a tutor that has read every lesson.**

[**Open the course**](https://physical-ai-textbook-puce-two.vercel.app) · [Urdu edition](https://physical-ai-textbook-puce-two.vercel.app/ur/) · [Report a problem](https://github.com/Ahsuu27488/physical-ai-textbook/issues)

</div>

## What's inside

- **11 modules, 52 lessons**: start here → foundations → robot fundamentals → ROS 2 → simulation → perception →
  motion → robot learning → humanoids → foundation models → capstone and career. Every lesson has objectives,
  teaching, a lab, exercises with solutions, a quiz and takeaways.
- **Rivet**, the course assistant: answers from the lessons with numbered citations, knows the page you're on,
  remembers the conversation. Free accounts only (it runs on a free model quota).
- **Your progress** is kept in the browser and synced to your account when you sign in.
- **Languages**: a full Urdu edition, plus any page translated on demand into 17 languages (signed in).
- **Personalize**: re-explain a page for your software and hardware background (signed in).

## How it's built

| Part | Stack | Hosted on |
|---|---|---|
| Site (`frontend/`) | Docusaurus 3.10 (Rspack), React, three.js hero, local search | Vercel (GitHub Pages redirects there) |
| API (`backend/`) | FastAPI, SQLAlchemy, Qdrant, Google Gemini (free tier) through the OpenAI SDK | Hugging Face Space (Docker) |
| Data | Postgres (accounts, progress, caches), Qdrant (lesson chunks for Rivet) | Neon, Qdrant Cloud free tier |

Merging to `main` deploys everything: Vercel builds the site, `deploy.yml` publishes the Pages redirects,
`deploy-hf.yml` pushes the API to the Space, which re-indexes the lessons on startup when they changed.

The map of the code is [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md); why things are the way they are is
[`docs/DECISIONS.md`](docs/DECISIONS.md); what works today and what's next is [`docs/STATUS.md`](docs/STATUS.md).

## Run it locally

```bash
cd frontend && npm ci && npm start            # the site on http://localhost:3000
```

```bash
cd backend && python -m venv .venv && .venv/Scripts/pip install -r requirements-dev.txt   # Windows path; bin/ on macOS/Linux
cd backend && .venv/Scripts/python -m uvicorn app.main:app --reload --port 8000
```

The API needs a `backend/.env` for the AI features (names in [`backend/.env.example`](backend/.env.example);
`GEMINI_API_KEY` for Rivet, `QDRANT_URL` + `QDRANT_API_KEY` for the lesson index). Without them, accounts and
progress still work and the AI endpoints answer 503. Point the site at a local API with `BACKEND_URL=http://localhost:8000 npm start`.

## Checks

```bash
cd frontend && npm run build && node scripts/check-mdx.mjs docs
cd backend && python -m pytest -q
```

## Contributing

Work on a branch and open a pull request; code commits update `docs/STATUS.md` and `docs/CHANGELOG.md` (a git
hook checks: run `git config core.hooksPath .githooks` once per clone). See [`AGENTS.md`](AGENTS.md).

Made by [Ahsan](https://github.com/Ahsuu27488).
