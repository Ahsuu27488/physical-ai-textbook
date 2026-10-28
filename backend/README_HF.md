---
title: Physical AI Textbook Backend
emoji: 📚
colorFrom: blue
colorTo: indigo
sdk: docker
app_port: 8000
pinned: false
license: mit
---

# Physical AI Textbook Backend

FastAPI backend for the Physical AI & Humanoid Robotics textbook platform.

## Environment Variables Required

Configure these in your Space Settings → Secrets:

```bash
GEMINI_API_KEY=...          # Google AI Studio, free tier
QDRANT_URL=https://...
QDRANT_API_KEY=...
NEON_DATABASE_URL=postgresql://...
SECRET_KEY=...              # long random string; signs login tokens
SITE_URL=https://...        # the site's origin, for CORS
```

## API Endpoints

- `GET /` - API information
- `GET /health` - Health check
- `POST /api/chat` - Chat with the textbook
- `POST /api/auth/register` - User registration
- `POST /api/auth/login` - User login

Built for the [Physical AI & Humanoid Robotics Textbook](https://github.com/Ahsuu27488/physical-ai-textbook).
