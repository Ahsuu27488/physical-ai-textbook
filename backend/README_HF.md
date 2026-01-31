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
OPENAI_API_KEY=sk-...
DATABASE_URL=postgresql://...
QDRANT_URL=https://...
QDRANT_API_KEY=...
JWT_SECRET=your-secret-key
```

## API Endpoints

- `GET /` - API information
- `GET /health` - Health check
- `POST /api/chat` - Chat with the textbook
- `POST /api/auth/register` - User registration
- `POST /api/auth/login` - User login

Built for the [Physical AI & Humanoid Robotics Textbook](https://github.com/Ahsuu27488/physical-ai-textbook).
