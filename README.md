<div align="center">

# Physical AI & Humanoid Robotics

### The World-Class AI-Native Learning Platform

[![Live Demo](https://img.shields.io/badge/Live%20Demo-GitHub%20Pages-blue?style=for-the-badge&logo=github)](https://ahsuu27488.github.io/physical-ai-textbook/)
[![Backend API](https://img.shields.io/badge/API-Railway-purple?style=for-the-badge&logo=railway)](https://physical-ai-textbook-production-fd94.up.railway.app/)
[![Docusaurus](https://img.shields.io/badge/Docusaurus-v3.9.2-green?style=for-the-badge&logo=docusaurus)](https://docusaurus.io/)
[![FastAPI](https://img.shields.io/badge/FastAPI-0.115.0-009688?style=for-the-badge&logo=fastapi)](https://fastapi.tiangolo.com/)
[![License](https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge)](LICENSE)

<br />

<img src="frontend/static/img/logo.png" alt="Physical AI Logo" width="200" />

**Master the intersection of AI and Robotics**

From ROS 2 fundamentals to Vision-Language-Action systems, build intelligent embodied AI systems that operate in the physical world.

[**Explore the Textbook**](https://ahsuu27488.github.io/physical-ai-textbook/) | [**API Documentation**](https://physical-ai-textbook-production-fd94.up.railway.app/docs) | [**Report Bug**](https://github.com/Ahsuu27488/physical-ai-textbook/issues)

</div>

---

## Overview

This platform is a comprehensive, interactive textbook for learning **Physical AI & Humanoid Robotics**. It bridges the gap between digital AI models and embodied intelligence that operates in the physical world.

### Why Physical AI?

> *"Humanoid robots are poised to excel in our human-centered world because they share our physical form and can be trained with abundant data from interacting in human environments."*

Physical AI represents a fundamental shift from AI systems that operate purely in digital spaces to systems that function in and interact with the physical world.

---

## Features

<table>
<tr>
<td width="50%">

### AI-Powered Learning
- **RAG Chatbot** - Floating AI assistant trained on textbook content
- **Personalized Content** - Adapts to your software/hardware background
- **Urdu Translation** - Full content translation with caching

</td>
<td width="50%">

### Modern Tech Stack
- **Futuristic UI/UX** - Glassmorphism, deep space aesthetics
- **Dark/Light Modes** - Seamless theme switching
- **Responsive Design** - Works on all devices

</td>
</tr>
</table>

### Key Capabilities

| Feature | Description |
|---------|-------------|
| **Interactive Lessons** | 200+ lessons with code examples, diagrams, and exercises |
| **RAG Chat Widget** | Ask questions and get answers from the textbook content |
| **Personalization** | Content adapts based on your experience level |
| **Multi-language** | Translate to Urdu (more languages coming) |
| **User Accounts** | Track progress and save preferences |

---

## Curriculum

<table>
<tr>
<td align="center" width="25%">
<img src="https://img.shields.io/badge/Module%201-ROS%202-blue?style=flat-square" /><br/>
<b>The Robotic Nervous System</b><br/>
<sub>Nodes, Topics, Services, Actions, URDF</sub>
</td>
<td align="center" width="25%">
<img src="https://img.shields.io/badge/Module%202-Gazebo-green?style=flat-square" /><br/>
<b>The Digital Twin</b><br/>
<sub>Physics simulation, sensors, Unity</sub>
</td>
<td align="center" width="25%">
<img src="https://img.shields.io/badge/Module%203-NVIDIA%20Isaac-76B900?style=flat-square" /><br/>
<b>The AI-Robot Brain</b><br/>
<sub>Isaac Sim, VSLAM, Nav2</sub>
</td>
<td align="center" width="25%">
<img src="https://img.shields.io/badge/Module%204-VLA-purple?style=flat-square" /><br/>
<b>Vision-Language-Action</b><br/>
<sub>LLMs, Whisper, Autonomous Humanoid</sub>
</td>
</tr>
</table>

---

## Tech Stack

### Frontend
```
Docusaurus v3.9.2  │  React 19  │  Tailwind CSS v4  │  Framer Motion
```

### Backend
```
FastAPI  │  Python 3.11+  │  SQLAlchemy  │  OpenAI API
```

### Infrastructure
```
Neon PostgreSQL  │  Qdrant Vector DB  │  GitHub Pages  │  Railway
```

---

## Project Structure

```
physical-ai-textbook/
├── frontend/                   # Docusaurus Site
│   ├── docs/                   # Course content (MDX)
│   │   ├── module-1/           # ROS 2 Fundamentals
│   │   ├── module-2/           # Gazebo & Unity
│   │   ├── module-3/           # NVIDIA Isaac
│   │   └── module-4/           # Vision-Language-Action
│   ├── src/
│   │   ├── components/         # React components
│   │   │   ├── HeroSection.jsx
│   │   │   ├── ChatWidget.js   # RAG AI assistant
│   │   │   ├── BentoGrid.jsx
│   │   │   └── ...
│   │   ├── hooks/useAuth.js    # Authentication
│   │   └── css/custom.css      # Tailwind + theme
│   └── docusaurus.config.js
│
├── backend/                    # FastAPI Backend
│   ├── app/
│   │   ├── api/v1/
│   │   │   ├── auth.py         # User authentication
│   │   │   ├── personalization.py
│   │   │   └── translation.py
│   │   ├── services/           # Business logic
│   │   ├── database/           # SQLAlchemy models
│   │   └── main.py             # FastAPI entry
│   └── requirements.txt
│
├── .claude/                    # Claude Code AI Skills
│   ├── skills/                 # Reusable capabilities
│   └── agents/                 # Autonomous subagents
│
└── specs/                      # Feature specifications
```

---

## Getting Started

### Prerequisites

- **Node.js** 20+
- **Python** 3.11+
- **Git**

### Installation

#### 1. Clone the repository
```bash
git clone https://github.com/Ahsuu27488/physical-ai-textbook.git
cd physical-ai-textbook
```

#### 2. Set up the Frontend
```bash
cd frontend
npm install
npm start
```
The site will be available at `http://localhost:3000`

#### 3. Set up the Backend (optional for local development)
```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

Create a `.env` file:
```env
NEON_DATABASE_URL=your_neon_postgres_url
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
FRONTEND_URL=http://localhost:3000
```

Start the backend:
```bash
uvicorn app.main:app --reload
```
API available at `http://localhost:8000`

---

## Environment Variables

### Backend (.env)
| Variable | Description |
|----------|-------------|
| `NEON_DATABASE_URL` | Neon PostgreSQL connection string |
| `OPENAI_API_KEY` | OpenAI API key for chat and translation |
| `QDRANT_URL` | Qdrant cloud URL |
| `QDRANT_API_KEY` | Qdrant API key |
| `FRONTEND_URL` | Frontend URL for CORS |
| `ACCESS_TOKEN_EXPIRE_MINUTES` | JWT token expiration (default: 30) |

---

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/chat` | POST | RAG chat with textbook content |
| `/api/v1/auth/register` | POST | User registration |
| `/api/v1/auth/token` | POST | Login and get JWT |
| `/api/v1/auth/me` | GET | Get current user |
| `/api/v1/translation/translate` | POST | Translate content |
| `/api/v1/personalization/apply/{chapter}` | POST | Personalize content |

Full API docs: [https://physical-ai-textbook-production-fd94.up.railway.app/docs](https://physical-ai-textbook-production-fd94.up.railway.app/docs)

---

## Deployment

### Frontend (GitHub Pages)
```bash
cd frontend
npm run build
npm run deploy
```

### Backend (Railway)
The backend is configured for Railway deployment via `railway.json`:
```json
{
  "deployments": {
    "services": [{
      "name": "backend",
      "dir": "backend",
      "command": "uvicorn app.main:app --host=0.0.0.0 --port=${PORT:-8000}"
    }]
  }
}
```

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        GitHub Pages                              │
│                    (Docusaurus Static Site)                      │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                         Railway                                  │
│                    (FastAPI Backend)                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │    Auth     │  │ Translation │  │    Personalization      │  │
│  │   Service   │  │   Service   │  │       Service           │  │
│  └─────────────┘  └─────────────┘  └─────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
           │                │                      │
           ▼                ▼                      ▼
┌──────────────────┐ ┌──────────────┐  ┌─────────────────────────┐
│ Neon PostgreSQL  │ │   OpenAI     │  │    Qdrant Vector DB     │
│  (User Data)     │ │    API       │  │     (RAG Content)       │
└──────────────────┘ └──────────────┘  └─────────────────────────┘
```

---

## Design System

### Color Palette

| Color | Light Mode | Dark Mode | Usage |
|-------|------------|-----------|-------|
| **Primary** | `#2563eb` | `#3b82f6` | Buttons, links |
| **Accent** | `#0d9488` | `#2dd4bf` | Highlights, hover states |
| **Background** | `#f8fafc` | `#0f172a` | Page background |
| **Surface** | `#ffffff` | `#1e293b` | Cards, modals |

### Design Tokens
- **Glassmorphism**: `backdrop-filter: blur(10px)` with semi-transparent backgrounds
- **Transitions**: `200-300ms ease` for smooth interactions
- **Typography**: Inter (sans-serif), Fira Code (monospace)

---

## Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

### Development Guidelines

- Follow the [Constitution](.specify/memory/constitution.md) principles
- Use atomic commits with clear messages
- Test locally before pushing
- Update documentation as needed

---

## Hardware Requirements

This course is computationally demanding. Recommended setup:

### Development Workstation
| Component | Requirement |
|-----------|-------------|
| **GPU** | NVIDIA RTX 4070 Ti (12GB VRAM) or higher |
| **CPU** | Intel Core i7 (13th Gen+) or AMD Ryzen 9 |
| **RAM** | 64 GB DDR5 |
| **OS** | Ubuntu 22.04 LTS |

### Physical AI Edge Kit
| Component | Hardware |
|-----------|----------|
| **Compute** | NVIDIA Jetson Orin Nano/NX |
| **Vision** | Intel RealSense D435i |
| **IMU** | USB IMU for balance |
| **Audio** | USB Microphone/Speaker array |

---

## Roadmap

- [x] Core textbook content (4 modules)
- [x] RAG chatbot with Qdrant
- [x] User authentication system
- [x] Content personalization
- [x] Urdu translation
- [x] Futuristic UI/UX redesign
- [ ] More language translations
- [ ] Progress tracking
- [ ] Interactive code playgrounds
- [ ] Video content integration
- [ ] Community features

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## Acknowledgments

- **Docusaurus** - Amazing documentation framework
- **OpenAI** - Powering the RAG chatbot and translation
- **Qdrant** - Vector database for semantic search
- **Neon** - Serverless PostgreSQL
- **Railway** - Seamless backend deployment
- **NVIDIA** - Isaac platform documentation

---

<div align="center">

**Built with passion for the future of robotics**

Made by [Ahsan](https://github.com/Ahsuu27488)

<br />

[![Stars](https://img.shields.io/github/stars/Ahsuu27488/physical-ai-textbook?style=social)](https://github.com/Ahsuu27488/physical-ai-textbook)
[![Follow](https://img.shields.io/github/followers/Ahsuu27488?style=social)](https://github.com/Ahsuu27488)

</div>
