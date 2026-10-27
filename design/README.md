# Design tooling

Scripts used to build and check the 2026-10 redesign. They drive the running dev site
(`cd frontend && npm start -- --port 3123`) with Playwright (`pip install playwright && playwright install chromium`).

| Script | What it does |
|---|---|
| `shoot.py <dir>` | Screenshots home, a lesson, 404, the open chat, sign-in and the Tools menu at 375 / 768 / 1440 px |
| `scroll_steps.py <dir> [path]` | Viewport shots while scrolling (scroll reveals need real scrolling); reports horizontal overflow |
| `modes.py <dir>` | Light theme, mobile menu, reduced motion, and a keyboard pass (tab order, chat focus + Esc) |
| `make_poster.py` | Renders the hero humanoid once and writes `frontend/static/img/hero-humanoid.webp`. Re-run after changing `humanoidScene.js` |
| `compare.py` | Side-by-side `before/` vs `after/` images into `compare/` |

Screenshot folders (`before/`, `after/`, `compare/`) are local output and not committed.
