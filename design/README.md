# Design tooling

Scripts used to build and check the 2026-10 redesign. They drive the running dev site
(`cd frontend && npm start -- --port 3140`, or a build: `npm run build && npm run serve -- --port 3142`) with Playwright (`pip install playwright && playwright install chromium`).

| Script | What it does |
|---|---|
| `shoot.py <dir> [url] [--only=a,b] [--widths=375,1440]` | Every surface and state (chat empty/thinking/streaming/answer/error/wide, auth, Tools, account, lesson parts, profile, 404, footer, search, mobile menu) at 375 / 768 / 1440, light at 375 / 1440, spot checks at 320 / 1920. The backend is mocked in the page. Search needs a production build |
| `scroll_steps.py <dir> [path]` | Viewport shots while scrolling (scroll reveals need real scrolling); reports horizontal overflow |
| `modes.py <dir>` | Light theme, mobile menu, reduced motion, and a keyboard pass (tab order, chat focus + Esc) |
| `make_poster.py` | Renders the hero humanoid once and writes `frontend/static/img/hero-humanoid.webp`. Re-run after changing `humanoidScene.js` |
| `compare.py` | Side-by-side `before/` vs `after/` images into `compare/` (every shot present in both) |
| `sheet.py <dir> <out.png> names…` | Contact sheet of several shots, for quick review |
| `ui-inventory.md` | Every UI component, its states, and what the 2026-10-28 pass did to it |

Screenshot folders (`before/`, `after/`, `compare/`) are local output and not committed.
