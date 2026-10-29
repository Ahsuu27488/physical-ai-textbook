# Project Status

> Current truth only: what works now, what doesn't, what's next. History goes in CHANGELOG.md, reasons in
> DECISIONS.md. Updated in the same commit as every code change (the commit-msg hook checks). Keep it short:
> it is loaded into every agent session.

**Last updated:** 2026-10-30 · **Code at:** fix/popover-scroll-mobile-keyboard
**In one line:** Kinemata live; popover scroll leaking isolated; mobile phone virtual keyboard gap fixed.

## What works (verified)

- Popover & Dropdown Scroll Containment (2026-10-30, Chromium dev server): Added data-lenis-prevent and wheel stopPropagation to PageTools menu, language selector (.langs), and AccountMenu; overscroll-behavior: contain added to popovers, search results, and dropdowns. Lenis smooth scroll delta remains 0 during popover scrolling.
- Mobile Virtual Keyboard Chat Fix (2026-10-30, Mobile viewport emulation): Rivet chat widget locks both body and documentElement overflow and pauses Lenis on mobile; anchored panel layout and dedicated backdrop overlay prevent page contents from peeking through during virtual keyboard appearance.
- Modal & Chat Scroll Isolation (2026-10-30, Chromium dev server): Lenis smooth scrolling paused and data-lenis-prevent added to all dialog overlays; wheel events inside Rivet chat panel contained; no background page scrolling when scrolling inside modals.
- Rivet UX & Background Streaming (2026-10-30): Closing the chat window or clicking outside minimizes the panel without aborting active response streams.
- Frontend: `npm run build` passes with full course in English and Urdu locales (2026-10-30).

- Frontend: `npm run build` passes with the full course, no broken links (2026-10-28); `check-mdx.mjs docs` → 64/64 ok (both re-run after the UI pass).
- Course: 11 modules, 52 lessons + overviews; Python examples in modules 00–09 were run by the writers (see
  CHANGELOG); ROS 2 / Gazebo / Isaac snippets were not run (no ROS install).
- Backend: `python -m pytest -q` → 102 passed (2026-10-28, no network/secrets); ingest dry-run → 64 files, 611 chunks.
- Rivet sign-in wall (2026-10-28, dev site + local backend, Chromium): signed out shows the wall, no composer; sign up →
  sign in → the waiting question is sent with the token and reaches the API (503 there only because no key locally).
- Theme switch (2026-10-28, Chromium): flips light ↔ dark with the circular reveal, choice kept after reload.
- Seen in the dev site (2026-10-28): lesson components, Rivet reading along on a lesson, profile page, search box.
- UI pass over every component (2026-10-28): before/after shots of 140+ states at 320–1920 px, dark + light, from a
  production build with the backend mocked; no horizontal scroll at 320/375/768 on any of the 64 docs pages; keyboard
  pass (Rivet, dialogs, Tools, account menu) and reduced motion checked in Chromium; console clean.

## In progress

- feat/urdu-edition: 25 Urdu lessons + UI strings; the other 39 lessons still to translate with flash models
  (`cd backend && .venv/Scripts/python -u scripts/translate_course.py --locale ur --workers 1`, resumable; check
  `node scripts/check-translation.mjs i18n/ur/docusaurus-plugin-content-docs/current` before committing).
- Urdu lessons: `backend/scripts/translate_course.py --locale ur` running (streams, 3 at a time, moves to the next
  model when one's daily quota is spent; owner's
  GEMINI_API_KEY is set in the system env for one-off jobs); output in `frontend/i18n/ur`, to commit when done.

## Live state

- Merged to main 2026-10-28 (PR #1 62bbbf1, PR #2 e5337ed: Kinemata; PR #7 4564795: waking-up state, Pages redirects; PR #8 1446bda: model chain; PR #9 4f154d9: saved chats, votes; PR #10 fb98e98: translations in place; PR #11 0cabf92: password reset; PR #12 d09e92d: dialogs above the phone keyboard; PR #13 c00f898: Firefox/WebKit checks); `design/live_smoke.py` passed on the Vercel site (landing, lesson,
  Rivet wall, theme switch, desktop + phone, no console errors or failed requests).
- Frontend: https://kinemata.ahsandev.site (Vercel, custom domain, DNS CNAME on Cloudflare); also https://physical-ai-textbook-puce-two.vercel.app; GitHub Pages redirects to it
- Backend: HF Space `Ahsuu27488/physical-ai-textbook-backend`: `/health` → postgres reachable (2026-10-28). Secrets:
  ADDITIONAL_CORS_ORIGIN, DATABASE_URL, FRONTEND_URL, JWT_SECRET_KEY, GEMINI_API_KEY, QDRANT_API_KEY, QDRANT_URL
  (owner set the Gemini key and removed OPENAI_API_KEY; QDRANT_URL points at the new cluster).
- Qdrant: free cluster `proprio-course` (AWS us-east-1), empty until `run_etl.py` runs.

## Known gaps

1. Free Gemini quota is small and per Google project: `gemini-3.8-flash` 20 requests/day (others unknown, see
   aistudio.google.com/rate-limit). If the PC's key and the Space's key are in the same project, one-off jobs (the
   Urdu translation) spend Rivet's daily quota. Rivet not yet tried live with a real account (owner's to-do 1).
2. `npm audit`: advisories in build/dev tooling under Docusaurus; nothing ships to the browser.
3. The HF Space proxy answers CORS for any origin (seen 2026-10-28; our allow-list is overridden). Low risk: sign-in
   is a Bearer token in the page, not a cookie.
3b. ROS 2 / Gazebo / Isaac / OpenCV / Open3D lesson code is unexecuted; needs a run on Ubuntu 24.04 + Jazzy.
4. Rivet's limits are per account but in memory, per process (reset on restart). Votes are only in the database
   (no page to read them yet: query `answer_feedback`).
5. Password reset email is off until the owner sets `MAIL_API_KEY` + `MAIL_FROM` (Brevo) on the Space; a reset does
   not sign out other devices (tokens last 7 days); no account email verification.
6. Phone Lighthouse (live, 2026-10-28, slow 4G + 4× CPU): home performance 75 (LCP 3.5 s, blocking 470 ms, Speed
   Index 4.7 s: the 3D hero now loads on phones, D-023; was ~89 / 220 ms before), lesson 86 (LCP 3.0 s, blocking
   350 ms); accessibility, best practices, SEO 100; layout shift ~0. Battery/heat on a real mid-range phone not checked.
7. Not checked on a real phone or real Safari. Playwright Firefox + WebKit (Safari's engine) pass the main flows at
   desktop and phone size (`design/engines_smoke.py`, 40/40); WebKit on Windows draws Geist's variable weights lighter
   (likely that build only). Firefox has no CSS scroll timelines, so the navbar lift and reading bar don't animate.

## Next (owner's to-do list, 2026-10-28; item "run ROS 2 code on Ubuntu" deferred)

1. Owner: test Rivet live with a real account (answers, citations, a follow-up).
2. Translate the remaining 39 Urdu lessons (flash models only; the free quota is shared with the owner's other
   projects, so it runs slowly with retries).
5. Owner: Brevo account → verify a sender address → API key → Space secrets `MAIL_API_KEY`, `MAIL_FROM`.
9. Owner: on a mid-range phone, check the landing page doesn't get hot or drain the battery (3D hero); if it
   does, phones can go back to the poster (D-023) or load the scene on first scroll instead.
10. Owner: open the site once in Safari on an iPhone (sign-in sheet with the keyboard, Rivet, Tools).
12. Rate limits are in memory (reset on restart): move to the database if traffic grows.

## How to run and check

- Run: `cd frontend && npm start` (site :3000), `cd backend && uvicorn app.main:app --reload` (API :8000)
- Check: `(cd frontend && npm run build) && (cd backend && python -m pytest -q)` (both pass 2026-10-28)
