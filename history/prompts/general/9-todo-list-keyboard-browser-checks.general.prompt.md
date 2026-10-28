---
id: 9
title: To-do list, phone keyboard fix, browser checks
stage: general
date: 2026-10-28
surface: agent
model: claude-opus-5-5
feature: none
branch: chore/browser-checks
user: Muhammad Ahsan
command: (chat)
labels: ["chatbot", "gemini", "auth", "i18n", "mobile", "cross-browser", "performance"]
links:
  spec: null
  ticket: null
  adr: null
  pr: https://github.com/Ahsuu27488/physical-ai-textbook/pull/13
files:
 - .github/workflows/{deploy.yml,keepalive.yml}
 - frontend/scripts/make-redirects.mjs
 - frontend/src/components/ChatWidget/{index.js,styles.module.css}
 - frontend/src/theme/NavbarItem/{PageTools.js,PageTools.module.css,AuthButtons.js,AuthButtons.module.css}
 - frontend/src/pages/{reset-password.js,reset-password.module.css}
 - frontend/src/hooks/useDialog.js
 - frontend/src/css/custom.css
 - frontend/docusaurus.config.js
 - backend/app/{llm.py,config.py,mail.py,main.py,models.py}
 - backend/app/api/v1/{chats.py,password_reset.py,personalization.py,auth.py}
 - backend/app/database/models.py
 - backend/app/services/{translation.py,personalize_ai.py}
 - backend/scripts/translate_course.py
 - backend/tests/{test_chats.py,test_password_reset.py,test_chat_api.py,test_api.py,test_account_progress.py,conftest.py}
 - design/engines_smoke.py
 - docs/{STATUS.md,CHANGELOG.md,DECISIONS.md,ARCHITECTURE.md}, docs/plans/2026-10-28-password-reset.md
 - README.md
tests:
 - backend pytest (102 passed)
 - frontend npm run build --locale en
 - Playwright mocked-API checks (waking state, saved chats + votes, in-place translation, password reset, simulated phone keyboard)
 - design/engines_smoke.py in Firefox + WebKit (40/40)
 - Lighthouse mobile on the live site
---

## Prompt

(After /compact) Urdu translation job is running in the background (log: scratchpad/urdu-translate.log, output: frontend/i18n/ur). When it finishes: check the "Kept in English" list, build en+ur, commit on a branch, PR, merge. Then work through docs/STATUS.md "Next" items 2-12.

I hit my usage limit while you were working, but it has reset now. Please continue from where you left off.

Profile working on this menus like sign up tasks, etc. You have to take a look when anyone in Android or an iPhone clicks on sign in or anything like that. His keyboard pops up from bottom. Just these windows that you created, these dialogues, the sign in form, is stuck to bottom. And now you have to make a user kind of text that after opening the keyboard overlay, our website's input form works well.

I haven't tested it yet, but I'm just telling you that test it if it's really a problem or not.

Bro, I'm running some tests with my Gemini API audio in a different project. So, hitting the rate limit is fine for now. We can wait, or we should do something else.

That's okay, let them run and give them the most retries it can, because we are already on a free tier, so it's probably fine. Don't stop them, let them run, but with maximum retries.

## Response

Merged PRs #7–#13: Rivet "waking up" state + keep-alive cron; GitHub Pages redirects to Vercel (D-028); cleanup of dead
personalization endpoints, Railway files, README; model chain for the free tier's per-model daily quotas (found
gemini-3.8-flash = 20 requests/day; D-029); saved chats on the account + thumbs up/down (D-030); translation and
personalization shown in place with an Original toggle; password reset by email over Brevo's HTTPS API, off until the
owner adds MAIL_API_KEY/MAIL_FROM (D-031), plus a fix for two sign-in dialogs opening on phones; dialogs kept above the
phone keyboard (confirmed the bug with a simulated keyboard first: sheet bottom 844 px vs visible 508 px; after: 508);
Firefox/WebKit smoke test 40/40, buttons now inherit the site font, decorative logo alt, phone Lighthouse numbers
recorded. Urdu job: 21/64 lessons done; rerunning with 40 retries per call and model rotation while the owner's other
project shares the Gemini quota.

## Outcome

- Impact: owner's to-do items 3–11 done; Urdu edition still in progress.
- Tests: 102 backend tests; browser checks with the API mocked; no real-phone or real-Safari test yet.
- Files: see front matter.
- Next prompts: commit the Urdu edition when the job ends; owner sets up Brevo and tests on a phone.
- Reflection: probe free-tier quotas before bulk jobs; simulate the phone keyboard by faking a live visualViewport.

## Evaluation notes (flywheel)

- Failure modes observed: two translation jobs ran at once on one log; a stash/rebase chain stopped midway; a stale fake
  visualViewport gave a false failure.
- Graders run and results (PASS/FAIL): pytest PASS; engines smoke PASS; Lighthouse recorded.
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): run the phone-keyboard check on a real device.
