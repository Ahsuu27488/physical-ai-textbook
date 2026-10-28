"""Main flows in Firefox and WebKit (Safari's engine) against a production build, API mocked.

Usage: python design/engines_smoke.py <out-dir> [base-url]
Serve a build first: cd frontend && npm run build && npx docusaurus serve --port 3124
Needs: python -m playwright install firefox webkit
"""
import json
import sys

from playwright.sync_api import sync_playwright

out = sys.argv[1]
BASE = sys.argv[2] if len(sys.argv) > 2 else "http://localhost:3124"
LESSON = "/docs/start-here/how-this-course-works"
USER = {"id": "u1", "email": "t@example.com", "name": "Test", "software_background": "beginner", "hardware_background": "beginner"}
SSE = (
    'event: meta\ndata: {"conversation_id": "c1", "sources": [{"n": 1, "title": "PID control", "section": "Tuning", "url": "/docs/motion/pid"}]}\n\n'
    'event: token\ndata: {"text": "A **PID** controller "}\n\n'
    'event: token\ndata: {"text": "adds three terms [1]. $K_p e(t)$"}\n\n'
    "event: done\ndata: {}\n\n"
)


def mock(page, signed_in):
    j = lambda body: (lambda r: r.fulfill(status=200, content_type="application/json", body=body))
    page.route("**/health", j('{"status":"healthy","email":true}'))
    page.route("**/api/v1/progress**", j('{"completed":[],"quizzes":{}}'))
    page.route("**/api/v1/chats**", j('{"id":"c1","title":"t","updated_at":null,"count":2}'))
    page.route("**/api/v1/translation/translate", j(json.dumps({"translated_content": "# 你好\n\n机器人**感知**世界。", "target_language": "zh"})))
    page.route("**/api/chat/stream", lambda r: r.fulfill(status=200, content_type="text/event-stream", body=SSE))
    if signed_in:
        page.add_init_script("localStorage.setItem('auth_token','fake'); localStorage.setItem('kinemata:translate-lang','zh')")
        page.route("**/api/v1/auth/me", j(json.dumps(USER)))


results = []


def check(engine, size, name, fn):
    try:
        ok = fn()
        results.append((engine, size, name, "ok" if ok in (True, None) else f"FAIL {ok}"))
    except Exception as exc:  # keep going; report every check
        results.append((engine, size, name, f"FAIL {type(exc).__name__}: {str(exc).splitlines()[0][:120]}"))


with sync_playwright() as p:
    for engine in ("firefox", "webkit"):
        browser = getattr(p, engine).launch()
        for size, (w, h) in (("desktop", (1440, 900)), ("phone", (390, 844))):
            opts = {"viewport": {"width": w, "height": h}}
            if engine == "webkit" and size == "phone":
                opts.update(is_mobile=True, has_touch=True, device_scale_factor=2)
            errors = []

            # signed out: landing, lesson, theme, sign-in dialog, reset page
            page = browser.new_page(**opts)
            page.on("pageerror", lambda e: errors.append(str(e)[:160]))
            mock(page, signed_in=False)
            page.goto(BASE + "/", wait_until="load")
            page.wait_for_timeout(3500)
            check(engine, size, "landing hero (canvas or poster)", lambda: page.locator("canvas, img[src*=hero]").count() > 0 or "no hero")
            page.screenshot(path=f"{out}/eng-{engine}-{size}-landing.png")
            page.goto(BASE + LESSON, wait_until="load")
            page.wait_for_timeout(1500)
            check(engine, size, "lesson renders", lambda: page.locator("article h1").count() == 1 or "no h1")
            check(engine, size, "no horizontal scroll", lambda: page.evaluate("document.documentElement.scrollWidth <= innerWidth + 1") or "overflows")
            before = page.evaluate("document.documentElement.dataset.theme")
            def theme():
                page.locator("button[aria-label='Dark mode']:visible").first.click()
                page.wait_for_timeout(900)
                after = page.evaluate("document.documentElement.dataset.theme")
                return after != before or f"{before}->{after}"
            check(engine, size, "theme switch", theme)
            def signin():
                page.evaluate("window.dispatchEvent(new CustomEvent('pa:open-sign-in', {detail: {mode: 'login'}}))")
                page.wait_for_timeout(700)
                page.fill("#pa-email", "a@example.com")
                n = page.locator(".pa-dialog").count()
                page.keyboard.press("Escape")
                page.wait_for_timeout(300)
                return (n == 1 and page.locator(".pa-dialog").count() == 0) or f"dialogs={n}"
            check(engine, size, "sign-in dialog (one, Esc closes)", signin)
            page.goto(BASE + "/reset-password?token=abcdefghijklmnopqrstuvwxyz", wait_until="load")
            page.wait_for_timeout(1200)
            check(engine, size, "reset-password page", lambda: page.locator("#pa-new-password").count() == 1 or "no field")
            page.close()

            # signed in: Rivet streaming answer, thumbs, in-place translation
            page = browser.new_page(**opts)
            page.on("pageerror", lambda e: errors.append(str(e)[:160]))
            mock(page, signed_in=True)
            page.goto(BASE + LESSON, wait_until="load")
            page.wait_for_timeout(3000)
            def rivet():
                page.evaluate("window.dispatchEvent(new CustomEvent('pa:open-assistant', {detail: {prompt: 'What is PID?'}}))")
                page.wait_for_timeout(3000)
                text = page.locator("[role=log]").inner_text()
                return ("adds three terms" in text and page.locator(".katex").count() > 0) or f"log={text[:80]!r}"
            check(engine, size, "Rivet streams + Markdown + math", rivet)
            check(engine, size, "Rivet thumbs shown", lambda: page.get_by_role("button", name="Helpful", exact=True).count() == 1 or "no thumbs")
            page.screenshot(path=f"{out}/eng-{engine}-{size}-rivet.png")
            page.keyboard.press("Escape")
            page.wait_for_timeout(500)
            def translate():
                if size == "phone":
                    page.locator("button.navbar__toggle").click()
                    page.wait_for_timeout(600)
                    back = page.get_by_role("button", name="Back to main menu")
                    if back.count():
                        back.click()
                        page.wait_for_timeout(400)
                else:
                    page.get_by_role("button", name="Tools").click()
                page.locator("button:visible", has_text="Translate to Chinese").click()
                page.wait_for_timeout(1500)
                shown = page.locator("[data-inplace]").count() == 1
                hidden = not page.locator("article .markdown:not([data-inplace])").is_visible()
                return (shown and hidden) or f"shown={shown} hidden={hidden}"
            check(engine, size, "translate in place", translate)
            page.screenshot(path=f"{out}/eng-{engine}-{size}-translate.png")
            page.close()
            results.append((engine, size, "page errors", "ok" if not errors else f"FAIL {errors[:2]}"))
        browser.close()

for r in results:
    print(" | ".join(r))
