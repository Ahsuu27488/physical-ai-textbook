"""Screenshot every UI surface and state at 375 / 768 / 1440 px (dark), plus light-theme and 320 / 1920 spot checks.

Usage: python design/shoot.py <out-dir> [base-url] [--only name,name] [--widths 375,1440]
Needs: pip install playwright && playwright install chromium, and the site running. Use a production build
(`npm run build && npm run serve -- --port 3141`) when you need the search box: local search is build-only.

The backend is mocked in the page (fetch is patched before the app loads), so chat streaming, citations,
errors, sign-in and translation render the same every run and nothing reaches the real API.
"""
import json
import re
import traceback
import sys
from pathlib import Path

from playwright.sync_api import sync_playwright

args = [a for a in sys.argv[1:] if not a.startswith("--")]
flags = {a.split("=")[0]: a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--") and "=" in a}
out = Path(args[0])
base = args[1] if len(args) > 1 else "http://localhost:3140"
only = set(flags["--only"].split(",")) if "--only" in flags else None
out.mkdir(parents=True, exist_ok=True)

ALL_WIDTHS = {"375": (375, 812), "768": (768, 1024), "1440": (1440, 900)}
WIDTHS = {k: v for k, v in ALL_WIDTHS.items() if "--widths" not in flags or k in flags["--widths"].split(",")}
LESSON = "/docs/robot-fundamentals/anatomy-of-a-robot"
LESSON2 = "/docs/ros2/nodes-topics-services"

ANSWER = r"""A **topic** is a one-way stream: a publisher sends messages and any number of subscribers listen [1]. A **service** is a single request with a single reply, so use it for commands like *reset the arm* [2].

| Pattern | Direction | Use it for |
|---|---|---|
| Topic | many → many | sensor streams, joint states |
| Service | one → one | quick commands with a reply |
| Action | one → one | long goals with feedback |

```python
from example_interfaces.srv import AddTwoInts
client = node.create_client(AddTwoInts, 'add_two_ints')
future = client.call_async(AddTwoInts.Request(a=2, b=3))
```

The latency budget for a control loop at rate $f$ is $\Delta t = 1/f$, so at 100 Hz:

$$\Delta t = \frac{1}{100\,\text{Hz}} = 10\,\text{ms}$$

> Rule of thumb: if you would poll it, make it a topic.

1. Start with topics.
2. Add a service when you need a reply [1]."""

MOCK = r"""
(() => {
  const ANSWER = __ANSWER__;
  const SOURCES = [
    {n: 1, title: 'Nodes, topics and services with rclpy', section: 'Topics', url: '/docs/ros2/nodes-topics-services#topics', score: 0.82},
    {n: 2, title: 'Nodes, topics and services with rclpy', section: 'Services', url: '/docs/ros2/nodes-topics-services#services', score: 0.77},
    {n: 3, title: 'ROS 2 architecture and DDS', section: 'The graph', url: '/docs/ros2/ros2-architecture-and-dds', score: 0.61},
  ];
  window.__mock = window.__mock || {chat: 'ok', auth: 'ok', signedIn: false};
  const sse = (e, d) => `event: ${e}\ndata: ${JSON.stringify(d)}\n\n`;
  const json = (body, status = 200) => new Response(JSON.stringify(body), {status, headers: {'Content-Type': 'application/json'}});
  const USER = {id: 1, email: 'learner@example.com', name: 'Sam Learner', software_background: 'intermediate', hardware_background: 'beginner', created_at: '2026-09-02T10:00:00Z'};
  const real = window.fetch.bind(window);
  window.fetch = async (input, init = {}) => {
    const url = typeof input === 'string' ? input : input.url;
    if (!/\/api\//.test(url) || url.startsWith(location.origin)) return real(input, init);
    const m = window.__mock;
    if (url.includes('/chat/stream')) {
      if (m.chat === 'error') return json({detail: 'Service unavailable'}, 503);
      if (m.chat === 'hang') return new Promise((_, reject) => init.signal?.addEventListener('abort', () => reject(new DOMException('Aborted', 'AbortError'))));
      const pieces = ANSWER.match(/[\s\S]{1,6}/g);
      const slow = m.chat === 'slow';
      const body = new ReadableStream({
        async start(c) {
          const enc = new TextEncoder();
          c.enqueue(enc.encode(sse('meta', {conversation_id: 'demo', sources: SOURCES})));
          for (const p of slow ? pieces.slice(0, 70) : pieces) {
            if (init.signal?.aborted) break;
            c.enqueue(enc.encode(sse('token', {text: p})));
            await new Promise((r) => setTimeout(r, slow ? 40 : 2));
          }
          if (slow) await new Promise((r) => setTimeout(r, 60000));
          c.enqueue(enc.encode(sse('done', {})));
          c.close();
        },
      });
      return new Response(body, {status: 200, headers: {'Content-Type': 'text/event-stream'}});
    }
    if (url.includes('/auth/me')) return m.signedIn ? json(USER) : json({detail: 'Not authenticated'}, 401);
    if (url.includes('/auth/login') || url.includes('/auth/token')) {
      await new Promise((r) => setTimeout(r, 400));
      return m.auth === 'bad' ? json({detail: 'Incorrect email or password'}, 401) : json({access_token: 'demo', token_type: 'bearer'});
    }
    if (url.includes('/progress')) return json({completed: [], quizzes: {}});
    if (url.includes('/translation/')) return json({translated_content: '## روبوٹ کی ساخت\n\nایک روبوٹ کے تین بنیادی حصے ہوتے ہیں: **سینسر**، **کنٹرولر** اور **ایکچویٹر**۔ ROS 2 ان حصوں کو آپس میں جوڑتا ہے۔\n\n- سینسر دنیا کو ناپتے ہیں۔\n- کنٹرولر فیصلہ کرتا ہے۔\n- ایکچویٹر حرکت کرتے ہیں۔'});
    return json({detail: 'mocked'}, 404);
  };
})();
""".replace("__ANSWER__", json.dumps(ANSWER))


def seed(page, signed_in=False, theme="dark", progress=True):
    page.evaluate(
        """([signedIn, theme, progress]) => {
          localStorage.setItem('theme', theme);
          if (signedIn) localStorage.setItem('auth_token', 'demo'); else localStorage.removeItem('auth_token');
          if (progress) localStorage.setItem('proprio:progress', JSON.stringify({
            completed: ['start-here/how-this-course-works', 'start-here/workstation-setup', 'robot-fundamentals/anatomy-of-a-robot'],
            quizzes: {'/docs/start-here/how-this-course-works': {best: 3, total: 4}},
            last: {path: '/docs/robot-fundamentals/frames-and-rigid-transforms', title: 'Frames and rigid transforms'}}));
          sessionStorage.removeItem('proprio:rivet');
        }""",
        [signed_in, theme, progress],
    )


def want(name):
    return only is None or any(name.startswith(o) for o in only)


def shot(page, name, full=False):
    page.screenshot(path=str(out / f"{name}.png"), full_page=full)


def scroll_to(page, selector, offset=110):
    ok = page.evaluate(
        """([sel, off]) => { const el = document.querySelector(sel); if (!el) return false;
           window.scrollTo(0, el.getBoundingClientRect().top + scrollY - off); return true; }""",
        [selector, offset],
    )
    page.wait_for_timeout(450)
    return ok


def open_chat(page, mode="ok", ask=None):
    page.evaluate(f"window.__mock.chat = '{mode}'")
    page.get_by_role("button", name=re.compile(r"^(Open chat|Ask Rivet, the course tutor)$")).first.click(force=True)
    page.wait_for_timeout(450)
    if ask:
        box = page.get_by_label("Your question")
        box.fill(ask)
        box.press("Enter")


def run_width(browser, wname, size, theme="dark", light_subset=False):
    w, h = size
    suffix = f"{wname}" if theme == "dark" else f"{wname}-light"
    ctx = browser.new_context(viewport={"width": w, "height": h}, color_scheme=theme, device_scale_factor=1)
    ctx.add_init_script(MOCK)
    page = ctx.new_page()
    errors = []
    page.on("pageerror", lambda e: errors.append(str(e)))
    desktop = w >= 997  # Docusaurus collapses the navbar below 997 px

    def go(path, signed_in=False, progress=True, wait=900):
        page.goto(base + "/404-seed", wait_until="domcontentloaded")
        seed(page, signed_in, theme, progress)
        if signed_in:
            page.add_init_script("window.__mock = {chat: 'ok', auth: 'ok', signedIn: true}")
        page.goto(base + path, wait_until="networkidle")
        page.wait_for_timeout(wait)

    try:
        if want("home"):
            go("/", progress=False, wait=3500)
            shot(page, f"home-{suffix}")
            if not light_subset:
                shot(page, f"home-full-{suffix}", full=True)
            go("/", progress=True, wait=1500)
            shot(page, f"home-continue-{suffix}")
    except Exception as exc:  # keep going: one broken surface must not stop the run
        print(suffix, "home failed:", str(exc).splitlines()[0])

    try:
        if want("lesson"):
            go(LESSON)
            shot(page, f"lesson-top-{suffix}")
            for key, sel in [
                ("objectives", "section[aria-label='Learning objectives']"),
                ("admonition", ".theme-admonition"),
                ("table", "article .markdown table"),
                ("math", ".katex-display"),
                ("mermaid", ".docusaurus-mermaid-container"),
                ("code", "article .theme-code-block"),
                ("steps", "article [class*='steps']"),
                ("keyidea", "article aside[class*='keyIdea']"),
            ]:
                if light_subset and key not in ("table", "code", "admonition"):
                    continue
                if scroll_to(page, sel):
                    if key == "code" and desktop:
                        page.hover("article .theme-code-block")
                        page.wait_for_timeout(200)
                    shot(page, f"lesson-{key}-{suffix}")
            if scroll_to(page, "section[aria-label='Quiz']"):
                opts = page.locator("section[aria-label='Quiz'] fieldset").first.locator("label")
                if opts.count() > 1:
                    opts.nth(0).click()
                    second = page.locator("section[aria-label='Quiz'] fieldset").nth(1).locator("label")
                    if second.count() > 1:
                        second.nth(1).click()
                    page.wait_for_timeout(300)
                shot(page, f"lesson-quiz-{suffix}")
            if not light_subset and scroll_to(page, "article section[class*='exercise']"):
                sol = page.locator("article details summary").first
                if sol.count():
                    sol.click()
                    page.wait_for_timeout(300)
                shot(page, f"lesson-exercise-{suffix}")
            if scroll_to(page, "[class*='complete']", 200):
                shot(page, f"lesson-footer-{suffix}")
            if not light_subset and desktop:
                term = page.locator("article [class*='term']").first
                if term.count():
                    term.scroll_into_view_if_needed()
                    page.evaluate("window.scrollBy(0, -200)")
                    term.hover()
                    page.wait_for_timeout(350)
                    shot(page, f"lesson-term-{suffix}")
                # select text → Ask Rivet bubble
                page.evaluate(
                    """() => { const p = document.querySelectorAll('article .markdown > p')[1]; if (!p) return;
                       p.scrollIntoView({block: 'center'}); const r = document.createRange();
                       r.setStart(p.firstChild, 0); r.setEnd(p.firstChild, Math.min(60, p.firstChild.length));
                       const s = getSelection(); s.removeAllRanges(); s.addRange(r);
                       document.dispatchEvent(new MouseEvent('mouseup', {bubbles: true})); }"""
                )
                page.wait_for_timeout(400)
                shot(page, f"selection-ask-{suffix}")
                page.evaluate("getSelection().removeAllRanges()")
    except Exception as exc:  # keep going: one broken surface must not stop the run
        print(suffix, "lesson failed:", str(exc).splitlines()[0])

    try:
        if want("sidebar") and desktop and not light_subset:
            go(LESSON)
            shot(page, f"sidebar-{suffix}", full=False)
    except Exception as exc:  # keep going: one broken surface must not stop the run
        print(suffix, "sidebar failed:", str(exc).splitlines()[0])

    try:
        if want("chat"):
            go(LESSON2)
            open_chat(page)
            shot(page, f"chat-empty-{suffix}")
            page.keyboard.press("Escape")
            page.wait_for_timeout(300)
            open_chat(page, "hang", "When should I use a service instead of a topic?")
            page.wait_for_timeout(500)
            shot(page, f"chat-thinking-{suffix}")
            page.get_by_role("button", name="Stop answering").first.click()
            page.wait_for_timeout(300)
            if not light_subset:
                page.evaluate("window.__mock.chat = 'slow'")
                box = page.get_by_label("Your question")
                box.fill("Compare topics, services and actions")
                box.press("Enter")
                page.wait_for_timeout(1900)
                shot(page, f"chat-streaming-{suffix}")
                page.get_by_role("button", name="Stop answering").first.click()
                page.wait_for_timeout(300)
            go(LESSON2)
            open_chat(page, "ok", "When should I use a service instead of a topic?")
            page.wait_for_timeout(2500)
            shot(page, f"chat-answer-{suffix}")
            log = page.locator("[role='log']").first
            log.evaluate("el => el.scrollTo(0, el.scrollHeight)")
            page.wait_for_timeout(300)
            shot(page, f"chat-answer-end-{suffix}")
            if not light_subset:
                log.evaluate("el => el.scrollTo(0, 120)")
                page.wait_for_timeout(400)
                shot(page, f"chat-scrolled-{suffix}")
                page.evaluate("window.__mock.chat = 'error'")
                box = page.get_by_label("Your question")
                box.fill("And what about actions?")
                box.press("Enter")
                page.wait_for_timeout(900)
                shot(page, f"chat-error-{suffix}")
                if wname == "1440":
                    wide = page.get_by_role("button", name="Make chat larger")
                    if wide.count():
                        wide.first.click()
                        page.wait_for_timeout(500)
                        shot(page, f"chat-wide-{suffix}")
                # composer with a long multi-line draft
                box.fill("Line one of a long question about TF2 frames\nline two\nline three\nline four\nline five")
                page.wait_for_timeout(200)
                shot(page, f"chat-composer-{suffix}")
            page.keyboard.press("Escape")
            page.wait_for_timeout(300)
            shot(page, f"launcher-{suffix}")
    except Exception as exc:  # keep going: one broken surface must not stop the run
        print(suffix, "chat failed:", str(exc).splitlines()[0])

    try:
        if want("auth"):
            go("/docs/")
            if desktop:
                btn = page.locator("button:visible", has_text=re.compile(r"^Sign in$"))
            else:
                page.get_by_role("button", name="Toggle navigation bar").click()
                page.wait_for_timeout(400)
                if not light_subset:
                    shot(page, f"mobile-menu-{suffix}")
                btn = page.locator("button:visible", has_text=re.compile(r"^Sign in$"))
            btn.first.evaluate("b => b.click()")
            page.wait_for_timeout(500)
            shot(page, f"auth-signin-{suffix}")
            if not light_subset:
                page.evaluate("window.__mock.auth = 'bad'")
                page.get_by_label("Email").fill("learner@example.com")
                page.get_by_label("Password", exact=True).fill("wrong-password")
                page.get_by_role("button", name="Sign in", exact=True).last.click()
                page.wait_for_timeout(900)
                shot(page, f"auth-error-{suffix}")
                page.get_by_role("tab", name="Create account").click()
                page.wait_for_timeout(300)
                shot(page, f"auth-register-{suffix}")
    except Exception as exc:  # keep going: one broken surface must not stop the run
        print(suffix, "auth failed:", str(exc).splitlines()[0])

    try:
        if want("tools"):
            go(LESSON)
            if desktop:
                page.get_by_role("button", name="Tools").first.click()
                page.wait_for_timeout(400)
                shot(page, f"tools-menu-{suffix}")
                page.get_by_role("button", name=re.compile(r"^Translate (page|to )")).first.click()
            else:
                page.get_by_role("button", name="Toggle navigation bar").click()
                page.wait_for_timeout(400)
                page.evaluate("document.querySelector('.navbar-sidebar__items') && document.querySelector('.navbar-sidebar__item.menu') && document.querySelector('.navbar-sidebar__back')?.click()")
                page.wait_for_timeout(400)
                shot(page, f"tools-mobile-{suffix}")
                t = page.get_by_role("button", name=re.compile(r"^Translate (page|to )"))
                if t.count():
                    t.first.click(force=True)
            page.wait_for_timeout(1200)
            shot(page, f"tools-result-{suffix}")
    except Exception as exc:  # keep going: one broken surface must not stop the run
        print(suffix, "tools failed:", str(exc).splitlines()[0])

    try:
        if want("account"):
            go("/docs/", signed_in=True)
            if desktop:
                acct = page.locator("[class*='accountButton']").first
                if acct.count():
                    acct.click()
                    page.wait_for_timeout(300)
                    shot(page, f"account-menu-{suffix}")
    except Exception as exc:  # keep going: one broken surface must not stop the run
        print(suffix, "account failed:", str(exc).splitlines()[0])

    try:
        if want("profile"):
            go("/profile")
            shot(page, f"profile-{suffix}")
            if not light_subset:
                go("/profile", signed_in=True)
                shot(page, f"profile-signedin-{suffix}", full=True)
    except Exception as exc:  # keep going: one broken surface must not stop the run
        print(suffix, "profile failed:", str(exc).splitlines()[0])

    try:
        if want("404"):
            go("/does-not-exist")
            shot(page, f"404-{suffix}")
    except Exception as exc:  # keep going: one broken surface must not stop the run
        print(suffix, "404 failed:", str(exc).splitlines()[0])

    try:
        if want("footer") and not light_subset:
            go("/docs/")
            page.evaluate("window.scrollTo(0, document.body.scrollHeight)")
            page.wait_for_timeout(500)
            shot(page, f"footer-{suffix}")
    except Exception as exc:  # keep going: one broken surface must not stop the run
        print(suffix, "footer failed:", str(exc).splitlines()[0])

    try:
        if want("search"):
            go(LESSON)
            sb = page.locator(".navbar__search-input, input[type='search']").first
            if not desktop:
                toggle = page.locator("[class*='searchBarContainer'], .navbar__search").first
            if sb.count():
                sb.evaluate("e => e.focus()")
                page.keyboard.type("kalman", delay=30)
                page.wait_for_timeout(1500)
                shot(page, f"search-{suffix}")
    except Exception as exc:  # keep going: one broken surface must not stop the run
        print(suffix, "search failed:", str(exc).splitlines()[0])

    if errors:
        print(suffix, "page errors:", errors[:3])
    ctx.close()


with sync_playwright() as p:
    browser = p.chromium.launch()
    for wname, size in WIDTHS.items():
        run_width(browser, wname, size)
        print("done", wname)
    if "--no-light" not in sys.argv and (only is None or "light" in only or True):
        for wname in ("375", "1440"):
            if wname in WIDTHS:
                run_width(browser, wname, ALL_WIDTHS[wname], theme="light", light_subset=True)
        print("done light")
    if "--no-extra" not in sys.argv:
        only_backup = only
        for wname, size in (("320", (320, 640)), ("1920", (1920, 1080))):
            ctx = browser.new_context(viewport={"width": size[0], "height": size[1]}, color_scheme="dark")
            ctx.add_init_script(MOCK)
            page = ctx.new_page()
            for name, path in (("home", "/"), ("lesson-top", LESSON)):
                page.goto(base + path, wait_until="networkidle")
                page.wait_for_timeout(1500)
                shot(page, f"{name}-{wname}")
            page.goto(base + LESSON2, wait_until="networkidle")
            page.wait_for_timeout(600)
            open_chat(page, "ok", "When should I use a service instead of a topic?")
            page.wait_for_timeout(2500)
            shot(page, f"chat-answer-{wname}")
            overflow = page.evaluate("document.documentElement.scrollWidth - innerWidth")
            print(wname, "horizontal overflow px:", overflow)
            ctx.close()
    browser.close()
print("saved to", out)
