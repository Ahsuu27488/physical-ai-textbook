"""Light theme, mobile menu, reduced motion and a keyboard pass.

Usage: python design/modes.py <out-dir> [base-url]
"""
import sys
from pathlib import Path

from playwright.sync_api import sync_playwright

out = Path(sys.argv[1])
base = sys.argv[2] if len(sys.argv) > 2 else "http://localhost:3123"
out.mkdir(parents=True, exist_ok=True)

with sync_playwright() as p:
    b = p.chromium.launch()

    # light theme: landing sections + a lesson
    ctx = b.new_context(viewport={"width": 1440, "height": 900}, color_scheme="light")
    page = ctx.new_page()
    page.goto(base + "/", wait_until="networkidle")
    page.evaluate("localStorage.setItem('theme','light')")
    page.reload(wait_until="networkidle")
    page.wait_for_timeout(1500)
    page.screenshot(path=str(out / "light-home.png"))
    page.evaluate("document.querySelector('#modules-title').scrollIntoView()")
    page.wait_for_timeout(900)
    page.screenshot(path=str(out / "light-modules.png"))
    page.goto(base + "/docs/module-1/lesson-2-nodes-topics-services", wait_until="networkidle")
    page.evaluate("window.scrollTo(0, 900)")
    page.wait_for_timeout(700)
    page.screenshot(path=str(out / "light-lesson.png"))
    ctx.close()

    # mobile menu (navbar items move into the sidebar)
    ctx = b.new_context(viewport={"width": 375, "height": 812}, color_scheme="dark")
    page = ctx.new_page()
    page.goto(base + "/docs/module-1/intro", wait_until="networkidle")
    page.get_by_role("button", name="Toggle navigation bar").click()
    page.wait_for_timeout(500)
    page.screenshot(path=str(out / "mobile-menu.png"))
    page.get_by_text("Back to main menu").first.click() if page.get_by_text("Back to main menu").count() else None
    page.wait_for_timeout(400)
    page.screenshot(path=str(out / "mobile-menu-main.png"))
    ctx.close()

    # reduced motion: hero must render a still frame, reveals show their end state
    ctx = b.new_context(viewport={"width": 1440, "height": 900}, color_scheme="dark", reduced_motion="reduce")
    page = ctx.new_page()
    page.goto(base + "/", wait_until="networkidle")
    page.wait_for_timeout(1500)
    page.screenshot(path=str(out / "reduced-hero.png"))
    page.evaluate("document.querySelector('#schedule-title').scrollIntoView()")
    page.wait_for_timeout(500)
    page.screenshot(path=str(out / "reduced-schedule.png"))

    # keyboard: tab through the first stops on the landing page and record what gets focus
    page.evaluate("window.scrollTo(0,0)")
    page.keyboard.press("Tab")
    stops = []
    for _ in range(12):
        stops.append(page.evaluate(
            "(() => { const e = document.activeElement; return (e.getAttribute('aria-label') || e.textContent || e.tagName).trim().slice(0, 40); })()"))
        page.keyboard.press("Tab")
    print("tab order:", " | ".join(stops))
    # chat by keyboard: open with Enter on the launcher, Esc closes and focus returns
    page.get_by_role("button", name="Open chat").focus()
    page.keyboard.press("Enter")
    page.wait_for_timeout(400)
    focused = page.evaluate("document.activeElement.tagName + '#' + document.activeElement.id")
    page.keyboard.press("Escape")
    page.wait_for_timeout(300)
    back = page.evaluate("document.activeElement.getAttribute('aria-label')")
    print("chat open focus:", focused, "| after Esc focus:", back)
    ctx.close()
    b.close()
print("saved to", out)
