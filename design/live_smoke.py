"""Smoke test of a deployed site without signing in: landing, a lesson, Rivet's wall, the theme switch, console errors.

Usage: python design/live_smoke.py <out-dir> <base-url>
"""
import sys
from pathlib import Path

from playwright.sync_api import sync_playwright

out = Path(sys.argv[1])
base = sys.argv[2].rstrip("/")
out.mkdir(parents=True, exist_ok=True)

with sync_playwright() as p:
    b = p.chromium.launch()
    for name, vp in (("desktop", {"width": 1440, "height": 900}), ("phone", {"width": 375, "height": 812})):
        page = b.new_page(viewport=vp, color_scheme="dark")
        errors, failed = [], []
        page.on("pageerror", lambda e: errors.append(str(e)[:160]))
        page.on("console", lambda m: m.type == "error" and errors.append(m.text[:160]))
        page.on("response", lambda r: r.status >= 400 and failed.append(f"{r.status} {r.url[:90]}"))
        page.goto(base + "/", wait_until="networkidle")
        page.wait_for_timeout(3500)
        page.screenshot(path=str(out / f"{name}-landing.png"))
        page.goto(base + "/docs/ros2/nodes-topics-services", wait_until="networkidle")
        page.wait_for_timeout(800)
        page.screenshot(path=str(out / f"{name}-lesson.png"))
        page.locator("button[aria-label^='Ask Rivet']").click()
        page.wait_for_timeout(700)
        wall = page.get_by_text("Sign in to start.").count() == 1
        page.screenshot(path=str(out / f"{name}-rivet-wall.png"))
        page.keyboard.press("Escape")
        page.wait_for_timeout(400)
        toggle = page.locator("button.pa-theme-toggle:visible").first
        if toggle.count():
            before = page.evaluate("document.documentElement.dataset.theme")
            toggle.click()
            page.wait_for_timeout(900)
            after = page.evaluate("document.documentElement.dataset.theme")
        else:
            before = after = "toggle in the mobile menu"
        print(f"{name}: wall={wall} theme {before}->{after}")
        print(f"  errors: {errors[:6]}")
        print(f"  failed requests: {failed[:6]}")
        page.close()
    b.close()
