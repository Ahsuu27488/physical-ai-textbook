"""Screenshot every main surface at 375 / 768 / 1440 px.

Usage: python design/shoot.py <out-dir> [base-url]
Needs: pip install playwright && playwright install chromium; the site running (npm start).
"""
import sys
from pathlib import Path

from playwright.sync_api import sync_playwright

out = Path(sys.argv[1])
base = sys.argv[2] if len(sys.argv) > 2 else "http://localhost:3123"
out.mkdir(parents=True, exist_ok=True)

WIDTHS = {"375": (375, 812), "768": (768, 1024), "1440": (1440, 900)}
PAGES = {"home": "/", "lesson": "/docs/module-1/lesson-1-ros2-architecture", "404": "/does-not-exist"}

with sync_playwright() as p:
    browser = p.chromium.launch()
    for wname, (w, h) in WIDTHS.items():
        ctx = browser.new_context(viewport={"width": w, "height": h}, color_scheme="dark", device_scale_factor=1)
        page = ctx.new_page()
        errors = []
        page.on("pageerror", lambda e: errors.append(str(e)))
        for pname, path in PAGES.items():
            page.goto(base + path, wait_until="networkidle")
            page.wait_for_timeout(4000 if pname == "home" else 1200)
            page.screenshot(path=str(out / f"{pname}-{wname}.png"))
            if pname == "home":
                page.screenshot(path=str(out / f"{pname}-{wname}-full.png"), full_page=True)
        # chat widget open
        page.goto(base + "/docs/module-1/intro", wait_until="networkidle")
        launcher = page.get_by_role("button", name="Open chat")
        if launcher.count():
            launcher.first.click(force=True, timeout=5000)
            page.wait_for_timeout(600)
            page.screenshot(path=str(out / f"chat-{wname}.png"))
            page.keyboard.press("Escape")
        # sign-in modal
        page.goto(base + "/", wait_until="networkidle")
        signin = page.get_by_role("button", name="Sign in", exact=False)
        if signin.count() and signin.first.is_visible():
            signin.first.click(force=True, timeout=5000)
            page.wait_for_timeout(500)
            page.screenshot(path=str(out / f"auth-{wname}.png"))
        # page tools menu (desktop navbar only)
        page.keyboard.press("Escape")
        page.goto(base + "/docs/module-1/intro", wait_until="networkidle")
        tools = page.get_by_role("button", name="Tools")
        if tools.count() and tools.first.is_visible():
            tools.first.click(timeout=5000)
            page.wait_for_timeout(400)
            page.screenshot(path=str(out / f"tools-{wname}.png"))
        if errors:
            print(wname, "page errors:", errors[:3])
        ctx.close()
    browser.close()
print("saved to", out)
