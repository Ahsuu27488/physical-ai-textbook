"""Viewport screenshots while stepping down a page (reveals and scroll timelines need real scrolling).

Usage: python design/scroll_steps.py <out-dir> [path] [base-url]
"""
import sys
from pathlib import Path

from playwright.sync_api import sync_playwright

out = Path(sys.argv[1])
path = sys.argv[2] if len(sys.argv) > 2 else "/"
base = sys.argv[3] if len(sys.argv) > 3 else "http://localhost:3123"
out.mkdir(parents=True, exist_ok=True)

with sync_playwright() as p:
    browser = p.chromium.launch()
    for w, h in [(1440, 900), (375, 812)]:
        page = browser.new_page(viewport={"width": w, "height": h}, color_scheme="dark")
        page.goto(base + path, wait_until="networkidle")
        page.wait_for_timeout(1500)
        total = page.evaluate("document.documentElement.scrollHeight - innerHeight")
        y, i = 0, 0
        while y <= total:
            page.evaluate(f"window.scrollTo(0, {y})")
            page.wait_for_timeout(700)
            page.screenshot(path=str(out / f"{w}-{i:02d}.png"))
            y += int(h * 0.85)
            i += 1
        overflow = page.evaluate("document.documentElement.scrollWidth > innerWidth")
        print(w, "steps:", i, "horizontal overflow:", overflow)
        page.close()
    browser.close()
