"""Phone checks: navbar right side (no overlapping controls), the menu's GitHub row, and the live hero on a phone.

Usage: python design/mobile_chrome.py <out-dir> [base-url]
"""
import sys
from pathlib import Path

from playwright.sync_api import sync_playwright

out = Path(sys.argv[1])
base = sys.argv[2].rstrip("/") if len(sys.argv) > 2 else "http://localhost:3123"
out.mkdir(parents=True, exist_ok=True)

BOXES = """() => [...document.querySelectorAll('.navbar__items--right > *')]
  .filter(e => getComputedStyle(e).display !== 'none' && e.getBoundingClientRect().width > 0)
  .map(e => { const r = e.getBoundingClientRect(); return [e.className.toString().slice(0, 40), Math.round(r.left), Math.round(r.right)]; })"""

with sync_playwright() as p:
    b = p.chromium.launch(args=["--use-gl=angle", "--enable-unsafe-swiftshader"])
    for scheme in ("dark", "light"):
        page = b.new_page(viewport={"width": 393, "height": 852}, device_scale_factor=3, color_scheme=scheme,
                          is_mobile=True, has_touch=True)
        errors = []
        page.on("pageerror", lambda e: errors.append(str(e)[:160]))
        page.goto(base + "/", wait_until="networkidle")
        page.wait_for_timeout(800)
        page.screenshot(path=str(out / f"header-{scheme}.png"), clip={"x": 0, "y": 0, "width": 393, "height": 90})
        boxes = page.evaluate(BOXES)
        overlap = any(a[2] > c[1] + 1 for a, c in zip(boxes, boxes[1:]))
        print(scheme, "right-side controls:", boxes, "overlap:", overlap)
        if scheme == "dark":
            stage = page.locator("[class*='stage_']").first
            stage.scroll_into_view_if_needed()
            page.wait_for_timeout(1200)
            page.screenshot(path=str(out / "hero-phone-building.png"))
            page.wait_for_timeout(3500)
            page.screenshot(path=str(out / "hero-phone-built.png"))
            live = page.evaluate("!!document.querySelector('[class*=canvas_] canvas')")
            print("live hero canvas on phone:", live)
            page.evaluate("window.scrollTo(0, 0)")
        page.locator(".navbar__toggle").click()
        page.wait_for_timeout(600)
        page.screenshot(path=str(out / f"menu-{scheme}.png"))
        gh = page.locator(".navbar-sidebar .pa-github-link").first
        if gh.count():
            gh.scroll_into_view_if_needed()
            page.wait_for_timeout(300)
            page.screenshot(path=str(out / f"menu-github-{scheme}.png"))
            print("menu GitHub row:", gh.inner_text() or "(label from CSS)", gh.bounding_box())
        print("page errors:", errors[:5])
        page.close()
    b.close()
