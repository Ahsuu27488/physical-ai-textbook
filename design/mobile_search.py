"""Phone search: the round button, the field expanded over the navbar, results, and opening a result.

Usage: python design/mobile_search.py <out-dir> [base-url]   (search needs a built site: npm run serve)
"""
import sys
from pathlib import Path

from playwright.sync_api import sync_playwright

out = Path(sys.argv[1])
base = sys.argv[2].rstrip("/") if len(sys.argv) > 2 else "http://localhost:3124"
out.mkdir(parents=True, exist_ok=True)

with sync_playwright() as p:
    b = p.chromium.launch()
    for scheme in ("dark", "light"):
        page = b.new_page(viewport={"width": 393, "height": 852}, device_scale_factor=3, color_scheme=scheme,
                          is_mobile=True, has_touch=True)
        page.goto(base + "/docs/ros2/nodes-topics-services", wait_until="networkidle")
        clip = {"x": 0, "y": 0, "width": 393, "height": 90}
        page.screenshot(path=str(out / f"search-closed-{scheme}.png"), clip=clip)
        page.locator(".navbar__search-input").tap()
        page.wait_for_timeout(400)
        page.screenshot(path=str(out / f"search-open-{scheme}.png"), clip=clip)
        page.keyboard.type("kalman filter", delay=25)
        page.wait_for_timeout(1500)
        page.screenshot(path=str(out / f"search-results-{scheme}.png"))
        if scheme == "dark":
            first = page.locator("[class*='dropdownMenu_'] [class*='suggestion_']").first
            print("results:", page.locator("[class*='dropdownMenu_'] [class*='suggestion_']").count())
            first.tap()
            page.wait_for_timeout(1500)
            print("opened:", page.url)
        page.close()
    b.close()
