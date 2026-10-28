"""The theme switch: icon in both themes (zoomed) and frames of the circular reveal after a click.

Usage: python design/theme_toggle.py <out-dir> [base-url]
"""
import sys
from pathlib import Path

from playwright.sync_api import sync_playwright

out = Path(sys.argv[1])
base = sys.argv[2] if len(sys.argv) > 2 else "http://localhost:3123"
out.mkdir(parents=True, exist_ok=True)

with sync_playwright() as p:
    b = p.chromium.launch()
    page = b.new_page(viewport={"width": 1440, "height": 900}, color_scheme="light", device_scale_factor=2)
    errors = []
    page.on("pageerror", lambda e: errors.append(str(e)))
    page.goto(base + "/docs/ros2/nodes-topics-services", wait_until="networkidle")
    toggle = page.locator("button.pa-theme-toggle").first
    box = toggle.bounding_box()
    clip = {"x": box["x"] - 40, "y": box["y"] - 12, "width": box["width"] + 80, "height": box["height"] + 24}
    page.screenshot(path=str(out / "toggle-light.png"), clip=clip)
    print("before:", page.evaluate("document.documentElement.dataset.theme"), toggle.get_attribute("aria-checked"))
    toggle.click()
    for ms in (60, 180, 320):
        page.wait_for_timeout(ms if ms == 60 else 120)
        page.screenshot(path=str(out / f"reveal-{ms}ms.png"), scale="css")
    page.wait_for_timeout(700)
    page.screenshot(path=str(out / "toggle-dark.png"), clip=clip)
    page.screenshot(path=str(out / "after-dark.png"), scale="css")
    print("after:", page.evaluate("document.documentElement.dataset.theme"), toggle.get_attribute("aria-checked"),
          "stored:", page.evaluate("localStorage.getItem('theme')"))
    toggle.hover()
    page.wait_for_timeout(400)
    page.screenshot(path=str(out / "toggle-dark-hover.png"), clip=clip)
    page.reload(wait_until="networkidle")
    print("after reload:", page.evaluate("document.documentElement.dataset.theme"))
    print("page errors:", errors[:5])
    b.close()
