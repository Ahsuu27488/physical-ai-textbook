"""Capture the hero intro as a sequence of frames: python design/intro_frames.py <out-dir> [base-url]"""
import sys
import time
from pathlib import Path

from playwright.sync_api import sync_playwright

out = Path(sys.argv[1])
base = sys.argv[2] if len(sys.argv) > 2 else "http://localhost:3123"
out.mkdir(parents=True, exist_ok=True)

with sync_playwright() as p:
    b = p.chromium.launch(args=["--use-angle=swiftshader", "--enable-unsafe-swiftshader"])
    page = b.new_page(viewport={"width": 1440, "height": 900}, color_scheme="dark")
    page.goto(base + "/", wait_until="domcontentloaded")
    page.wait_for_selector("[data-ready='true'] canvas", timeout=30000)
    t0 = time.time()
    for t in [0.4, 1.2, 2.0, 2.8, 4.0, 6.0]:
        time.sleep(max(0, t - (time.time() - t0)))
        page.screenshot(path=str(out / f"intro-{t:.1f}s.png"), clip={"x": 700, "y": 60, "width": 740, "height": 840})
    page.mouse.move(1050, 420)
    time.sleep(0.6)
    page.screenshot(path=str(out / "hover.png"), clip={"x": 700, "y": 60, "width": 740, "height": 840})
    b.close()
print("saved", out)
