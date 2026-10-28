"""Render the hero humanoid once and save it as a transparent WebP poster.

The poster is what phones, reduced-motion users and slow devices see instead of live WebGL, and what
desktop shows until the first WebGL frame. Re-run after changing humanoidScene.js:
    python design/make_poster.py [base-url]
"""
import io
import sys
from pathlib import Path

from PIL import Image
from playwright.sync_api import sync_playwright

base = sys.argv[1] if len(sys.argv) > 1 else "http://localhost:3123"
out = Path(__file__).resolve().parents[1] / "frontend" / "static" / "img" / "hero-humanoid.webp"

with sync_playwright() as p:
    b = p.chromium.launch()
    # reduced motion = one deterministic still frame (assembled figure, scan line at the waist)
    page = b.new_page(viewport={"width": 1440, "height": 900}, device_scale_factor=2, color_scheme="dark",
                      reduced_motion="reduce")
    page.goto(base + "/?poster", wait_until="networkidle")
    page.wait_for_selector("[data-ready='true'] canvas", timeout=20000)
    page.add_style_tag(content="html, body, main, section, figure, div { background: transparent !important; }"
                               " section::before, section::after { display: none !important; }"
                               " figcaption, figure > span, [aria-label='Open chat'] { visibility: hidden !important; }")
    page.wait_for_timeout(300)
    png = page.locator("[data-ready='true'] canvas").screenshot(omit_background=True)
    b.close()

img = Image.open(io.BytesIO(png)).convert("RGBA")
# trim only the side margins: full height keeps the figure the same size as the live render
# (the camera's field of view is vertical), and the component centres it with object-fit: contain
left, _, right, _ = img.getbbox()
img = img.crop((left, 0, right, img.height))
w = 560
img = img.resize((w, round(img.height * w / img.width)), Image.LANCZOS)
img.save(out, "WEBP", quality=72, method=6)
print(out, img.size, f"{out.stat().st_size // 1024} KB")
