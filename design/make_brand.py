"""Render the brand rasters from frontend/static/img/logo.svg: favicons, app icons and the social card.

Usage: python design/make_brand.py      (re-run after changing logo.svg or the card text)
"""
import io
from pathlib import Path

from PIL import Image
from playwright.sync_api import sync_playwright

img = Path(__file__).resolve().parents[1] / "frontend" / "static" / "img"
svg = (img / "logo.svg").read_text(encoding="utf-8")
poster = (img / "hero-humanoid.webp").as_uri()

CARD = f"""<!doctype html><html><head><meta charset="utf-8"><style>
@font-face {{ font-family: Geist; src: url('{(img.parent.parent / 'src/fonts/geist-latin-wght-normal.woff2').as_uri()}'); font-weight: 100 900; }}
@font-face {{ font-family: Serif; src: url('{(img.parent.parent / 'src/fonts/instrument-serif-latin-400-italic.woff2').as_uri()}'); font-style: italic; }}
html, body {{ margin: 0; }}
body {{ width: 1200px; height: 630px; background: #0b0e13; color: #f1f4f8; font-family: Geist; position: relative; overflow: hidden; }}
.grid {{ position: absolute; inset: 0; background-image: linear-gradient(rgba(255,255,255,.05) 1px, transparent 1px),
  linear-gradient(90deg, rgba(255,255,255,.05) 1px, transparent 1px); background-size: 48px 48px;
  -webkit-mask-image: radial-gradient(60% 70% at 70% 50%, #000, transparent 80%); }}
.glow {{ position: absolute; right: 70px; bottom: -60px; width: 520px; height: 200px;
  background: radial-gradient(closest-side, rgba(198,244,50,.25), transparent); }}
.fig {{ position: absolute; right: 60px; top: 30px; height: 570px; }}
.brand {{ position: absolute; left: 80px; top: 72px; display: flex; align-items: center; gap: 18px; font-size: 34px; font-weight: 650; letter-spacing: -0.02em; }}
.brand svg {{ width: 56px; height: 56px; }}
h1 {{ position: absolute; left: 80px; top: 200px; margin: 0; width: 640px; font-size: 84px; line-height: .98; letter-spacing: -0.045em; font-weight: 650; }}
h1 em {{ font-family: Serif; font-weight: 400; color: #c6f432; letter-spacing: -0.01em; }}
p {{ position: absolute; left: 80px; top: 420px; width: 560px; margin: 0; font-size: 26px; line-height: 1.4; color: #b7c0cf; }}
.tag {{ position: absolute; left: 80px; bottom: 64px; font-size: 20px; color: #8d97a8; }}
</style></head><body><div class="grid"></div><div class="glow"></div><img class="fig" src="{poster}">
<div class="brand">{svg}Proprio</div>
<h1>Give AI a <em>body</em>.</h1>
<p>The open course in Physical AI &amp; humanoid robotics, from zero to expert.</p>
<div class="tag">11 modules · 52 lessons · labs, quizzes and an AI tutor</div>
</body></html>"""

with sync_playwright() as p:
    b = p.chromium.launch()
    page = b.new_page(viewport={"width": 512, "height": 512})
    page.set_content(f"<html><body style='margin:0;background:transparent'>{svg.replace('<svg ', '<svg width=\"512\" height=\"512\" ', 1)}</body></html>")
    big = Image.open(io.BytesIO(page.locator("svg").screenshot(omit_background=True))).convert("RGBA")
    for name, size in [("android-chrome-512x512.png", 512), ("android-chrome-192x192.png", 192),
                       ("apple-touch-icon.png", 180), ("favicon-32x32.png", 32), ("favicon-16x16.png", 16)]:
        big.resize((size, size), Image.LANCZOS).save(img / name, optimize=True)
    big.resize((64, 64), Image.LANCZOS).save(img / "favicon.ico", sizes=[(16, 16), (32, 32), (48, 48), (64, 64)])

    card = b.new_page(viewport={"width": 1200, "height": 630})
    tmp = Path(__file__).parent / "_card.html"  # a file page may load the local fonts and figure
    tmp.write_text(CARD, encoding="utf-8")
    card.goto(tmp.as_uri(), wait_until="networkidle")
    card.wait_for_timeout(300)
    card.screenshot(path=str(img / "social-card.png"))
    b.close()
    tmp.unlink()

(img / "site.webmanifest").write_text(
    '{"name":"Proprio — Physical AI & humanoid robotics","short_name":"Proprio",'
    '"icons":[{"src":"android-chrome-192x192.png","sizes":"192x192","type":"image/png"},'
    '{"src":"android-chrome-512x512.png","sizes":"512x512","type":"image/png"}],'
    '"theme_color":"#0b0e13","background_color":"#0b0e13","display":"standalone","start_url":"../"}\n',
    encoding="utf-8")
print("brand assets written to", img)
