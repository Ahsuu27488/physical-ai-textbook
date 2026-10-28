"""Side-by-side before/after images: python design/compare.py  ->  design/compare/*.png"""
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

root = Path(__file__).parent
out = root / "compare"
out.mkdir(exist_ok=True)
try:
    font = ImageFont.truetype("arial.ttf", 28)
except OSError:
    font = ImageFont.load_default()

# every shot present in both folders (full-page shots are skipped: too tall to compare side by side)
names = sorted(f.stem for f in (root / "after").glob("*.png") if (root / "before" / f.name).exists() and "-full-" not in f.stem)
for name in names:
    before, after = root / "before" / f"{name}.png", root / "after" / f"{name}.png"
    if not (before.exists() and after.exists()):
        continue
    a, b = Image.open(before).convert("RGB"), Image.open(after).convert("RGB")
    gap, head = 24, 56
    canvas = Image.new("RGB", (a.width + b.width + gap, max(a.height, b.height) + head), (24, 26, 32))
    canvas.paste(a, (0, head))
    canvas.paste(b, (a.width + gap, head))
    d = ImageDraw.Draw(canvas)
    d.text((12, 12), "Before", fill=(200, 205, 215), font=font)
    d.text((a.width + gap + 12, 12), "After", fill=(198, 244, 50), font=font)
    if canvas.width > 2400:
        canvas = canvas.resize((2400, round(canvas.height * 2400 / canvas.width)), Image.LANCZOS)
    canvas.save(out / f"{name}.png", optimize=True)
    print("wrote", out / f"{name}.png")
