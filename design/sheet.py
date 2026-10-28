"""Contact sheet: python design/sheet.py <dir> <out.png> name1 name2 ... [--scale=0.5] [--cols=2]"""
import sys
from pathlib import Path
from PIL import Image, ImageDraw

args = [a for a in sys.argv[1:] if not a.startswith("--")]
opts = dict(a[2:].split("=") for a in sys.argv[1:] if a.startswith("--"))
d, outp, names = Path(args[0]), args[1], args[2:]
scale, cols = float(opts.get("scale", 0.5)), int(opts.get("cols", 2))
ims = []
for n in names:
    f = d / f"{n}.png"
    if f.exists():
        im = Image.open(f).convert("RGB")
        ims.append((n, im.resize((round(im.width * scale), round(im.height * scale)))))
if not ims:
    sys.exit("no images")
cw = max(i.width for _, i in ims); rows = [ims[i:i + cols] for i in range(0, len(ims), cols)]
H = sum(max(i.height for _, i in r) + 22 for r in rows)
c = Image.new("RGB", (cw * cols + 8 * (cols - 1), H), (40, 40, 48)); dr = ImageDraw.Draw(c); y = 0
for r in rows:
    x = 0
    for n, im in r:
        dr.text((x + 4, y + 4), n, fill=(230, 230, 120)); c.paste(im, (x, y + 22)); x += cw + 8
    y += max(i.height for _, i in r) + 22
c.save(outp)
