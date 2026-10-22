#!/usr/bin/env python3
"""Sklada obrazy PNG z zapisow symulatora (shots/*.txt) - tylko do podgladu ukladu ekranu na hoscie.
Tekst jest rysowany zastepcza czcionka, wiec szerokosci sa orientacyjne (na plytce czcionki TFT_eSPI wygladaja inaczej)."""
import sys, glob, os
from PIL import Image, ImageDraw, ImageFont
S = 3
def rgb(v):
    r = (v >> 11) & 31; g = (v >> 5) & 63; b = v & 31
    return (r * 255 // 31, g * 255 // 63, b * 255 // 31)
fonts = {1: ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 8 * S),
         2: ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 13 * S),
         4: ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 21 * S)}
def render(path, out):
    L = open(path).read().split("\n")
    w, h = map(int, L[0].split())
    im = Image.new("RGB", (w, h))
    px = im.load()
    for y in range(h):
        vals = L[1 + y].split()
        for x in range(w):
            px[x, y] = rgb(int(vals[x], 16))
    im = im.resize((w * S, h * S), Image.NEAREST)
    d = ImageDraw.Draw(im)
    n = int(L[1 + h])
    fh = {1: 8, 2: 16, 4: 26}
    for i in range(n):
        head, txt = L[2 + h + i].split("|", 1)
        x0, y0, font, datum, fg, bg = map(int, head.split())
        if not txt:
            continue
        fw = {1: 6, 2: 8, 4: 14}[font] * len(txt)
        d.rectangle([x0 * S, y0 * S, (x0 + fw) * S - 1, (y0 + fh[font]) * S - 1], fill=rgb(bg))
        d.text((x0 * S, y0 * S - (1 if font == 1 else 0) * S), txt, font=fonts[font], fill=rgb(fg))
    im.save(out)
if __name__ == "__main__":
    src = sys.argv[1] if len(sys.argv) > 1 else "shots"
    for f in sorted(glob.glob(os.path.join(src, "*.txt"))):
        render(f, f[:-4] + ".png")
