#!/usr/bin/env python3
"""
convert_logo.py  —  Convert PNG/JPG to RGB565 C array for ST7735 TFT
Usage: python convert_logo.py <image_path> <width> <height>
Output: logo.h in the same folder

RGB565 format: RRRRRGGGGGGBBBBB  (16 bits per pixel, big-endian)
"""

from PIL import Image
import sys
import os

# ── Config ──────────────────────────────────────────────────────────────────
IMAGE_PATH = r"C:\Users\Admin\.gemini\antigravity\brain\f9d6dc90-e854-46e1-ac46-5d7ab2937e9b\bannari_amman_logo_1775575500381.png"
OUT_W = 100   # Target width  (keep it <= 128)
OUT_H = 100   # Target height (keep it <= 160)
OUT_FILE = r"c:\Users\Admin\OneDrive\Desktop\mplabnew_secure\src\logo.h"
ARRAY_NAME = "bannari_logo"
# ────────────────────────────────────────────────────────────────────────────

img = Image.open(IMAGE_PATH).convert("RGBA")

# White-fill transparent background
bg = Image.new("RGBA", img.size, (255, 255, 255, 255))
bg.paste(img, mask=img.split()[3])
img = bg.convert("RGB")

# Resize to target
img = img.resize((OUT_W, OUT_H), Image.LANCZOS)

pixels = list(img.getdata())
total  = len(pixels)

def to_rgb565(r, g, b):
    """Convert 8-bit R,G,B to packed 16-bit RGB565 (big-endian word)."""
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)

lines = []
lines.append("// Auto-generated RGB565 bitmap — DO NOT EDIT")
lines.append("// Source: Bannari Amman Institute of Technology logo")
lines.append(f"// Size: {OUT_W} x {OUT_H} pixels  ({total*2} bytes)")
lines.append("")
lines.append("#pragma once")
lines.append("#include <stdint.h>")
lines.append("")
lines.append(f"#define LOGO_W  {OUT_W}")
lines.append(f"#define LOGO_H  {OUT_H}")
lines.append("")
lines.append(f"const uint16_t {ARRAY_NAME}[{total}] = {{")

row_data = []
for idx, (r, g, b) in enumerate(pixels):
    word = to_rgb565(r, g, b)
    row_data.append(f"0x{word:04X}")
    if len(row_data) == 10 or idx == total - 1:
        lines.append("    " + ", ".join(row_data) + ",")
        row_data = []

lines.append("};")
lines.append("")

output = "\n".join(lines)
with open(OUT_FILE, "w") as f:
    f.write(output)

print(f"Done! {total} pixels written to:")
print(f"  {OUT_FILE}")
print(f"  Array: {ARRAY_NAME}[{total}]  ({OUT_W}x{OUT_H})")
