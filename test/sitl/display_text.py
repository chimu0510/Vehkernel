# SPDX-License-Identifier: GPL-3.0-or-later
"""Read the text on SITL's display image (ADR-0022).

SITL's --display writes what the simulated SSD1306 shows as a binary PBM
file. The local display draws 21 columns by 8 lines in its font's 6 x 8
cells, one pixel in from the left edge, so each cell is matched against the
glyphs of the firmware's own font table in modules/local_display/src/font.cpp.
"""

from __future__ import annotations

import re
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
FONT = REPO / "modules" / "local_display" / "src" / "font.cpp"
COLUMNS, LINES = 21, 8
CELL_W, CELL_H, GLYPH_W, GLYPH_H = 6, 8, 5, 7
LEFT = 1


def load_glyphs(font: Path = FONT) -> dict[tuple[str, ...], str]:
    """Each glyph's seven rows, mapped to its character, from the font table."""
    rows = re.findall(r'\{("[.#]{5}"(?:, "[.#]{5}"){6})\},', font.read_text(encoding="utf-8"))
    if len(rows) != 95:
        raise ValueError(f"expected 95 glyphs in {font}, found {len(rows)}")
    return {tuple(r.strip('"') for r in row.split(", ")): chr(0x20 + i) for i, row in enumerate(rows)}


def read_pbm(path: Path) -> list[list[bool]]:
    data = path.read_bytes()
    match = re.match(rb"P4\s+(\d+)\s+(\d+)\s", data)
    if match is None:
        raise ValueError(f"{path} is not a binary PBM file")
    width, height = int(match.group(1)), int(match.group(2))
    pixels = data[match.end():]
    stride = (width + 7) // 8
    return [[bool(pixels[y * stride + x // 8] & (0x80 >> (x % 8))) for x in range(width)] for y in range(height)]


def screen_text(path: Path) -> list[str]:
    """The display's eight lines, trailing spaces removed; '\\x00' for a cell no glyph matches."""
    image = read_pbm(path)
    glyphs = load_glyphs()
    lines = []
    for line in range(LINES):
        text = ""
        for column in range(COLUMNS):
            x0, y0 = LEFT + column * CELL_W, line * CELL_H
            rows = tuple("".join("#" if image[y0 + y][x0 + x] else "." for x in range(GLYPH_W))
                         for y in range(GLYPH_H))
            text += glyphs.get(rows, "\x00")
        lines.append(text.rstrip())
    return lines
