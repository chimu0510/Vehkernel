"""Command line of the board generator."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .board import load_board
from .diagnostics import BoardError
from .emit import outputs


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="generate_board.py",
        description="Validate a board definition against its MCU database and emit board outputs.")
    parser.add_argument("board", type=Path, help="board definition, for example boards/c50c-v1_1/board.toml")
    parser.add_argument("--mcu-root", type=Path, required=True, help="directory holding <mcu>/mcu.json")
    parser.add_argument("--output-dir", type=Path, required=True, help="directory for generated files")
    args = parser.parse_args(argv)

    try:
        board = load_board(args.board, args.mcu_root)
    except BoardError as error:
        for diagnostic in error.diagnostics:
            print(diagnostic, file=sys.stderr)
        return 1
    except OSError as error:
        print(f"{args.board}: error: {error.strerror}", file=sys.stderr)
        return 1

    args.output_dir.mkdir(parents=True, exist_ok=True)
    for name, text in outputs(board).items():
        target = args.output_dir / name
        # Leave unchanged files alone so the build does not recompile needlessly.
        if not target.exists() or target.read_text(encoding="utf-8") != text:
            target.write_text(text, encoding="utf-8")
    return 0
