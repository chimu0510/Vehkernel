# SPDX-License-Identifier: GPL-3.0-or-later
"""Command line of the schema generator."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .emit import outputs
from .schema import SchemaError, load_schema


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="generate_schema.py",
        description="Validate .msg schema files and emit typed C++ topics.")
    parser.add_argument("messages", type=Path, nargs="*", help=".msg files, for example schema/msg/imu_sample.msg")
    parser.add_argument("--output-dir", type=Path, required=True, help="directory for generated files")
    args = parser.parse_args(argv)

    try:
        messages = load_schema(args.messages)
    except SchemaError as error:
        for diagnostic in error.diagnostics:
            print(diagnostic, file=sys.stderr)
        return 1

    for name, text in outputs(messages).items():
        target = args.output_dir / name
        target.parent.mkdir(parents=True, exist_ok=True)
        # Leave unchanged files alone so the build does not recompile needlessly.
        if not target.exists() or target.read_text(encoding="utf-8") != text:
            target.write_text(text, encoding="utf-8")
    return 0
