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
        description="Validate .msg and .enum schema files and emit typed C++ topics and enums.")
    parser.add_argument("files", type=Path, nargs="*",
                        help=".msg and .enum files, for example schema/msg/imu_sample.msg schema/enum/mode.enum")
    parser.add_argument("--output-dir", type=Path, required=True, help="directory for generated files")
    args = parser.parse_args(argv)

    try:
        schema = load_schema(args.files)
    except SchemaError as error:
        for diagnostic in error.diagnostics:
            print(diagnostic, file=sys.stderr)
        return 1

    for name, text in outputs(schema).items():
        target = args.output_dir / name
        target.parent.mkdir(parents=True, exist_ok=True)
        # Leave unchanged files alone so the build does not recompile needlessly.
        if not target.exists() or target.read_text(encoding="utf-8") != text:
            target.write_text(text, encoding="utf-8")
    return 0
