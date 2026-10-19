"""Command line of the parameter generator."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .emit import outputs
from .params import ParamsError, compiled_in, load_definitions, read_config


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="generate_params.py",
        description="Validate parameter definitions and emit parameter groups, the parameter table "
                    "and the metadata file (ADR-0012).")
    parser.add_argument("definitions", type=Path, nargs="*",
                        help="every params.toml in the repository, for example modules/estimation/params.toml")
    parser.add_argument("--config", type=Path,
                        help="solved Kconfig .config; without it every definition is compiled in")
    parser.add_argument("--output-dir", type=Path, required=True, help="directory for generated files")
    args = parser.parse_args(argv)

    try:
        groups = load_definitions(args.definitions)
        enabled = read_config(args.config) if args.config is not None else None
        groups = compiled_in(groups, enabled)
    except ParamsError as error:
        for diagnostic in error.diagnostics:
            print(diagnostic, file=sys.stderr)
        return 1

    for name, text in outputs(groups).items():
        target = args.output_dir / name
        target.parent.mkdir(parents=True, exist_ok=True)
        # Leave unchanged files alone so the build does not recompile needlessly.
        if not target.exists() or target.read_text(encoding="utf-8") != text:
            target.write_text(text, encoding="utf-8")
    return 0
