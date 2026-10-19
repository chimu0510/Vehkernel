#!/usr/bin/env python3
"""Parameter generator entry point: validates params.toml files and emits parameter groups (ADR-0012)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from vkparams.cli import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
