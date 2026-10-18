#!/usr/bin/env python3
"""Schema generator entry point: validates .msg files and emits typed C++ topics (ADR-0006)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from vkschema.cli import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
