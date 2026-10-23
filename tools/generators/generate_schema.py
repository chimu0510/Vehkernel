#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Schema generator entry point: validates .msg files and emits typed C++ topics (ADR-0006)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from vkschema.cli import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
