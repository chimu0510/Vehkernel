#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Board generator entry point: validates a board definition and emits board outputs (ADR-0011)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from vkboard.cli import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
