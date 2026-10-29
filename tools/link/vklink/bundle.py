# SPDX-License-Identifier: GPL-3.0-or-later
"""Bundle root helper for host tools (spec 0014, ADR-0026)."""

from __future__ import annotations

from pathlib import Path


def bundle_root(start: Path) -> Path | None:
    """Returns the bundle root if start is within a bundle, else None.

    A bundle has manifest.json in the directory two levels above the tool's
    own directory: <bundle>/tools/<dir>/<tool>.py gives <bundle>.
    """
    path = start.resolve()
    tool_dir = path if path.is_dir() else path.parent
    candidate = tool_dir.parent.parent
    if (candidate / "manifest.json").is_file():
        return candidate
    return None
