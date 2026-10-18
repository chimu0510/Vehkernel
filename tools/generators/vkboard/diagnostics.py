"""Diagnostics in compiler style: ``file:line: error: message``."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class Loc:
    """Where an entry was written: a board definition file and line."""

    path: str
    line: int | None = None

    def refer(self, from_loc: Loc | None) -> str:
        """How to mention this location in a message about ``from_loc``."""
        if from_loc is not None and from_loc.path == self.path:
            return f"line {self.line}" if self.line else "another entry"
        return f"{self.path}:{self.line}" if self.line else self.path


@dataclass(frozen=True)
class Diagnostic:
    path: str
    line: int | None
    message: str

    def __str__(self) -> str:
        location = f"{self.path}:{self.line}" if self.line else self.path
        return f"{location}: error: {self.message}"


@dataclass
class Diagnostics:
    """Collects every error so one run reports all of them."""

    path: str
    errors: list[Diagnostic] = field(default_factory=list)

    def error(self, message: str, loc: Loc | None = None) -> None:
        if loc is None:
            self.errors.append(Diagnostic(self.path, None, message))
        else:
            self.errors.append(Diagnostic(loc.path, loc.line, message))

    def __bool__(self) -> bool:
        return bool(self.errors)


class BoardError(Exception):
    """Raised when a board definition cannot be processed."""

    def __init__(self, diagnostics: list[Diagnostic]):
        super().__init__("\n".join(map(str, diagnostics)))
        self.diagnostics = diagnostics
