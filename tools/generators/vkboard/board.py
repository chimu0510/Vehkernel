"""Load a board definition and validate it against the MCU database (ADR-0011)."""

from __future__ import annotations

import json
import re
import tomllib
from dataclasses import dataclass
from pathlib import Path

from .diagnostics import BoardError, Diagnostics

GPIO_FUNCTIONS = ("GPIO_INPUT", "GPIO_OUTPUT")
PIN_KEYS = {"pin", "role", "func"}
TOP_LEVEL_KEYS = {"name", "mcu", "pins"}

_ROLE_RE = re.compile(r"^[A-Z][A-Z0-9_]*$")
_PIN_RE = re.compile(r"^P([A-K])(\d+)$")
# One pin per line is guaranteed by TOML 1.0 inline tables, so the n-th
# inline table holding a "pin" key is the n-th entry of the pins array.
_PIN_LINE_RE = re.compile(r"\{[^}]*\bpin\s*=\s*\"")
_TOML_POSITION_RE = re.compile(r"\s*\(at line (\d+), column \d+\)")


@dataclass(frozen=True)
class Pin:
    pin: str
    port: str
    number: int
    role: str
    func: str
    kind: str  # "af", "analog", "gpio_input" or "gpio_output"
    af: int | None
    line: int | None


@dataclass(frozen=True)
class Board:
    name: str
    mcu: str
    source: str
    pins: tuple[Pin, ...]


def _pin_lines(text: str) -> list[int]:
    lines = []
    for number, line in enumerate(text.splitlines(), start=1):
        code = line.split("#", 1)[0]
        lines.extend(number for _ in _PIN_LINE_RE.finditer(code))
    return lines


def load_mcu_database(mcu_root: Path, mcu: str) -> dict:
    return json.loads((mcu_root / mcu / "mcu.json").read_text(encoding="utf-8"))


def load_board(path: Path, mcu_root: Path, display_path: str | None = None) -> Board:
    """Parse and validate a board definition, raising BoardError with every problem found."""
    shown = display_path or str(path)
    diag = Diagnostics(shown)
    text = path.read_text(encoding="utf-8")
    try:
        data = tomllib.loads(text)
    except tomllib.TOMLDecodeError as error:
        # Python 3.12 reports the position only inside the message text.
        match = _TOML_POSITION_RE.search(str(error))
        message = _TOML_POSITION_RE.sub("", str(error)).strip()
        diag.error(f"invalid TOML: {message}", int(match.group(1)) if match else None)
        raise BoardError(diag.errors) from None

    for key in sorted(set(data) - TOP_LEVEL_KEYS):
        diag.error(f"unknown key '{key}'")
    name, mcu = data.get("name"), data.get("mcu")
    if not isinstance(name, str) or not name:
        diag.error("missing board 'name'")
    if not isinstance(mcu, str) or not mcu:
        diag.error("missing 'mcu'")
    raw_pins = data.get("pins", [])
    if not isinstance(raw_pins, list):
        diag.error("'pins' must be an array of inline tables")
    if not isinstance(name, str) or not isinstance(mcu, str) or not isinstance(raw_pins, list) or not mcu:
        raise BoardError(diag.errors)

    try:
        database = load_mcu_database(mcu_root, mcu)
    except FileNotFoundError:
        diag.error(f"unknown MCU '{mcu}': the MCU root has no {mcu}/mcu.json")
        raise BoardError(diag.errors) from None

    lines = _pin_lines(text)
    if len(lines) != len(raw_pins):
        lines = [None] * len(raw_pins)

    pins = [p for p in (_check_pin(entry, line, database, diag) for entry, line in zip(raw_pins, lines)) if p]
    _check_unique(pins, diag)
    if diag:
        raise BoardError(diag.errors)
    return Board(name=name, mcu=mcu, source=shown, pins=tuple(pins))


def _check_pin(entry: object, line: int | None, database: dict, diag: Diagnostics) -> Pin | None:
    if not isinstance(entry, dict):
        diag.error("each pin must be an inline table", line)
        return None
    ok = True
    for key in sorted(set(entry) - PIN_KEYS):
        diag.error(f"unknown key '{key}' in pin entry", line)
        ok = False
    for key in sorted(PIN_KEYS - set(entry)):
        diag.error(f"pin entry is missing '{key}'", line)
        ok = False
    if not ok:
        return None
    pin, role, func = entry["pin"], entry["role"], entry["func"]

    if not _ROLE_RE.match(role):
        diag.error(f"role '{role}' must be upper case letters, digits and underscores", line)
        return None
    match = _PIN_RE.match(pin)
    if match is None or pin not in database["pins"]:
        diag.error(f"pin {pin} does not exist on {database['mcu']} ({database['package']})", line)
        return None

    if func in GPIO_FUNCTIONS:
        kind, af = func.lower(), None
    elif func in database.get("analog", {}).get(pin, []):
        kind, af = "analog", None
    else:
        afs = [e["af"] for e in database["af"][pin] if e["function"] == func]
        if not afs:
            available = ", ".join(e["function"] for e in database["af"][pin]) or "none"
            diag.error(f"function {func} is not available on pin {pin} (available: {available})", line)
            return None
        kind, af = "af", afs[0]
    return Pin(pin=pin, port=match.group(1), number=int(match.group(2)),
               role=role, func=func, kind=kind, af=af, line=line)


def _where(pin: Pin) -> str:
    return f"line {pin.line}" if pin.line else "another entry"


def _check_unique(pins: list[Pin], diag: Diagnostics) -> None:
    by_pin: dict[str, Pin] = {}
    by_role: dict[str, Pin] = {}
    by_signal: dict[str, Pin] = {}
    for p in pins:
        if p.pin in by_pin:
            first = by_pin[p.pin]
            diag.error(f"pin {p.pin} is already assigned to role {first.role} ({_where(first)})", p.line)
        else:
            by_pin[p.pin] = p
        if p.role in by_role:
            diag.error(f"role {p.role} is already defined ({_where(by_role[p.role])})", p.line)
        else:
            by_role[p.role] = p
        # A peripheral signal (for example a timer channel) can drive only one pin.
        if p.kind == "af":
            if p.func in by_signal:
                first = by_signal[p.func]
                diag.error(f"signal {p.func} is already used by pin {first.pin} "
                           f"for role {first.role} ({_where(first)})", p.line)
            else:
                by_signal[p.func] = p
