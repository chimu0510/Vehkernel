"""Board ID and bootloader (ADR-0023).

- ``board_id``: the number a firmware image's app descriptor and ``.apj`` file
  carry, numbered as in the board list ArduPilot and PX4 share. A board with
  a Flash layout builds firmware and must declare it.
- ``bootloader``: which bootloader the board has. ``VEHKERNEL`` lists the
  serial ports it listens on, in order, and may name a ``stay`` input that
  keeps the board in the bootloader. ``ARDUPILOT`` and ``PX4`` keep the
  bootloader the board ships with.

A board with the Vehkernel bootloader takes an ID that no board in
ArduPilot's list uses, outside the range ArduPilot manages; a board keeping
an ArduPilot or PX4 bootloader takes the ID its existing firmware uses, so
it must be in the list. No two board definitions beside each other share an
ID. The list is a copy of ArduPilot's ``board_types.txt``.
"""

from __future__ import annotations

import re
import tomllib
from dataclasses import dataclass
from pathlib import Path

from .diagnostics import Diagnostics, Loc

BOOTLOADER_KEYS = {"kind", "ports", "stay"}
BOOTLOADER_KINDS = ("VEHKERNEL", "ARDUPILOT", "PX4")
STAY_KEYS = {"pin", "active"}
LEVELS = ("HIGH", "LOW")
BOARD_ID_MAX = 65535
# ArduPilot allocates these only through its list (board_types.txt).
MANAGED_RANGE = (1000, 19999)
BOARD_TYPES = Path(__file__).with_name("board_types.txt")

_RESERVED_RANGE_RE = re.compile(r"IDs?\s+(\d+)\s*[-–]\s*(\d+)\s+(?:are\s+)?reserved", re.IGNORECASE)


@dataclass(frozen=True)
class Stay:
    pin: str  # pin role
    active: str  # level that keeps the board in the bootloader: HIGH or LOW


@dataclass(frozen=True)
class Bootloader:
    kind: str
    ports: tuple[str, ...] = ()  # logical serial ports, for VEHKERNEL
    stay: Stay | None = None


def registered_ids(path: Path = BOARD_TYPES) -> tuple[set[int], list[tuple[int, int]]]:
    """IDs the list gives a board, and the ranges its comments reserve for vendors."""
    ids, ranges = set(), []
    for line in path.read_text(encoding="utf-8").splitlines():
        code, _, comment = line.partition("#")
        match = _RESERVED_RANGE_RE.search(comment)
        if match:
            ranges.append((int(match.group(1)), int(match.group(2))))
        words = code.split()
        if len(words) >= 2 and words[-1].isdigit():
            ids.add(int(words[-1]))
    return ids, ranges


def _is_int(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def check_board_id(board_id: object, kind: str | None, loc: Loc, diag: Diagnostics) -> int | None:
    if not _is_int(board_id) or not 1 <= board_id <= BOARD_ID_MAX:
        diag.error(f"'board_id' must be an integer from 1 to {BOARD_ID_MAX}, got {board_id!r}", loc)
        return None
    ids, ranges = registered_ids()
    listed = board_id in ids
    if kind == "VEHKERNEL":
        low, high = MANAGED_RANGE
        if listed:
            diag.error(f"board ID {board_id} is used by a board in ArduPilot's list; a board with the "
                       f"Vehkernel bootloader needs an unused one", loc)
            return None
        if low <= board_id <= high:
            diag.error(f"board ID {board_id} lies in the range {low} to {high} that ArduPilot allocates; "
                       f"a board with the Vehkernel bootloader needs one outside it", loc)
            return None
        reserved = next((r for r in ranges if r[0] <= board_id <= r[1]), None)
        if reserved:
            diag.error(f"board ID {board_id} lies in IDs {reserved[0]}-{reserved[1]}, which ArduPilot's list "
                       f"reserves for a vendor", loc)
            return None
    elif kind in ("ARDUPILOT", "PX4") and not listed:
        diag.error(f"board ID {board_id} is not in ArduPilot's list; a board keeping an ArduPilot or PX4 "
                   f"bootloader uses the ID its existing firmware uses", loc)
        return None
    return board_id


def _stay(entry: object, roles: dict, loc: Loc, diag: Diagnostics) -> tuple[bool, Stay | None]:
    if entry is None:
        return True, None
    if not isinstance(entry, dict) or set(entry) != STAY_KEYS:
        diag.error("the bootloader's 'stay' needs exactly 'pin' and 'active'", loc)
        return False, None
    pin = roles.get(entry["pin"]) if isinstance(entry["pin"], str) else None
    if pin is None or pin.kind != "gpio_input":
        diag.error(f"the bootloader's stay pin must be a pin role with the GPIO_INPUT function, "
                   f"not {entry['pin']!r}", loc)
        return False, None
    if entry["active"] not in LEVELS:
        diag.error(f"the bootloader's stay level must be one of: {', '.join(LEVELS)}, got {entry['active']!r}", loc)
        return False, None
    return True, Stay(entry["pin"], entry["active"])


def check_bootloader(entry: object, loc: Loc, ports: set[str], roles: dict,
                     diag: Diagnostics) -> Bootloader | None:
    if not isinstance(entry, dict):
        diag.error("'bootloader' must be an inline table", loc)
        return None
    ok = True
    for key in sorted(set(entry) - BOOTLOADER_KEYS):
        diag.error(f"unknown key '{key}' in bootloader", loc)
        ok = False
    kind = entry.get("kind")
    if kind not in BOOTLOADER_KINDS:
        diag.error(f"bootloader kind must be one of: {', '.join(BOOTLOADER_KINDS)}, got {kind!r}", loc)
        return None
    if kind != "VEHKERNEL":
        for key in sorted({"ports", "stay"} & set(entry)):
            diag.error(f"'{key}' belongs to the Vehkernel bootloader, not to a kept {kind} bootloader", loc)
            ok = False
        return Bootloader(kind) if ok else None
    listened = entry.get("ports")
    if not isinstance(listened, list) or not listened or not all(isinstance(p, str) for p in listened):
        diag.error("the Vehkernel bootloader needs 'ports': a non-empty list of serial ports", loc)
        return None
    unknown = sorted({p for p in listened if p not in ports})
    if unknown:
        diag.error(f"the bootloader lists serial ports the board does not define: {', '.join(unknown)}", loc)
        ok = False
    if len(set(listened)) != len(listened):
        diag.error("the bootloader lists a serial port twice", loc)
        ok = False
    stay_ok, stay = _stay(entry.get("stay"), roles, loc, diag)
    if not ok or not stay_ok:
        return None
    return Bootloader(kind, tuple(listened), stay)


def check_unique_id(board_id: int, path: Path, loc: Loc, diag: Diagnostics) -> None:
    """Reports another board definition beside this one with the same ID."""
    here = path.resolve()
    for other in sorted(here.parent.parent.glob("*/board.toml")):
        if other.resolve() == here:
            continue
        try:
            data = tomllib.loads(other.read_text(encoding="utf-8"))
        except (OSError, tomllib.TOMLDecodeError):
            continue  # an unreadable board reports its own problems
        if data.get("board_id") == board_id:
            diag.error(f"board ID {board_id} is already used by board {other.parent.name}", loc)
            return
