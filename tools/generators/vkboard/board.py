"""Load a board definition and validate it against the MCU database (ADR-0010, ADR-0011).

A board definition may ``extends`` another board in a sibling directory. The
child's pin entries replace base entries for the same pin, ``remove`` drops
base pins, and its ``dma`` entries replace base entries for the same request.

A pin may declare alt configs: ``alt = [{ config = 1, role = ..., func = ... }]``.
One alt config is selected at boot by a parameter, config 0 being the default
pins; every config is checked for role and peripheral signal conflicts.

A ``GPIO_INPUT`` pin may declare ``pull = "UP"`` or ``"DOWN"``, the pull
resistor its default function needs; without it the input has no pull.
"""

from __future__ import annotations

import json
import os
import re
import tomllib
from dataclasses import dataclass, field
from pathlib import Path

from .diagnostics import BoardError, Diagnostics, Loc
from .dma import DmaAssignment, assign, collect_requests
from .clock import CLOCK_KEYS, Clock, solve_clock
from .flash import FlashRegion, RamRegion, check_layout, ram_regions
from .hardware import Hardware, check_hardware

GPIO_FUNCTIONS = ("GPIO_INPUT", "GPIO_OUTPUT")
PIN_KEYS = {"pin", "role", "func", "alt", "pull"}
PULLS = ("UP", "DOWN")
ALT_KEYS = {"config", "role", "func"}
TOP_LEVEL_KEYS = {"name", "mcu", "extends", "remove", "pins", "dma", "flash",
                  "serial", "devices", "storage", "clock", "timing", "defaults",
                  "revision", "actuator_power", "actuators", "user_key"}
ARRAY_SECTIONS = {"pins": "pin", "dma": "request", "flash": "region", "serial": "port",
                  "devices": "device", "storage": "storage", "actuators": "output"}
TABLE_SECTIONS = ("clock", "timing", "defaults", "revision", "actuator_power", "user_key")

_ROLE_RE = re.compile(r"^[A-Z][A-Z0-9_]*$")
_PIN_RE = re.compile(r"^P([A-K])(\d+)$")
_BOARD_NAME_RE = re.compile(r"^[a-z0-9][a-z0-9_-]*$")
_TOML_POSITION_RE = re.compile(r"\s*\(at line (\d+), column \d+\)")


@dataclass(frozen=True)
class Function:
    role: str
    func: str
    kind: str  # "af", "analog", "gpio_input" or "gpio_output"
    af: int | None


@dataclass(frozen=True)
class Pin:
    pin: str
    port: str
    number: int
    role: str
    func: str
    kind: str
    af: int | None
    loc: Loc
    alt: tuple[tuple[int, Function], ...] = ()  # (config, function) sorted by config
    pull: str | None = None  # of the default function, a GPIO_INPUT: UP or DOWN

    @property
    def default(self) -> Function:
        return Function(self.role, self.func, self.kind, self.af)

    def in_config(self, config: int) -> Function:
        return dict(self.alt).get(config, self.default)


@dataclass(frozen=True)
class Board:
    name: str
    mcu: str
    source: str
    pins: tuple[Pin, ...]
    dma: tuple[DmaAssignment, ...] = ()
    alt_configs: tuple[int, ...] = field(default=())
    flash: tuple[FlashRegion, ...] = ()
    ram: tuple[RamRegion, ...] = ()
    hardware: Hardware = Hardware()
    clock: Clock | None = None
    peripherals: tuple[tuple[str, dict], ...] = ()  # (instance, MCU database entry), sorted


@dataclass
class _Entry:
    data: object
    loc: Loc


@dataclass
class _Resolved:
    name: object
    mcu: object
    arrays: dict[str, list[_Entry]]  # by section name, see ARRAY_SECTIONS
    tables: dict[str, _Entry]  # see TABLE_SECTIONS


def _entry_lines(text: str, key: str, count: int) -> list[int | None]:
    """Line of each inline table holding ``key``, in order.

    TOML 1.0 keeps an inline table on one line, so the n-th inline table with
    ``key`` is the n-th entry of its array. Falls back to no line numbers if
    the text does not match that shape.
    """
    pattern = re.compile(r"\{[^}]*\b" + key + r"\s*=")
    # A top-level inline table such as ``user_key = { pin = ... }`` is not an array entry.
    table = re.compile(r"^\s*[A-Za-z_]+\s*=\s*\{")
    lines = []
    for number, line in enumerate(text.splitlines(), start=1):
        code = line.split("#", 1)[0]
        if table.match(code):
            continue
        lines.extend(number for _ in pattern.finditer(code))
    return lines if len(lines) == count else [None] * count


def load_mcu_database(mcu_root: Path, mcu: str) -> dict:
    return json.loads((mcu_root / mcu / "mcu.json").read_text(encoding="utf-8"))


def _read(path: Path, shown: str, diag: Diagnostics) -> tuple[dict, str] | None:
    try:
        text = path.read_text(encoding="utf-8")
    except FileNotFoundError:
        diag.error(f"board definition {shown} does not exist")
        return None
    try:
        return tomllib.loads(text), text
    except tomllib.TOMLDecodeError as error:
        # Python 3.12 reports the position only inside the message text.
        match = _TOML_POSITION_RE.search(str(error))
        message = _TOML_POSITION_RE.sub("", str(error)).strip()
        diag.error(f"invalid TOML: {message}", Loc(shown, int(match.group(1)) if match else None))
        return None


def _resolve(path: Path, shown: str, diag: Diagnostics,
             chain: tuple[tuple[Path, str], ...]) -> _Resolved | None:
    loaded = _read(path, shown, diag)
    if loaded is None:
        return None
    data, text = loaded
    top = Loc(shown)
    for key in sorted(set(data) - TOP_LEVEL_KEYS):
        diag.error(f"unknown key '{key}'", top)
    for key in (*ARRAY_SECTIONS, "remove"):
        if not isinstance(data.get(key, []), list):
            diag.error(f"'{key}' must be an array", top)
            return None
    arrays = {section: [_Entry(e, Loc(shown, line)) for e, line in
                        zip(data.get(section, []), _entry_lines(text, key, len(data.get(section, []))))]
              for section, key in ARRAY_SECTIONS.items()}
    tables = {section: _Entry(data[section], Loc(shown, _key_line(text, section)))
              for section in TABLE_SECTIONS if section in data}
    removed = data.get("remove", [])

    base_name = data.get("extends")
    if base_name is None:
        if removed:
            diag.error("'remove' needs 'extends'", top)
        return _Resolved(data.get("name"), data.get("mcu"), arrays, tables)

    if not isinstance(base_name, str) or not _BOARD_NAME_RE.match(base_name):
        diag.error("'extends' must name a board directory", top)
        return None
    base_path = path.resolve().parent.parent / base_name / "board.toml"
    # Show the base the way the board itself was named: relative stays relative.
    base_shown = str(base_path) if os.path.isabs(shown) else os.path.relpath(base_path)
    if any(base_path == real for real, _ in chain):
        names = " -> ".join([s for _, s in chain] + [base_shown])
        diag.error(f"board definitions extend each other in a cycle: {names}", top)
        return None
    base = _resolve(base_path, base_shown, diag, chain + ((base_path, base_shown),))
    if base is None:
        return None

    def key_of(entry: _Entry, key: str) -> object:
        return entry.data.get(key) if isinstance(entry.data, dict) else None

    base_pins = {key_of(e, "pin") for e in base.arrays["pins"]}
    for pin in removed:
        if pin not in base_pins:
            diag.error(f"cannot remove pin {pin}: {base_shown} does not define it", top)
    merged = {}
    # Pins: child entries replace base entries for the same pin, in place.
    pins = [e for e in base.arrays["pins"] if key_of(e, "pin") not in removed]
    for entry in arrays["pins"]:
        index = next((i for i, e in enumerate(pins)
                      if key_of(e, "pin") == key_of(entry, "pin") and e.loc.path != shown), None)
        if index is None:
            pins.append(entry)
        else:
            pins[index] = entry
    merged["pins"] = pins
    # A Flash layout is replaced as a whole, never merged region by region.
    merged["flash"] = arrays["flash"] or base.arrays["flash"]
    # Other arrays: child entries replace base entries with the same key.
    for section in ("dma", "serial", "devices", "storage", "actuators"):
        key = ARRAY_SECTIONS[section]
        child_keys = {key_of(e, key) for e in arrays[section]}
        merged[section] = [e for e in base.arrays[section] if key_of(e, key) not in child_keys] + arrays[section]

    merged_tables = dict(base.tables)
    for section, entry in tables.items():
        if section == "defaults" and "defaults" in base.tables \
                and isinstance(entry.data, dict) and isinstance(base.tables["defaults"].data, dict):
            merged_tables[section] = _Entry({**base.tables["defaults"].data, **entry.data}, entry.loc)
        else:
            merged_tables[section] = entry
    return _Resolved(data.get("name"), data.get("mcu", base.mcu), merged, merged_tables)


def _key_line(text: str, key: str) -> int | None:
    pattern = re.compile(r"^\s*" + key + r"\s*=")
    return next((n for n, line in enumerate(text.splitlines(), start=1) if pattern.match(line)), None)


def load_board(path: Path, mcu_root: Path, display_path: str | None = None) -> Board:
    """Parse and validate a board definition, raising BoardError with every problem found."""
    shown = display_path or str(path)
    diag = Diagnostics(shown)
    resolved = _resolve(path, shown, diag, ((path.resolve(), shown),))
    if resolved is None:
        raise BoardError(diag.errors)
    name, mcu = resolved.name, resolved.mcu
    if not isinstance(name, str) or not name:
        diag.error("missing board 'name'")
    if not isinstance(mcu, str) or not mcu:
        diag.error("missing 'mcu'")
    if not isinstance(name, str) or not isinstance(mcu, str) or not mcu:
        raise BoardError(diag.errors)

    try:
        database = load_mcu_database(mcu_root, mcu)
    except FileNotFoundError:
        diag.error(f"unknown MCU '{mcu}': the MCU root has no {mcu}/mcu.json")
        raise BoardError(diag.errors) from None

    arrays, tables = resolved.arrays, resolved.tables
    flash = check_layout([e.data for e in arrays["flash"]], [e.loc for e in arrays["flash"]],
                         database["flash"], Loc(shown), diag)
    pins = [p for p in (_check_pin(e.data, e.loc, database, diag) for e in arrays["pins"]) if p]
    configs = sorted({config for p in pins for config, _ in p.alt})
    _check_unique(pins, diag)
    for config in configs:
        _check_config(pins, config, diag)

    # DMA covers the functions of every alt config, so any selection works at boot.
    funcs = sorted({f.func for p in pins for f in (p.default, *(fn for _, fn in p.alt)) if f.kind == "af"})
    requests = collect_requests([e.data for e in arrays["dma"]], [e.loc for e in arrays["dma"]],
                                funcs, database["dma"], diag)

    def table(section: str) -> _Entry:
        return tables.get(section, _Entry(None, Loc(shown)))

    hardware = check_hardware([(e.data, e.loc) for e in arrays["serial"]],
                              [(e.data, e.loc) for e in arrays["devices"]],
                              [(e.data, e.loc) for e in arrays["storage"]],
                              table("timing").data, table("defaults").data or {},
                              {p.role: p for p in pins}, Loc(shown), table("timing").loc,
                              table("defaults").loc, diag,
                              (table("revision").data, table("revision").loc),
                              (table("actuator_power").data, table("actuator_power").loc),
                              [(e.data, e.loc) for e in arrays["actuators"]],
                              (table("user_key").data, table("user_key").loc))
    clock = _check_clock(table("clock"), database, diag)
    used = {f.func.rsplit("_", 1)[0] for p in pins for f in (p.default, *(fn for _, fn in p.alt))
            if f.kind in ("af", "analog")}
    used.update(d.peripheral for d in hardware.devices if d.peripheral)
    peripherals = tuple((name, database["peripherals"][name]) for name in sorted(used)
                        if name in database.get("peripherals", {}))
    if diag:
        raise BoardError(diag.errors)
    dma = assign(requests, diag)
    if diag:
        raise BoardError(diag.errors)
    return Board(name=name, mcu=mcu, source=shown, pins=tuple(pins), dma=tuple(dma),
                 alt_configs=tuple(configs), flash=flash, ram=ram_regions(database),
                 hardware=hardware, clock=clock, peripherals=peripherals)


def _check_clock(entry: _Entry, database: dict, diag: Diagnostics) -> Clock | None:
    if entry.data is None:
        return None
    data = entry.data
    if not isinstance(data, dict) or set(data) != CLOCK_KEYS \
            or not all(isinstance(v, int) and not isinstance(v, bool) for v in data.values()):
        diag.error("'clock' needs exactly integer 'hse_hz' and 'sysclk_hz'", entry.loc)
        return None
    if database["family"] != "STM32F4":
        diag.error(f"clock solving supports STM32F4 only, not {database['family']}", entry.loc)
        return None
    result = solve_clock(data["hse_hz"], data["sysclk_hz"])
    if isinstance(result, str):
        diag.error(result, entry.loc)
        return None
    return result


def _function(pin: str, role: object, func: object, loc: Loc, database: dict,
              diag: Diagnostics) -> Function | None:
    if not isinstance(role, str) or not _ROLE_RE.match(role):
        diag.error(f"role '{role}' must be upper case letters, digits and underscores", loc)
        return None
    if func in GPIO_FUNCTIONS:
        return Function(role, func, func.lower(), None)
    if func in database.get("analog", {}).get(pin, []):
        return Function(role, func, "analog", None)
    afs = [e["af"] for e in database["af"][pin] if e["function"] == func]
    if not afs:
        available = ", ".join(e["function"] for e in database["af"][pin]) or "none"
        diag.error(f"function {func} is not available on pin {pin} (available: {available})", loc)
        return None
    return Function(role, func, "af", afs[0])


def _check_pin(entry: object, loc: Loc, database: dict, diag: Diagnostics) -> Pin | None:
    if not isinstance(entry, dict):
        diag.error("each pin must be an inline table", loc)
        return None
    ok = True
    for key in sorted(set(entry) - PIN_KEYS):
        diag.error(f"unknown key '{key}' in pin entry", loc)
        ok = False
    for key in sorted((PIN_KEYS - {"alt", "pull"}) - set(entry)):
        diag.error(f"pin entry is missing '{key}'", loc)
        ok = False
    if not ok:
        return None
    pin = entry["pin"]
    match = _PIN_RE.match(pin) if isinstance(pin, str) else None
    if match is None or pin not in database["pins"]:
        diag.error(f"pin {pin} does not exist on {database['mcu']} ({database['package']})", loc)
        return None
    default = _function(pin, entry["role"], entry["func"], loc, database, diag)

    alts: dict[int, Function] = {}
    raw_alts = entry.get("alt", [])
    if not isinstance(raw_alts, list):
        diag.error(f"'alt' of pin {pin} must be an array of inline tables", loc)
        raw_alts = []
    for raw in raw_alts:
        if not isinstance(raw, dict) or set(raw) != ALT_KEYS:
            diag.error(f"each alt config of pin {pin} needs exactly 'config', 'role' and 'func'", loc)
            continue
        config = raw["config"]
        if not isinstance(config, int) or isinstance(config, bool) or config < 1:
            diag.error(f"alt config number of pin {pin} must be an integer of at least 1", loc)
            continue
        if config in alts:
            diag.error(f"pin {pin} declares alt config {config} twice", loc)
            continue
        function = _function(pin, raw["role"], raw["func"], loc, database, diag)
        if function:
            alts[config] = function
    if default is None:
        return None
    pull = entry.get("pull")
    if pull is not None and pull not in PULLS:
        diag.error(f"pull of pin {pin} must be one of: {', '.join(PULLS)}, got {pull!r}", loc)
        return None
    if pull is not None and default.kind != "gpio_input":
        diag.error(f"pull of pin {pin} needs the GPIO_INPUT function, not {default.func}", loc)
        return None
    return Pin(pin=pin, port=match.group(1), number=int(match.group(2)), role=default.role,
               func=default.func, kind=default.kind, af=default.af, loc=loc,
               alt=tuple(sorted(alts.items())), pull=pull)


def _check_unique(pins: list[Pin], diag: Diagnostics) -> None:
    by_pin: dict[str, Pin] = {}
    for p in pins:
        if p.pin in by_pin:
            first = by_pin[p.pin]
            diag.error(f"pin {p.pin} is already assigned to role {first.role} ({first.loc.refer(p.loc)})", p.loc)
        else:
            by_pin[p.pin] = p
    _check_config(pins, 0, diag)


def _check_config(pins: list[Pin], config: int, diag: Diagnostics) -> None:
    """Roles and peripheral signals must be unique among the pins of one config.

    Conflicts between two default functions are reported once, under config 0.
    """
    in_config = "" if config == 0 else f" in alt config {config}"

    def report(first: Pin, p: Pin) -> bool:
        return config == 0 or first.in_config(config) != first.default or p.in_config(config) != p.default

    by_role: dict[str, Pin] = {}
    by_signal: dict[str, Pin] = {}
    for p in pins:
        f = p.in_config(config)
        first = by_role.setdefault(f.role, p)
        if first is not p and report(first, p):
            diag.error(f"role {f.role} is already defined{in_config} ({first.loc.refer(p.loc)})", p.loc)
        # A peripheral signal (for example a timer channel) can drive only one pin.
        if f.kind == "af":
            first = by_signal.setdefault(f.func, p)
            if first is not p and report(first, p):
                diag.error(f"signal {f.func} is already used by pin {first.pin} for role "
                           f"{first.in_config(config).role}{in_config} ({first.loc.refer(p.loc)})", p.loc)
