"""Load and validate parameter definitions (ADR-0012).

Each module or driver with settings has a ``params.toml`` next to it::

    # Parameters of the attitude estimator (ADR-0014).
    prefix = "ATT"
    kconfig = "ESTIMATOR_MAHONY"   # compiled in only when this symbol is set

    [[param]]
    name = "ATT_KP"
    type = "FLOAT"                 # FLOAT or INT32
    default = 1.0
    min = 0.0
    max = 10.0
    unit = "1/s"
    description = "Attitude correction gain of the Mahony filter."

An INT32 parameter may instead list enum ``values`` (``[{ name = "NONE",
value = 0 }, ...]``, the default given by name) or bitmask ``bits`` (``[{
name = "LINK", bit = 0 }, ...]``); a boolean is an enum with two values. Enum
values may carry their own ``kconfig`` symbol. ``reboot_required`` and
``locked_when_armed`` are the safety attributes.

Every definition in the repository is checked, compiled in or not: prefixes,
names, hash collisions and defaults. Only the compiled-in ones are emitted.
"""

from __future__ import annotations

import hashlib
import json
import keyword
import math
import re
import tomllib
from dataclasses import dataclass, field
from pathlib import Path

from vkboard.diagnostics import Diagnostic, Diagnostics, Loc
from vkschema.schema import CPP_KEYWORDS

TYPES = ("INT32", "FLOAT")
MAX_NAME_LENGTH = 16  # MAVLink parameter protocol (ADR-0012)
INT32_MIN, INT32_MAX = -(1 << 31), (1 << 31) - 1

TOP_LEVEL_KEYS = {"prefix", "kconfig", "param"}
PARAM_KEYS = {"name", "type", "default", "min", "max", "unit", "description", "values", "bits",
              "reboot_required", "locked_when_armed"}
VALUE_KEYS = {"name", "value", "kconfig"}
BIT_KEYS = {"name", "bit"}

_PREFIX_RE = re.compile(r"[A-Z][A-Z0-9]*\Z")
_NAME_RE = re.compile(r"[A-Z][A-Z0-9_]*\Z")
_SYMBOL_RE = re.compile(r"[A-Z][A-Z0-9_]*\Z")
_TOML_POSITION_RE = re.compile(r"\s*\(at line (\d+), column \d+\)")


def name_hash(name: str) -> int:
    """FNV-1a, 32 bits, over the name's ASCII bytes: the key of stored values."""
    h = 0x811C9DC5
    for byte in name.encode("ascii"):
        h ^= byte
        h = (h * 0x01000193) & 0xFFFFFFFF
    return h


@dataclass(frozen=True)
class EnumValue:
    name: str
    value: int
    kconfig: str | None


@dataclass(frozen=True)
class Bit:
    name: str
    bit: int


@dataclass
class Param:
    name: str
    type: str
    default: int | float
    minimum: int | float
    maximum: int | float
    unit: str | None
    description: str
    values: list[EnumValue]
    bits: list[Bit]
    reboot_required: bool
    locked_when_armed: bool
    loc: Loc
    member: str = ""
    default_name: str | None = None  # enum default as written

    @property
    def hash(self) -> int:
        return name_hash(self.name)

    @property
    def is_enum(self) -> bool:
        return bool(self.values)

    @property
    def is_bitmask(self) -> bool:
        return bool(self.bits)


@dataclass
class Group:
    prefix: str
    kconfig: str | None
    source: str
    doc: list[str]
    params: list[Param] = field(default_factory=list)
    prefix_line: int | None = None

    @property
    def type_name(self) -> str:
        return self.prefix.capitalize()

    @property
    def instance(self) -> str:
        return self.prefix.lower()


class ParamsError(Exception):
    """Raised when parameter definitions cannot be processed."""

    def __init__(self, diagnostics: list[Diagnostic]):
        super().__init__("\n".join(map(str, diagnostics)))
        self.diagnostics = diagnostics


def read_config(path: Path) -> set[str]:
    """The Kconfig symbols set to y in a solved .config."""
    enabled = set()
    for line in path.read_text(encoding="utf-8").splitlines():
        key, _, value = line.partition("=")
        if key.startswith("CONFIG_") and value == "y":
            enabled.add(key.removeprefix("CONFIG_"))
    return enabled


def _key_lines(text: str, key: str) -> list[int]:
    """Lines where ``key = ...`` starts, outside comments."""
    pattern = re.compile(r"^\s*" + key + r"\s*=")
    return [n for n, line in enumerate(text.splitlines(), start=1) if pattern.match(line.split("#", 1)[0])]


def _doc(text: str) -> list[str]:
    """The comment lines that open the file."""
    doc = []
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped.startswith("#"):
            break
        doc.append(stripped.lstrip("#").strip())
    return doc


def _is_int(value) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _is_number(value) -> bool:
    return (_is_int(value) or isinstance(value, float)) and math.isfinite(value)


def _member(prefix: str, name: str) -> str:
    rest = name.removeprefix(prefix + "_")
    member = rest.lower() if rest != name and not rest[:1].isdigit() else name.lower()
    if member in CPP_KEYWORDS or keyword.iskeyword(member):
        member = name.lower()
    return member


def _symbol(value, what: str, loc: Loc, diag: Diagnostics) -> str | None:
    if value is None:
        return None
    if not isinstance(value, str) or not _SYMBOL_RE.match(value):
        diag.error(f"{what} must name a Kconfig symbol without CONFIG_, got {value!r}", loc)
        return None
    return value


def _parse_values(entry: dict, loc: Loc, diag: Diagnostics) -> list[EnumValue] | None:
    raw = entry["values"]
    if not isinstance(raw, list) or not raw:
        diag.error("'values' must be a non-empty array of { name, value } tables", loc)
        return None
    values: list[EnumValue] = []
    for item in raw:
        if not isinstance(item, dict) or set(item) - VALUE_KEYS or not {"name", "value"} <= set(item):
            diag.error("each enum value is { name = \"...\", value = <int> } with an optional kconfig", loc)
            return None
        name, value = item["name"], item["value"]
        if not isinstance(name, str) or not _NAME_RE.match(name):
            diag.error(f"enum value name {name!r} must be UPPER_SNAKE_CASE", loc)
            return None
        if not _is_int(value) or not INT32_MIN <= value <= INT32_MAX:
            diag.error(f"enum value {name} must be a 32-bit integer, got {value!r}", loc)
            return None
        if any(v.name == name for v in values):
            diag.error(f"enum value name {name} is listed twice", loc)
            return None
        if any(v.value == value for v in values):
            diag.error(f"enum value {value} is listed twice", loc)
            return None
        values.append(EnumValue(name, value, _symbol(item.get("kconfig"), f"kconfig of {name}", loc, diag)))
    return values


def _parse_bits(entry: dict, loc: Loc, diag: Diagnostics) -> list[Bit] | None:
    raw = entry["bits"]
    if not isinstance(raw, list) or not raw:
        diag.error("'bits' must be a non-empty array of { name, bit } tables", loc)
        return None
    bits: list[Bit] = []
    for item in raw:
        if not isinstance(item, dict) or set(item) != BIT_KEYS:
            diag.error("each bit is { name = \"...\", bit = <0..31> }", loc)
            return None
        name, bit = item["name"], item["bit"]
        if not isinstance(name, str) or not _NAME_RE.match(name):
            diag.error(f"bit name {name!r} must be UPPER_SNAKE_CASE", loc)
            return None
        if not _is_int(bit) or not 0 <= bit <= 31:
            diag.error(f"bit {name} must be a bit number from 0 to 31, got {bit!r}", loc)
            return None
        if any(b.name == name or b.bit == bit for b in bits):
            diag.error(f"bit {name} ({bit}) is listed twice", loc)
            return None
        bits.append(Bit(name, bit))
    return bits


def _parse_param(entry, prefix: str, loc: Loc, diag: Diagnostics) -> Param | None:
    if not isinstance(entry, dict):
        diag.error("each [[param]] must be a table", loc)
        return None
    errors_before = len(diag.errors)
    for key in sorted(set(entry) - PARAM_KEYS):
        diag.error(f"unknown key '{key}'; expected one of: {', '.join(sorted(PARAM_KEYS))}", loc)
    name = entry.get("name")
    if not isinstance(name, str) or not _NAME_RE.match(name):
        diag.error(f"parameter name {name!r} must be UPPER_SNAKE_CASE", loc)
        return None
    if len(name) > MAX_NAME_LENGTH:
        diag.error(f"parameter name {name} has {len(name)} characters; at most {MAX_NAME_LENGTH} "
                   "fit the MAVLink parameter protocol (ADR-0012)", loc)
    rest = name[len(prefix):]
    if not name.startswith(prefix) or not rest[:1] or not (rest[0] == "_" or rest[0].isdigit()):
        diag.error(f"parameter name {name} must start with the prefix {prefix}_ (or {prefix} and a digit)", loc)
    type_ = entry.get("type")
    if type_ not in TYPES:
        diag.error(f"{name}: type must be one of {', '.join(TYPES)}, got {type_!r}", loc)
        return None
    description = entry.get("description")
    if not isinstance(description, str) or not description.strip():
        diag.error(f"{name}: a description is required", loc)
    unit = entry.get("unit")
    if unit is not None and (not isinstance(unit, str) or not unit):
        diag.error(f"{name}: unit must be a non-empty string", loc)
    flags = {}
    for key in ("reboot_required", "locked_when_armed"):
        flags[key] = entry.get(key, False)
        if not isinstance(flags[key], bool):
            diag.error(f"{name}: {key} must be true or false", loc)
    if "default" not in entry:
        diag.error(f"{name}: a default is required", loc)
        return None
    default = entry["default"]

    values: list[EnumValue] = []
    bits: list[Bit] = []
    default_name = None
    if "values" in entry or "bits" in entry:
        if type_ != "INT32":
            diag.error(f"{name}: only INT32 parameters may have enum values or bits", loc)
            return None
        if "values" in entry and "bits" in entry:
            diag.error(f"{name}: a parameter has either enum values or bits, not both", loc)
            return None
        if "min" in entry or "max" in entry:
            diag.error(f"{name}: an enum or bitmask takes its range from its values; drop min and max", loc)
        if "values" in entry:
            parsed = _parse_values(entry, loc, diag)
            if parsed is None:
                return None
            values = parsed
            if not isinstance(default, str):
                diag.error(f"{name}: the default of an enum is a value name, got {default!r}", loc)
                return None
            match = next((v for v in values if v.name == default), None)
            if match is None:
                diag.error(f"{name}: default {default} is not one of its values: "
                           f"{', '.join(v.name for v in values)}", loc)
                return None
            default_name, default = default, match.value
            minimum, maximum = min(v.value for v in values), max(v.value for v in values)
        else:
            parsed_bits = _parse_bits(entry, loc, diag)
            if parsed_bits is None:
                return None
            bits = parsed_bits
            mask = sum(1 << b.bit for b in bits)
            if not _is_int(default) or default < 0 or default & ~mask:
                diag.error(f"{name}: default {default!r} sets bits that are not listed", loc)
                return None
            minimum, maximum = 0, mask
    else:
        if "min" not in entry or "max" not in entry:
            diag.error(f"{name}: min and max are required", loc)
            return None
        minimum, maximum = entry["min"], entry["max"]
        checks = [("default", default), ("min", minimum), ("max", maximum)]
        if type_ == "INT32":
            for key, value in checks:
                if not _is_int(value) or not INT32_MIN <= value <= INT32_MAX:
                    diag.error(f"{name}: {key} must be a 32-bit integer, got {value!r}", loc)
                    return None
        else:
            for key, value in checks:
                if not _is_number(value):
                    diag.error(f"{name}: {key} must be a finite number, got {value!r}", loc)
                    return None
            default, minimum, maximum = float(default), float(minimum), float(maximum)
        if minimum > maximum:
            diag.error(f"{name}: min {minimum} is above max {maximum}", loc)
            return None
        if not minimum <= default <= maximum:
            diag.error(f"{name}: default {default} is outside its range [{minimum}, {maximum}]", loc)
            return None

    if len(diag.errors) != errors_before:
        return None
    return Param(name=name, type=type_, default=default, minimum=minimum, maximum=maximum, unit=unit,
                 description=description.strip(), values=values, bits=bits,
                 reboot_required=flags["reboot_required"], locked_when_armed=flags["locked_when_armed"],
                 loc=loc, member=_member(prefix, name), default_name=default_name)


def parse_file(path: Path, display: str, diag: Diagnostics) -> Group | None:
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as error:
        diag.error(error.strerror or str(error))
        return None
    try:
        data = tomllib.loads(text)
    except tomllib.TOMLDecodeError as error:
        message = str(error)
        match = _TOML_POSITION_RE.search(message)
        diag.error(_TOML_POSITION_RE.sub("", message),
                   Loc(display, int(match.group(1))) if match else None)
        return None

    errors_before = len(diag.errors)
    for key in sorted(set(data) - TOP_LEVEL_KEYS):
        diag.error(f"unknown key '{key}'; expected one of: {', '.join(sorted(TOP_LEVEL_KEYS))}")
    prefix_lines = _key_lines(text, "prefix")
    prefix_loc = Loc(display, prefix_lines[0] if prefix_lines else None)
    prefix = data.get("prefix")
    if not isinstance(prefix, str) or not _PREFIX_RE.match(prefix):
        diag.error(f"prefix must be upper case letters and digits, got {prefix!r}", prefix_loc)
        return None
    kconfig = _symbol(data.get("kconfig"), "kconfig", Loc(display, (_key_lines(text, "kconfig") or [None])[0]),
                      diag)
    entries = data.get("param", [])
    if not isinstance(entries, list) or not entries:
        diag.error("a parameter definition file needs at least one [[param]]")
        return None
    name_lines = _key_lines(text, "name")
    # [[param]] tables keep 'name' on its own line; enum values and bits are
    # inline tables, so their 'name' never starts a line.
    lines = name_lines if len(name_lines) == len(entries) else [None] * len(entries)
    group = Group(prefix=prefix, kconfig=kconfig, source=display, doc=_doc(text), prefix_line=prefix_loc.line)
    for entry, line in zip(entries, lines):
        param = _parse_param(entry, prefix, Loc(display, line), diag)
        if param is None:
            continue
        for other in group.params:
            if other.name == param.name:
                diag.error(f"parameter {param.name} is already defined on line {other.loc.line}", param.loc)
                break
            if other.member == param.member:
                diag.error(f"parameters {other.name} and {param.name} map to the same member '{param.member}'",
                           param.loc)
                break
        else:
            group.params.append(param)
    return group if len(diag.errors) == errors_before else None


def load_definitions(paths: list[Path]) -> list[Group]:
    """Parse and cross-check every definition file, reporting all errors at once.

    Sorted by prefix.
    """
    errors: list[Diagnostic] = []
    groups: list[Group] = []
    for path in paths:
        display = str(path)
        diag = Diagnostics(display)
        group = parse_file(path, display, diag)
        errors += diag.errors
        if group is not None:
            groups.append(group)

    by_prefix: dict[str, Group] = {}
    for group in groups:
        other = by_prefix.get(group.prefix)
        if other is not None:
            errors.append(Diagnostic(group.source, group.prefix_line,
                                     f"prefix {group.prefix} is already declared in {other.source}"))
            continue
        for known in by_prefix.values():
            short, long_ = sorted((known, group), key=lambda g: len(g.prefix))
            if long_.prefix.startswith(short.prefix):
                errors.append(Diagnostic(group.source, group.prefix_line,
                                         f"prefix {group.prefix} overlaps prefix {known.prefix} of {known.source}"))
        by_prefix[group.prefix] = group

    seen: dict[int, Param] = {}
    for group in groups:
        for param in group.params:
            other = seen.get(param.hash)
            if other is not None:
                kind = "is already defined" if other.name == param.name else \
                    f"has the same name hash 0x{param.hash:08X} as {other.name}"
                errors.append(Diagnostic(param.loc.path, param.loc.line,
                                         f"parameter {param.name} {kind} ({other.loc.path}:{other.loc.line})"))
            else:
                seen[param.hash] = param
    if errors:
        raise ParamsError(errors)
    return sorted(groups, key=lambda g: g.prefix)


def compiled_in(groups: list[Group], enabled: set[str] | None) -> list[Group]:
    """The groups and enum values the build contains; None keeps everything.

    An enum whose default is not compiled in is an error.
    """
    if enabled is None:
        return groups
    errors: list[Diagnostic] = []
    result = []
    for group in groups:
        if group.kconfig is not None and group.kconfig not in enabled:
            continue
        params = []
        for param in group.params:
            if param.is_enum:
                values = [v for v in param.values if v.kconfig is None or v.kconfig in enabled]
                if not any(v.value == param.default for v in values):
                    errors.append(Diagnostic(param.loc.path, param.loc.line,
                                             f"{param.name}: default {param.default_name} is not compiled in"))
                    continue
                param = Param(**{**param.__dict__, "values": values,
                                 "minimum": min(v.value for v in values),
                                 "maximum": max(v.value for v in values)})
            params.append(param)
        result.append(Group(group.prefix, group.kconfig, group.source, group.doc, params, group.prefix_line))
    if errors:
        raise ParamsError(errors)
    return result


def all_params(groups: list[Group]) -> list[Param]:
    """Every parameter, sorted by name: the order of the parameter table."""
    return sorted((p for g in groups for p in g.params), key=lambda p: p.name)


def metadata(param: Param) -> dict:
    entry = {
        "name": param.name,
        "hash": param.hash,
        "type": param.type,
        "default": param.default,
        "min": param.minimum,
        "max": param.maximum,
        "unit": param.unit,
        "description": param.description,
        "reboot_required": param.reboot_required,
        "locked_when_armed": param.locked_when_armed,
    }
    if param.is_enum:
        entry["values"] = [{"name": v.name, "value": v.value} for v in param.values]
    if param.is_bitmask:
        entry["bits"] = [{"name": b.name, "bit": b.bit} for b in param.bits]
    return entry


def set_hash(params: list[Param]) -> int:
    """First 64 bits of SHA-256 over the canonical metadata, sorted by name (ADR-0012)."""
    canonical = json.dumps([metadata(p) for p in params], sort_keys=True, separators=(",", ":"))
    return int.from_bytes(hashlib.sha256(canonical.encode()).digest()[:8], "big")
