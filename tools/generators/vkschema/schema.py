# SPDX-License-Identifier: GPL-3.0-or-later
"""Parse and validate .msg and .enum files.

A .msg file defines one message; its file name is the topic name. The syntax
follows ROS .msg files, with annotations on their own lines::

    # IMU sample from one device, in the body frame.
    @queue 4
    @rate 200
    uint64 timestamp            # time of sampling
    float32[3] accel  @unit m/s^2
    uint8 mode  @enum mode

Message annotations are ``@queue N`` (a queue of N samples instead of the
latest value), ``@instances N`` and ``@rate HZ`` (nominal publish rate). A
field may carry ``@unit <unit>`` and ``@enum <enum>``, the name of the enum
whose values it takes; the field's integer type must hold every one of them.

A .enum file defines one enum; its file name is the enum name. ``@type``
gives the integer type of its values, and each value takes a line of its
own, with an optional short name for narrow screens::

    # Why the vehicle last disarmed.
    @type uint8
    NONE = 0              @short -
    EMERGENCY_STOP = 2    @short ESTOP   # engaged for 5 s while armed

``#`` starts a comment; comment lines before the first field or value document
the message or the enum, except ``SPDX-`` license tags.
"""

from __future__ import annotations

import hashlib
import keyword
import re
from dataclasses import dataclass, field
from pathlib import Path

from vkboard.diagnostics import Diagnostic, Diagnostics, Loc

# Schema type -> C++ type.
TYPES = {
    "bool": "bool",
    "int8": "std::int8_t",
    "int16": "std::int16_t",
    "int32": "std::int32_t",
    "int64": "std::int64_t",
    "uint8": "std::uint8_t",
    "uint16": "std::uint16_t",
    "uint32": "std::uint32_t",
    "uint64": "std::uint64_t",
    "float32": "float",
    "float64": "double",
}

# Schema type -> (bytes on the wire, Python struct code). The custom protocol
# packs fields little-endian with no padding (ADR-0006).
WIRE = {
    "bool": (1, "?"),
    "int8": (1, "b"),
    "int16": (2, "h"),
    "int32": (4, "i"),
    "int64": (8, "q"),
    "uint8": (1, "B"),
    "uint16": (2, "H"),
    "uint32": (4, "I"),
    "uint64": (8, "Q"),
    "float32": (4, "f"),
    "float64": (8, "d"),
}

# Largest encoded message a Link frame carries.
MAX_ENCODED_SIZE = 240

# C++ keywords a field name must not collide with (Python's are checked too,
# since tool-side parsers are generated from the same schema).
CPP_KEYWORDS = {
    "alignas", "alignof", "and", "asm", "auto", "bool", "break", "case", "catch", "char",
    "class", "concept", "const", "consteval", "constexpr", "constinit", "continue",
    "co_await", "co_return", "co_yield", "decltype", "default", "delete", "do", "double",
    "else", "enum", "explicit", "export", "extern", "false", "float", "for", "friend",
    "goto", "if", "inline", "int", "long", "mutable", "namespace", "new", "noexcept", "not",
    "nullptr", "operator", "or", "private", "protected", "public", "register", "requires",
    "return", "short", "signed", "sizeof", "static", "struct", "switch", "template", "this",
    "throw", "true", "try", "typedef", "typename", "union", "unsigned", "using", "virtual",
    "void", "volatile", "while", "xor",
}

# Integer schema type -> the range of its values, for enums.
INTEGER_RANGES = {
    "int8": (-(1 << 7), (1 << 7) - 1),
    "int16": (-(1 << 15), (1 << 15) - 1),
    "int32": (-(1 << 31), (1 << 31) - 1),
    "int64": (-(1 << 63), (1 << 63) - 1),
    "uint8": (0, (1 << 8) - 1),
    "uint16": (0, (1 << 16) - 1),
    "uint32": (0, (1 << 32) - 1),
    "uint64": (0, (1 << 64) - 1),
}

NAME = re.compile(r"[a-z][a-z0-9_]*\Z")
VALUE_NAME = re.compile(r"[A-Z][A-Z0-9_]*\Z")
INTEGER = re.compile(r"-?[0-9]+\Z")
NOTES = r"(?P<notes>(?:\s+@\S+\s+\S+)*)"
FIELD = re.compile(r"(?P<type>[a-z0-9]+)(?:\[(?P<len>[^\]]*)\])?\s+(?P<name>\S+)" + NOTES + r"\Z")
VALUE = re.compile(r"(?P<name>[^\s=]+)\s*=\s*(?P<value>\S+)" + NOTES + r"\Z")
NOTE = re.compile(r"@(?P<key>\S+)\s+(?P<value>\S+)")
ANNOTATION = re.compile(r"@(?P<key>\S+)(?:\s+(?P<value>\S+))?\Z")
MAX_QUEUE = 255
# Names the generated files and declarations take for themselves.
RESERVED_NAMES = {"registry", "enums", "wire", "topics", "enum_value", "enum_values", "topic_list"}
MAX_INSTANCES = 32


def type_name(name: str) -> str:
    """The C++ and Python type of a message or an enum: imu_sample -> ImuSample."""
    return "".join(part.capitalize() for part in name.split("_"))


@dataclass(frozen=True)
class Field:
    type: str
    length: int | None  # None for a scalar
    name: str
    unit: str | None
    comment: str | None
    line: int
    enum: str | None = None  # name of the enum whose values the field takes

    @property
    def cpp_type(self) -> str:
        base = TYPES[self.type]
        return base if self.length is None else f"std::array<{base}, {self.length}>"

    @property
    def schema_type(self) -> str:
        return self.type if self.length is None else f"{self.type}[{self.length}]"

    @property
    def count(self) -> int:
        return 1 if self.length is None else self.length

    @property
    def wire_size(self) -> int:
        return WIRE[self.type][0] * self.count

    @property
    def struct_code(self) -> str:
        code = WIRE[self.type][1]
        return code if self.length is None else f"{self.length}{code}"


@dataclass
class Message:
    name: str
    source: str
    doc: list[str] = field(default_factory=list)
    fields: list[Field] = field(default_factory=list)
    queue: int = 1
    instances: int = 1
    rate_hz: float = 0.0

    @property
    def type_name(self) -> str:
        return type_name(self.name)

    @property
    def encoded_size(self) -> int:
        return sum(f.wire_size for f in self.fields)

    @property
    def schema_hash(self) -> int:
        """64-bit hash of the definition as it appears on the wire: the message
        name and each field's type and name, in order. Comments, units and
        MCU-side annotations do not change it, nor does the enum a field takes
        or that enum's values (ADR-0006)."""
        canonical = "\n".join([self.name] + [f"{f.schema_type} {f.name}" for f in self.fields])
        return int.from_bytes(hashlib.sha256(canonical.encode()).digest()[:8], "big")


@dataclass(frozen=True)
class EnumValue:
    name: str
    value: int
    short: str | None
    comment: str | None
    line: int

    @property
    def cpp_name(self) -> str:
        """BOARD_NOT_VERIFIED -> BoardNotVerified."""
        return type_name(self.name.lower())

    @property
    def short_name(self) -> str:
        return self.short or self.name


@dataclass
class Enum:
    name: str
    source: str
    doc: list[str] = field(default_factory=list)
    type: str | None = None
    values: list[EnumValue] = field(default_factory=list)

    @property
    def type_name(self) -> str:
        return type_name(self.name)

    @property
    def cpp_type(self) -> str:
        return TYPES[self.type or "uint8"]


@dataclass
class Schema:
    messages: list[Message]
    enums: list[Enum]


class SchemaError(Exception):
    """Raised when a schema cannot be processed."""

    def __init__(self, diagnostics: list[Diagnostic]):
        super().__init__("\n".join(map(str, diagnostics)))
        self.diagnostics = diagnostics


def _positive_int(value: str | None, maximum: int) -> int | None:
    if value is None or not value.isdigit():
        return None
    number = int(value)
    return number if 1 <= number <= maximum else None


def _parse_annotation(message: Message, match: re.Match, loc: Loc, seen: dict[str, int],
                      diag: Diagnostics) -> None:
    key, value = match["key"], match["value"]
    if key in seen:
        diag.error(f"@{key} is already set on line {seen[key]}", loc)
        return
    seen[key] = loc.line
    if key == "queue":
        number = _positive_int(value, MAX_QUEUE)
        if number is None:
            diag.error(f"@queue needs a length from 1 to {MAX_QUEUE}, got '{value or ''}'", loc)
        else:
            message.queue = number
    elif key == "instances":
        number = _positive_int(value, MAX_INSTANCES)
        if number is None:
            diag.error(f"@instances needs a count from 1 to {MAX_INSTANCES}, got '{value or ''}'", loc)
        else:
            message.instances = number
    elif key == "rate":
        try:
            rate = float(value or "")
        except ValueError:
            rate = 0.0
        if not rate > 0.0:
            diag.error(f"@rate needs a positive rate in Hz, got '{value or ''}'", loc)
        else:
            message.rate_hz = rate
    else:
        diag.error(f"unknown annotation @{key}; expected @queue, @instances or @rate", loc)


def _parse_notes(text: str, allowed: tuple[str, ...], loc: Loc, diag: Diagnostics) -> dict[str, str] | None:
    """The ``@key value`` annotations after a field or an enum value."""
    notes: dict[str, str] = {}
    ok = True
    for note in NOTE.finditer(text):
        key, value = note["key"], note["value"]
        if key not in allowed:
            expected = " or ".join(f"@{a}" for a in allowed)
            diag.error(f"unknown annotation @{key}; expected {expected}", loc)
            ok = False
        elif key in notes:
            diag.error(f"@{key} is given twice", loc)
            ok = False
        else:
            notes[key] = value
    return notes if ok else None


def _parse_field(match: re.Match, comment: str | None, loc: Loc, message: Message,
                 diag: Diagnostics) -> None:
    type_, length_text, name = match["type"], match["len"], match["name"]
    notes = _parse_notes(match["notes"], ("unit", "enum"), loc, diag)
    ok = notes is not None
    notes = notes or {}
    if type_ not in TYPES:
        diag.error(f"unknown type '{type_}'; expected one of: {', '.join(TYPES)}", loc)
        ok = False
    length = None
    if length_text is not None:
        length = _positive_int(length_text, 1 << 16)
        if length is None:
            diag.error(f"array length must be a positive integer, got '{length_text}'", loc)
            ok = False
    if not NAME.match(name):
        diag.error(f"field name '{name}' must be lower_snake_case", loc)
        ok = False
    elif name in CPP_KEYWORDS or keyword.iskeyword(name):
        diag.error(f"field name '{name}' is a reserved word", loc)
        ok = False
    else:
        for other in message.fields:
            if other.name == name:
                diag.error(f"field '{name}' is already defined on line {other.line}", loc)
                ok = False
    if ok:
        message.fields.append(Field(type_, length, name, notes.get("unit"), comment, loc.line,
                                    notes.get("enum")))


def parse_message(path: Path, display: str, diag: Diagnostics) -> Message | None:
    name = path.stem
    message = Message(name=name, source=display)
    if not NAME.match(name):
        diag.error(f"message file name '{path.name}' must be lower_snake_case with the .msg suffix")
    elif name in RESERVED_NAMES:
        diag.error(f"message name '{name}' is reserved for generated code")
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        diag.error(error.strerror or str(error))
        return None

    seen: dict[str, int] = {}
    errors_before = len(diag.errors)
    first_field_line = None
    for number, raw in enumerate(lines, start=1):
        loc = Loc(display, number)
        text, _, comment = raw.partition("#")
        text, comment = text.strip(), comment.strip() or None
        if not text:
            if (comment is not None and first_field_line is None and not message.fields
                    and not comment.startswith("SPDX-")):
                message.doc.append(comment)
            continue
        if text.startswith("@"):
            match = ANNOTATION.match(text)
            if match is None:
                diag.error(f"cannot parse annotation '{text}'", loc)
            else:
                _parse_annotation(message, match, loc, seen, diag)
            continue
        match = FIELD.match(text)
        if match is None:
            diag.error(f"cannot parse '{text}'; expected '<type> <name>' or '<type>[<length>] <name>'", loc)
            continue
        if first_field_line is None:
            first_field_line = number
        _parse_field(match, comment, loc, message, diag)

    if message.encoded_size > MAX_ENCODED_SIZE:
        diag.error(f"message encodes to {message.encoded_size} bytes; a Link frame carries at most "
                   f"{MAX_ENCODED_SIZE}", Loc(display, message.fields[-1].line))

    first = message.fields[0] if message.fields else None
    if first_field_line is None:
        diag.error("message has no fields; the first field must be 'uint64 timestamp' (ADR-0006)")
    elif first is not None and first.line == first_field_line and (
            first.name != "timestamp" or first.schema_type != "uint64"):
        diag.error(f"the first field must be 'uint64 timestamp' (ADR-0006), got "
                   f"'{first.schema_type} {first.name}'", Loc(display, first.line))
    return message if len(diag.errors) == errors_before else None


def _parse_value(match: re.Match, comment: str | None, loc: Loc, enum: Enum, diag: Diagnostics) -> None:
    name, value_text = match["name"], match["value"]
    notes = _parse_notes(match["notes"], ("short",), loc, diag)
    ok = notes is not None
    notes = notes or {}
    if not VALUE_NAME.match(name):
        diag.error(f"value name '{name}' must be UPPER_SNAKE_CASE", loc)
        ok = False
    elif keyword.iskeyword(name):
        diag.error(f"value name '{name}' is a reserved word", loc)
        ok = False
    if not INTEGER.match(value_text):
        diag.error(f"value of '{name}' must be a decimal integer, got '{value_text}'", loc)
        ok = False
    short = notes.get("short")
    if short is not None and not all(" " < c <= "~" and c not in "\"\\" for c in short):
        diag.error(f"short name '{short}' must be printable ASCII without quotes or backslashes", loc)
        ok = False
    if not ok:
        return
    value = int(value_text)
    for other in enum.values:
        if other.name == name:
            diag.error(f"value '{name}' is already defined on line {other.line}", loc)
            return
        if other.value == value:
            diag.error(f"value {value} is already taken by '{other.name}' on line {other.line}", loc)
            return
    enum.values.append(EnumValue(name, value, short, comment, loc.line))


def parse_enum(path: Path, display: str, diag: Diagnostics) -> Enum | None:
    name = path.stem
    enum = Enum(name=name, source=display)
    if not NAME.match(name):
        diag.error(f"enum file name '{path.name}' must be lower_snake_case with the .enum suffix")
    elif name in RESERVED_NAMES:
        diag.error(f"enum name '{name}' is reserved for generated code")
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        diag.error(error.strerror or str(error))
        return None

    errors_before = len(diag.errors)
    type_line = None
    for number, raw in enumerate(lines, start=1):
        loc = Loc(display, number)
        text, _, comment = raw.partition("#")
        text, comment = text.strip(), comment.strip() or None
        if not text:
            if comment is not None and not enum.values and not comment.startswith("SPDX-"):
                enum.doc.append(comment)
            continue
        if text.startswith("@"):
            match = ANNOTATION.match(text)
            if match is None:
                diag.error(f"cannot parse annotation '{text}'", loc)
            elif match["key"] != "type":
                diag.error(f"unknown annotation @{match['key']}; expected @type", loc)
            elif type_line is not None:
                diag.error(f"@type is already set on line {type_line}", loc)
            else:
                type_line = number
                if match["value"] in INTEGER_RANGES:
                    enum.type = match["value"]
                else:
                    diag.error(f"@type needs an integer type: {', '.join(INTEGER_RANGES)}; "
                               f"got '{match['value'] or ''}'", loc)
            continue
        match = VALUE.match(text)
        if match is None:
            diag.error(f"cannot parse '{text}'; expected '<NAME> = <value>'", loc)
            continue
        _parse_value(match, comment, loc, enum, diag)

    if type_line is None:
        diag.error("enum has no @type; it needs the integer type of its values")
    elif enum.type is not None:
        low, high = INTEGER_RANGES[enum.type]
        for value in enum.values:
            if not low <= value.value <= high:
                diag.error(f"value {value.value} of '{value.name}' does not fit @type {enum.type}",
                           Loc(display, value.line))
    if not enum.values and len(diag.errors) == errors_before:
        diag.error("enum has no values")
    return enum if len(diag.errors) == errors_before else None


def _check_enum_fields(message: Message, enums: dict[str, Enum], errors: list[Diagnostic]) -> None:
    for f in message.fields:
        if f.enum is None:
            continue
        enum = enums.get(f.enum)
        if enum is None:
            errors.append(Diagnostic(message.source, f.line, f"field '{f.name}' takes unknown enum '{f.enum}'"))
        elif f.type not in INTEGER_RANGES:
            errors.append(Diagnostic(message.source, f.line,
                                     f"field '{f.name}' takes enum '{f.enum}' but its type {f.type} is not "
                                     f"an integer type"))
        else:
            low, high = INTEGER_RANGES[f.type]
            for value in enum.values:
                if not low <= value.value <= high:
                    errors.append(Diagnostic(message.source, f.line,
                                             f"field '{f.name}' of type {f.type} cannot hold value "
                                             f"{value.value} of enum '{f.enum}' ({value.name})"))
                    break


def load_schema(paths: list[Path]) -> Schema:
    """Parse every .msg and .enum file, reporting all errors at once. Sorted by name."""
    errors: list[Diagnostic] = []
    messages: dict[str, Message] = {}
    enums: dict[str, Enum] = {}
    for path in paths:
        display = str(path)
        diag = Diagnostics(display)
        if path.suffix == ".enum":
            enum = parse_enum(path, display, diag)
            errors += diag.errors
            if enum is None:
                continue
            if enum.name in enums:
                errors.append(Diagnostic(display, None, f"enum '{enum.name}' is already defined in "
                                                        f"{enums[enum.name].source}"))
                continue
            enums[enum.name] = enum
            continue
        message = parse_message(path, display, diag)
        errors += diag.errors
        if message is None:
            continue
        if message.name in messages:
            errors.append(Diagnostic(display, None, f"message '{message.name}' is already defined in "
                                                    f"{messages[message.name].source}"))
            continue
        messages[message.name] = message
    for name in sorted(set(enums) & set(messages)):
        errors.append(Diagnostic(enums[name].source, None,
                                 f"enum '{name}' has the name of the message in {messages[name].source}"))
    for name in sorted(messages):
        _check_enum_fields(messages[name], enums, errors)
    if errors:
        raise SchemaError(errors)
    return Schema([messages[name] for name in sorted(messages)], [enums[name] for name in sorted(enums)])
