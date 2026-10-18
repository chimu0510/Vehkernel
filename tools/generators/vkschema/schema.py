"""Parse and validate .msg files.

A .msg file defines one message; its file name is the topic name. The syntax
follows ROS .msg files, with annotations on their own lines::

    # IMU sample from one device, in the body frame.
    @queue 4
    @rate 200
    uint64 timestamp            # time of sampling
    float32[3] accel  @unit m/s^2

Message annotations are ``@queue N`` (a queue of N samples instead of the
latest value), ``@instances N`` and ``@rate HZ`` (nominal publish rate). A
field may carry ``@unit <unit>``. ``#`` starts a comment; comment lines before
the first field document the message.
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

NAME = re.compile(r"[a-z][a-z0-9_]*\Z")
FIELD = re.compile(r"(?P<type>[a-z0-9]+)(?:\[(?P<len>[^\]]*)\])?\s+(?P<name>\S+)(?:\s+@unit\s+(?P<unit>\S+))?\Z")
ANNOTATION = re.compile(r"@(?P<key>\S+)(?:\s+(?P<value>\S+))?\Z")
MAX_QUEUE = 255
RESERVED_MESSAGES = {"registry"}
MAX_INSTANCES = 32


@dataclass(frozen=True)
class Field:
    type: str
    length: int | None  # None for a scalar
    name: str
    unit: str | None
    comment: str | None
    line: int

    @property
    def cpp_type(self) -> str:
        base = TYPES[self.type]
        return base if self.length is None else f"std::array<{base}, {self.length}>"

    @property
    def schema_type(self) -> str:
        return self.type if self.length is None else f"{self.type}[{self.length}]"


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
        return "".join(part.capitalize() for part in self.name.split("_"))

    @property
    def schema_hash(self) -> int:
        """64-bit hash of the definition as it appears on the wire: the message
        name and each field's type and name, in order. Comments, units and
        MCU-side annotations do not change it."""
        canonical = "\n".join([self.name] + [f"{f.schema_type} {f.name}" for f in self.fields])
        return int.from_bytes(hashlib.sha256(canonical.encode()).digest()[:8], "big")


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


def _parse_field(match: re.Match, comment: str | None, loc: Loc, message: Message,
                 diag: Diagnostics) -> None:
    type_, length_text, name, unit = match["type"], match["len"], match["name"], match["unit"]
    ok = True
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
        message.fields.append(Field(type_, length, name, unit, comment, loc.line))


def parse_message(path: Path, display: str, diag: Diagnostics) -> Message | None:
    name = path.stem
    message = Message(name=name, source=display)
    if not NAME.match(name):
        diag.error(f"message file name '{path.name}' must be lower_snake_case with the .msg suffix")
    elif name in RESERVED_MESSAGES:
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
            if comment is not None and first_field_line is None and not message.fields:
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

    first = message.fields[0] if message.fields else None
    if first_field_line is None:
        diag.error("message has no fields; the first field must be 'uint64 timestamp' (ADR-0006)")
    elif first is not None and first.line == first_field_line and (
            first.name != "timestamp" or first.schema_type != "uint64"):
        diag.error(f"the first field must be 'uint64 timestamp' (ADR-0006), got "
                   f"'{first.schema_type} {first.name}'", Loc(display, first.line))
    return message if len(diag.errors) == errors_before else None


def load_schema(paths: list[Path]) -> list[Message]:
    """Parse every .msg file, reporting all errors at once. Sorted by name."""
    errors: list[Diagnostic] = []
    messages: dict[str, Message] = {}
    for path in paths:
        display = str(path)
        diag = Diagnostics(display)
        message = parse_message(path, display, diag)
        errors += diag.errors
        if message is None:
            continue
        if message.name in messages:
            errors.append(Diagnostic(display, None, f"message '{message.name}' is already defined in "
                                                    f"{messages[message.name].source}"))
            continue
        messages[message.name] = message
    if errors:
        raise SchemaError(errors)
    return [messages[name] for name in sorted(messages)]
