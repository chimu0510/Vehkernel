# SPDX-License-Identifier: GPL-3.0-or-later
"""Parse, validate and calculate bandwidth for log profiles (spec 0011, ADR-0019)."""

from __future__ import annotations

import re
import tomllib
from dataclasses import dataclass, field
from pathlib import Path

from vkboard.diagnostics import Diagnostics
from .schema import Message, Schema, type_name

PROFILE_NAME = re.compile(r"[a-z][a-z0-9_]*\Z")
KCONFIG_NAME = re.compile(r"[A-Z][A-Z0-9_]*\Z")

# A per-sample topic without @rate runs at the board's base rate (200 Hz on the
# C50C, timing.base_rate_hz in boards/c50c-v1_1/board.toml), and the Link checks
# the real bandwidth when logging starts (spec 0011, a later ticket).
DEFAULT_BASE_RATE_HZ = 200.0


def wire_bytes(encoded_size: int) -> int:
    """Bytes on the wire for a topic data frame with Link COBS framing (ADR-0002)."""
    # Header: version(1) + type(1) + seq(2) = 4
    # Body: topic_id(2) + instance(1) + encoded_size
    # CRC: 2
    frame = 4 + 3 + encoded_size + 2
    cobs = frame + frame // 254 + 1
    return cobs + 1  # trailing zero delimiter


@dataclass(frozen=True)
class ProfileEntry:
    topic: str
    rate: float | str  # float Hz, or "every_sample", or "as_published"
    instance: int | None = None  # None means all instances
    kconfig: str | None = None

    @property
    def rate_kind(self) -> str:
        if self.rate == "every_sample":
            return "every_sample"
        if self.rate == "as_published":
            return "as_published"
        return "rate_hz"

    def rate_hz(self, message: Message) -> float:
        if self.rate == "every_sample":
            return message.rate_hz if message.rate_hz > 0 else DEFAULT_BASE_RATE_HZ
        if self.rate == "as_published":
            return message.rate_hz
        return float(self.rate)

    def instance_count(self, message: Message) -> int:
        return 1 if self.instance is not None else message.instances

    def bandwidth(self, message: Message) -> float:
        return self.rate_hz(message) * wire_bytes(message.encoded_size) * self.instance_count(message)


@dataclass
class Profile:
    name: str
    description: str
    entries: list[ProfileEntry] = field(default_factory=list)
    source: str = ""

    @property
    def type_name(self) -> str:
        return type_name(self.name)

    def bandwidth(self, schema: Schema, kconfig: set[str] | None = None) -> float:
        """Total bandwidth on the wire in bytes per second for active kconfig symbols."""
        msgs = {m.name: m for m in schema.messages}
        active = kconfig if kconfig is not None else set()
        total = 0.0
        for entry in self.entries:
            if entry.kconfig is not None and entry.kconfig not in active:
                continue
            msg = msgs.get(entry.topic)
            if msg is not None:
                total += entry.bandwidth(msg)
        return total

    def bandwidth_c50c(self, schema: Schema) -> float:
        return self.bandwidth(schema, kconfig=set())

    def bandwidth_sitl(self, schema: Schema) -> float:
        return self.bandwidth(schema, kconfig={"PLATFORM_SITL"})


def load_profiles(path: Path, schema: Schema, diag: Diagnostics) -> list[Profile] | None:
    """Parse a log_profiles.toml file and check every entry against the schema."""
    display = str(path)
    try:
        text = path.read_text(encoding="utf-8")
        data = tomllib.loads(text)
    except OSError as error:
        diag.error(error.strerror or str(error))
        return None
    except tomllib.TOMLDecodeError as error:
        diag.error(f"cannot parse TOML: {error}")
        return None

    msgs: dict[str, Message] = {m.name: m for m in schema.messages}
    profiles_dict: dict[str, dict] = data
    errors_before = len(diag.errors)
    profiles: list[Profile] = []

    for name, content in profiles_dict.items():
        if not isinstance(content, dict):
            continue
        if not PROFILE_NAME.match(name):
            diag.error(f"profile name '{name}' must be lower_snake_case")
            continue

        desc = str(content.get("description", ""))
        entries_data = content.get("entries")
        if not isinstance(entries_data, list):
            diag.error(f"profile '{name}': entries must be an array of tables")
            continue

        entries: list[ProfileEntry] = []
        for i, item in enumerate(entries_data):
            if not isinstance(item, dict):
                diag.error(f"profile '{name}': entry {i} must be a table")
                continue

            unknown_keys = set(item) - {"topic", "rate", "instance", "kconfig"}
            if unknown_keys:
                diag.error(f"profile '{name}': unknown entry keys: {', '.join(sorted(unknown_keys))}")

            topic = item.get("topic")
            if not topic or not isinstance(topic, str):
                diag.error(f"profile '{name}': entry missing 'topic'")
                continue

            msg = msgs.get(topic)
            if msg is None:
                diag.error(f"profile '{name}': unknown topic '{topic}'")
                continue

            rate = item.get("rate")
            rate_val: float | str
            if rate == "every_sample":
                if msg.queue <= 1:
                    diag.error(f"profile '{name}': topic '{topic}' has @queue {msg.queue}; every_sample needs @queue")
                rate_val = "every_sample"
            elif rate == "as_published":
                rate_val = "as_published"
            elif isinstance(rate, (int, float)) and not isinstance(rate, bool) and rate > 0:
                rate_val = float(rate)
            else:
                diag.error(f"profile '{name}': topic '{topic}' has malformed rate '{rate}'; "
                           f"expected a positive frequency in Hz, 'every_sample' or 'as_published'")
                continue

            instance = item.get("instance")
            if instance is not None:
                if not isinstance(instance, int) or isinstance(instance, bool) or instance < 0 or instance >= msg.instances:
                    diag.error(f"profile '{name}': topic '{topic}' has invalid instance '{instance}'; "
                               f"must be 0 <= instance < {msg.instances}")
                    continue

            kconfig = item.get("kconfig")
            if kconfig is not None:
                if not isinstance(kconfig, str) or kconfig.startswith("CONFIG_") or not KCONFIG_NAME.match(kconfig):
                    diag.error(f"profile '{name}': topic '{topic}' has malformed kconfig '{kconfig}'; "
                               f"expected UPPER_SNAKE_CASE symbol without CONFIG_ prefix")
                    continue

            entries.append(ProfileEntry(
                topic=topic,
                rate=rate_val,
                instance=instance,
                kconfig=kconfig,
            ))

        profiles.append(Profile(name=name, description=desc, entries=entries, source=display))

    return profiles if len(diag.errors) == errors_before else None
