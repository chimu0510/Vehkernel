"""Internal Flash layout and memory regions (ADR-0020).

A board's ``flash`` array splits internal Flash into named regions, each a
whole number of sectors from the MCU database. The generator turns the layout
and the MCU's RAM regions into the linker script's MEMORY block, whose region
sizes are the budgets the linker enforces (ADR-0017).
"""

from __future__ import annotations

import re
from dataclasses import dataclass

from .diagnostics import Diagnostics, Loc

FLASH_KEYS = {"region", "start", "size"}
REGIONS = ("BOOT", "PARAMS", "EVENTLOG", "FIRMWARE")
REQUIRED_REGIONS = ("BOOT", "FIRMWARE")
_SIZE_RE = re.compile(r"^(\d+)([KM]?)$")


@dataclass(frozen=True)
class FlashRegion:
    name: str
    start: int
    size: int
    first_sector: int
    last_sector: int

    @property
    def end(self) -> int:
        return self.start + self.size


@dataclass(frozen=True)
class RamRegion:
    name: str  # "RAM" or "CCM"
    start: int
    size: int


def parse_size(value: object) -> int | None:
    if isinstance(value, int) and not isinstance(value, bool):
        return value if value > 0 else None
    if isinstance(value, str):
        match = _SIZE_RE.match(value)
        if match:
            size = int(match.group(1)) * {"": 1, "K": 1024, "M": 1024 * 1024}[match.group(2)]
            return size or None
    return None


def _sector_containing(sectors: list[dict], address: int) -> dict | None:
    return next((s for s in sectors if s["address"] <= address < s["address"] + s["size"]), None)


def _describe(sector: dict) -> str:
    return f"S{sector['index']} (0x{sector['address']:08X}-0x{sector['address'] + sector['size'] - 1:08X})"


def check_layout(raw: list, locs: list[Loc], flash: dict, top: Loc, diag: Diagnostics) -> tuple[FlashRegion, ...]:
    sectors = flash["sectors"]
    starts = {s["address"]: s for s in sectors}
    ends = {s["address"] + s["size"]: s for s in sectors}
    regions: list[tuple[FlashRegion, Loc]] = []
    seen: dict[str, Loc] = {}
    for entry, loc in zip(raw, locs):
        if not isinstance(entry, dict) or set(entry) != FLASH_KEYS:
            diag.error("each flash entry needs exactly 'region', 'start' and 'size'", loc)
            continue
        name, start, size = entry["region"], entry["start"], parse_size(entry["size"])
        if name not in REGIONS:
            diag.error(f"unknown Flash region '{name}' (expected one of: {', '.join(REGIONS)})", loc)
            continue
        if name in seen:
            diag.error(f"Flash region {name} is already defined ({seen[name].refer(loc)})", loc)
            continue
        seen[name] = loc
        if not isinstance(start, int) or isinstance(start, bool) or size is None:
            diag.error(f"Flash region {name} needs an integer 'start' and a positive 'size' "
                       f"such as 16384, \"16K\" or \"1M\"", loc)
            continue
        end = start + size
        if start < flash["base"] or end > flash["base"] + flash["size"]:
            diag.error(f"Flash region {name} (0x{start:08X}-0x{end - 1:08X}) lies outside Flash "
                       f"(0x{flash['base']:08X}-0x{flash['base'] + flash['size'] - 1:08X})", loc)
            continue
        if start not in starts:
            diag.error(f"Flash region {name} is not aligned to sectors: it starts inside sector "
                       f"{_describe(_sector_containing(sectors, start))}", loc)
            continue
        if end not in ends:
            diag.error(f"Flash region {name} is not aligned to sectors: it ends inside sector "
                       f"{_describe(_sector_containing(sectors, end - 1))}", loc)
            continue
        regions.append((FlashRegion(name, start, size, starts[start]["index"], ends[end]["index"]), loc))

    regions.sort(key=lambda item: item[0].start)
    for (previous, _), (current, loc) in zip(regions, regions[1:]):
        if current.start < previous.end:
            diag.error(f"Flash region {current.name} overlaps {previous.name}", loc)
    if raw and not diag:
        for name in REQUIRED_REGIONS:
            if name not in seen:
                diag.error(f"the Flash layout needs a {name} region", top)
    return tuple(region for region, _ in regions)


def ram_regions(database: dict) -> tuple[RamRegion, ...]:
    """Merge adjacent executable RAM into RAM; non-executable RAM becomes CCM."""
    merged: list[RamRegion] = []
    for r in sorted(database["ram"], key=lambda r: r["address"]):
        name = "RAM" if r["executable"] else "CCM"
        last = merged[-1] if merged else None
        if last and last.name == name and last.start + last.size == r["address"]:
            merged[-1] = RamRegion(name, last.start, last.size + r["size"])
        else:
            merged.append(RamRegion(name, r["address"], r["size"]))
    names = [r.name for r in merged]
    if names.count("RAM") != 1 or names.count("CCM") > 1:
        raise ValueError(f"unsupported RAM map for {database['mcu']}: {names}")
    return tuple(merged)
