#!/usr/bin/env python3
"""Convert an stm32-data chip description into a Vehkernel MCU database.

Runs offline on a chip JSON downloaded from embassy-rs/stm32-data-generated and
committed under ``<mcu_dir>/source/`` with a manifest naming its upstream
commit, so the build never needs the network and the result is reproducible
(ADR-0011, ADR-0016). Usage::

    tools/mcudb/convert_stm32_data.py mcu/stm32f407vet6          # write mcu.json
    tools/mcudb/convert_stm32_data.py mcu/stm32f407vet6 --check  # verify it

The MCU database holds what the board generator validates against:

- ``af``: alternate functions per pin, named ``<PERIPHERAL>_<SIGNAL>``.
- ``analog``: additional (non-AF) analog functions per pin, such as ADC inputs.
- ``dma``: the DMA stream/channel pairs each request may use. stm32-data calls
  an F4 stream a "channel" and the channel a "request"; this tool uses the
  reference manual's terms.
- ``flash``: the Flash sector table.
- ``ram``: RAM regions, including CCM.
- ``peripherals``: base address, register block kind and RCC bus clock of
  each peripheral instance.

Output is deterministic: identical input gives byte-identical output.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

FORMAT_VERSION = 1
UPSTREAM_PROJECT = "embassy-rs/stm32-data-generated"

_PIN_RE = re.compile(r"^P([A-K])(\d+)$")
_DMA_STREAM_RE = re.compile(r"^(DMA\d)_CH(\d)$")


class ConversionError(Exception):
    """The upstream data does not have the expected shape."""


def pin_sort_key(pin: str) -> tuple[str, int]:
    match = _PIN_RE.match(pin)
    if match is None:
        raise ConversionError(f"unexpected pin name '{pin}'")
    return match.group(1), int(match.group(2))


def _find_package(chip: dict, package_name: str) -> dict:
    for package in chip["packages"]:
        if package["name"] == package_name:
            return package
    known = ", ".join(p["name"] for p in chip["packages"])
    raise ConversionError(f"package '{package_name}' not found; available: {known}")


def _package_gpios(package: dict) -> set[str]:
    gpios = set()
    for pad in package["pins"]:
        for signal in pad["signals"]:
            # Pads can carry suffixes such as "PC14-OSC32_IN"; keep the GPIO name.
            name = signal.split("-")[0]
            if _PIN_RE.match(name):
                gpios.add(name)
    return gpios


def _flash_sectors(memory: list[dict]) -> dict:
    regions = sorted(
        (m for m in memory if m["kind"] == "flash" and m["name"].startswith("BANK_")),
        key=lambda m: m["address"],
    )
    if not regions:
        raise ConversionError("no Flash bank regions in the memory map")
    sectors = []
    for region in regions:
        erase_size = region["settings"]["erase_size"]
        if erase_size <= 0 or region["size"] % erase_size:
            raise ConversionError(f"Flash region {region['name']} is not a whole number of sectors")
        for offset in range(0, region["size"], erase_size):
            sectors.append({
                "index": len(sectors),
                "address": region["address"] + offset,
                "size": erase_size,
            })
    for previous, current in zip(sectors, sectors[1:]):
        if previous["address"] + previous["size"] != current["address"]:
            raise ConversionError("Flash sectors are not contiguous")
    settings = {(r["settings"]["write_size"], r["settings"]["erase_value"]) for r in regions}
    if len(settings) != 1:
        raise ConversionError("Flash regions disagree on write size or erase value")
    write_size, erase_value = settings.pop()
    return {
        "base": sectors[0]["address"],
        "size": sum(s["size"] for s in sectors),
        "write_size": write_size,
        "erase_value": erase_value,
        "sectors": sectors,
    }


def _ram_regions(memory: list[dict]) -> list[dict]:
    regions = []
    for m in sorted((m for m in memory if m["kind"] == "ram"), key=lambda m: m["address"]):
        access = m.get("access", {})
        regions.append({
            "name": m["name"],
            "address": m["address"],
            "size": m["size"],
            "executable": access.get("execute", True),
        })
    return regions


def _peripherals(peripherals: list[dict]) -> dict:
    table = {}
    for p in sorted(peripherals, key=lambda p: p["name"]):
        if "registers" not in p:
            continue
        table[p["name"]] = {
            "address": p["address"],
            "kind": p["registers"]["kind"],
            "bus_clock": p.get("rcc", {}).get("bus_clock"),
        }
    return table


def convert(chip: dict, *, mcu: str, package_name: str, commit: str, source_path: str) -> dict:
    package = _find_package(chip, package_name)
    gpios = _package_gpios(package)
    if len(chip["cores"]) != 1:
        raise ConversionError("only single-core MCUs are supported")
    core = chip["cores"][0]
    # stm32-data lists one memory map per bank configuration; dual-bank parts are not supported.
    if len(chip["memory"]) != 1:
        raise ConversionError("only MCUs with a single memory map are supported")
    memory = chip["memory"][0]

    af: dict[str, list[dict]] = {pin: [] for pin in gpios}
    analog: dict[str, list[str]] = {pin: [] for pin in gpios}
    dma: dict[str, list[dict]] = {}
    for peripheral in core["peripherals"]:
        name = peripheral["name"]
        for entry in peripheral.get("pins", []):
            pin = entry["pin"]
            if pin not in gpios:
                continue
            function = f"{name}_{entry['signal']}"
            if "af" in entry:
                af[pin].append({"function": function, "af": entry["af"]})
            else:
                analog[pin].append(function)
        for entry in peripheral.get("dma_channels", []):
            match = _DMA_STREAM_RE.match(entry["channel"])
            if match is None:
                raise ConversionError(f"unexpected DMA stream '{entry['channel']}' for {name}")
            request = f"{name}_{entry['signal']}"
            dma.setdefault(request, []).append({
                "controller": match.group(1),
                "stream": int(match.group(2)),
                "channel": entry["request"],
            })

    def sort_af(entries: list[dict]) -> list[dict]:
        return sorted(entries, key=lambda e: (e["af"], e["function"]))

    def sort_dma(entries: list[dict]) -> list[dict]:
        return sorted(entries, key=lambda e: (e["controller"], e["stream"], e["channel"]))

    pins = sorted(gpios, key=pin_sort_key)
    return {
        "format": FORMAT_VERSION,
        "mcu": mcu,
        "family": chip["family"],
        "package": package["package"],
        "source": {
            "project": UPSTREAM_PROJECT,
            "commit": commit,
            "path": source_path,
            "chip": chip["name"],
            "package": package_name,
        },
        "pins": pins,
        "af": {pin: sort_af(af[pin]) for pin in pins},
        "analog": {pin: sorted(analog[pin]) for pin in pins if analog[pin]},
        "dma": {request: sort_dma(dma[request]) for request in sorted(dma)},
        "flash": _flash_sectors(memory),
        "ram": _ram_regions(memory),
        "peripherals": _peripherals(core["peripherals"]),
    }


def render(database: dict) -> str:
    return json.dumps(database, indent=2, sort_keys=False) + "\n"


def regenerate(mcu_dir: Path) -> str:
    """Convert the upstream data described by ``<mcu_dir>/source/manifest.json``."""
    manifest = json.loads((mcu_dir / "source" / "manifest.json").read_text(encoding="utf-8"))
    chip = json.loads((mcu_dir / "source" / manifest["file"]).read_text(encoding="utf-8"))
    return render(convert(chip, mcu=manifest["mcu"], package_name=manifest["package"],
                          commit=manifest["commit"], source_path=manifest["path"]))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("mcu_dir", type=Path,
                        help="MCU directory holding source/manifest.json, for example mcu/stm32f407vet6")
    parser.add_argument("--check", action="store_true",
                        help="fail if mcu.json differs from a fresh conversion instead of writing it")
    args = parser.parse_args(argv)

    output = args.mcu_dir / "mcu.json"
    try:
        text = regenerate(args.mcu_dir)
    except (OSError, ValueError, KeyError, ConversionError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1

    if args.check:
        current = output.read_text(encoding="utf-8") if output.exists() else ""
        if current != text:
            print(f"error: {output} is out of date; rerun without --check", file=sys.stderr)
            return 1
        return 0

    output.write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
