#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""L0 check for board examples (spec 0013, ADR-0025).

Checks examples.toml against the parsed board definition, verifying that:
- Every board component named exists in the board definition.
- Every device, actuator output, encoder, serial port and flash region of the
  board definition is introduced by exactly one example or listed in
  [not_covered] (or [pending] during development), not both.
- Every pin role not covered by a higher-level component is introduced or
  listed as not covered.
- Components used by an example are introduced by that example or an earlier one.
- Firmware components used are declared in [firmware].
- Example numbering starts at 01 with no gaps.
- Example directories match examples.toml exactly, each containing main.cpp
  and README.md with the required lesson sections.
- Examples declaring sitl = true match the targets built by sitl-examples.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys
import tomllib
from typing import Sequence

# Allow importing vkboard from tools/generators
REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT / "tools" / "generators") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "tools" / "generators"))

from vkboard.board import Board, load_board  # noqa: E402

LESSON_SECTIONS = [
    "What you learn",
    "Before this",
    "Hardware",
    "Build and flash",
    "What you see",
    "The code",
    "Exercises",
    "Read further",
]


BOARD_PREFIXES = ("device:", "output:", "encoder:", "serial:", "flash:", "pin:")
BOARD_SPECIAL = ("user_key", "actuator_power")


def is_board_component(comp: str) -> bool:
    return comp.startswith(BOARD_PREFIXES) or comp in BOARD_SPECIAL


def check_examples(
    examples_toml: Path,
    board_toml: Path,
    examples_dir: Path,
    soc_root: Path | None = None,
    sitl_targets: Sequence[str] | None = None,
) -> list[str]:
    """Validate examples.toml and example directories against the board definition.

    Returns a list of error diagnostic strings (empty if all checks pass).
    """
    errors: list[str] = []

    if soc_root is None:
        soc_root = REPO_ROOT / "soc"

    try:
        board: Board = load_board(board_toml, soc_root)
    except Exception as e:
        return [f"failed to load board definition {board_toml}: {e}"]

    try:
        text = examples_toml.read_text(encoding="utf-8")
        data = tomllib.loads(text)
    except Exception as e:
        return [f"failed to load examples TOML {examples_toml}: {e}"]

    # 1. Firmware components declared
    firmware_defs = data.get("firmware", {})
    if not isinstance(firmware_defs, dict):
        errors.append("[firmware] must be a table")
        firmware_defs = {}

    # 2. Extract valid board components from board definition
    valid_devices = {f"device:{d.device}" for d in board.hardware.devices}
    valid_outputs = {f"output:{a.output}" for a in board.hardware.actuators}
    valid_encoders = {f"encoder:{a.output}" for a in board.hardware.actuators}
    valid_serials = {f"serial:{s.port}" for s in board.hardware.serial}
    valid_flashes = {f"flash:{f.name}" for f in board.flash}
    valid_special = set()
    if board.hardware.user_key is not None:
        valid_special.add("user_key")
    if board.hardware.actuator_power is not None:
        valid_special.add("actuator_power")

    all_pin_roles = {p.role for p in board.pins}
    for p in board.pins:
        for _, fn in p.alt:
            all_pin_roles.add(fn.role)
    valid_pins = {f"pin:{role}" for role in all_pin_roles}

    all_valid_board_components = (
        valid_devices
        | valid_outputs
        | valid_encoders
        | valid_serials
        | valid_flashes
        | valid_special
        | valid_pins
    )

    # 3. Pin roles covered by higher-level components
    covered_pin_roles: set[str] = set()
    for d in board.hardware.devices:
        covered_pin_roles.update(d.pins)
    for a in board.hardware.actuators:
        covered_pin_roles.add(a.pwm)
        covered_pin_roles.add(a.in1)
        covered_pin_roles.add(a.in2)
        covered_pin_roles.update(a.encoder)
    for s in board.hardware.serial:
        if s.tx:
            covered_pin_roles.add(s.tx)
        if s.rx:
            covered_pin_roles.add(s.rx)
    if board.hardware.user_key is not None:
        covered_pin_roles.add(board.hardware.user_key.pin)
    if board.hardware.actuator_power is not None:
        covered_pin_roles.add(board.hardware.actuator_power.pin)

    # Required board components that must be introduced or not covered
    uncovered_pin_components = {f"pin:{r}" for r in (all_pin_roles - covered_pin_roles)}
    required_board_components = (
        valid_devices
        | valid_outputs
        | valid_encoders
        | valid_serials
        | valid_flashes
        | valid_special
        | uncovered_pin_components
    )

    # 4. Parse not_covered and pending tables
    not_covered = data.get("not_covered", {})
    if not isinstance(not_covered, dict):
        errors.append("[not_covered] must be a table")
        not_covered = {}

    pending = data.get("pending", {})
    if not isinstance(pending, dict):
        errors.append("[pending] must be a table")
        pending = {}

    for comp in not_covered:
        if comp not in all_valid_board_components:
            errors.append(f"unknown board component '{comp}' in [not_covered]")

    for comp in pending:
        if comp not in all_valid_board_components:
            errors.append(f"unknown board component '{comp}' in [pending]")
        if comp in not_covered:
            errors.append(f"component '{comp}' both listed in [pending] and [not_covered]")

    # 5. Parse examples list
    examples_raw = data.get("example", [])
    if not isinstance(examples_raw, list):
        errors.append("[[example]] must be an array of tables")
        examples_raw = []

    seen_introduced: dict[str, str] = {}
    available_so_far: set[str] = set()

    for idx, ex in enumerate(examples_raw):
        if not isinstance(ex, dict):
            errors.append(f"example entry {idx} must be a table")
            continue

        name = ex.get("name")
        if not isinstance(name, str):
            errors.append(f"example entry {idx} missing 'name'")
            name = f"example_{idx}"

        # Numbering check: NN-name from 01 with no gaps
        match = re.match(r"^(\d{2})-[a-z0-9_-]+$", name)
        expected_num = f"{idx + 1:02d}"
        if not match or match.group(1) != expected_num:
            errors.append(
                f"gap in numbering: example at index {idx} has name '{name}', "
                f"expected prefix '{expected_num}-'"
            )

        # Introduces check
        introduces = ex.get("introduces", [])
        if not isinstance(introduces, list):
            errors.append(f"example '{name}' 'introduces' must be a list")
            introduces = []

        for comp in introduces:
            if not isinstance(comp, str):
                errors.append(f"example '{name}' introduces non-string component: {comp}")
                continue

            if is_board_component(comp):
                if comp not in all_valid_board_components:
                    errors.append(f"unknown board component '{comp}' introduced by '{name}'")
            else:
                if comp not in firmware_defs:
                    errors.append(f"undeclared firmware component '{comp}' introduced by '{name}'")

            if comp in seen_introduced:
                errors.append(
                    f"component '{comp}' introduced twice: by '{seen_introduced[comp]}' and '{name}'"
                )
            else:
                seen_introduced[comp] = name

            if comp in not_covered:
                errors.append(
                    f"component '{comp}' both introduced by '{name}' and listed in [not_covered]"
                )
            if comp in pending:
                errors.append(
                    f"component '{comp}' both introduced by '{name}' and listed in [pending]"
                )

        available_so_far.update(introduces)

        # Uses check
        uses = ex.get("uses", [])
        if not isinstance(uses, list):
            errors.append(f"example '{name}' 'uses' must be a list")
            uses = []

        for comp in uses:
            if not isinstance(comp, str):
                errors.append(f"example '{name}' uses non-string component: {comp}")
                continue

            if is_board_component(comp):
                if comp not in all_valid_board_components:
                    errors.append(f"unknown board component '{comp}' used by '{name}'")
                elif comp not in available_so_far:
                    errors.append(f"component '{comp}' used by '{name}' before it is introduced")
            else:
                if comp not in firmware_defs:
                    errors.append(f"undeclared firmware component '{comp}' used by '{name}'")
                elif comp not in available_so_far:
                    errors.append(f"component '{comp}' used by '{name}' before it is introduced")

    # 6. Check full coverage of required board components
    for comp in sorted(required_board_components):
        if comp not in seen_introduced and comp not in not_covered and comp not in pending:
            errors.append(f"board component '{comp}' neither introduced nor listed as not covered")

    # 7. Check directory structure and lesson sections
    if examples_dir.is_dir():
        disk_dirs = {
            p.name
            for p in examples_dir.iterdir()
            if p.is_dir() and not p.name.startswith((".", "__"))
        }
        toml_names = {ex.get("name") for ex in examples_raw if isinstance(ex, dict) and "name" in ex}

        for d in sorted(disk_dirs - toml_names):
            errors.append(f"directory '{d}' exists on disk but is not listed in examples.toml")

        for d in sorted(toml_names - disk_dirs):
            errors.append(f"example '{d}' listed in examples.toml has no directory on disk")

        for ex in examples_raw:
            if not isinstance(ex, dict) or "name" not in ex:
                continue
            name = ex["name"]
            edir = examples_dir / name
            if not edir.is_dir():
                continue

            main_file = edir / "main.cpp"
            readme_file = edir / "README.md"

            if not main_file.is_file():
                errors.append(f"example '{name}' missing main.cpp")

            if not readme_file.is_file():
                errors.append(f"example '{name}' missing README.md")
            else:
                readme_text = readme_file.read_text(encoding="utf-8")
                for section in LESSON_SECTIONS:
                    pattern = rf"(?m)^#+\s+{re.escape(section)}\b"
                    if not re.search(pattern, readme_text, re.IGNORECASE):
                        errors.append(f"example '{name}' lesson missing section '{section}'")

    # 8. SITL target comparison
    sitl_examples = {
        ex["name"]
        for ex in examples_raw
        if isinstance(ex, dict) and ex.get("sitl") is True and "name" in ex
    }
    if sitl_targets is not None:
        expected_sitl = set(sitl_targets)
        if sitl_examples != expected_sitl:
            errors.append(
                f"sitl-examples targets mismatch: examples.toml has {sorted(sitl_examples)}, "
                f"but sitl-examples builds {sorted(expected_sitl)}"
            )

    return errors


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--examples-toml", type=Path, required=True, help="Path to board's examples.toml")
    parser.add_argument("--board-toml", type=Path, required=True, help="Path to board definition board.toml")
    parser.add_argument("--examples-dir", type=Path, required=True, help="Path to board's examples directory")
    parser.add_argument("--soc-root", type=Path, default=None, help="Path to SoC database root")
    parser.add_argument("--sitl-targets", nargs="*", default=None, help="Targets built by sitl-examples")

    args = parser.parse_args(argv)

    errors = check_examples(
        examples_toml=args.examples_toml,
        board_toml=args.board_toml,
        examples_dir=args.examples_dir,
        soc_root=args.soc_root,
        sitl_targets=args.sitl_targets,
    )

    if errors:
        for err in errors:
            print(f"error: {err}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
