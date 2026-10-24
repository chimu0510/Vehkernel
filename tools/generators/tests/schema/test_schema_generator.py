# SPDX-License-Identifier: GPL-3.0-or-later
"""Schema generator tests, run through its command line.

Each directory under ``cases/`` holds .msg and .enum files and the expected result:
``expected/`` with the generated files for a valid schema, or
``expected_stderr.txt`` with the diagnostics for an invalid one. Run with
``VK_UPDATE_GOLDEN=1`` to rewrite the expected files after an intended
change, then review the diff.
"""

import importlib.util
import os
import subprocess
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
GENERATOR = REPO / "tools" / "generators" / "generate_schema.py"
CASES = sorted(p for p in (HERE / "cases").iterdir() if p.is_dir())
UPDATE = os.environ.get("VK_UPDATE_GOLDEN") == "1"


LINK_ENUMS = REPO / "tools" / "link" / "vklink" / "enums.py"


def schema_files_in(directory: Path) -> list[str]:
    return sorted(str(p.relative_to(directory)) for suffix in ("*.msg", "*.enum") for p in directory.rglob(suffix))


def generate(schema_dir: Path, output_dir: Path):
    # Run from the schema directory so diagnostics read "<file>.msg:<line>".
    return subprocess.run(
        [sys.executable, str(GENERATOR), *schema_files_in(schema_dir), "--output-dir", str(output_dir)],
        cwd=schema_dir, capture_output=True, text=True, check=False)


def files_in(directory: Path) -> list[str]:
    return sorted(str(p.relative_to(directory)) for p in directory.rglob("*") if p.is_file())


def check_golden(path: Path, actual: str):
    if UPDATE:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(actual, encoding="utf-8")
    assert path.exists(), f"missing golden file {path}; run with VK_UPDATE_GOLDEN=1"
    assert actual == path.read_text(encoding="utf-8")


@pytest.mark.parametrize("case", CASES, ids=[c.name for c in CASES])
def test_case(case: Path, tmp_path: Path):
    result = generate(case, tmp_path)
    if case.name.startswith("error_"):
        assert result.returncode == 1
        assert not any(tmp_path.iterdir()), "no outputs may be written for an invalid schema"
        check_golden(case / "expected_stderr.txt", result.stderr)
    else:
        assert result.returncode == 0, result.stderr
        produced = files_in(tmp_path)
        if not UPDATE:
            assert produced == files_in(case / "expected")
        for name in produced:
            check_golden(case / "expected" / name, (tmp_path / name).read_text(encoding="utf-8"))


def test_output_is_deterministic(tmp_path: Path):
    schema = REPO / "schema"
    first, second = tmp_path / "first", tmp_path / "second"
    assert generate(schema, first).returncode == 0
    assert generate(schema, second).returncode == 0
    assert files_in(first) == files_in(second)
    for name in files_in(first):
        assert (first / name).read_bytes() == (second / name).read_bytes()


def test_link_tools_keep_the_generated_enums(tmp_path: Path):
    # The Link tools run without a build, so they keep a copy of the enums
    # generated from the repository's schema; it must match the schema.
    assert generate(REPO / "schema", tmp_path).returncode == 0
    check_golden(LINK_ENUMS, (tmp_path / "python" / "vkenums.py").read_text(encoding="utf-8"))


def test_python_enums_match_the_cpp_tables(tmp_path: Path):
    assert generate(REPO / "schema", tmp_path).returncode == 0
    spec = importlib.util.spec_from_file_location("vkenums", tmp_path / "python" / "vkenums.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    header = (tmp_path / "vk" / "msg" / "enums.hpp").read_text(encoding="utf-8")
    for path in sorted((REPO / "schema" / "enum").glob("*.enum")):
        enum_type = getattr(module, "".join(part.capitalize() for part in path.stem.split("_")))
        for value in enum_type:
            cpp_name = "".join(part.capitalize() for part in value.name.lower().split("_"))
            assert f"    {cpp_name} = {value.value}," in header
            assert f'{{.value = {enum_type.__name__}::{cpp_name}, .name = "{value.name}", ' in header


def test_unchanged_outputs_are_not_rewritten(tmp_path: Path):
    case = HERE / "cases" / "valid_basic"
    assert generate(case, tmp_path).returncode == 0
    header = tmp_path / "vk" / "msg" / "all_types.hpp"
    os.utime(header, (0, 0))
    assert generate(case, tmp_path).returncode == 0
    assert header.stat().st_mtime == 0


def schema_hash(schema_dir: Path, tmp_path: Path, name: str) -> str:
    assert generate(schema_dir, tmp_path).returncode == 0
    text = (tmp_path / "vk" / "msg" / f"{name}.hpp").read_text(encoding="utf-8")
    return next(line for line in text.splitlines() if "schema_hash" in line)


def write(directory: Path, name: str, text: str) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    (directory / f"{name}.msg").write_text(text, encoding="utf-8")
    return directory


@pytest.mark.parametrize("changed, same", [
    ("uint64 timestamp\nfloat64 value\n", False),       # field type
    ("uint64 timestamp\nfloat32 speed\n", False),       # field name
    ("uint64 timestamp\nfloat32[2] value\n", False),    # array length
    ("uint64 timestamp\nfloat32 value\nuint8 x\n", False),  # added field
    ("# Docs.\n@queue 3\n@rate 50\nuint64 timestamp\nfloat32 value  @unit m  # note\n", True),
], ids=["type", "name", "array", "added", "comments-units-annotations"])
def test_schema_hash_tracks_the_wire_definition(tmp_path: Path, changed: str, same: bool):
    base = schema_hash(write(tmp_path / "a", "topic", "uint64 timestamp\nfloat32 value\n"), tmp_path / "out_a", "topic")
    other = schema_hash(write(tmp_path / "b", "topic", changed), tmp_path / "out_b", "topic")
    assert (base == other) is same


def test_license_tag_is_not_documentation(tmp_path: Path):
    doc = "# The topic.\nuint64 timestamp\nfloat32 value\n"
    plain = write(tmp_path / "plain", "topic", doc)
    # Split so that reuse lint does not take it for this file's own tag.
    tag = "SPDX-" + "License-Identifier: GPL-3.0-or-later"
    tagged = write(tmp_path / "tagged", "topic", f"# {tag}\n" + doc)
    assert generate(plain, tmp_path / "out_plain").returncode == 0
    assert generate(tagged, tmp_path / "out_tagged").returncode == 0
    assert files_in(tmp_path / "out_plain") == files_in(tmp_path / "out_tagged")
    for name in files_in(tmp_path / "out_plain"):
        assert (tmp_path / "out_plain" / name).read_bytes() == (tmp_path / "out_tagged" / name).read_bytes()


@pytest.mark.parametrize("values", [
    "@type uint8\nOK = 0\nFAILED = 1\n",
    "# Docs.\n@type uint8\nOK = 0  @short -\nFAILED = 2\nLATER = 3\n",
], ids=["base", "renumbered-added-short"])
def test_enums_do_not_change_the_schema_hash(tmp_path: Path, values: str):
    plain = schema_hash(write(tmp_path / "plain", "topic", "uint64 timestamp\nuint8 state\n"), tmp_path / "out_plain", "topic")
    schema = write(tmp_path / "enum", "topic", "uint64 timestamp\nuint8 state  @enum result\n")
    (schema / "result.enum").write_text(values, encoding="utf-8")
    assert schema_hash(schema, tmp_path / "out_enum", "topic") == plain
