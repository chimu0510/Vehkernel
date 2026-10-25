# SPDX-License-Identifier: GPL-3.0-or-later
"""Parameter generator tests, run through its command line.

Each directory under ``cases/`` holds parameter definitions (any
``params.toml`` below it), an optional solved Kconfig ``config``, optional
board defaults (``board_defaults.json`` with the ``board.toml`` they come
from), and the expected result: ``expected/`` with the generated files, or
``expected_stderr.txt`` with the diagnostics. Run with ``VK_UPDATE_GOLDEN=1``
to rewrite the expected files after an intended change, then review the diff.
"""

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
GENERATOR = REPO / "tools" / "generators" / "generate_params.py"
CASES = sorted(p for p in (HERE / "cases").iterdir() if p.is_dir())
UPDATE = os.environ.get("VK_UPDATE_GOLDEN") == "1"

sys.path.insert(0, str(REPO / "tools" / "generators"))
from vkparams.params import name_hash  # noqa: E402


def definitions_in(directory: Path) -> list[str]:
    return sorted(str(p.relative_to(directory)) for p in directory.rglob("params.toml"))


def generate(case_dir: Path, output_dir: Path, definitions: list[str] | None = None):
    # Run from the case directory so diagnostics read "<dir>/params.toml:<line>".
    config = ["--config", "config"] if (case_dir / "config").exists() else []
    if (case_dir / "board_defaults.json").exists():
        config += ["--board-defaults", "board_defaults.json", "--board", "board.toml"]
    files = definitions_in(case_dir) if definitions is None else definitions
    return subprocess.run(
        [sys.executable, str(GENERATOR), *files, *config, "--output-dir", str(output_dir)],
        cwd=case_dir, capture_output=True, text=True, check=False)


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
        assert not any(tmp_path.iterdir()), "no outputs may be written for invalid definitions"
        check_golden(case / "expected_stderr.txt", result.stderr)
    else:
        assert result.returncode == 0, result.stderr
        produced = files_in(tmp_path)
        if not UPDATE:
            assert produced == files_in(case / "expected")
        for name in produced:
            check_golden(case / "expected" / name, (tmp_path / name).read_text(encoding="utf-8"))


def test_output_is_deterministic(tmp_path: Path):
    case = HERE / "cases" / "valid_basic"
    first, second = tmp_path / "first", tmp_path / "second"
    assert generate(case, first).returncode == 0
    assert generate(case, second, list(reversed(definitions_in(case)))).returncode == 0
    assert files_in(first) == files_in(second)
    for name in files_in(first):
        assert (first / name).read_bytes() == (second / name).read_bytes()


def test_unchanged_outputs_are_not_rewritten(tmp_path: Path):
    case = HERE / "cases" / "valid_basic"
    assert generate(case, tmp_path).returncode == 0
    header = tmp_path / "vk" / "params" / "params.hpp"
    os.utime(header, (0, 0))
    assert generate(case, tmp_path).returncode == 0
    assert header.stat().st_mtime == 0


def test_name_hash_is_fnv1a_32():
    # Reference values of FNV-1a 32 (http://www.isthe.com/chongo/tech/comp/fnv/).
    assert name_hash("") == 0x811C9DC5
    assert name_hash("a") == 0xE40C292C
    assert name_hash("foobar") == 0xBF9CF968


def write(directory: Path, text: str) -> Path:
    (directory / "a").mkdir(parents=True, exist_ok=True)
    (directory / "a" / "params.toml").write_text(text, encoding="utf-8")
    return directory


GAIN = """prefix = "TA"

[[param]]
name = "TA_GAIN"
type = "FLOAT"
default = 0.5
min = 0.0
max = 1.0
description = "A gain."
"""


def set_hash(case_dir: Path, output_dir: Path) -> str:
    assert generate(case_dir, output_dir).returncode == 0
    return json.loads((output_dir / "params.json").read_text(encoding="utf-8"))["set_hash"]


@pytest.mark.parametrize("old, new", [
    ('description = "A gain."', 'description = "Another gain."'),
    ("default = 0.5", "default = 0.25"),
    ("max = 1.0", "max = 2.0"),
    ('description = "A gain."', 'description = "A gain."\nunit = "m"'),
    ('description = "A gain."', 'description = "A gain."\nreboot_required = true'),
], ids=["description", "default", "range", "unit", "flag"])
def test_set_hash_covers_the_whole_metadata(tmp_path: Path, old: str, new: str):
    base = set_hash(write(tmp_path / "a", GAIN), tmp_path / "out_a")
    changed = set_hash(write(tmp_path / "b", GAIN.replace(old, new)), tmp_path / "out_b")
    assert base != changed


def test_set_hash_ignores_comments_and_file_order(tmp_path: Path):
    base = set_hash(write(tmp_path / "a", GAIN), tmp_path / "out_a")
    commented = set_hash(write(tmp_path / "b", "# A comment.\n" + GAIN), tmp_path / "out_b")
    assert base == commented


def test_the_c50c_board_sets_the_link_baud_rate(tmp_path: Path):
    """The repository's definitions with the C50C V1.1's golden board defaults."""
    board_defaults = REPO / "tools" / "generators" / "tests" / "boards" / "c50c-v1_1" / "expected" / "board_defaults.json"
    definitions = sorted(str(p.relative_to(REPO)) for layer in ("core", "drivers", "modules", "platform")
                         for p in (REPO / layer).rglob("params.toml"))
    config = tmp_path / "config"
    config.write_text("CONFIG_PLATFORM_STM32F4=y\nCONFIG_ESTIMATOR_MAHONY=y\nCONFIG_VEHICLE_MODEL_WHEELED=y\n",
                      encoding="utf-8")
    result = subprocess.run(
        [sys.executable, str(GENERATOR), *definitions, "--config", str(config), "--board-defaults", str(board_defaults),
         "--board", "boards/c50c-v1_1/board.toml", "--output-dir", str(tmp_path / "out")],
        cwd=REPO, capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr
    params = {p["name"]: p for p in json.loads((tmp_path / "out" / "params.json").read_text())["parameters"]}
    assert params["SERIAL1_BAUD"]["default"] == 921600
    assert params["ATT_KP"]["default"] == 1.0


def generate_frame(tmp_path: Path, config_text: str):
    """The repository's vehicle description with a solved Kconfig."""
    definitions = sorted(str(p.relative_to(REPO)) for p in (REPO / "modules" / "vehicle_model").rglob("params.toml"))
    config = tmp_path / "config"
    config.write_text(config_text, encoding="utf-8")
    return subprocess.run(
        [sys.executable, str(GENERATOR), *definitions, "--config", str(config), "--output-dir", str(tmp_path / "out")],
        cwd=REPO, capture_output=True, text=True, check=False)


def test_frame_types_follow_the_compiled_in_vehicle_models(tmp_path: Path):
    """ADR-0013, ADR-0017: FRAME_CLASS and FRAME_TYPE offer only the compiled-in models' values."""
    result = generate_frame(tmp_path, "CONFIG_VEHICLE_MODEL_WHEELED=y\n")
    assert result.returncode == 0, result.stderr
    params = {p["name"]: p for p in json.loads((tmp_path / "out" / "params.json").read_text())["parameters"]}
    assert [v["name"] for v in params["FRAME_CLASS"]["values"]] == ["WHEELED"]
    assert [v["name"] for v in params["FRAME_TYPE"]["values"]] == ["DIFF_2WD", "SKID_4WD"]
    assert params["WHL_RADIUS"]["reboot_required"]

    # Without the WHEELED model its frame types are not offered, and its
    # geometry is not compiled in: the default frame type is refused.
    result = generate_frame(tmp_path, "CONFIG_VEHICLE_MODEL_ACKERMANN=y\n")
    assert result.returncode == 1
    assert "FRAME_CLASS: default WHEELED is not compiled in" in result.stderr
    assert "FRAME_TYPE: default DIFF_2WD is not compiled in" in result.stderr
