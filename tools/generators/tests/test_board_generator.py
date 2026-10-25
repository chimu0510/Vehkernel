# SPDX-License-Identifier: GPL-3.0-or-later
"""Board generator tests, run through its command line.

Each directory under ``cases/`` holds a ``board.toml`` and the expected result:
``expected/`` with the generated files for a valid board, or ``expected_stderr.txt``
with the diagnostics for an invalid one. Run with ``VK_UPDATE_GOLDEN=1`` to
rewrite the expected files after an intended change, then review the diff.
"""

import os
import subprocess
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
GENERATOR = REPO / "tools" / "generators" / "generate_board.py"
FIXTURE_SOC_ROOT = HERE / "fixtures" / "soc"
CASES = sorted(p for p in (HERE / "cases").iterdir() if p.is_dir())
UPDATE = os.environ.get("VK_UPDATE_GOLDEN") == "1"


def generate(board_dir: Path, output_dir: Path, soc_root: Path = FIXTURE_SOC_ROOT):
    # Run from the board directory so diagnostics read "board.toml:<line>".
    return subprocess.run(
        [sys.executable, str(GENERATOR), "board.toml", "--soc-root", str(soc_root),
         "--output-dir", str(output_dir)],
        cwd=board_dir, capture_output=True, text=True, check=False)


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
        assert not any(tmp_path.iterdir()), "no outputs may be written for an invalid board"
        check_golden(case / "expected_stderr.txt", result.stderr)
    else:
        assert result.returncode == 0, result.stderr
        produced = sorted(p.name for p in tmp_path.iterdir())
        if not UPDATE:
            assert produced == sorted(p.name for p in (case / "expected").iterdir())
        for name in produced:
            check_golden(case / "expected" / name, (tmp_path / name).read_text(encoding="utf-8"))


def test_a_soc_is_found_under_its_vendor_and_family(tmp_path: Path):
    root = tmp_path / "soc"
    (root / "vendor" / "family" / "testsoc").mkdir(parents=True)
    (root / "vendor" / "family" / "testsoc" / "soc.json").write_bytes(
        (FIXTURE_SOC_ROOT / "testsoc" / "soc.json").read_bytes())
    result = generate(HERE / "cases" / "valid_basic", tmp_path / "out", root)
    assert result.returncode == 0, result.stderr
    assert '"soc_dir": "vendor/family/testsoc"' in (tmp_path / "out" / "board.json").read_text()


def test_two_socs_with_one_name_are_ambiguous(tmp_path: Path):
    root = tmp_path / "soc"
    for family in ("a", "b"):
        (root / "vendor" / family / "testsoc").mkdir(parents=True)
        (root / "vendor" / family / "testsoc" / "soc.json").write_bytes(
            (FIXTURE_SOC_ROOT / "testsoc" / "soc.json").read_bytes())
    result = generate(HERE / "cases" / "valid_basic", tmp_path / "out", root)
    assert result.returncode == 1
    assert "SoC 'testsoc' is ambiguous: vendor/a/testsoc, vendor/b/testsoc" in result.stderr


@pytest.mark.parametrize("board_dir, soc_root", [
    (HERE / "cases" / "valid_basic", FIXTURE_SOC_ROOT),
    (REPO / "boards" / "c50c-v1_1", REPO / "soc"),
], ids=["valid_basic", "c50c-v1_1"])
def test_output_is_deterministic(board_dir: Path, soc_root: Path, tmp_path: Path):
    first, second = tmp_path / "first", tmp_path / "second"
    assert generate(board_dir, first, soc_root).returncode == 0
    assert generate(board_dir, second, soc_root).returncode == 0
    names = sorted(p.name for p in first.iterdir())
    assert names == sorted(p.name for p in second.iterdir())
    for name in names:
        assert (first / name).read_bytes() == (second / name).read_bytes()


def test_unchanged_outputs_are_not_rewritten(tmp_path: Path):
    board = HERE / "cases" / "valid_basic"
    assert generate(board, tmp_path).returncode == 0
    header = tmp_path / "board.hpp"
    os.utime(header, (0, 0))
    assert generate(board, tmp_path).returncode == 0
    assert header.stat().st_mtime == 0


def test_two_boards_beside_each_other_cannot_share_a_board_id(tmp_path: Path):
    result = generate(HERE / "fixtures" / "duplicate_ids" / "second", tmp_path)
    assert result.returncode == 1
    assert result.stderr == "board.toml:4: error: board ID 9 is already used by board first\n"


REAL_BOARDS = sorted(p.parent.name for p in (REPO / "boards").glob("*/board.toml"))


@pytest.mark.parametrize("board", REAL_BOARDS)
def test_real_board_outputs_are_locked(board: str, tmp_path: Path):
    """The outputs of every board in boards/ are golden, so any change to them is reviewed."""
    result = subprocess.run(
        [sys.executable, str(GENERATOR), f"boards/{board}/board.toml", "--soc-root", "soc",
         "--output-dir", str(tmp_path)],
        cwd=REPO, capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr
    expected = HERE / "boards" / board / "expected"
    produced = sorted(p.name for p in tmp_path.iterdir())
    if not UPDATE:
        assert produced == sorted(p.name for p in expected.iterdir())
    for name in produced:
        check_golden(expected / name, (tmp_path / name).read_text(encoding="utf-8"))
