"""Bring-up runner tests, through its command line, with a fake OpenOCD and a
minimal ELF file holding the result block symbol."""

import os
import stat
import struct
import subprocess
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
RUNNER = HERE.parent / "run_bringup.py"
ADDRESS = 0x20000100
BLOCK_WORDS = 4 + 4 * 6


def minimal_elf(symbol: str, address: int, size: int) -> bytes:
    """An ELF32 little-endian file with only a symbol table."""
    strtab = b"\0" + symbol.encode() + b"\0"
    symtab = b"\0" * 16 + struct.pack("<IIIBBH", 1, address, size, 0x11, 0, 1)
    shstrtab = b"\0.symtab\0.strtab\0.shstrtab\0"
    data_offset = 52
    offsets = [data_offset, data_offset + len(symtab), data_offset + len(symtab) + len(strtab)]
    shoff = offsets[2] + len(shstrtab)
    header = b"\x7fELF" + bytes([1, 1, 1]) + b"\0" * 9 + struct.pack(
        "<HHIIIIIHHHHHH", 2, 40, 1, 0, 0, shoff, 0, 52, 0, 0, 40, 4, 3)
    sections = [b"\0" * 40,
                struct.pack("<IIIIIIIIII", 1, 2, 0, 0, offsets[0], len(symtab), 2, 1, 4, 16),
                struct.pack("<IIIIIIIIII", 9, 3, 0, 0, offsets[1], len(strtab), 0, 0, 1, 0),
                struct.pack("<IIIIIIIIII", 17, 3, 0, 0, offsets[2], len(shstrtab), 0, 0, 1, 0)]
    return header + symtab + strtab + shstrtab + b"".join(sections)


def block(cases: list[tuple[int, int, list[int]]], magic=0x55424B56, version=1, complete=1) -> list[int]:
    words = [magic, version, len(cases), complete]
    for case_id, status, values in cases:
        words += [case_id, status] + [v & 0xFFFFFFFF for v in values]
    return words + [0] * (BLOCK_WORDS - len(words))


def fake_openocd(tmp_path: Path, words: list[int]) -> Path:
    """A script that accepts any OpenOCD command line and answers memory reads."""
    script = tmp_path / "openocd"
    reply = " ".join(f"0x{w:x}" for w in words)
    script.write_text(f"#!{sys.executable}\nimport sys\n"
                      f"if any('read_memory' in a for a in sys.argv):\n    print('VKWORDS:{reply}')\n")
    script.chmod(script.stat().st_mode | stat.S_IEXEC)
    return script


def run(tmp_path: Path, words: list[int], symbol: str = "vk_bringup_results"):
    elf = tmp_path / "vehkernel.elf"
    elf.write_bytes(minimal_elf(symbol, ADDRESS, BLOCK_WORDS * 4))
    return subprocess.run(
        [sys.executable, str(RUNNER), str(elf), "--openocd", str(fake_openocd(tmp_path, words)), "--wait", "0", "--timeout", "0"],
        capture_output=True, text=True, check=False, env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})


PASSING = [(1, 1, [0xEA, 1, 0, 0]), (2, 1, [450000, 470000, 2222, 0])]


def test_all_cases_passing_exits_zero(tmp_path: Path):
    result = run(tmp_path, block(PASSING))
    assert result.returncode == 0, result.stdout + result.stderr
    assert "WHO_AM_I = 0xEA" in result.stdout
    assert "achievable 2222 Hz" in result.stdout
    assert result.stdout.strip().endswith("PASS")


def test_a_failed_case_exits_non_zero(tmp_path: Path):
    result = run(tmp_path, block([(1, 2, [0xFF, 0, 0, 0]), PASSING[1]]))
    assert result.returncode == 1
    assert "FAIL" in result.stdout
    assert "WHO_AM_I = 0xFF" in result.stdout


def test_a_missing_block_exits_non_zero(tmp_path: Path):
    result = run(tmp_path, [0] * BLOCK_WORDS)
    assert result.returncode == 1
    assert "no result block" in result.stdout


def test_an_incomplete_run_exits_non_zero(tmp_path: Path):
    result = run(tmp_path, block(PASSING, complete=0))
    assert result.returncode == 1
    assert "did not finish" in result.stdout


def test_a_different_block_version_exits_non_zero(tmp_path: Path):
    result = run(tmp_path, block(PASSING, version=2))
    assert result.returncode == 1
    assert "version 2" in result.stdout


def test_an_image_without_the_block_is_a_usage_error(tmp_path: Path):
    result = run(tmp_path, block(PASSING), symbol="something_else")
    assert result.returncode == 2
    assert "bring-up build" in result.stderr


@pytest.mark.parametrize("value", [-1, -123456])
def test_negative_values_are_decoded(tmp_path: Path, value: int):
    result = run(tmp_path, block([(3, 1, [value, 0, 0, 0])]))
    assert str(value) in result.stdout
