# SPDX-License-Identifier: GPL-3.0-or-later
"""Unit tests for the host tools bundle packer (ADR-0026, spec 0014)."""

from __future__ import annotations

import json
from pathlib import Path
import stat
import sys
import tarfile

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from make_bundle import BundleError, make_bundle  # noqa: E402


def setup_fake_source_and_build(root: Path) -> tuple[Path, Path]:
    src = root / "source"
    build = root / "build"
    src.mkdir(parents=True)
    build.mkdir(parents=True)

    # modules/link/include/vk/link/frame.hpp
    hpp = src / "modules" / "link" / "include" / "vk" / "link" / "frame.hpp"
    hpp.parent.mkdir(parents=True)
    hpp.write_text("// header\ninline constexpr std::uint8_t protocol_version = 5;\n", encoding="utf-8")

    # tools/link/vklink/
    vklink = src / "tools" / "link" / "vklink"
    vklink.mkdir(parents=True)
    (vklink / "frame.py").write_text("PROTOCOL_VERSION = 5\n", encoding="utf-8")
    (vklink / "cobs.py").write_text("# cobs\n", encoding="utf-8")

    # __pycache__ and tests/ to ensure they are excluded
    pycache = vklink / "__pycache__"
    pycache.mkdir()
    (pycache / "cobs.cpython-310.pyc").write_bytes(b"\x00\x00\x00\x00")

    tests_dir = src / "tools" / "link" / "tests"
    tests_dir.mkdir(parents=True)
    (tests_dir / "test_frame.py").write_text("# test\n", encoding="utf-8")

    # tools/link/link_monitor.py (executable script)
    monitor = src / "tools" / "link" / "link_monitor.py"
    monitor.write_text("#!/usr/bin/env python3\nprint('monitor')\n", encoding="utf-8")
    monitor.chmod(0o755)

    # tools/log/
    log_dir = src / "tools" / "log"
    vkulog = log_dir / "vkulog"
    vkulog.mkdir(parents=True)
    (vkulog / "reader.py").write_text("# reader\n", encoding="utf-8")
    (log_dir / "compare.py").write_text("#!/usr/bin/env python3\n", encoding="utf-8")
    (log_dir / "compare.py").chmod(0o755)
    (log_dir / "cut.py").write_text("#!/usr/bin/env python3\n", encoding="utf-8")
    (log_dir / "cut.py").chmod(0o644)  # non-executable in repo
    (log_dir / "mag_cal.py").write_text("#!/usr/bin/env python3\n", encoding="utf-8")
    (log_dir / "mag_cal.py").chmod(0o755)
    (log_dir / "replay.py").write_text("#!/usr/bin/env python3\n# replay stays out\n", encoding="utf-8")
    (log_dir / "replay.py").chmod(0o755)

    # tools/flash/
    flash_dir = src / "tools" / "flash"
    flash_dir.mkdir(parents=True)
    (flash_dir / "flash_serial.py").write_text("#!/usr/bin/env python3\n", encoding="utf-8")
    (flash_dir / "flash_serial.py").chmod(0o755)
    (flash_dir / "run_uploader.py").write_text("#!/usr/bin/env python3\n", encoding="utf-8")
    (flash_dir / "run_uploader.py").chmod(0o755)

    # tools/bootloader/external/uploader.py
    uploader = src / "tools" / "bootloader" / "external" / "uploader.py"
    uploader.parent.mkdir(parents=True)
    uploader.write_text("#!/usr/bin/env python3\n", encoding="utf-8")
    uploader.chmod(0o755)

    # tools/bundle/README.md
    bundle_readme = src / "tools" / "bundle" / "README.md"
    bundle_readme.parent.mkdir(parents=True)
    bundle_readme.write_text("# Bundle README\n", encoding="utf-8")

    # tools/generators/ should stay out
    generators = src / "tools" / "generators"
    generators.mkdir(parents=True)
    (generators / "generate_schema.py").write_text("#!/usr/bin/env python3\n", encoding="utf-8")

    # Build outputs: generated/schema/python/
    schema_py = build / "generated" / "schema" / "python"
    schema_py.mkdir(parents=True)
    (schema_py / "vkmsg.py").write_text("# vkmsg\n", encoding="utf-8")
    (schema_py / "vkenums.py").write_text("# vkenums\n", encoding="utf-8")
    schema_pycache = schema_py / "__pycache__"
    schema_pycache.mkdir()
    (schema_pycache / "vkmsg.pyc").write_bytes(b"\x00")

    # generated/params/params.json
    params_dir = build / "generated" / "params"
    params_dir.mkdir(parents=True)
    (params_dir / "params.json").write_text(
        json.dumps({"generator": "gen", "set_hash": "0xABCDEF0123456789", "parameters": []}),
        encoding="utf-8",
    )

    return src, build


def test_make_bundle_sitl(tmp_path: Path) -> None:
    src, build = setup_fake_source_and_build(tmp_path)
    output_dir = tmp_path / "bundle_out"
    archive_path = tmp_path / "vehkernel-sitl-testcommit.tar.gz"

    make_bundle(
        source_dir=src,
        build_dir=build,
        preset="sitl",
        board=None,
        output_dir=output_dir,
        archive_path=archive_path,
        commit_override="testcommit",
        commit_time_override=1700000000,
    )

    assert output_dir.is_dir()
    assert archive_path.is_file()

    # Verify manifest
    manifest_file = output_dir / "manifest.json"
    assert manifest_file.is_file()
    manifest = json.loads(manifest_file.read_text(encoding="utf-8"))

    assert manifest["preset"] == "sitl"
    assert manifest["board"] is None
    assert manifest["commit"] == "testcommit"
    assert manifest["protocol_version"] == 5
    assert manifest["param_set_hash"] == "0xABCDEF0123456789"
    assert manifest["python_min"] == "3.10"

    expected_files = {
        "tools/link/link_monitor.py",
        "tools/link/vklink/frame.py",
        "tools/link/vklink/cobs.py",
        "tools/log/vkulog/reader.py",
        "tools/log/compare.py",
        "tools/log/cut.py",
        "tools/log/mag_cal.py",
        "tools/flash/flash_serial.py",
        "tools/flash/run_uploader.py",
        "tools/bootloader/external/uploader.py",
        "tools/bundle/README.md",
        "generated/schema/python/vkmsg.py",
        "generated/schema/python/vkenums.py",
        "generated/params/params.json",
    }

    assert set(manifest["files"]) == expected_files
    # Manifest itself is excluded from files list
    assert "manifest.json" not in manifest["files"]

    # Exclusions checked:
    all_unpacked = [p.relative_to(output_dir).as_posix() for p in output_dir.rglob("*") if p.is_file()]
    assert "tools/log/replay.py" not in all_unpacked
    assert not any("__pycache__" in p for p in all_unpacked)
    assert not any(p.endswith(".pyc") for p in all_unpacked)
    assert not any("tests" in p for p in all_unpacked)
    assert not any("generators" in p for p in all_unpacked)
    assert not any("app/" in p for p in all_unpacked)

    # Check file modes in output directory
    for script_name in (
        "tools/link/link_monitor.py",
        "tools/log/compare.py",
        "tools/log/mag_cal.py",
        "tools/flash/flash_serial.py",
        "tools/flash/run_uploader.py",
        "tools/bootloader/external/uploader.py",
    ):
        p = output_dir / script_name
        mode = stat.S_IMODE(p.stat().st_mode)
        assert mode == 0o755, f"{script_name} has mode {oct(mode)}, expected 0755"

    for non_exec in (
        "tools/log/cut.py",
        "tools/bundle/README.md",
        "generated/params/params.json",
        "generated/schema/python/vkmsg.py",
        "manifest.json",
    ):
        p = output_dir / non_exec
        mode = stat.S_IMODE(p.stat().st_mode)
        assert mode == 0o644, f"{non_exec} has mode {oct(mode)}, expected 0644"

    # Check archive tar entries
    with tarfile.open(archive_path, "r:gz") as tar:
        members = tar.getmembers()
        assert len(members) == len(expected_files) + 1  # includes manifest.json
        names = [m.name for m in members]
        assert all(n.startswith("vehkernel-sitl-testcommit/") for n in names)
        # Check sorted order of entries
        assert names == sorted(names)

        for m in members:
            assert m.mtime == 1700000000
            assert m.uid == 0
            assert m.gid == 0
            assert m.uname == ""
            assert m.gname == ""
            rel = m.name.removeprefix("vehkernel-sitl-testcommit/")
            if rel in (
                "tools/link/link_monitor.py",
                "tools/log/compare.py",
                "tools/log/mag_cal.py",
                "tools/flash/flash_serial.py",
                "tools/flash/run_uploader.py",
                "tools/bootloader/external/uploader.py",
            ):
                assert m.mode == 0o755
            else:
                assert m.mode == 0o644


def test_make_bundle_board(tmp_path: Path) -> None:
    src, build = setup_fake_source_and_build(tmp_path)
    output_dir = tmp_path / "bundle_board_out"
    archive_path = tmp_path / "vehkernel-c50c-v1_1-testcommit.tar.gz"

    # Add board-specific files
    board_dir = build / "generated" / "board"
    board_dir.mkdir(parents=True)
    (board_dir / "board.json").write_text('{"name": "c50c-v1_1"}\n', encoding="utf-8")

    app_dir = build / "app"
    app_dir.mkdir(parents=True)
    (app_dir / "vehkernel.bin").write_bytes(b"\x00" * 32)
    (app_dir / "vehkernel.apj").write_text("{}", encoding="utf-8")
    (app_dir / "vehkernel.elf").write_bytes(b"\x7fELF")
    (app_dir / "vehkernel.hex").write_text(":00000001FF\n", encoding="ascii")
    (app_dir / "vehkernel_with_bl.hex").write_text(":00000001FF\n", encoding="ascii")

    make_bundle(
        source_dir=src,
        build_dir=build,
        preset="c50c-v1_1",
        board="c50c-v1_1",
        output_dir=output_dir,
        archive_path=archive_path,
        commit_override="testcommit",
        commit_time_override=1700000000,
    )

    manifest = json.loads((output_dir / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["board"] == "c50c-v1_1"
    assert "generated/board/board.json" in manifest["files"]
    assert "app/vehkernel.bin" in manifest["files"]
    assert "app/vehkernel.apj" in manifest["files"]
    assert "app/vehkernel.elf" in manifest["files"]
    assert "app/vehkernel.hex" in manifest["files"]
    assert "app/vehkernel_with_bl.hex" in manifest["files"]


def test_protocol_version_mismatch(tmp_path: Path) -> None:
    src, build = setup_fake_source_and_build(tmp_path)
    py_path = src / "tools" / "link" / "vklink" / "frame.py"
    py_path.write_text("PROTOCOL_VERSION = 99\n", encoding="utf-8")

    with pytest.raises(BundleError, match="Protocol version mismatch"):
        make_bundle(
            source_dir=src,
            build_dir=build,
            preset="sitl",
            board=None,
            output_dir=tmp_path / "bundle_out",
            archive_path=tmp_path / "archive.tar.gz",
            commit_override="testcommit",
            commit_time_override=1700000000,
        )


def test_byte_identical_archives(tmp_path: Path) -> None:
    src, build = setup_fake_source_and_build(tmp_path)
    out1 = tmp_path / "out1"
    out2 = tmp_path / "out2"
    archive1 = tmp_path / "archive1.tar.gz"
    archive2 = tmp_path / "archive2.tar.gz"

    make_bundle(
        source_dir=src,
        build_dir=build,
        preset="sitl",
        board=None,
        output_dir=out1,
        archive_path=archive1,
        commit_override="reprocommit",
        commit_time_override=1700000000,
    )

    make_bundle(
        source_dir=src,
        build_dir=build,
        preset="sitl",
        board=None,
        output_dir=out2,
        archive_path=archive2,
        commit_override="reprocommit",
        commit_time_override=1700000000,
    )

    bytes1 = archive1.read_bytes()
    bytes2 = archive2.read_bytes()
    assert bytes1 == bytes2, "Repeated bundle packaging must produce byte-identical archives"
