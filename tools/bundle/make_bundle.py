#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Host tools bundle packer (ADR-0026, spec 0014).

Packages the host tools, generated metadata, and firmware images into
build/<preset>/bundle/ and build/<preset>/vehkernel-<preset>-<commit>.tar.gz.
"""

from __future__ import annotations

import argparse
import gzip
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile


class BundleError(Exception):
    """Raised when bundle packaging fails."""


def is_executable_script(path: Path) -> bool:
    """Returns True if the file has an executable mode and a shebang."""
    try:
        if (path.stat().st_mode & 0o111) == 0:
            return False
        with open(path, "rb") as f:
            header = f.read(2)
        return header == b"#!"
    except (OSError, PermissionError):
        return False


def check_protocol_version(source_dir: Path) -> int:
    """Reads protocol_version from frame.hpp and PROTOCOL_VERSION from frame.py,

    verifying that they match.
    """
    hpp_path = source_dir / "modules" / "link" / "include" / "vk" / "link" / "frame.hpp"
    py_path = source_dir / "tools" / "link" / "vklink" / "frame.py"

    if not hpp_path.is_file():
        raise BundleError(f"Missing {hpp_path}")
    if not py_path.is_file():
        raise BundleError(f"Missing {py_path}")

    hpp_text = hpp_path.read_text(encoding="utf-8")
    m_hpp = re.search(r"protocol_version\s*=\s*(\d+)\s*;", hpp_text)
    if not m_hpp:
        raise BundleError(f"Could not find protocol_version in {hpp_path}")
    hpp_ver = int(m_hpp.group(1))

    py_text = py_path.read_text(encoding="utf-8")
    m_py = re.search(r"PROTOCOL_VERSION\s*=\s*(\d+)", py_text)
    if not m_py:
        raise BundleError(f"Could not find PROTOCOL_VERSION in {py_path}")
    py_ver = int(m_py.group(1))

    if hpp_ver != py_ver:
        raise BundleError(
            f"Protocol version mismatch: {hpp_path} has {hpp_ver}, "
            f"but {py_path} has {py_ver}"
        )
    return hpp_ver


def read_param_set_hash(build_dir: Path) -> str:
    params_json = build_dir / "generated" / "params" / "params.json"
    if not params_json.is_file():
        raise BundleError(f"Missing {params_json}; build a preset first")
    try:
        data = json.loads(params_json.read_text(encoding="utf-8"))
        val = data["set_hash"]
        if isinstance(val, int):
            return f"0x{val:016X}"
        return str(val)
    except Exception as e:
        raise BundleError(f"Failed to read param_set_hash from {params_json}: {e}") from e


def git_commit(source_dir: Path) -> str:
    try:
        res = subprocess.run(
            ["git", "-C", str(source_dir), "describe", "--always", "--dirty"],
            capture_output=True,
            text=True,
            check=False,
        )
        if res.returncode == 0 and res.stdout.strip():
            return res.stdout.strip()
    except (FileNotFoundError, OSError):
        pass
    return "unknown"


def git_commit_time(source_dir: Path) -> int:
    try:
        res = subprocess.run(
            ["git", "-C", str(source_dir), "log", "-1", "--format=%ct"],
            capture_output=True,
            text=True,
            check=False,
        )
        if res.returncode == 0 and res.stdout.strip():
            return int(res.stdout.strip())
    except (FileNotFoundError, OSError, ValueError):
        pass
    return 0


def collect_files(source_dir: Path, build_dir: Path, board: str | None) -> list[tuple[Path, str]]:
    entries: list[tuple[Path, str]] = []

    def should_skip(rel: str) -> bool:
        parts = Path(rel).parts
        if "__pycache__" in parts:
            return True
        if "tests" in parts or "test" in parts:
            return True
        if any(part == "generators" for part in parts):
            return True
        if rel.endswith(".pyc") or rel.endswith(".pyo"):
            return True
        return False

    # 1. tools/link/link_monitor.py
    entries.append((source_dir / "tools" / "link" / "link_monitor.py", "tools/link/link_monitor.py"))

    # 2. tools/link/vklink/**/*.py
    vklink_dir = source_dir / "tools" / "link" / "vklink"
    if vklink_dir.is_dir():
        for p in sorted(vklink_dir.rglob("*.py")):
            rel = p.relative_to(source_dir).as_posix()
            if not should_skip(rel):
                entries.append((p, rel))

    # 3. tools/log/vkulog/**/*.py
    vkulog_dir = source_dir / "tools" / "log" / "vkulog"
    if vkulog_dir.is_dir():
        for p in sorted(vkulog_dir.rglob("*.py")):
            rel = p.relative_to(source_dir).as_posix()
            if not should_skip(rel):
                entries.append((p, rel))

    # 4. tools/log/compare.py, cut.py, mag_cal.py (replay.py and tests/ left out)
    for name in ("compare.py", "cut.py", "mag_cal.py"):
        p = source_dir / "tools" / "log" / name
        if p.is_file():
            entries.append((p, f"tools/log/{name}"))

    # 5. tools/flash/flash_serial.py, run_uploader.py
    for name in ("flash_serial.py", "run_uploader.py"):
        p = source_dir / "tools" / "flash" / name
        if p.is_file():
            entries.append((p, f"tools/flash/{name}"))

    # 6. tools/bootloader/external/uploader.py
    uploader = source_dir / "tools" / "bootloader" / "external" / "uploader.py"
    if uploader.is_file():
        entries.append((uploader, "tools/bootloader/external/uploader.py"))

    # 7. tools/bundle/README.md
    readme = source_dir / "tools" / "bundle" / "README.md"
    if readme.is_file():
        entries.append((readme, "tools/bundle/README.md"))

    # 8. generated/schema/python/**/*.py
    schema_py = build_dir / "generated" / "schema" / "python"
    if schema_py.is_dir():
        for p in sorted(schema_py.rglob("*.py")):
            rel = p.relative_to(build_dir).as_posix()
            if not should_skip(rel):
                entries.append((p, rel))

    # 9. generated/params/params.json
    params_json = build_dir / "generated" / "params" / "params.json"
    if params_json.is_file():
        entries.append((params_json, "generated/params/params.json"))

    # 10. Board outputs
    if board:
        board_json = build_dir / "generated" / "board" / "board.json"
        if board_json.is_file():
            entries.append((board_json, "generated/board/board.json"))

        for img in ("vehkernel.bin", "vehkernel.apj", "vehkernel.elf", "vehkernel.hex"):
            img_path = build_dir / "app" / img
            if img_path.is_file():
                entries.append((img_path, f"app/{img}"))

        with_bl = build_dir / "app" / "vehkernel_with_bl.hex"
        if with_bl.is_file():
            entries.append((with_bl, "app/vehkernel_with_bl.hex"))

    return entries


def make_bundle(
    source_dir: Path,
    build_dir: Path,
    preset: str,
    board: str | None,
    output_dir: Path,
    archive_path: Path | None = None,
    commit_override: str | None = None,
    commit_time_override: int | None = None,
) -> Path:
    """Writes the bundle and its archive; returns the archive's path.

    Without archive_path, the archive is build_dir/vehkernel-<preset>-<commit>.tar.gz, and older
    archives of the preset there are removed, so the build tree holds the current commit's only.
    """
    source_dir = source_dir.resolve()
    build_dir = build_dir.resolve()
    output_dir = output_dir.resolve()

    protocol_version = check_protocol_version(source_dir)
    param_set_hash = read_param_set_hash(build_dir)
    commit = commit_override if commit_override is not None else git_commit(source_dir)
    commit_time = commit_time_override if commit_time_override is not None else git_commit_time(source_dir)
    if archive_path is None:
        archive_path = archive_path_for(build_dir, preset, commit)
        for old in build_dir.glob(f"vehkernel-{preset}-*.tar.gz"):
            if old != archive_path:
                old.unlink()
    archive_path = archive_path.resolve()

    # Clean previous output directory
    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    file_entries = collect_files(source_dir, build_dir, board)
    for src, rel in file_entries:
        if not src.is_file():
            raise BundleError(f"Missing required source file: {src}")
        dest = output_dir / rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(src, dest)
        mode = 0o755 if is_executable_script(src) else 0o644
        os.chmod(dest, mode)

    # Generate manifest.json (files sorted, excluding manifest.json itself)
    files = sorted([
        p.relative_to(output_dir).as_posix()
        for p in output_dir.rglob("*")
        if p.is_file() and p.name != "manifest.json"
    ])
    manifest = {
        "board": board if board else None,
        "commit": commit,
        "files": files,
        "param_set_hash": param_set_hash,
        "preset": preset,
        "protocol_version": protocol_version,
        "python_min": "3.10",
    }
    manifest_path = output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.chmod(manifest_path, 0o644)

    # Write archive
    archive_path.parent.mkdir(parents=True, exist_ok=True)
    top_dir = f"vehkernel-{preset}-{commit}"
    all_files = sorted([
        p.relative_to(output_dir).as_posix()
        for p in output_dir.rglob("*")
        if p.is_file()
    ])

    with open(archive_path, "wb") as f_out:
        with gzip.GzipFile(filename="", mode="wb", fileobj=f_out, mtime=0.0) as gz_out:
            with tarfile.open(mode="w:", fileobj=gz_out) as tar:
                for rel_path in all_files:
                    full_path = output_dir / rel_path
                    arcname = f"{top_dir}/{rel_path}"
                    ti = tar.gettarinfo(str(full_path), arcname=arcname)
                    ti.mtime = commit_time
                    ti.uid = 0
                    ti.gid = 0
                    ti.uname = ""
                    ti.gname = ""
                    ti.mode = 0o755 if is_executable_script(full_path) else 0o644
                    with open(full_path, "rb") as f_in:
                        tar.addfile(ti, f_in)
    return archive_path


def archive_path_for(build_dir: Path, preset: str, commit: str) -> Path:
    """The archive a build writes for a commit."""
    return build_dir / f"vehkernel-{preset}-{commit}.tar.gz"


def built_archive(build_dir: Path) -> Path:
    """The archive of the bundle last written in build_dir, from its manifest."""
    manifest = json.loads((build_dir / "bundle" / "manifest.json").read_text(encoding="utf-8"))
    return archive_path_for(build_dir, manifest["preset"], manifest["commit"])


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Package the Vehkernel host tools bundle.")
    parser.add_argument("--source-dir", type=Path, required=True, help="repository root")
    parser.add_argument("--build-dir", type=Path, required=True, help="CMake build directory")
    parser.add_argument("--preset", required=True, help="CMake preset name")
    parser.add_argument("--board", default="", help="target board name (empty for SITL)")
    parser.add_argument("--output-dir", type=Path, required=True, help="output directory (<build>/bundle)")
    parser.add_argument("--archive", type=Path, help="output archive path (default: <build>/vehkernel-<preset>-<commit>.tar.gz)")
    parser.add_argument("--commit", help="override git commit string")
    parser.add_argument("--commit-time", type=int, help="override commit timestamp")
    args = parser.parse_args(argv)

    try:
        make_bundle(
            source_dir=args.source_dir,
            build_dir=args.build_dir,
            preset=args.preset,
            board=args.board if args.board else None,
            output_dir=args.output_dir,
            archive_path=args.archive,
            commit_override=args.commit,
            commit_time_override=args.commit_time,
        )
    except BundleError as error:
        print(f"make_bundle.py: error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
