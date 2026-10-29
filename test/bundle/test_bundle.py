# SPDX-License-Identifier: GPL-3.0-or-later
"""L0 test for the host tools bundle (ADR-0026, spec 0014 Seam A)."""

from __future__ import annotations

import json
import os
from pathlib import Path
import re
import sys
import tarfile
import tempfile

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "tools" / "bundle"))
sys.path.insert(0, str(REPO_ROOT / "tools" / "link"))

from make_bundle import built_archive, make_bundle  # noqa: E402
from vklink.bundle import bundle_root  # noqa: E402


def check_bundle_contents(
    archive_path: Path,
    source_dir: Path,
    build_dir: Path,
    work_dir: Path,
) -> None:
    assert archive_path.is_file(), f"Bundle archive {archive_path} does not exist"

    # 1. Unpack into an empty directory
    extract_target = work_dir / "unpacked"
    extract_target.mkdir(parents=True)
    with tarfile.open(archive_path, "r:gz") as tar:
        tar.extractall(extract_target)

    bundle_dirs = [d for d in extract_target.iterdir() if d.is_dir()]
    assert len(bundle_dirs) == 1, f"Expected 1 top-level bundle directory, got {bundle_dirs}"
    bundle_dir = bundle_dirs[0]

    # 2. Check manifest exists and is valid
    manifest_file = bundle_dir / "manifest.json"
    assert manifest_file.is_file(), f"No manifest.json in {bundle_dir}"
    manifest = json.loads(manifest_file.read_text(encoding="utf-8"))

    # 3. Check the file list equals manifest's 'files'
    extracted_files = sorted([
        p.relative_to(bundle_dir).as_posix()
        for p in bundle_dir.rglob("*")
        if p.is_file() and p.name != "manifest.json"
    ])
    assert extracted_files == manifest["files"], (
        f"Extracted files mismatch manifest 'files':\n"
        f"Extra in extracted: {set(extracted_files) - set(manifest['files'])}\n"
        f"Missing in extracted: {set(manifest['files']) - set(extracted_files)}"
    )

    # 4. Check param_set_hash equals build's params.json
    build_params_file = build_dir / "generated" / "params" / "params.json"
    assert build_params_file.is_file(), f"Build params.json missing at {build_params_file}"
    build_params = json.loads(build_params_file.read_text(encoding="utf-8"))
    assert manifest["param_set_hash"] == build_params["set_hash"]

    # 5. Check protocol_version equals the firmware's
    hpp_file = source_dir / "modules" / "link" / "include" / "vk" / "link" / "frame.hpp"
    m = re.search(r"protocol_version\s*=\s*(\d+)\s*;", hpp_file.read_text(encoding="utf-8"))
    assert m, f"Could not find protocol_version in {hpp_file}"
    firmware_proto = int(m.group(1))
    assert manifest["protocol_version"] == firmware_proto

    # 6. Check that bundle_root resolves for tools/link/link_monitor.py inside the bundle
    monitor_path = bundle_dir / "tools" / "link" / "link_monitor.py"
    assert monitor_path.is_file(), f"Missing {monitor_path}"
    assert bundle_root(monitor_path) == bundle_dir.resolve()

    # 7. Repack the same build into a second directory and check archive bytes are equal
    repack_output_dir = work_dir / "repack_bundle"
    repack_archive = work_dir / "repack.tar.gz"

    make_bundle(
        source_dir=source_dir,
        build_dir=build_dir,
        preset=manifest["preset"],
        board=manifest.get("board"),
        output_dir=repack_output_dir,
        archive_path=repack_archive,
    )

    orig_bytes = archive_path.read_bytes()
    repack_bytes = repack_archive.read_bytes()
    assert orig_bytes == repack_bytes, "Repacked bundle archive is not byte-identical to the build archive"


def test_bundle(tmp_path: Path) -> None:
    build_env = os.environ.get("VK_BUILD_DIR")
    if not build_env:
        pytest.skip("VK_BUILD_DIR environment variable not set")

    build_dir = Path(build_env)
    archive_path = built_archive(build_dir)
    source_dir = Path(os.environ.get("VK_SOURCE_DIR", str(REPO_ROOT)))

    check_bundle_contents(archive_path, source_dir, build_dir, tmp_path)


def main() -> int:
    build_env = os.environ.get("VK_BUILD_DIR")
    if not build_env:
        print("error: VK_BUILD_DIR environment variable not set", file=sys.stderr)
        return 2

    build_dir = Path(build_env)
    archive_path = built_archive(build_dir)
    source_dir = Path(os.environ.get("VK_SOURCE_DIR", str(REPO_ROOT)))

    with tempfile.TemporaryDirectory() as tmp_str:
        try:
            check_bundle_contents(archive_path, source_dir, build_dir, Path(tmp_str))
            print("OK: bundle verification passed")
            return 0
        except AssertionError as err:
            print(f"FAIL: {err}", file=sys.stderr)
            return 1


if __name__ == "__main__":
    sys.exit(main())
