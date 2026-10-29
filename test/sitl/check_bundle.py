#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""L3 SITL bundle test (ADR-0026, spec 0014 Seam C): bundle tools against SITL.

Unpacks the bundle archive into a temporary directory outside the repository,
starts SITL with its UDP Link, and runs the bundle's link_monitor.py as a
subprocess with no --msg-dir, --param-meta or VK_* environment variables set.
Verifies --check, param get ATT_KP with unit, param set SENS_BOARD_ROT YAW_90
by enum name, and recording a log read back by the bundle's vkulog.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tarfile
import tempfile


def free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("sitl", help="vehkernel-sitl executable")
    parser.add_argument("build_dir", type=Path, help="the build tree that wrote the bundle")
    args = parser.parse_args()
    # The archive the build wrote is named after the commit in its bundle's manifest.
    manifest = json.loads((args.build_dir / "bundle" / "manifest.json").read_text(encoding="utf-8"))
    args.archive = args.build_dir / f"vehkernel-{manifest['preset']}-{manifest['commit']}.tar.gz"

    if not args.archive.is_file():
        print(f"FAIL: bundle archive does not exist at {args.archive}", file=sys.stderr)
        return 1

    failures: list[str] = []

    # Temporary directory outside the repository for the unpacked bundle
    with tempfile.TemporaryDirectory() as tmp_str:
        tmp_dir = Path(tmp_str)
        with tarfile.open(args.archive, "r:gz") as tar:
            tar.extractall(tmp_dir)

        bundle_dirs = [d for d in tmp_dir.iterdir() if d.is_dir()]
        if len(bundle_dirs) != 1:
            print(f"FAIL: expected 1 top-level bundle directory in archive, found {bundle_dirs}", file=sys.stderr)
            return 1
        bundle_dir = bundle_dirs[0]

        monitor = bundle_dir / "tools" / "link" / "link_monitor.py"
        if not monitor.is_file():
            print(f"FAIL: link_monitor.py missing in bundle at {monitor}", file=sys.stderr)
            return 1

        port = free_udp_port()
        address = f"127.0.0.1:{port}"

        # Start SITL with UDP Link
        sitl = subprocess.Popen([args.sitl, "--duration", "0", "--udp", address],
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        first_line = sitl.stdout.readline()
        print(first_line, end="")

        # Clean environment: remove any VK_* variables and PYTHONPATH
        clean_env = {
            k: v for k, v in os.environ.items()
            if not k.startswith("VK_") and k not in ("PYTHONPATH",)
        }

        try:
            # Step 1: --check --duration 3 with no mismatch
            print("$ tools/link/link_monitor.py --check --duration 3")
            res_check = subprocess.run(
                [sys.executable, str(monitor), "--udp", address, "--handshake-timeout", "5", "--check", "--duration", "3"],
                cwd=bundle_dir,
                env=clean_env,
                capture_output=True,
                text=True,
                timeout=30,
                check=False,
            )
            print(res_check.stdout, end="")
            if res_check.stderr:
                print(res_check.stderr, end="", file=sys.stderr)
            if res_check.returncode != 0:
                failures.append(f"--check failed with exit code {res_check.returncode}")
            if "mismatches=0" not in res_check.stdout:
                failures.append("--check did not report mismatches=0")

            # Step 2: param get ATT_KP showing its unit
            print("$ tools/link/link_monitor.py param get ATT_KP")
            res_get = subprocess.run(
                [sys.executable, str(monitor), "--udp", address, "--handshake-timeout", "5", "param", "get", "ATT_KP"],
                cwd=bundle_dir,
                env=clean_env,
                capture_output=True,
                text=True,
                timeout=30,
                check=False,
            )
            print(res_get.stdout, end="")
            if res_get.stderr:
                print(res_get.stderr, end="", file=sys.stderr)
            if res_get.returncode != 0:
                failures.append(f"param get ATT_KP failed with exit code {res_get.returncode}")
            if "unit=1/s" not in res_get.stdout:
                failures.append(f"param get ATT_KP did not show unit=1/s:\n{res_get.stdout}")

            # Step 3: param set SENS_BOARD_ROT YAW_90 by enum name
            print("$ tools/link/link_monitor.py param set SENS_BOARD_ROT YAW_90")
            res_set = subprocess.run(
                [sys.executable, str(monitor), "--udp", address, "--handshake-timeout", "5", "param", "set", "SENS_BOARD_ROT", "YAW_90"],
                cwd=bundle_dir,
                env=clean_env,
                capture_output=True,
                text=True,
                timeout=30,
                check=False,
            )
            print(res_set.stdout, end="")
            if res_set.stderr:
                print(res_set.stderr, end="", file=sys.stderr)
            if res_set.returncode != 0:
                failures.append(f"param set SENS_BOARD_ROT YAW_90 failed with exit code {res_set.returncode}")
            if "value_name=YAW_90" not in res_set.stdout:
                failures.append(f"param set SENS_BOARD_ROT YAW_90 did not show value_name=YAW_90:\n{res_set.stdout}")

            # Step 4: record x.ulg --profile estimator_replay --duration 2
            print("$ tools/link/link_monitor.py record x.ulg --profile estimator_replay --duration 2")
            res_record = subprocess.run(
                [sys.executable, str(monitor), "--udp", address, "--handshake-timeout", "5",
                 "record", "x.ulg", "--profile", "estimator_replay", "--duration", "2"],
                cwd=bundle_dir,
                env=clean_env,
                capture_output=True,
                text=True,
                timeout=30,
                check=False,
            )
            print(res_record.stdout, end="")
            if res_record.stderr:
                print(res_record.stderr, end="", file=sys.stderr)
            if res_record.returncode != 0:
                failures.append(f"record command failed with exit code {res_record.returncode}")

            log_file = bundle_dir / "x.ulg"
            if not log_file.is_file() or log_file.stat().st_size == 0:
                failures.append(f"record command did not create log file at {log_file}")
            else:
                # Read using bundle's vkulog
                print("$ reading x.ulg using bundle's vkulog")
                read_code = (
                    "import sys\n"
                    "sys.path.insert(0, 'tools/log')\n"
                    "import vkulog\n"
                    "reader = vkulog.ULogReader('x.ulg')\n"
                    "assert len(reader.records) > 0, 'No records in log'\n"
                    "print(f'Read {len(reader.records)} records successfully from x.ulg')\n"
                )
                res_read = subprocess.run(
                    [sys.executable, "-c", read_code],
                    cwd=bundle_dir,
                    env=clean_env,
                    capture_output=True,
                    text=True,
                    timeout=30,
                    check=False,
                )
                print(res_read.stdout, end="")
                if res_read.stderr:
                    print(res_read.stderr, end="", file=sys.stderr)
                if res_read.returncode != 0:
                    failures.append(f"bundle's vkulog could not read x.ulg (exit {res_read.returncode})")
        finally:
            sitl.terminate()
            sitl.communicate(timeout=10)

    for failure in failures:
        print(f"FAIL: {failure}", file=sys.stderr)

    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
