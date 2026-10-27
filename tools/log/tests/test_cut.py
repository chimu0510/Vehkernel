# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests for tools/log/cut.py (spec 0011, ADR-0018 L2)."""

from pathlib import Path
import struct
import sys

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools" / "log"))

from cut import cut  # noqa: E402
import vkulog  # noqa: E402


def write_log(path: Path, restart_us: int = 1_000_000, with_restart: bool = True) -> None:
    with vkulog.ULogWriter(path, start_timestamp_us=900_000) as w:
        w.write_format(vkulog.FORMATS["imu_sample"])
        w.write_format(vkulog.FORMATS["estimator_restart"])
        w.write_format(vkulog.FORMATS["work_item_status"])
        w.write_info("ver_git", "abcdef01")
        w.write_parameter("ATT_KP", 1.0)
        w.write_add_logged(1, "imu_sample", 0)
        w.write_add_logged(2, "estimator_restart", 0)
        w.write_add_logged(3, "work_item_status", 0)
        # One sample before the restart, then one every second for ten seconds.
        w.write_data(1, struct.pack("<QB3f3ff", restart_us - 500, 0, 0.0, 0.0, 9.8, 0.0, 0.0, 0.0, 25.0))
        if with_restart:
            w.write_data(2, struct.pack("<Q", restart_us))
        for i in range(10):
            ts = restart_us + i * 1_000_000
            w.write_data(1, struct.pack("<QB3f3ff", ts + 5_000, 0, 0.0, 0.0, 9.8, 0.0, 0.0, 0.0, 25.0))
            w.write_data(3, struct.pack("<Q", ts))
        w.write_parameter("ATT_KP", 2.0)


def test_cut_keeps_the_window_and_the_topics(tmp_path: Path):
    src = tmp_path / "in.ulg"
    dst = tmp_path / "out.ulg"
    write_log(src)

    cut(src, dst, 3.5)

    reader = vkulog.ULogReader(dst)
    names = {sub.message_name for sub in reader.subscriptions.values()}
    assert names == {"imu_sample", "estimator_restart"}
    assert set(reader.formats) == {"imu_sample", "estimator_restart"}
    assert reader.info["ver_git"] == "abcdef01"
    assert reader.parameters["ATT_KP"] == 1.0
    assert reader.start_timestamp_us == 900_000

    imu_ids = [i for i, sub in reader.subscriptions.items() if sub.message_name == "imu_sample"]
    imu = [m.timestamp_us for m in reader.data_messages if m.msg_id in imu_ids]
    # The sample before the restart and the ones after the cut are gone.
    assert imu == [1_005_000, 2_005_000, 3_005_000, 4_005_000]
    # A parameter changed after the cut is gone too.
    assert reader.parameter_changes == []


def test_cut_without_a_restart_fails(tmp_path: Path):
    src = tmp_path / "in.ulg"
    write_log(src, with_restart=False)
    with pytest.raises(ValueError):
        cut(src, tmp_path / "out.ulg", 3.0)
