# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests for log profile validation, bandwidth calculation, and format generation."""

import subprocess
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
sys.path.insert(0, str(REPO / "tools" / "generators"))

from vkboard.diagnostics import Diagnostics
from vkschema.emit import python_ulog_formats
from vkschema.profiles import load_profiles
from vkschema.schema import load_schema
SCHEMA_DIR = REPO / "schema"
GENERATOR = REPO / "tools" / "generators" / "generate_schema.py"
VKULOG_FORMATS = REPO / "tools" / "log" / "vkulog" / "formats.py"


@pytest.fixture
def repo_schema():
    messages = sorted((SCHEMA_DIR / "msg").glob("*.msg"))
    enums = sorted((SCHEMA_DIR / "enum").glob("*.enum"))
    return load_schema(messages + enums)


def test_estimator_replay_bandwidth(repo_schema):
    """The generated bandwidth of estimator_replay is within 5 % of a hand count
    written in the test, for the C50C's table and for SITL's (spec 0011)."""
    diag = Diagnostics(str(SCHEMA_DIR / "log_profiles.toml"))
    profiles = load_profiles(SCHEMA_DIR / "log_profiles.toml", repo_schema, diag)
    assert not diag.errors
    assert profiles is not None

    profile_map = {p.name: p for p in profiles}
    assert "estimator_replay" in profile_map
    replay = profile_map["estimator_replay"]

    # Hand count for C50C (bytes/second):
    # - imu_sample: 200 Hz * 48 B * 1 inst = 9600 B/s
    # - body_velocity_measurement: 200 Hz * 56 B * 1 inst = 11200 B/s
    # - vehicle_attitude: 50 Hz * 84 B * 1 inst = 4200 B/s
    # - vehicle_local_odometry: 50 Hz * 100 B * 1 inst = 5000 B/s
    # - actuator_outputs: 50 Hz * 51 B * 1 inst = 2550 B/s
    # - vehicle_status: 5 Hz * 46 B * 1 inst = 230 B/s
    # - work_item_status: 1 Hz * 60 B * 16 insts = 960 B/s
    # - link_status: 1 Hz * 57 B * 2 insts = 114 B/s
    # - parameter_update: 0 Hz = 0 B/s
    # Sum: 9600 + 11200 + 4200 + 5000 + 2550 + 230 + 960 + 114 = 33854 B/s (~33 KB/s)
    hand_count_c50c = 33854.0

    # Hand count for SITL adds:
    # - vehicle_attitude_groundtruth: 50 Hz * 47 B * 1 inst = 2350 B/s
    # - vehicle_local_odometry_groundtruth: 50 Hz * 71 B * 1 inst = 3550 B/s
    # Sum: 33854 + 5900 = 39754 B/s
    hand_count_sitl = 39754.0

    bw_c50c = replay.bandwidth_c50c(repo_schema)
    bw_sitl = replay.bandwidth_sitl(repo_schema)

    assert abs(bw_c50c - hand_count_c50c) / hand_count_c50c < 0.05
    assert abs(bw_sitl - hand_count_sitl) / hand_count_sitl < 0.05


def test_build_fails_on_unknown_topic(repo_schema, tmp_path: Path):
    """The build fails on a profile naming an unknown topic."""
    profile_toml = tmp_path / "log_profiles.toml"
    profile_toml.write_text(
        """[test_profile]
description = "Profile with unknown topic"
entries = [
    { topic = "non_existent_topic", rate = "as_published" },
]
""",
        encoding="utf-8",
    )
    diag = Diagnostics(str(profile_toml))
    profiles = load_profiles(profile_toml, repo_schema, diag)
    assert profiles is None
    assert any("unknown topic 'non_existent_topic'" in str(e) for e in diag.errors)

    # CLI also returns 1
    result = subprocess.run(
        [
            sys.executable,
            str(GENERATOR),
            *sorted(str(p) for p in (SCHEMA_DIR / "msg").glob("*.msg")),
            *sorted(str(p) for p in (SCHEMA_DIR / "enum").glob("*.enum")),
            "--profiles",
            str(profile_toml),
            "--output-dir",
            str(tmp_path / "out"),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 1
    assert "unknown topic 'non_existent_topic'" in result.stderr


def test_build_fails_on_every_sample_on_unqueued_topic(repo_schema, tmp_path: Path):
    """The build fails on every_sample on an unqueued topic."""
    # vehicle_status has no @queue (queue == 1)
    profile_toml = tmp_path / "log_profiles.toml"
    profile_toml.write_text(
        """[test_profile]
description = "Profile with unqueued every_sample"
entries = [
    { topic = "vehicle_status", rate = "every_sample" },
]
""",
        encoding="utf-8",
    )
    diag = Diagnostics(str(profile_toml))
    profiles = load_profiles(profile_toml, repo_schema, diag)
    assert profiles is None
    assert any("every_sample needs @queue" in str(e) for e in diag.errors)

    result = subprocess.run(
        [
            sys.executable,
            str(GENERATOR),
            *sorted(str(p) for p in (SCHEMA_DIR / "msg").glob("*.msg")),
            *sorted(str(p) for p in (SCHEMA_DIR / "enum").glob("*.enum")),
            "--profiles",
            str(profile_toml),
            "--output-dir",
            str(tmp_path / "out"),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 1
    assert "every_sample needs @queue" in result.stderr


@pytest.mark.parametrize(
    "bad_rate",
    ["malformed_string", -5, 0, 0.0, True, False, []],
)
def test_build_fails_on_malformed_rate(repo_schema, tmp_path: Path, bad_rate):
    """The build fails on a malformed rate."""
    profile_toml = tmp_path / "log_profiles.toml"
    rate_str = str(bad_rate).lower() if isinstance(bad_rate, bool) else repr(bad_rate)
    profile_toml.write_text(
        f"""[test_profile]
description = "Profile with bad rate"
entries = [
    {{ topic = "imu_sample", rate = {rate_str} }},
]
""",
        encoding="utf-8",
    )
    diag = Diagnostics(str(profile_toml))
    profiles = load_profiles(profile_toml, repo_schema, diag)
    assert profiles is None
    assert any("malformed rate" in str(e) for e in diag.errors)


@pytest.mark.parametrize(
    "bad_kconfig",
    ["CONFIG_PLATFORM_SITL", "platform_sitl", "123_BAD", "", "PLATFORM-SITL"],
)
def test_build_fails_on_malformed_kconfig(repo_schema, tmp_path: Path, bad_kconfig):
    """The build fails on a malformed kconfig symbol name."""
    profile_toml = tmp_path / "log_profiles.toml"
    profile_toml.write_text(
        f"""[test_profile]
description = "Profile with bad kconfig"
entries = [
    {{ topic = "imu_sample", rate = "every_sample", kconfig = {bad_kconfig!r} }},
]
""",
        encoding="utf-8",
    )
    diag = Diagnostics(str(profile_toml))
    profiles = load_profiles(profile_toml, repo_schema, diag)
    assert profiles is None
    assert any("malformed kconfig" in str(e) for e in diag.errors)


def test_vkulog_formats_match_schema(repo_schema):
    """tools/log/vkulog/formats.py matches the schema generated formats."""
    # REUSE-IgnoreStart
    tag = "SPDX-License-Identifier"
    expected = f"# {tag}: GPL-3.0-or-later\n" + python_ulog_formats(repo_schema)
    # REUSE-IgnoreEnd
    actual = VKULOG_FORMATS.read_text(encoding="utf-8")
    assert actual == expected
