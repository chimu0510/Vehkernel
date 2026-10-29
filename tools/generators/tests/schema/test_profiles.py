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
    """The generated bandwidth of estimator_replay matches a hand count and fits
    under 80 % of 921600 baud (spec 0011, spec 0012)."""
    diag = Diagnostics(str(SCHEMA_DIR / "log_profiles.toml"))
    profiles = load_profiles(SCHEMA_DIR / "log_profiles.toml", repo_schema, diag)
    assert not diag.errors
    assert profiles is not None

    profile_map = {p.name: p for p in profiles}
    assert "estimator_replay" in profile_map
    replay = profile_map["estimator_replay"]

    # Hand count table for C50C (bytes/second):
    # Framing per record: wire_bytes(encoded_size) = frame + (frame // 254) + 2, where frame = 9 + encoded_size
    #
    # | Topic                     | Encoded | Wire  | Rate (Hz)      | Insts | Bandwidth (B/s)       |
    # |---------------------------|---------|-------|----------------|-------|-----------------------|
    # | imu_sample                | 37 B    | 48 B  | 200 Hz         | 1     | 200 * 48 * 1  = 9600  |
    # | body_velocity_measurement | 45 B    | 56 B  | 200 Hz         | 1     | 200 * 56 * 1  = 11200 |
    # | magnetometer_sample       | 29 B    | 40 B  | 100 Hz         | 1     | 100 * 40 * 1  = 4000  |
    # | vehicle_attitude          | 86 B    | 97 B  | 50 Hz          | 1     | 50 * 97 * 1   = 4850  |
    # | estimator_attitude        | 86 B    | 97 B  | 50 Hz          | 2     | 50 * 97 * 2   = 9700  |
    # | vehicle_local_odometry    | 89 B    | 100 B | 50 Hz          | 1     | 50 * 100 * 1  = 5000  |
    # | actuator_outputs          | 40 B    | 51 B  | 50 Hz          | 1     | 50 * 51 * 1   = 2550  |
    # | vehicle_status            | 36 B    | 47 B  | 5 Hz           | 1     | 5 * 47 * 1    = 235   |
    # | work_item_status          | 50 B    | 61 B  | 1 Hz           | 16    | 1 * 61 * 16   = 976   |
    # | link_status               | 46 B    | 57 B  | 1 Hz           | 2     | 1 * 57 * 2    = 114   |
    # | parameter_update          | 16 B    | 27 B  | 0 Hz           | 1     | 0 * 27 * 1    = 0     |
    # | estimator_restart         | 8 B     | 19 B  | 0 Hz           | 1     | 0 * 19 * 1    = 0     |
    # Sum: 9600 + 11200 + 4000 + 4850 + 9700 + 5000 + 2550 + 235 + 976 + 114 = 48225 B/s (~48.2 KB/s)
    hand_count_c50c = 48225.0

    # Hand count for SITL adds (PLATFORM_SITL):
    # | vehicle_attitude_groundtruth       | 36 B | 47 B | 50 Hz | 1 | 50 * 47 * 1 = 2350 |
    # | vehicle_local_odometry_groundtruth | 60 B | 71 B | 50 Hz | 1 | 50 * 71 * 1 = 3550 |
    # Sum: 48225 + 2350 + 3550 = 54125 B/s (~54.1 KB/s)
    hand_count_sitl = 54125.0

    bw_c50c = replay.bandwidth_c50c(repo_schema)
    bw_sitl = replay.bandwidth_sitl(repo_schema)

    assert bw_c50c == hand_count_c50c
    assert bw_sitl == hand_count_sitl

    # Budget check on SERIAL1 at 921600 baud with 8N1 framing (10 bits per byte)
    baud_rate = 921600
    bits_per_byte = 10
    budget_80_pct = 0.80 * (baud_rate / bits_per_byte)  # 73728.0 B/s
    assert bw_c50c < budget_80_pct


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


def test_log_profile_streams_header(repo_schema):
    """The generator emits typed stream holders for each profile."""
    from vkschema.emit import log_profile_streams_header
    diag = Diagnostics(str(SCHEMA_DIR / "log_profiles.toml"))
    profiles = load_profiles(SCHEMA_DIR / "log_profiles.toml", repo_schema, diag)
    assert not diag.errors
    assert profiles is not None

    header = log_profile_streams_header(profiles, repo_schema)
    assert "class EstimatorReplayStreams" in header
    assert "estimator_replay_stream_count = 31;" in header
    assert "estimator_replay_stream_count = 29;" in header
    assert "TopicStreamOf<msg::ImuSample>" in header
    assert "TopicStreamOf<msg::BodyVelocityMeasurement>" in header
    assert "TopicStreamOf<msg::MagnetometerSample>" in header
    assert "TopicStreamOf<msg::VehicleAttitude>" in header
    assert "TopicStreamOf<msg::EstimatorAttitude>" in header
    assert "CONFIG_PLATFORM_SITL" in header
