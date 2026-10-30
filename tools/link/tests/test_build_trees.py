# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests for build tree discovery on schema hash mismatch (spec 0014, ADR-0026)."""

import importlib.util
import sys
from pathlib import Path

from test_monitor import FakeMcu, SAMPLE_HASH
from vklink.monitor import find_matching_build_trees, format_build_tree_hint

MONITOR = Path(__file__).resolve().parents[1] / "link_monitor.py"


def make_vkmsg(dir_path: Path, hashes: dict[str, int]) -> Path:
    """Generates a stub vkmsg.py with the given {topic_name: schema_hash} mapping."""
    dir_path.mkdir(parents=True, exist_ok=True)
    vkmsg_path = dir_path / "vkmsg.py"
    lines = ["import struct", ""]
    topics = []
    for idx, (name, h) in enumerate(hashes.items()):
        cls_name = "".join(p.capitalize() for p in name.split("_"))
        lines.extend([
            f"class {cls_name}:",
            f"    NAME = {name!r}",
            f"    TOPIC_ID = {idx}",
            f"    SCHEMA_HASH = 0x{h:016X}",
            "    ENCODED_SIZE = 12",
            "    FIELDS = ('timestamp', 'value')",
            "    def __init__(self, timestamp=0, value=0):",
            "        self.timestamp, self.value = timestamp, value",
            "    @classmethod",
            "    def decode(cls, data):",
            "        return cls(*struct.unpack('<QI', data))",
            "",
        ])
        topics.append(f"{idx}: {cls_name}")
    lines.append(f"TOPICS = {{{', '.join(topics)}}}")
    lines.append("BY_NAME = {cls.NAME: cls for cls in TOPICS.values()}")
    lines.append("")
    vkmsg_path.write_text("\n".join(lines), encoding="utf-8")
    return vkmsg_path


def load_link_monitor():
    """Imports link_monitor.py under a test-specific module name."""
    spec = importlib.util.spec_from_file_location("link_monitor_test", MONITOR)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_hint_names_matching_build_tree_and_not_other(tmp_path, monkeypatch):
    """A unit test with two message directories, one matching the MCU's hashes,

    checks the hint names the matching one and not the other, and that no hint
    is printed when none matches (spec 0014, Seam D).
    """
    monkeypatch.chdir(tmp_path)
    matching_msg_dir = tmp_path / "build" / "c50c-v1_1" / "generated" / "schema" / "python"
    differing_msg_dir = tmp_path / "build" / "sitl" / "generated" / "schema" / "python"

    make_vkmsg(matching_msg_dir, {"sample": SAMPLE_HASH})
    make_vkmsg(differing_msg_dir, {"sample": 0xDEADBEEF00000000})

    # One matching the MCU's hashes: names the matching one and not the other
    matching = find_matching_build_trees(tmp_path, {"sample": SAMPLE_HASH})
    assert matching == [matching_msg_dir]
    assert differing_msg_dir not in matching

    hints = [format_build_tree_hint(d) for d in matching]
    assert len(hints) == 1
    assert "build/c50c-v1_1" in hints[0]
    assert "--msg-dir build/c50c-v1_1/generated/schema/python" in hints[0]
    assert "sitl" not in hints[0]
    expected_hint = (
        "hint: the MCU matches the build tree in build/c50c-v1_1; "
        "pass --msg-dir build/c50c-v1_1/generated/schema/python"
    )
    assert hints[0] == expected_hint

    # None matches: no matching directory and no hint
    none_matching = find_matching_build_trees(tmp_path, {"sample": 0x9999999999999999})
    assert none_matching == []
    no_hints = [format_build_tree_hint(d) for d in none_matching]
    assert no_hints == []


def test_link_monitor_prints_matching_hint_and_no_hint_when_none_matches(tmp_path, monkeypatch, capsys):
    """link_monitor.py outputs the hint on mismatch and omits it when no tree matches."""
    monkeypatch.chdir(tmp_path)
    current_msg_dir = tmp_path / "build" / "stale" / "generated" / "schema" / "python"
    matching_msg_dir = tmp_path / "build" / "c50c-v1_1" / "generated" / "schema" / "python"
    differing_msg_dir = tmp_path / "build" / "sitl_alt" / "generated" / "schema" / "python"

    make_vkmsg(current_msg_dir, {"sample": 0xDEAD})
    make_vkmsg(matching_msg_dir, {"sample": SAMPLE_HASH})
    make_vkmsg(differing_msg_dir, {"sample": 0x9999})

    monitor_mod = load_link_monitor()
    monkeypatch.setattr(monitor_mod, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(
        monitor_mod.LinkMonitor,
        "run",
        lambda self, duration: (_ for _ in ()).throw(KeyboardInterrupt),
    )

    with FakeMcu() as mcu:
        monkeypatch.setattr(
            sys,
            "argv",
            [
                "link_monitor.py",
                "--udp",
                f"127.0.0.1:{mcu.port}",
                "--msg-dir",
                str(current_msg_dir),
                "--handshake-timeout",
                "2",
            ],
        )
        assert monitor_mod.main() == 0

    err = capsys.readouterr().err
    assert "warning: topic sample has a different schema hash on the MCU" in err
    expected_hint = (
        "hint: the MCU matches the build tree in build/c50c-v1_1; "
        "pass --msg-dir build/c50c-v1_1/generated/schema/python"
    )
    assert expected_hint in err
    assert "sitl_alt" not in err
    assert "stale" not in err

    # When no build tree matches the MCU's hash:
    make_vkmsg(matching_msg_dir, {"sample": 0x1111})
    with FakeMcu() as mcu:
        monkeypatch.setattr(
            sys,
            "argv",
            [
                "link_monitor.py",
                "--udp",
                f"127.0.0.1:{mcu.port}",
                "--msg-dir",
                str(current_msg_dir),
                "--handshake-timeout",
                "2",
            ],
        )
        assert monitor_mod.main() == 0

    err = capsys.readouterr().err
    assert "warning: topic sample has a different schema hash on the MCU" in err
    assert "hint:" not in err


def test_hint_path_works_from_another_directory(tmp_path, monkeypatch, capsys):
    """When run from outside the repository root, the printed --msg-dir works as typed."""
    matching_msg_dir = tmp_path / "build" / "c50c-v1_1" / "generated" / "schema" / "python"
    stale_msg_dir = tmp_path / "build" / "stale" / "generated" / "schema" / "python"
    make_vkmsg(matching_msg_dir, {"sample": SAMPLE_HASH})
    make_vkmsg(stale_msg_dir, {"sample": 0xDEAD})

    workdir = tmp_path / "tools" / "link"
    workdir.mkdir(parents=True)
    monkeypatch.chdir(workdir)

    # 1. Direct function check
    matching = find_matching_build_trees(tmp_path, {"sample": SAMPLE_HASH})
    assert matching == [matching_msg_dir]

    hint = format_build_tree_hint(matching[0])
    prefix = "pass --msg-dir "
    assert prefix in hint
    suggested_msg_dir = Path(hint.split(prefix)[1].strip())
    assert suggested_msg_dir.resolve() == matching_msg_dir.resolve()
    assert (suggested_msg_dir / "vkmsg.py").is_file()

    # 2. CLI check: running from tools/link/ outputs a working --msg-dir
    monitor_mod = load_link_monitor()
    monkeypatch.setattr(monitor_mod, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(
        monitor_mod.LinkMonitor,
        "run",
        lambda self, duration: (_ for _ in ()).throw(KeyboardInterrupt),
    )

    with FakeMcu() as mcu:
        monkeypatch.setattr(
            sys,
            "argv",
            [
                "link_monitor.py",
                "--udp",
                f"127.0.0.1:{mcu.port}",
                "--msg-dir",
                str(stale_msg_dir),
                "--handshake-timeout",
                "2",
            ],
        )
        assert monitor_mod.main() == 0

    err = capsys.readouterr().err
    assert "hint: the MCU matches the build tree in " in err
    cli_suggested = Path(err.split(prefix)[1].splitlines()[0].strip())
    assert cli_suggested.resolve() == matching_msg_dir.resolve()
    assert (cli_suggested / "vkmsg.py").is_file()


def test_skip_current_message_directory(tmp_path):
    """The message directory already in use is skipped even if its hashes match."""
    msg_dir = tmp_path / "build" / "sitl" / "generated" / "schema" / "python"
    make_vkmsg(msg_dir, {"sample": SAMPLE_HASH})

    matching = find_matching_build_trees(
        tmp_path,
        {"sample": SAMPLE_HASH},
        current_msg_dir=msg_dir,
    )
    assert matching == []


def test_tree_matches_only_when_every_mismatched_topic_matches(tmp_path):
    """A tree matches only when every mismatched topic exists in it with the MCU's hash."""
    tree_both = tmp_path / "build" / "tree_both" / "generated" / "schema" / "python"
    tree_partial = tmp_path / "build" / "tree_partial" / "generated" / "schema" / "python"
    tree_differ = tmp_path / "build" / "tree_differ" / "generated" / "schema" / "python"

    make_vkmsg(tree_both, {"sample": SAMPLE_HASH, "imu_sample": 0x2222})
    make_vkmsg(tree_partial, {"sample": SAMPLE_HASH})  # missing imu_sample
    make_vkmsg(tree_differ, {"sample": SAMPLE_HASH, "imu_sample": 0x9999})  # differing imu_sample

    mcu_hashes = {"sample": SAMPLE_HASH, "imu_sample": 0x2222}

    matching = find_matching_build_trees(tmp_path, mcu_hashes)
    assert matching == [tree_both]


def test_skip_corrupt_and_missing_vkmsg(tmp_path):
    """Trees without vkmsg.py or whose vkmsg.py cannot be loaded are skipped cleanly."""
    build_dir = tmp_path / "build"

    # 1. No vkmsg.py
    empty_tree = build_dir / "empty" / "generated" / "schema" / "python"
    empty_tree.mkdir(parents=True)

    # 2. Syntax error in vkmsg.py
    broken_tree = build_dir / "broken" / "generated" / "schema" / "python"
    broken_tree.mkdir(parents=True)
    (broken_tree / "vkmsg.py").write_text("def broken(:\n", encoding="utf-8")

    # 3. Not a directory
    (build_dir / "random_file.txt").write_text("hello", encoding="utf-8")

    # 4. Valid matching tree
    valid_tree = build_dir / "valid" / "generated" / "schema" / "python"
    make_vkmsg(valid_tree, {"sample": SAMPLE_HASH})

    matching = find_matching_build_trees(tmp_path, {"sample": SAMPLE_HASH})
    assert matching == [valid_tree]


def test_no_build_directory_returns_empty(tmp_path):
    """Running outside a build tree (such as from a bundle) returns empty without error."""
    bundle_dir = tmp_path / "bundle"
    bundle_dir.mkdir()
    assert find_matching_build_trees(bundle_dir, {"sample": SAMPLE_HASH}) == []


def test_no_mismatches_returns_empty(tmp_path):
    """When there are no mismatched topics, no build tree matches."""
    msg_dir = tmp_path / "build" / "c50c" / "generated" / "schema" / "python"
    make_vkmsg(msg_dir, {"sample": SAMPLE_HASH})
    assert find_matching_build_trees(tmp_path, {}) == []


def test_stable_order_multiple_matches(tmp_path):
    """Multiple matching build trees are returned in stable alphabetical order."""
    tree_c = tmp_path / "build" / "charlie" / "generated" / "schema" / "python"
    tree_a = tmp_path / "build" / "alpha" / "generated" / "schema" / "python"
    tree_b = tmp_path / "build" / "bravo" / "generated" / "schema" / "python"

    make_vkmsg(tree_c, {"sample": SAMPLE_HASH})
    make_vkmsg(tree_a, {"sample": SAMPLE_HASH})
    make_vkmsg(tree_b, {"sample": SAMPLE_HASH})

    matching = find_matching_build_trees(tmp_path, {"sample": SAMPLE_HASH})
    assert matching == [tree_a, tree_b, tree_c]
