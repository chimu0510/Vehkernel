# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests for agy_watch.py: decoding steps and reading a conversation from a copy."""

from __future__ import annotations

from pathlib import Path
import sqlite3
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import agy_watch  # noqa: E402

CONVERSATION = "69a4d068-88bc-4eab-9b16-61a35488d8d5"


def varint(value: int) -> bytes:
    out = bytearray()
    while True:
        byte = value & 0x7F
        value >>= 7
        out.append(byte | (0x80 if value else 0))
        if not value:
            return bytes(out)


def field(number: int, value: bytes | str | int) -> bytes:
    if isinstance(value, int):
        return varint(number << 3) + varint(value)
    data = value.encode() if isinstance(value, str) else value
    return varint(number << 3 | 2) + varint(len(data)) + data


def user_step(text: str) -> bytes:
    return field(19, field(2, text) + field(5, 7))


def model_step(thought: str, calls: list[tuple[str, str]]) -> bytes:
    body = field(3, thought) + b"".join(field(7, field(1, "call_1") + field(2, n) + field(3, a)) for n, a in calls)
    return field(20, body)


def tool_step(tool: str, args: dict[str, str], result: str) -> bytes:
    arg_fields = b"".join(field(1, field(1, k) + field(2, v)) for k, v in args.items())
    return field(140, arg_fields + field(2, field(1, result) + field(6, field(1, f"type.googleapis.com/x.{tool}"))))


def make_state(tmp_path: Path, rows: list[tuple[int, int, int, bytes]]) -> tuple[Path, sqlite3.Connection]:
    conversations = tmp_path / "conversations"
    conversations.mkdir()
    database = conversations / f"{CONVERSATION}.db"
    connection = sqlite3.connect(database)
    connection.execute("PRAGMA journal_mode=WAL")
    connection.execute("CREATE TABLE steps (idx integer, step_type integer, status integer, step_payload blob)")
    connection.executemany("INSERT INTO steps VALUES (?, ?, ?, ?)", rows)
    connection.commit()
    # Left open, as agy leaves it: the rows are in the -wal file only.
    return tmp_path, connection


def test_protobuf_strings_reads_nested_text_with_paths():
    data = field(1, 150) + field(2, field(3, "inner")) + field(4, "outer")
    assert agy_watch.protobuf_strings(data) == [((2, 3), "inner"), ((4,), "outer")]


def test_protobuf_strings_rejects_what_is_not_a_message():
    with pytest.raises(ValueError):
        agy_watch.protobuf_strings(b"\x0a\x05ab")


def test_describe_names_each_kind_of_step():
    prompt = agy_watch.describe(0, agy_watch.STEP_USER, 3, user_step("Implement ticket #151"))
    assert (prompt.kind, prompt.status, prompt.lines) == ("prompt", "done", ["Implement ticket #151"])

    model = agy_watch.describe(1, agy_watch.STEP_MODEL, 8, model_step("Reading the spec", [("read_file", '{"Path":"a"}')]))
    assert (model.kind, model.status) == ("agy", "running")
    assert model.lines == ["Reading the spec", '-> read_file {"Path":"a"}']

    tool = agy_watch.describe(2, agy_watch.STEP_TOOL, 3,
                              tool_step("RunCommand", {"CommandLine": "ls", "toolSummary": "list"}, "exit 0"))
    assert (tool.kind, tool.lines) == ("RunCommand", ["CommandLine=ls", "exit 0"])


def test_describe_shows_a_file_content_by_its_length():
    tool = agy_watch.describe(3, agy_watch.STEP_TOOL, 3, tool_step("FileEdit", {"Content": "a\nb"}, "Created file."))
    assert tool.lines[0] == "Content=<3 chars>"


def test_describe_survives_an_unknown_or_broken_step():
    assert agy_watch.describe(4, 999, 42, b"\x0a\x05ab") == agy_watch.Step(4, "step type 999", "status 42", [])


def test_read_steps_reads_a_copy_and_leaves_the_conversation_unchanged(tmp_path):
    state, connection = make_state(tmp_path, [
        (0, agy_watch.STEP_USER, 3, user_step("go")),
        (1, agy_watch.STEP_MODEL, 8, model_step("thinking", [])),
    ])
    files = sorted((tmp_path / "conversations").iterdir())
    before = {p.name: (p.stat().st_mtime_ns, p.read_bytes()) for p in files}

    steps = agy_watch.read_steps(state, CONVERSATION)
    assert [(s.index, s.kind, s.status) for s in steps] == [(0, "prompt", "done"), (1, "agy", "running")]
    assert [s.index for s in agy_watch.read_steps(state, CONVERSATION, after=0)] == [1]

    after = {p.name: (p.stat().st_mtime_ns, p.read_bytes()) for p in sorted((tmp_path / "conversations").iterdir())}
    assert after == before
    connection.close()


def test_read_steps_refuses_a_path_instead_of_an_id(tmp_path):
    with pytest.raises(agy_watch.WatchError, match="not a conversation id"):
        agy_watch.read_steps(tmp_path, "../conversations/x")


def test_main_reports_a_missing_conversation(tmp_path, capsys):
    (tmp_path / "conversations").mkdir()
    assert agy_watch.main([CONVERSATION, "--state-dir", str(tmp_path)]) == 1
    assert "no conversation" in capsys.readouterr().err


def test_main_prints_the_last_steps(tmp_path, capsys):
    state, connection = make_state(tmp_path, [
        (i, agy_watch.STEP_USER, 3, user_step(f"message {i}")) for i in range(5)
    ])
    assert agy_watch.main([CONVERSATION, "--state-dir", str(state), "--last", "2"]) == 0
    out = capsys.readouterr().out
    assert "message 3" in out and "message 4" in out and "message 2" not in out
    connection.close()
