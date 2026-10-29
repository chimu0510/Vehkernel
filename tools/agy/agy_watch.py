#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Shows what an agy conversation is doing, without touching it (docs/guide/delegating-to-agy.md).

agy keeps each conversation in a SQLite database, `conversations/<id>.db`
with its `-wal`, under its state directory. Opening that conversation in
another agy (`/resume`) interrupts a delegation that is running in it. This
tool never runs agy and never opens agy's files for writing: it copies the
database and its write-ahead log into a private temporary directory, reads
the copy, and deletes it. It takes no lock and leaves agy's presence files
alone, so agy cannot tell it was there.

The steps are protobuf messages with no published schema; the field numbers
below were read from agy 1.4.9's databases. A field this tool does not know
is skipped, so a newer agy shows less, never something wrong.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import re
import shutil
import sqlite3
import sys
import tempfile
import time

DEFAULT_STATE_DIR = Path.home() / ".gemini" / "antigravity-cli"
CONVERSATION_ID = re.compile(r"^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$")

# Step types and statuses seen in agy 1.4.9.
STEP_USER = 14
STEP_MODEL = 15
STEP_SYSTEM = 101
STEP_TOOL = 132
STATUS_NAMES = {3: "done", 6: "stopped", 8: "running"}


class WatchError(Exception):
    pass


def _varint(data: bytes, pos: int) -> tuple[int, int]:
    value = shift = 0
    while True:
        if pos >= len(data):
            raise ValueError("truncated varint")
        byte = data[pos]
        pos += 1
        value |= (byte & 0x7F) << shift
        shift += 7
        if byte < 0x80:
            return value, pos
        if shift > 63:
            raise ValueError("varint too long")


def protobuf_strings(data: bytes, path: tuple[int, ...] = (), depth: int = 0) -> list[tuple[tuple[int, ...], str]]:
    """Returns every text field of a protobuf message, with its field path, in order.

    Without a schema a length-delimited field is either a nested message or
    bytes; it is read as a message when it parses as one, else as UTF-8 text.
    Raises ValueError when `data` is not a protobuf message.
    """
    out: list[tuple[tuple[int, ...], str]] = []
    pos = 0
    while pos < len(data):
        key, pos = _varint(data, pos)
        field, wire = key >> 3, key & 7
        if field == 0:
            raise ValueError("field 0")
        if wire == 0:
            _, pos = _varint(data, pos)
        elif wire == 1:
            pos += 8
        elif wire == 5:
            pos += 4
        elif wire == 2:
            length, pos = _varint(data, pos)
            value = data[pos:pos + length]
            pos += length
            if pos > len(data):
                raise ValueError("truncated field")
            nested = None
            if depth < 12 and length > 1:
                try:
                    nested = protobuf_strings(value, path + (field,), depth + 1)
                except ValueError:
                    nested = None
            if nested:
                out.extend(nested)
            else:
                try:
                    text = value.decode("utf-8")
                except UnicodeDecodeError:
                    continue
                if text and (text.isprintable() or "\n" in text):
                    out.append((path + (field,), text))
        else:
            raise ValueError(f"wire type {wire}")
        if pos > len(data):
            raise ValueError("truncated message")
    return out


@dataclass
class Step:
    index: int
    kind: str
    status: str
    lines: list[str]


def _first(fields: list[tuple[tuple[int, ...], str]], path: tuple[int, ...]) -> str | None:
    return next((text for p, text in fields if p == path), None)


def _all(fields: list[tuple[tuple[int, ...], str]], path: tuple[int, ...]) -> list[str]:
    return [text for p, text in fields if p == path]


def describe(index: int, step_type: int, status: int, payload: bytes | None) -> Step:
    """Turns one row of the steps table into what a person reads."""
    try:
        fields = protobuf_strings(payload or b"")
    except ValueError:
        fields = []
    status_name = STATUS_NAMES.get(status, f"status {status}")
    if step_type == STEP_USER:
        text = _first(fields, (19, 2)) or ""
        return Step(index, "prompt", status_name, [text])
    if step_type == STEP_MODEL:
        lines = []
        thought = _first(fields, (20, 3))
        if thought:
            lines.append(thought)
        names = _all(fields, (20, 7, 2))
        args = _all(fields, (20, 7, 3))
        for name, arg in zip(names, args):
            lines.append(f"-> {name} {arg}")
        return Step(index, "agy", status_name, lines)
    if step_type == STEP_TOOL:
        tool = _first(fields, (140, 2, 6, 1)) or ""
        tool = tool.rsplit(".", 1)[-1] or "tool"
        keys = _all(fields, (140, 1, 1))
        values = _all(fields, (140, 1, 2))
        args = {k: v for k, v in zip(keys, values) if k not in ("toolSummary", "WaitMsBeforeAsync")}
        # A file's content is shown by its length; the diff shows the rest.
        lines = [" ".join(f"{k}=<{len(v)} chars>" if "\n" in v or len(v) > 200 else f"{k}={v}"
                          for k, v in args.items())]
        result = _first(fields, (140, 2, 1))
        if result:
            lines.append(result)
        return Step(index, tool, status_name, lines)
    if step_type == STEP_SYSTEM:
        text = _first(fields, (114, 2, 10, 1)) or _first(fields, (114, 1)) or ""
        return Step(index, "system", status_name, [text])
    return Step(index, f"step type {step_type}", status_name, [])


def read_steps(state_dir: Path, conversation_id: str, after: int = -1) -> list[Step]:
    """Reads the steps of a conversation after index `after`, from a private copy of its database."""
    if not CONVERSATION_ID.match(conversation_id):
        raise WatchError(f"not a conversation id: {conversation_id!r}")
    database = state_dir / "conversations" / f"{conversation_id}.db"
    if not database.is_file():
        raise WatchError(f"no conversation {conversation_id} in {state_dir / 'conversations'}")
    with tempfile.TemporaryDirectory(prefix="agy-watch-") as tmp:
        copy = Path(tmp) / "conversation.db"
        # The database first, then its log: a log copied mid-write ends in
        # frames whose checksums fail, and SQLite ignores them.
        shutil.copyfile(database, copy)
        wal = database.with_name(database.name + "-wal")
        if wal.is_file():
            shutil.copyfile(wal, copy.with_name(copy.name + "-wal"))
        connection = sqlite3.connect(copy)
        try:
            rows = connection.execute(
                "SELECT idx, step_type, status, step_payload FROM steps WHERE idx > ? ORDER BY idx", (after,)
            ).fetchall()
        except sqlite3.DatabaseError as error:
            raise WatchError(f"cannot read the copy of {database}: {error}") from error
        finally:
            connection.close()
    return [describe(*row) for row in rows]


def format_step(step: Step, width: int, full: bool) -> str:
    head = f"[{step.index:3d}] {step.kind:<14} {step.status:<8}"
    out = [head]
    for line in step.lines:
        if full:
            out.extend("    " + part for part in line.splitlines())
            continue
        text = " ".join(line.split())
        if len(text) > width:
            text = text[:width - 3] + "..."
        out.append("    " + text)
    return "\n".join(out)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("conversation_id", help="the agy conversation id, as the AGY_USAGE line of a round's log gives it")
    parser.add_argument("--state-dir", type=Path, default=DEFAULT_STATE_DIR,
                        help=f"agy's state directory (default {DEFAULT_STATE_DIR})")
    parser.add_argument("--last", type=int, default=0, metavar="N", help="show only the last N steps")
    parser.add_argument("--follow", action="store_true", help="keep showing new steps until interrupted")
    parser.add_argument("--interval", type=float, default=5.0, help="seconds between reads with --follow (default 5)")
    parser.add_argument("--width", type=int, default=160, help="cut each line to this width (default 160)")
    parser.add_argument("--full", action="store_true", help="show whole texts and tool outputs")
    args = parser.parse_args(argv)

    try:
        steps = read_steps(args.state_dir, args.conversation_id)
        if args.last > 0:
            steps = steps[-args.last:]
        for step in steps:
            print(format_step(step, args.width, args.full), flush=True)
        if not args.follow:
            return 0
        # A step's status changes after it is first shown (running, then
        # done), so the last one shown is read again on the next pass.
        shown = steps[-1].index - 1 if steps else -1
        last = steps[-1] if steps else None
        while True:
            time.sleep(args.interval)
            for step in read_steps(args.state_dir, args.conversation_id, shown):
                if last is not None and step.index == last.index and step == last:
                    continue
                print(format_step(step, args.width, args.full), flush=True)
                last = step
            if last is not None:
                shown = last.index - 1
    except WatchError as error:
        print(f"agy_watch: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
