#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Cuts a log from its estimator_restart to a length, keeping the replay's topics (spec 0011, ADR-0018 L2)."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "log"))

import vkulog  # noqa: E402
from vkulog.reader import DataMessage, ParameterChange  # noqa: E402

# What the replay reads and what it is compared with (replay.py, compare.py).
REPLAY_TOPICS = (
    "imu_sample",
    "body_velocity_measurement",
    "estimator_restart",
    "vehicle_attitude",
    "vehicle_local_odometry",
)


def cut(
    in_path: str | Path,
    out_path: str | Path,
    duration_s: float,
    topics: tuple[str, ...] = REPLAY_TOPICS,
) -> None:
    """Writes the part of a log from its first estimator_restart for `duration_s`, and only `topics`.

    The formats, information and initial parameters are kept whole; a parameter that changes inside the cut
    is kept at its time. Message ids stay as they were.
    """
    reader = vkulog.ULogReader(in_path)
    restart_ids = [i for i, sub in reader.subscriptions.items() if sub.message_name == "estimator_restart"]
    restarts = [r.timestamp_us for r in reader.data_messages if r.msg_id in restart_ids]
    if not restarts:
        raise ValueError(f"{in_path} holds no estimator_restart")
    start_us = min(restarts)
    end_us = start_us + round(duration_s * 1e6)

    kept = {i: sub for i, sub in reader.subscriptions.items() if sub.message_name in topics}

    with vkulog.ULogWriter(out_path, start_timestamp_us=reader.start_timestamp_us) as writer:
        for name in topics:
            if name in reader.formats and any(sub.message_name == name for sub in kept.values()):
                writer.write_format(reader.formats[name].raw)
        for key, value in reader.info.items():
            writer.write_info(key, value)
        for name, value in reader.parameters.items():
            writer.write_parameter(name, value)
        for msg_id, sub in kept.items():
            writer.write_add_logged(msg_id, sub.message_name, sub.multi_id)

        # Records are in time order; a parameter change takes the time of the data message before it.
        for record in reader.records:
            if isinstance(record, DataMessage):
                if record.msg_id in kept and start_us <= record.timestamp_us <= end_us:
                    writer.write_data(record.msg_id, record.data)
            elif isinstance(record, ParameterChange):
                if start_us <= record.timestamp_us <= end_us:
                    writer.write_parameter(record.name, record.value)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("in_log", help="log to cut")
    parser.add_argument("out_log", help="path for the cut")
    parser.add_argument("--duration", type=float, required=True, help="seconds to keep, from the first estimator_restart")
    parser.add_argument(
        "--topics",
        default=",".join(REPLAY_TOPICS),
        help="comma-separated topics to keep (default: the replay's inputs and outputs)",
    )
    args = parser.parse_args()
    try:
        cut(args.in_log, args.out_log, args.duration, tuple(args.topics.split(",")))
    except ValueError as err:
        print(f"ERROR: {err}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
