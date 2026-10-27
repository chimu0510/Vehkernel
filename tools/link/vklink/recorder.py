# SPDX-License-Identifier: GPL-3.0-or-later
"""Host ULog recorder over a Link (spec 0011, ADR-0019)."""

from __future__ import annotations

import struct
import sys
from pathlib import Path
from typing import TYPE_CHECKING

try:
    import vkulog
except ImportError:
    log_dir = Path(__file__).resolve().parents[2] / "log"
    if log_dir.exists():
        sys.path.insert(0, str(log_dir))
    import vkulog
from vklink.frame import Frame, FrameType, LogDropout, LogStartResult
from vklink.session import ParamValue, name_hash

if TYPE_CHECKING:
    from vklink.monitor import LinkMonitor

PARAM_TIMEOUT_S = 5.0
SYNC_INTERVAL_S = 1.0


class ULogRecorder:
    """Records a ULog file over a LinkMonitor for a chosen log profile."""

    def __init__(
        self,
        monitor: LinkMonitor,
        profile_name: str,
        output_file: Path | str,
        duration: float | None = None,
        msg_dir: str | Path | None = None,
        handshake_timeout: float = 5.0,
    ) -> None:
        self.monitor = monitor
        self.profile_name = profile_name
        self.output_file = Path(output_file)
        self.duration = duration
        self.msg_dir = str(msg_dir) if msg_dir is not None else None
        self.handshake_timeout = handshake_timeout

    def record(self) -> int:
        """Runs the recorder; returns 0 on success, non-zero on failure."""
        try:
            from vklog_profiles import PROFILES  # type: ignore
        except ImportError:
            if self.msg_dir is not None:
                sys.path.insert(0, self.msg_dir)
            from vklog_profiles import PROFILES  # type: ignore

        profile = PROFILES.get(self.profile_name)
        if profile is None:
            print(f"error: unknown log profile '{self.profile_name}'", file=sys.stderr)
            return 2

        # 1. Read parameters before logging starts
        params = self.monitor.list_params(PARAM_TIMEOUT_S)
        if params is None:
            print("FAIL: could not read parameters from MCU", file=sys.stderr)
            return 1

        hash_to_name: dict[int, str] = {name_hash(p.name): p.name for p in params}

        # 2. Determine initial start timestamp for ULog header
        now_host = self.monitor.clock()
        if self.monitor.time_sync is not None:
            start_timestamp_us = max(0, int(now_host * 1e6) + self.monitor.time_sync.clock_offset_us)
        else:
            start_timestamp_us = max(0, int(now_host * 1e6))

        # 3. Request log_start from the MCU
        log_info = self.monitor.start_logging(int(profile.id), self.handshake_timeout)
        if log_info is None:
            print("FAIL: no log_info received from MCU", file=sys.stderr)
            return 1

        if log_info.status != LogStartResult.OK:
            print(f"FAIL: log_start refused: {log_info.status.name.lower()}", file=sys.stderr)
            return 1

        writer = vkulog.ULogWriter(self.output_file, start_timestamp_us=start_timestamp_us)

        # 4. Write Formats (F)
        written_formats: set[str] = set()
        for entry in profile.entries:
            if entry.topic in vkulog.FORMATS and entry.topic not in written_formats:
                writer.write_format(vkulog.FORMATS[entry.topic])
                written_formats.add(entry.topic)

        # 5. Write Info (I) from log_info and schema hashes
        writer.write_info("sys_name", log_info.board_name)
        writer.write_info("board_id", log_info.board_id)
        writer.write_info("ver_sw", f"{log_info.version_major}.{log_info.version_minor}")
        writer.write_info("ver_git", f"{log_info.git_hash:08x}")
        writer.write_info("bandwidth", log_info.bandwidth)
        for peer in self.monitor.session.peer_topics.values():
            writer.write_info(f"sys_schema_{peer.name}", f"0x{peer.schema_hash:016X}")

        # 6. Write initial Parameters (P)
        for p in params:
            writer.write_parameter(p.name, p.value)

        # 7. Write Add Logged subscriptions (A)
        topic_map_msg_id: dict[tuple[int, int], int] = {}
        entry_rates: dict[tuple[int, int], float] = {}
        next_msg_id = 1
        for entry in profile.entries:
            inst = entry.instance if entry.instance is not None else 0
            topic_map_msg_id[(entry.topic_id, inst)] = next_msg_id
            entry_rates[(entry.topic_id, inst)] = entry.rate_hz
            writer.write_add_logged(next_msg_id, entry.topic, multi_id=inst)
            next_msg_id += 1

        param_update_topic = self.monitor.session.peer_topics_by_name.get("parameter_update")
        param_update_id = param_update_topic.id if param_update_topic else None

        # 8. Data section tracking
        prev_lost = self.monitor.reader.stats.lost
        last_sync = self.monitor.clock()
        start_time = self.monitor.clock()

        last_frame_mcu_ts_us: int | None = None
        last_frame_host_s: float = start_time
        gap_pending: bool = False
        gap_start_mcu_us: int | None = None
        gap_start_host_s: float = start_time

        pending_params: list[str] = []

        def on_frame(frame: Frame) -> None:
            nonlocal prev_lost, last_frame_mcu_ts_us, last_frame_host_s
            nonlocal gap_pending, gap_start_mcu_us, gap_start_host_s

            lost_diff = self.monitor.reader.stats.lost - prev_lost
            if lost_diff > 0:
                gap_pending = True
                gap_start_mcu_us = last_frame_mcu_ts_us
                gap_start_host_s = last_frame_host_s
                prev_lost = self.monitor.reader.stats.lost

            now_host_frame = self.monitor.clock()

            if frame.type == FrameType.TOPIC_DATA and len(frame.body) >= 3:
                topic_id = int.from_bytes(frame.body[:2], "little")
                instance = frame.body[2]
                msg_data = frame.body[3:]

                cur_mcu_ts_us: int | None = None
                if len(msg_data) >= 8:
                    cur_mcu_ts_us = struct.unpack("<Q", msg_data[:8])[0]

                if gap_pending:
                    if cur_mcu_ts_us is not None and gap_start_mcu_us is not None:
                        dur_ms = int(round((cur_mcu_ts_us - gap_start_mcu_us) / 1000.0))
                    else:
                        dur_ms = int(round((now_host_frame - gap_start_host_s) * 1000.0))
                    writer.write_dropout(max(1, dur_ms))
                    gap_pending = False

                if cur_mcu_ts_us is not None:
                    last_frame_mcu_ts_us = cur_mcu_ts_us
                last_frame_host_s = now_host_frame

                key = (topic_id, instance)
                if key in topic_map_msg_id:
                    writer.write_data(topic_map_msg_id[key], msg_data)

                if param_update_id is not None and topic_id == param_update_id and len(msg_data) >= 12:
                    _ts, p_hash = struct.unpack("<QI", msg_data[:12])
                    pname = hash_to_name.get(p_hash)
                    if pname and pname not in pending_params:
                        pending_params.append(pname)

            elif frame.type == FrameType.LOG_DROPOUT and len(frame.body) >= 5:
                drop = LogDropout.parse(frame.body)
                if drop is not None:
                    rate = entry_rates.get((drop.topic_id, drop.instance), 50.0)
                    period_ms = 1000.0 / rate if rate > 0 else 1.0
                    writer.write_dropout(max(1, int(round(drop.count * period_ms))))

        self.monitor.frame_listeners.append(on_frame)
        try:
            while True:
                now = self.monitor.clock()
                if self.duration is not None and now - start_time >= self.duration:
                    break

                if now - last_sync >= SYNC_INTERVAL_S:
                    writer.write_sync()
                    last_sync = now

                # Drain queued parameters without handler re-entrancy
                while pending_params:
                    pname = pending_params.pop(0)
                    pv = self.monitor.get_param(pname, timeout_s=1.0)
                    if isinstance(pv, ParamValue):
                        writer.write_parameter(pname, pv.value)

                remaining = (start_time + self.duration - now) if self.duration is not None else 0.05
                self.monitor.poll(min(0.05, max(0.0, remaining)))
        finally:
            self.monitor.stop_logging()
            writer.close()
            if on_frame in self.monitor.frame_listeners:
                self.monitor.frame_listeners.remove(on_frame)

        return 0


def record(
    monitor: LinkMonitor,
    profile_name: str,
    output_file: Path | str,
    duration: float | None = None,
    msg_dir: str | Path | None = None,
    handshake_timeout: float = 5.0,
) -> int:
    """Functional wrapper around ULogRecorder."""
    recorder = ULogRecorder(
        monitor,
        profile_name,
        output_file,
        duration=duration,
        msg_dir=msg_dir,
        handshake_timeout=handshake_timeout,
    )
    return recorder.record()
