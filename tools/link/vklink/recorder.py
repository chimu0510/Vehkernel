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

        self.writer: vkulog.ULogWriter | None = None
        self._on_frame_fn = None
        self._last_sync: float = 0.0
        self._start_time: float = 0.0
        self._pending_params: list[str] = []
        self._hash_to_name: dict[int, str] = {}
        self._is_recording: bool = False

    def start(self) -> bool:
        """Starts recording session: requests log_start, sets up writer and frame listeners."""
        if self._is_recording:
            return True

        try:
            from vklog_profiles import PROFILES  # type: ignore
        except ImportError:
            if self.msg_dir is not None:
                sys.path.insert(0, self.msg_dir)
            from vklog_profiles import PROFILES  # type: ignore

        profile = PROFILES.get(self.profile_name)
        if profile is None:
            print(f"error: unknown log profile '{self.profile_name}'", file=sys.stderr)
            return False

        # 1. Read parameters before logging starts
        params = self.monitor.list_params(PARAM_TIMEOUT_S)
        if params is None:
            print("FAIL: could not read parameters from MCU", file=sys.stderr)
            return False

        self._hash_to_name = {name_hash(p.name): p.name for p in params}

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
            return False

        if log_info.status != LogStartResult.OK:
            print(f"FAIL: log_start refused: {log_info.status.name.lower()}", file=sys.stderr)
            return False

        self.writer = vkulog.ULogWriter(self.output_file, start_timestamp_us=start_timestamp_us)

        # 4. Write Formats (F)
        written_formats: set[str] = set()
        for entry in profile.entries:
            if entry.topic in vkulog.FORMATS and entry.topic not in written_formats:
                self.writer.write_format(vkulog.FORMATS[entry.topic])
                written_formats.add(entry.topic)

        # 5. Write Info (I) from log_info and schema hashes
        self.writer.write_info("sys_name", log_info.board_name)
        self.writer.write_info("board_id", log_info.board_id)
        self.writer.write_info("ver_sw", f"{log_info.version_major}.{log_info.version_minor}")
        self.writer.write_info("ver_git", f"{log_info.git_hash:08x}")
        self.writer.write_info("bandwidth", log_info.bandwidth)
        for peer in self.monitor.session.peer_topics.values():
            self.writer.write_info(f"sys_schema_{peer.name}", f"0x{peer.schema_hash:016X}")

        # 6. Write initial Parameters (P)
        for p in params:
            self.writer.write_parameter(p.name, p.value)

        # 7. Write Add Logged subscriptions (A)
        topic_map_msg_id: dict[tuple[int, int], int] = {}
        entry_rates: dict[tuple[int, int], float] = {}
        next_msg_id = 1
        for entry in profile.entries:
            inst = entry.instance if entry.instance is not None else 0
            topic_map_msg_id[(entry.topic_id, inst)] = next_msg_id
            entry_rates[(entry.topic_id, inst)] = entry.rate_hz
            self.writer.write_add_logged(next_msg_id, entry.topic, multi_id=inst)
            next_msg_id += 1

        param_update_topic = self.monitor.session.peer_topics_by_name.get("parameter_update")
        param_update_id = param_update_topic.id if param_update_topic else None

        # 8. Data section tracking
        prev_lost = self.monitor.reader.stats.lost
        self._last_sync = self.monitor.clock()
        self._start_time = self.monitor.clock()

        last_frame_mcu_ts_us: int | None = None
        last_frame_host_s: float = self._start_time
        gap_pending: bool = False
        gap_start_mcu_us: int | None = None
        gap_start_host_s: float = self._start_time

        self._pending_params = []

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
                    if self.writer is not None:
                        self.writer.write_dropout(max(1, dur_ms))
                    gap_pending = False

                if cur_mcu_ts_us is not None:
                    last_frame_mcu_ts_us = cur_mcu_ts_us
                last_frame_host_s = now_host_frame

                key = (topic_id, instance)
                if key in topic_map_msg_id and self.writer is not None:
                    self.writer.write_data(topic_map_msg_id[key], msg_data)

                if param_update_id is not None and topic_id == param_update_id and len(msg_data) >= 12:
                    _ts, p_hash = struct.unpack("<QI", msg_data[:12])
                    pname = self._hash_to_name.get(p_hash)
                    if pname and pname not in self._pending_params:
                        self._pending_params.append(pname)

            elif frame.type == FrameType.LOG_DROPOUT and len(frame.body) >= 5:
                drop = LogDropout.parse(frame.body)
                if drop is not None and self.writer is not None:
                    rate = entry_rates.get((drop.topic_id, drop.instance), 50.0)
                    period_ms = 1000.0 / rate if rate > 0 else 1.0
                    self.writer.write_dropout(max(1, int(round(drop.count * period_ms))))

        self._on_frame_fn = on_frame
        self.monitor.frame_listeners.append(on_frame)
        self._is_recording = True
        return True

    def update(self) -> None:
        """Flushes sync markers and drains pending parameter changes."""
        if not self._is_recording or self.writer is None:
            return
        now = self.monitor.clock()
        if now - self._last_sync >= SYNC_INTERVAL_S:
            self.writer.write_sync()
            self._last_sync = now

        while self._pending_params:
            pname = self._pending_params.pop(0)
            pv = self.monitor.get_param(pname, timeout_s=1.0)
            if isinstance(pv, ParamValue):
                self.writer.write_parameter(pname, pv.value)

    def stop(self) -> None:
        """Stops recording session: sends log_stop, unregisters listeners and closes writer."""
        if not self._is_recording:
            return
        self.monitor.stop_logging()
        if self._on_frame_fn in self.monitor.frame_listeners:
            self.monitor.frame_listeners.remove(self._on_frame_fn)
        self._on_frame_fn = None
        if self.writer is not None:
            self.writer.close()
            self.writer = None
        self._is_recording = False

    def record(self) -> int:
        """Runs the recorder; returns 0 on success, non-zero on failure."""
        if not self.start():
            return 1
        try:
            while True:
                now = self.monitor.clock()
                if self.duration is not None and now - self._start_time >= self.duration:
                    break

                self.update()

                remaining = (self._start_time + self.duration - now) if self.duration is not None else 0.05
                self.monitor.poll(min(0.05, max(0.0, remaining)))
        finally:
            self.stop()

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
