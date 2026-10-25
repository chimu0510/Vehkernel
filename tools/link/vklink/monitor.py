# SPDX-License-Identifier: GPL-3.0-or-later
"""Link monitor for the development host (spec 0003).

Connects to a Link, performs the handshake and time synchronization, decodes
every topic with the Python module generated from the schema, and keeps each
topic instance's rate and latest values. It also lists, reads and sets
parameters (spec 0005), described by the metadata file the parameter
generator writes into the build tree, and sends direct-actuator setpoints
(spec 0006).
"""

from __future__ import annotations

import fcntl
import importlib.util
import json
import math
import os
import select
import socket
import struct
import termios
import time
import tty
from dataclasses import dataclass, field
from pathlib import Path
from types import ModuleType
from typing import Callable, Protocol

from .enums import HandshakeState, MenuState
from .frame import Frame, FrameReader, FrameType
from .session import (
    ACTUATOR_POWER_STATES,
    ARMING_REJECT_REASONS,
    DISARM_REASONS,
    FAULTS,
    MODES,
    REJECT_REASONS,
    RESET_CAUSES,
    BringupReply,
    ControlReply,
    HostSession,
    ParamReject,
    ParamType,
    ParamValue,
    TimeSyncResult,
    check_names,
    value_texts,
)


class Connection(Protocol):
    def send(self, data: bytes) -> None: ...

    def receive(self, timeout_s: float) -> bytes: ...

    def close(self) -> None: ...


class UdpConnection:
    """A Link over UDP, as SITL offers by default (ADR-0018)."""

    def __init__(self, host: str, port: int):
        self._address = (host, port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setblocking(False)

    def send(self, data: bytes) -> None:
        self._socket.sendto(data, self._address)

    def receive(self, timeout_s: float) -> bytes:
        readable, _, _ = select.select([self._socket], [], [], max(timeout_s, 0.0))
        if not readable:
            return b""
        chunks = []
        while True:
            try:
                chunks.append(self._socket.recv(65536))
            except BlockingIOError:
                break
        return b"".join(chunks)

    def close(self) -> None:
        self._socket.close()


class SerialConnection:
    """A Link over a serial port: a board's USB serial adapter, or SITL's pty."""

    def __init__(self, path: str, baud: int = 921600):
        speed = getattr(termios, f"B{baud}", None)
        if speed is None:
            raise ValueError(f"unsupported baud rate {baud}")
        self._fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            # Raw 8N1: every byte passes unchanged, nothing is echoed.
            tty.setraw(self._fd)
            mode = termios.tcgetattr(self._fd)
            mode[2] |= termios.CLOCAL | termios.CREAD
            mode[4] = mode[5] = speed
            termios.tcsetattr(self._fd, termios.TCSANOW, mode)
            termios.tcflush(self._fd, termios.TCIOFLUSH)  # bytes from before we opened it
            self._release_modem_lines()
        except BaseException:
            os.close(self._fd)
            raise

    def _release_modem_lines(self) -> None:
        """Releases DTR and RTS, which opening the port asserts.

        On a port wired for one-key download, such as the C50C's SERIAL2, RTS
        asserted holds BOOT0 high, so a reset while the monitor is connected
        would start the system bootloader. Both are released in one request:
        DTR released first, with RTS still asserted, would reset the board.
        A pty has no modem lines.
        """
        lines = struct.pack("I", termios.TIOCM_DTR | termios.TIOCM_RTS)
        try:
            fcntl.ioctl(self._fd, termios.TIOCMBIC, lines)
        except OSError:
            pass

    def send(self, data: bytes) -> None:
        while data:
            try:
                data = data[os.write(self._fd, data):]
            except BlockingIOError:
                select.select([], [self._fd], [], 0.1)

    def receive(self, timeout_s: float) -> bytes:
        readable, _, _ = select.select([self._fd], [], [], max(timeout_s, 0.0))
        if not readable:
            return b""
        chunks = []
        while True:
            try:
                chunk = os.read(self._fd, 4096)
            except BlockingIOError:
                break
            if not chunk:
                break
            chunks.append(chunk)
        return b"".join(chunks)

    def close(self) -> None:
        os.close(self._fd)


def load_messages(directory: str | Path) -> ModuleType:
    """Imports the generated vkmsg module from a build directory."""
    path = Path(directory) / "vkmsg.py"
    if not path.is_file():
        raise FileNotFoundError(f"{path} does not exist; build a preset first, or pass --msg-dir")
    spec = importlib.util.spec_from_file_location("vkmsg", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# Flags of param_value: vk::core::param_state.
PARAM_PENDING_REBOOT = 1
PARAM_UNSAVED = 2


def vehicle_status_line(status) -> str:
    """vehicle_status as a space-separated key=value line."""
    source = "none" if status.source == 0xFF else str(status.source)
    skipped = ",".join(check_names(status.skipped_checks)) or "none"
    return (f"vehicle_status mode={MODES.get(status.mode, status.mode)} armed={int(status.armed)} "
            f"source={source} board_verified={int(status.board_verified)} "
            f"emergency_stop={int(status.emergency_stop)} "
            f"actuator_power={ACTUATOR_POWER_STATES.get(status.actuator_power, status.actuator_power).replace(' ', '_')} "
            f"skipped_checks={skipped} "
            f"arming_refusal={ARMING_REJECT_REASONS.get(status.arming_refusal, 'none').replace(' ', '_')} "
            f"disarm_reason={DISARM_REASONS.get(status.disarm_reason, status.disarm_reason).replace(' ', '_')} "
            f"output_watchdog_trips={status.output_watchdog_trips} "
            f"reset_cause={RESET_CAUSES.get(status.reset_cause, status.reset_cause).replace(' ', '_')} "
            f"fault={FAULTS.get(status.fault, status.fault).replace(' ', '_')} "
            f"fault_address=0x{status.fault_address:08X}")


# The local menu's states (local_menu.state) and what its last Save returned.
MENU_STATES = value_texts(MenuState)
NO_MENU_ITEM = 0xFFFF
NO_MENU_RESULT = 0xFF


def local_menu_line(menu) -> str:
    """local_menu as a space-separated key=value line."""
    item = "none" if menu.item == NO_MENU_ITEM else str(menu.item)
    if menu.last_result == NO_MENU_RESULT:
        result = "none"
    else:
        result = "ok" if menu.last_result == 0 else REJECT_REASONS.get(menu.last_result, str(menu.last_result))
    parameter = "none" if menu.last_parameter == NO_MENU_ITEM else str(menu.last_parameter)
    return (f"local_menu state={MENU_STATES.get(menu.state, menu.state)} item={item} "
            f"last_result={result.replace(' ', '_')} last_parameter={parameter} "
            f"reboot_required={int(menu.reboot_required)}")


def speeds_text(values) -> str:
    """Four floats as a comma-separated list, NaN as nan."""
    return ",".join(f"{v:.3f}" for v in values)


def wheel_encoders_line(encoders) -> str:
    """wheel_encoders as a space-separated key=value line."""
    return (f"wheel_encoders speed={speeds_text(encoders.speed)} angle={speeds_text(encoders.angle)} "
            f"valid=0x{encoders.valid:X}")


def actuator_outputs_line(outputs) -> str:
    """actuator_outputs as a space-separated key=value line."""
    return f"actuator_outputs output={speeds_text(outputs.output)} target={speeds_text(outputs.target)}"


def param_status_line(status) -> str:
    """parameter_status as a space-separated key=value line."""
    return (f"param_status count={status.parameter_count} sector={status.active_sector} "
            f"bytes_used={status.bytes_used} sector_size={status.sector_size} unsaved={status.unsaved} "
            f"pending_reboot={status.pending_reboot} skipped_unknown={status.skipped_unknown} "
            f"skipped_wrong_type={status.skipped_wrong_type} skipped_out_of_range={status.skipped_out_of_range} "
            f"skipped_corrupt={status.skipped_corrupt} failed={int(status.failed)}")


@dataclass(frozen=True)
class ParamMetadata:
    """The parameter generator's metadata file (ADR-0012)."""

    set_hash: int
    by_name: dict[str, dict]


def load_param_metadata(path: str | Path) -> ParamMetadata:
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    return ParamMetadata(int(document["set_hash"], 16), {p["name"]: p for p in document["parameters"]})


def parse_param_text(text: str, param_type: int, meta: dict | None) -> int | float:
    """A value typed by a user: an enum value name, or a number of the parameter's type."""
    for value in (meta or {}).get("values", []):
        if value["name"] == text:
            return value["value"]
    if param_type == ParamType.FLOAT:
        return float(text)
    try:
        return int(text, 0)
    except ValueError:
        names = ", ".join(v["name"] for v in (meta or {}).get("values", []))
        raise ValueError(f"expected an integer{' or one of ' + names if names else ''}, got '{text}'") from None


def format_number(value: int | float) -> str:
    return f"{value:.7g}" if isinstance(value, float) else str(value)


def format_param(param: ParamValue, meta: dict | None) -> str:
    """One parameter as a space-separated key=value line; the description, quoted, comes last."""
    line = f"param name={param.name} value={format_number(param.value)}"
    names = {v["value"]: v["name"] for v in (meta or {}).get("values", [])}
    if param.value in names:
        line += f" value_name={names[param.value]}"
    line += f" type={ParamType(param.type).name if param.type in (1, 2) else param.type}"
    if param.flags & PARAM_PENDING_REBOOT:
        line += " pending_reboot=1"
    if param.flags & PARAM_UNSAVED:
        line += " unsaved=1"
    if meta is not None:
        if meta.get("unit"):
            line += f" unit={meta['unit']}"
        line += f" default={format_number(meta['default'])} min={format_number(meta['min'])} " \
                f"max={format_number(meta['max'])}"
        if meta.get("reboot_required"):
            line += " reboot_required=1"
        if meta.get("locked_when_armed"):
            line += " locked_when_armed=1"
        line += f" description={json.dumps(meta['description'])}"
    return line


@dataclass
class TopicStats:
    """What arrived of one topic instance."""

    name: str
    instance: int
    messages: int = 0
    first_timestamp_us: int | None = None
    last_timestamp_us: int | None = None
    latest: object | None = None

    def add(self, message: object | None) -> None:
        self.messages += 1
        self.latest = message
        timestamp = getattr(message, "timestamp", None)
        if timestamp is None:
            return
        if self.first_timestamp_us is None:
            self.first_timestamp_us = timestamp
        self.last_timestamp_us = timestamp

    @property
    def rate_hz(self) -> float:
        """Average rate by the MCU's timestamps, so host scheduling does not skew it."""
        if self.messages < 2 or self.first_timestamp_us is None or self.last_timestamp_us <= self.first_timestamp_us:
            return 0.0
        return (self.messages - 1) * 1e6 / (self.last_timestamp_us - self.first_timestamp_us)


HANDSHAKE_STATES = value_texts(HandshakeState)


def euler_degrees(q) -> tuple[float, float, float]:
    """Roll, pitch and yaw in degrees of a body-to-level quaternion (w, x, y, z), ZYX convention."""
    w, x, y, z = q
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    if abs(sin_pitch) > 0.99999:
        # Gimbal lock: roll and yaw turn about the same axis; yaw carries it.
        roll, pitch, yaw = 0.0, math.copysign(math.pi / 2, sin_pitch), 2.0 * math.atan2(z, w)
    else:
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(sin_pitch)
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    yaw = math.remainder(yaw, 2.0 * math.pi)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def work_item_name(status: object) -> str:
    """The name in a work_item_status message, or its index when it has none."""
    raw = getattr(status, "name", None)
    if raw is None:
        return f"#{status.work_item}"
    return bytes(raw).rstrip(b"\0").decode("ascii", errors="replace")


class WorkItemOverruns:
    """Overruns of each work item over a check, telling host stalls from real overruns.

    The timing monitor reports every work item at once, each report stamped
    with the same MCU time, and the Link carries them at 1 Hz. Between two
    reports that every work item is in, overruns of most work items at once
    mean the host paused SITL: the POSIX port runs the whole MCU in one
    process, so a host that does not schedule it delays every periodic work
    item together. Such an interval is a stall and its overruns do not count.
    Not every work item, since one triggered by a topic or slower than the
    pause cannot overrun in it. Overruns of one work item, or a few, while
    the others keep time count. Overruns before the first report are
    start-up's and do not count either.
    """

    def __init__(self) -> None:
        self._reports: dict[int, dict[str, int]] = {}

    def record(self, status: object) -> None:
        """Adds one work_item_status message."""
        self._reports.setdefault(status.timestamp, {})[work_item_name(status)] = status.overruns

    def judge(self) -> tuple[dict[str, int], int] | None:
        """Real overruns by work item and the number of stalls; None with fewer than two whole reports."""
        names = set().union(*self._reports.values())
        whole = [report for _, report in sorted(self._reports.items()) if report.keys() == names]
        if len(whole) < 2:
            return None
        overruns = dict.fromkeys(names, 0)
        stalls = 0
        for before, after in zip(whole, whole[1:]):
            counts = {name: after[name] - before[name] for name in names}
            if 2 * sum(1 for count in counts.values() if count > 0) >= len(names):
                stalls += 1
                continue
            for name, count in counts.items():
                overruns[name] += count
        return overruns, stalls


@dataclass
class Mismatch:
    name: str
    mcu_hash: int
    local_hash: int


# Replies to mode, arming and emergency stop requests.
CONTROL_REPLIES = (FrameType.MODE_REPLY, FrameType.MODE_REJECT, FrameType.ARMING_REPLY,
                   FrameType.ARMING_REJECT, FrameType.ESTOP_REPLY, FrameType.REBOOT_REPLY,
                   FrameType.REBOOT_REJECT)


class LinkMonitor:
    """The development host's end of one Link."""

    HELLO_INTERVAL_S = 0.25
    TIME_SYNC_INTERVAL_S = 0.5

    def __init__(self, connection: Connection, messages: ModuleType, clock: Callable[[], float] = time.monotonic):
        self.connection = connection
        self.messages = messages
        self.clock = clock
        self.local_hashes = {cls.NAME: cls.SCHEMA_HASH for cls in messages.TOPICS.values()}
        self.session = HostSession(local_schema=self.local_hashes)
        self.reader = FrameReader()
        self.topics: dict[tuple[str, int], TopicStats] = {}
        self.time_sync: TimeSyncResult | None = None
        # Called with (topic name, instance, message) for every decoded message.
        self.listeners: list[Callable[[str, int, object], None]] = []
        self._next_time_sync = 0.0
        # Frames lost before the handshake are not the Link's: a serial adapter
        # may still hold the end of the previous session, whose sequence numbers
        # are far behind. Losses count from the handshake on.
        self.lost_at_handshake = 0
        self.crc_errors_at_handshake = 0
        # The MCU's drops count from the second link_status of each instance
        # after the handshake, not from boot: link_status is published once a
        # second, so the first may predate the handshake and miss what its
        # burst of topic_info frames dropped.
        self.first_link_status: dict[int, object] = {}
        self._link_status_seen: dict[int, int] = {}
        # Parameter replies, as they arrive.
        self.params: dict[str, ParamValue] = {}
        self.param_rejects: list[ParamReject] = []
        # Mode, arming, emergency stop and reboot replies, as they arrive.
        self.control_replies: list[ControlReply] = []
        self.bringup_replies: list[BringupReply] = []

    @property
    def connected(self) -> bool:
        return self.session.handshake_done

    @property
    def lost(self) -> int:
        """Frames lost since the handshake."""
        return self.reader.stats.lost - self.lost_at_handshake

    @property
    def crc_errors(self) -> int:
        """Frames with a wrong CRC since the handshake; one before it may be a stale frame cut in two."""
        return self.reader.stats.crc_errors - self.crc_errors_at_handshake

    @property
    def mismatches(self) -> list[Mismatch]:
        return [
            Mismatch(name, self.session.peer_topics_by_name[name].schema_hash, self.local_hashes[name])
            for name in self.session.mismatched_topics
        ]

    def handshake(self, timeout_s: float) -> bool:
        """Sends hello until the MCU answers with every topic; False on timeout."""
        deadline = self.clock() + timeout_s
        next_hello = 0.0
        while not self.session.handshake_done:
            now = self.clock()
            if now >= deadline:
                return False
            if now >= next_hello:
                self.connection.send(self.session.make_hello())
                next_hello = now + self.HELLO_INTERVAL_S
            self._receive(min(next_hello, deadline) - now)
        self.lost_at_handshake = self.reader.stats.lost
        self.crc_errors_at_handshake = self.reader.stats.crc_errors
        return True

    def run(self, duration_s: float) -> None:
        """Receives for `duration_s`, keeping the session alive with time sync."""
        end = self.clock() + duration_s
        while (now := self.clock()) < end:
            self.poll(min(0.05, end - now))

    def poll(self, timeout_s: float) -> None:
        now = self.clock()
        if self.session.handshake_done and now >= self._next_time_sync:
            self.connection.send(self.session.make_time_sync(int(now * 1e6)))
            self._next_time_sync = now + self.TIME_SYNC_INTERVAL_S
        self._receive(timeout_s)

    def _receive(self, timeout_s: float) -> None:
        for frame in self.reader.feed(self.connection.receive(timeout_s)):
            self._handle(frame)

    def _handle(self, frame: Frame) -> None:
        if frame.type == FrameType.TOPIC_DATA:
            self._on_topic_data(frame.body)
        elif frame.type == FrameType.TIME_SYNC_REPLY:
            self.time_sync = HostSession.parse_time_sync(frame, int(self.clock() * 1e6))
        elif frame.type == FrameType.PARAM_VALUE:
            value = ParamValue.parse(frame.body)
            if value is not None:
                self.params[value.name] = value
        elif frame.type == FrameType.PARAM_REJECT:
            reject = ParamReject.parse(frame.body)
            if reject is not None:
                self.param_rejects.append(reject)
        elif frame.type in (FrameType.BRINGUP_REJECT, FrameType.BRINGUP_RESULT):
            bringup = BringupReply.parse(frame)
            if bringup is not None:
                self.bringup_replies.append(bringup)
        elif frame.type in CONTROL_REPLIES:
            reply = ControlReply.parse(frame)
            if reply is not None:
                self.control_replies.append(reply)
        else:
            self.session.handle_frame(frame)

    def _on_topic_data(self, body: bytes) -> None:
        if len(body) < 3:
            return
        topic_id = int.from_bytes(body[0:2], "little")
        instance = body[2]
        peer = self.session.peer_topics.get(topic_id)
        if peer is None:
            return  # Not announced at the handshake.
        message = None
        cls = self.messages.BY_NAME.get(peer.name)
        if cls is not None and peer.name not in self.session.mismatched_topics and len(body) - 3 == cls.ENCODED_SIZE:
            message = cls.decode(body[3:])
        key = (peer.name, instance)
        if key not in self.topics:
            self.topics[key] = TopicStats(peer.name, instance)
        self.topics[key].add(message)
        if message is not None and peer.name == "link_status":
            self._link_status_seen[instance] = self._link_status_seen.get(instance, 0) + 1
            if self._link_status_seen[instance] == 2:
                self.first_link_status[instance] = message
        if message is not None:
            for listener in self.listeners:
                listener(peer.name, instance, message)

    REQUEST_INTERVAL_S = 0.5

    def list_params(self, timeout_s: float) -> list[ParamValue] | None:
        """Every parameter of the MCU, by index; None when some did not arrive in time.

        Asks again from the first missing index, so a frame lost on the way
        costs a retry, not the list.
        """
        count = self.session.param_count
        deadline = self.clock() + timeout_s
        next_request = 0.0
        while True:
            by_index = {p.index: p for p in self.params.values() if p.count == count}
            missing = [i for i in range(count) if i not in by_index]
            if not missing:
                return [by_index[i] for i in range(count)]
            now = self.clock()
            if now >= deadline:
                return None
            if now >= next_request:
                self.connection.send(self.session.make_param_list_request(missing[0]))
                next_request = now + self.REQUEST_INTERVAL_S
            self.poll(min(0.05, deadline - now))

    def _await_param(self, name: str, request: Callable[[], bytes], timeout_s: float) -> ParamValue | ParamReject | None:
        self.params.pop(name, None)
        rejects = len(self.param_rejects)
        deadline = self.clock() + timeout_s
        next_request = 0.0
        while True:
            if name in self.params:
                return self.params[name]
            for reject in self.param_rejects[rejects:]:
                if reject.name == name:
                    return reject
            now = self.clock()
            if now >= deadline:
                return None
            if now >= next_request:
                self.connection.send(request())
                next_request = now + self.REQUEST_INTERVAL_S
            self.poll(min(0.05, deadline - now))

    def get_param(self, name: str, timeout_s: float) -> ParamValue | ParamReject | None:
        """A parameter's value, the MCU's refusal, or None when nothing came back in time."""
        return self._await_param(name, lambda: self.session.make_param_get(name), timeout_s)

    def set_param(self, name: str, param_type: ParamType, value: int | float,
                  timeout_s: float) -> ParamValue | ParamReject | None:
        """Changes a parameter; the reply holds the value now in effect, or why it was refused."""
        return self._await_param(name, lambda: self.session.make_param_set(name, param_type, value), timeout_s)

    def _await_control(self, request: Callable[[], bytes], answers: tuple[FrameType, ...],
                       timeout_s: float) -> ControlReply | None:
        seen = len(self.control_replies)
        deadline = self.clock() + timeout_s
        next_request = 0.0
        while True:
            for reply in self.control_replies[seen:]:
                if reply.type in answers:
                    return reply
            now = self.clock()
            if now >= deadline:
                return None
            if now >= next_request:
                # Requests are idempotent, so one may go again.
                self.connection.send(request())
                next_request = now + self.REQUEST_INTERVAL_S
            self.poll(min(0.05, deadline - now))

    def request_mode(self, mode: int, timeout_s: float) -> ControlReply | None:
        """Changes the mode; the reply holds the mode now in effect, or why it was refused."""
        return self._await_control(lambda: self.session.make_mode_request(mode),
                                   (FrameType.MODE_REPLY, FrameType.MODE_REJECT), timeout_s)

    def request_arming(self, arm: bool, timeout_s: float) -> ControlReply | None:
        """Arms or disarms; the reply holds the armed state and the skipped checks, or why it was refused."""
        return self._await_control(lambda: self.session.make_arming_request(arm),
                                   (FrameType.ARMING_REPLY, FrameType.ARMING_REJECT), timeout_s)

    def run_bringup(self, bringup_case: int, timeout_s: float) -> BringupReply | None:
        """Runs a bring-up case; the refusal, the result when it ends, or None after the timeout.

        Sent once: a case is not idempotent, and a repeat would start it again.
        """
        seen = len(self.bringup_replies)
        self.connection.send(self.session.make_bringup_run(bringup_case))
        deadline = self.clock() + timeout_s
        while (now := self.clock()) < deadline:
            if len(self.bringup_replies) > seen:
                return self.bringup_replies[seen]
            self.poll(min(0.05, deadline - now))
        return self.bringup_replies[seen] if len(self.bringup_replies) > seen else None

    def request_estop(self, engage: bool, timeout_s: float) -> ControlReply | None:
        """Engages or releases the emergency stop; the reply says whether it is engaged."""
        return self._await_control(lambda: self.session.make_estop_request(engage),
                                   (FrameType.ESTOP_REPLY,), timeout_s)

    def request_reboot(self, action: int, timeout_s: float) -> ControlReply | None:
        """Reboots the board, or reboots it into the bootloader; the reply echoes the action, or says why not.

        Sent again until a reply comes: a repeat while the reboot is pending changes nothing.
        """
        return self._await_control(lambda: self.session.make_reboot_request(action),
                                   (FrameType.REBOOT_REPLY, FrameType.REBOOT_REJECT), timeout_s)

    def sim_actuator_power(self, powered: bool) -> None:
        """SITL only: cuts or restores the simulated actuator power. The MCU does not answer."""
        self.connection.send(self.session.make_sim_actuator_power(powered))

    def press_key(self, duration_s: float) -> None:
        """SITL only: holds the simulated user key down for the duration, keeping the Link alive."""
        self.connection.send(self.session.make_sim_user_key(True))
        self.run(duration_s)
        self.connection.send(self.session.make_sim_user_key(False))

    def send_topic(self, name: str, message: object, instance: int = 0) -> None:
        """Sends a message as inbound topic data; the MCU drops it unless its topic map declares it inbound.

        Raises KeyError when the MCU did not announce the topic at the handshake.
        """
        self.connection.send(self.session.make_topic_data(name, message.encode(), instance))

    SETPOINT_RATE_HZ = 20.0

    def drive(self, wheel_speed: tuple[float, ...], duration_s: float,
              rate_hz: float = SETPOINT_RATE_HZ) -> object | None:
        """Sends actuator_setpoint at `rate_hz` for `duration_s`, then one with every target NaN.

        Stops early when vehicle_status reports the vehicle disarmed after it
        reported it armed; returns that vehicle_status, or None.
        """
        setpoint = self.messages.BY_NAME["actuator_setpoint"]
        return self._send_setpoints("actuator_setpoint", setpoint(wheel_speed=tuple(wheel_speed)),
                                    setpoint(wheel_speed=(math.nan,) * len(wheel_speed)), duration_s, rate_hz)

    def drive_body(self, twist: tuple[float, float, float], duration_s: float,
                   rate_hz: float = SETPOINT_RATE_HZ) -> object | None:
        """Sends body_velocity_setpoint (vx, vy, yaw_rate) at `rate_hz` for
        `duration_s`, then one with every field NaN; as drive()."""
        setpoint = self.messages.BY_NAME["body_velocity_setpoint"]
        vx, vy, yaw_rate = twist
        return self._send_setpoints("body_velocity_setpoint", setpoint(vx=vx, vy=vy, yaw_rate=yaw_rate),
                                    setpoint(vx=math.nan, vy=math.nan, yaw_rate=math.nan), duration_s, rate_hz)

    def _send_setpoints(self, name: str, message: object, stop: object, duration_s: float,
                        rate_hz: float) -> object | None:
        armed_seen = False
        disarmed = None

        def watch(name: str, _instance: int, message: object) -> None:
            nonlocal armed_seen, disarmed
            if name != "vehicle_status":
                return
            if message.armed:
                armed_seen = True
            elif armed_seen and disarmed is None:
                disarmed = message

        self.listeners.append(watch)
        try:
            end = self.clock() + duration_s
            next_send = 0.0
            while disarmed is None and (now := self.clock()) < end:
                if now >= next_send:
                    self.send_topic(name, message)
                    next_send = now + 1.0 / rate_hz
                self.poll(min(next_send, end) - now)
        finally:
            self.listeners.remove(watch)
        self.send_topic(name, stop)
        return disarmed

    def latest(self, name: str) -> list[tuple[int, object]]:
        """The latest message of every instance of a topic that arrived, by instance."""
        return [
            (instance, stats.latest)
            for (topic, instance), stats in sorted(self.topics.items())
            if topic == name and stats.latest is not None
        ]

    def summary(self) -> list[str]:
        """The state of the Link as space-separated key=value lines."""
        stats = self.reader.stats
        link = (
            f"link handshake={'ok' if self.connected else 'failed'} version={self.session.version} "
            f"mismatches={len(self.session.mismatched_topics)} params={self.session.param_count} "
            f"param_set_hash=0x{self.session.param_set_hash:016X} frames={stats.frames} "
            f"crc_errors={self.crc_errors} malformed={stats.malformed} lost={self.lost}"
        )
        if self.time_sync is not None:
            link += f" rtt_us={self.time_sync.rtt_us} clock_offset_us={self.time_sync.clock_offset_us}"
        lines = [link]
        lines += [
            f"mismatch topic={m.name} mcu_hash=0x{m.mcu_hash:016X} local_hash=0x{m.local_hash:016X}"
            for m in self.mismatches
        ]
        lines += [
            f"topic name={t.name} instance={t.instance} messages={t.messages} rate_hz={t.rate_hz:.2f}"
            for _, t in sorted(self.topics.items())
        ]
        # Attitudes as angles, the MCU's own view of each Link, and every work item's timing.
        for (name, instance), topic in sorted(self.topics.items()):
            q = getattr(topic.latest, "q", None)
            if q is not None:
                roll, pitch, yaw = euler_degrees(q)
                lines.append(f"attitude name={name} instance={instance} roll_deg={roll:.2f} "
                             f"pitch_deg={pitch:.2f} yaw_deg={yaw:.2f}")
        for instance, status in self.latest("link_status"):
            lines.append(
                f"mcu_link instance={instance} frames_sent={status.frames_sent} bytes_sent={status.bytes_sent} "
                f"frames_received={status.frames_received} bytes_received={status.bytes_received} "
                f"drops={status.drops} crc_errors={status.crc_errors} malformed={status.malformed} "
                f"lost={status.lost} handshake_state={HANDSHAKE_STATES.get(status.handshake_state, '?').replace(' ', '_')} "
                f"rate_scale={getattr(status, 'rate_scale', 100)}"
            )
        for _, status in self.latest("parameter_status"):
            lines.append(param_status_line(status))
        for _, status in self.latest("vehicle_status"):
            lines.append(vehicle_status_line(status))
        for _, menu in self.latest("local_menu"):
            lines.append(local_menu_line(menu))
        for _, encoders in self.latest("wheel_encoders"):
            lines.append(wheel_encoders_line(encoders))
        for _, outputs in self.latest("actuator_outputs"):
            lines.append(actuator_outputs_line(outputs))
        for _, status in self.latest("work_item_status"):
            lines.append(
                f"work_item name={work_item_name(status)} runs={status.runs} overruns={status.overruns} "
                f"run_time_last_us={status.run_time_last_us} run_time_max_us={status.run_time_max_us} "
                f"run_time_mean_us={status.run_time_mean_us} jitter_max_us={status.jitter_max_us}"
            )
        return lines


@dataclass(frozen=True)
class RateExpectation:
    name: str
    instance: int
    rate_hz: float

    @classmethod
    def parse(cls, text: str) -> "RateExpectation":
        try:
            target, rate = text.split("=")
            name, instance = target.split(":")
            return cls(name, int(instance), float(rate))
        except ValueError:
            raise ValueError(f"expected <topic>:<instance>=<rate_hz>, got '{text}'") from None


@dataclass
class CheckResult:
    failures: list[str] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        return not self.failures


def check(monitor: LinkMonitor, expectations: list[RateExpectation], tolerance: float) -> CheckResult:
    """Judges a finished run: the handshake, schema hashes, rates and frame loss."""
    result = CheckResult()
    if not monitor.connected:
        result.failures.append("no handshake with the MCU")
        return result
    for mismatch in monitor.mismatches:
        result.failures.append(f"topic {mismatch.name} has a different schema hash on the MCU")
    if monitor.lost:
        result.failures.append(f"{monitor.lost} frames lost")
    if monitor.crc_errors:
        result.failures.append(f"{monitor.crc_errors} frames with CRC errors")
    for instance, status in monitor.latest("link_status"):
        first = monitor.first_link_status.get(instance, status)
        if status.drops > first.drops:
            result.failures.append(f"the MCU dropped {status.drops - first.drops} frames on Link {instance}")
    # The MCU scales its rates to fit a slow serial port (ADR-0002); expect the scaled rate.
    scale = min((getattr(status, "rate_scale", 100) for _, status in monitor.latest("link_status")), default=100)
    for expected in expectations:
        topic = monitor.topics.get((expected.name, expected.instance))
        if topic is None:
            result.failures.append(f"topic {expected.name}:{expected.instance} never arrived")
            continue
        rate_hz = max(1.0, expected.rate_hz * scale / 100) if scale < 100 else expected.rate_hz
        if abs(topic.rate_hz - rate_hz) > rate_hz * tolerance:
            scaled = f" (scaled to {scale} %)" if scale < 100 else ""
            result.failures.append(
                f"topic {expected.name}:{expected.instance} arrived at {topic.rate_hz:.2f} Hz, "
                f"expected {rate_hz:g} Hz{scaled} within {tolerance:.0%}"
            )
    return result
