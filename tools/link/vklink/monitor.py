"""Link monitor for the development host (spec 0003).

Connects to a Link, performs the handshake and time synchronization, decodes
every topic with the Python module generated from the schema, and keeps each
topic instance's rate and latest values.
"""

from __future__ import annotations

import importlib.util
import os
import select
import socket
import termios
import time
import tty
from dataclasses import dataclass, field
from pathlib import Path
from types import ModuleType
from typing import Callable, Protocol

from .frame import Frame, FrameReader, FrameType
from .session import HostSession, TimeSyncResult


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
        except BaseException:
            os.close(self._fd)
            raise

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


HANDSHAKE_STATES = {0: "waiting", 1: "connected", 2: "peer_silent"}


def work_item_name(status: object) -> str:
    """The name in a work_item_status message, or its index when it has none."""
    raw = getattr(status, "name", None)
    if raw is None:
        return f"#{status.work_item}"
    return bytes(raw).rstrip(b"\0").decode("ascii", errors="replace")


@dataclass
class Mismatch:
    name: str
    mcu_hash: int
    local_hash: int


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
        self._next_time_sync = 0.0

    @property
    def connected(self) -> bool:
        return self.session.handshake_done

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
            f"mismatches={len(self.session.mismatched_topics)} frames={stats.frames} "
            f"crc_errors={stats.crc_errors} malformed={stats.malformed} lost={stats.lost}"
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
        # The MCU's own view of each Link, and every work item's timing.
        for instance, status in self.latest("link_status"):
            lines.append(
                f"mcu_link instance={instance} frames_sent={status.frames_sent} bytes_sent={status.bytes_sent} "
                f"frames_received={status.frames_received} bytes_received={status.bytes_received} "
                f"drops={status.drops} crc_errors={status.crc_errors} malformed={status.malformed} "
                f"lost={status.lost} handshake_state={HANDSHAKE_STATES.get(status.handshake_state, '?')}"
            )
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
    stats = monitor.reader.stats
    if stats.lost:
        result.failures.append(f"{stats.lost} frames lost")
    if stats.crc_errors:
        result.failures.append(f"{stats.crc_errors} frames with CRC errors")
    for instance, status in monitor.latest("link_status"):
        if status.drops:
            result.failures.append(f"the MCU dropped {status.drops} frames on Link {instance}")
    for expected in expectations:
        topic = monitor.topics.get((expected.name, expected.instance))
        if topic is None:
            result.failures.append(f"topic {expected.name}:{expected.instance} never arrived")
            continue
        if abs(topic.rate_hz - expected.rate_hz) > expected.rate_hz * tolerance:
            result.failures.append(
                f"topic {expected.name}:{expected.instance} arrived at {topic.rate_hz:.2f} Hz, "
                f"expected {expected.rate_hz:g} Hz within {tolerance:.0%}"
            )
    return result
