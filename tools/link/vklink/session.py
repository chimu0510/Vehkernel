"""Link session management (ADR-0002).

Implements both Host-side Link session and MCU-side Link session for tools and tests.
"""

import struct
from dataclasses import dataclass, field

from .frame import PROTOCOL_VERSION, Frame, FrameType, FrameWriter


@dataclass(frozen=True)
class TopicDescription:
    id: int
    instances: int
    schema_hash: int
    name: str


@dataclass(frozen=True)
class TimeSyncResult:
    host_send_us: int
    mcu_recv_us: int
    mcu_send_us: int
    host_recv_us: int
    rtt_us: int
    clock_offset_us: int


class HostSession:
    """Host side of a Link session (ADR-0002).

    Initiates handshake, verifies schema hashes, manages time sync requests, and receives topics.
    """

    def __init__(
        self,
        local_schema: dict[str, int] | None = None,
        supported_versions: list[int] | None = None,
    ):
        self.local_schema = local_schema or {}
        self.supported_versions = supported_versions or [PROTOCOL_VERSION]
        self.writer = FrameWriter()
        self.version = 0
        self.handshake_done = False
        self.expected_topic_count = 0
        self.peer_topics: dict[int, TopicDescription] = {}
        self.peer_topics_by_name: dict[str, TopicDescription] = {}
        self.mismatched_topics: list[str] = []

    def make_hello(self) -> bytes:
        """Encodes a Hello wire frame proposing supported protocol versions."""
        body = struct.pack("<B", len(self.supported_versions))
        for v in self.supported_versions:
            body += struct.pack("<B", v)
        return self.writer.write(FrameType.HELLO, body)

    def make_time_sync(self, host_send_time_us: int) -> bytes:
        """Encodes a TimeSyncRequest wire frame."""
        body = struct.pack("<Q", host_send_time_us)
        return self.writer.write(FrameType.TIME_SYNC_REQUEST, body)

    def handle_frame(self, frame: Frame) -> None:
        """Consumes one frame received from the MCU."""
        if frame.type == FrameType.HELLO_REPLY:
            if len(frame.body) < 3:
                return
            chosen, topic_count = struct.unpack("<BH", frame.body[:3])
            if chosen == 0 or chosen not in self.supported_versions:
                self.handshake_done = False
                self.version = 0
                return
            self.version = chosen
            self.expected_topic_count = topic_count
            self.peer_topics.clear()
            self.peer_topics_by_name.clear()
            self.mismatched_topics.clear()
            if topic_count == 0:
                self.handshake_done = True

        elif frame.type == FrameType.TOPIC_INFO:
            if len(frame.body) < 12:
                return
            topic_id, instances, schema_hash, name_len = struct.unpack("<HBQB", frame.body[:12])
            name = frame.body[12 : 12 + name_len].decode("utf-8", errors="replace")
            desc = TopicDescription(
                id=topic_id,
                instances=instances,
                schema_hash=schema_hash,
                name=name,
            )
            self.peer_topics[topic_id] = desc
            self.peer_topics_by_name[name] = desc
            if name in self.local_schema and self.local_schema[name] != schema_hash:
                self.mismatched_topics.append(name)
            if len(self.peer_topics) == self.expected_topic_count:
                self.handshake_done = True

    @staticmethod
    def parse_time_sync(frame: Frame, host_recv_us: int) -> TimeSyncResult:
        """Extracts MCU times from a TimeSyncReply and computes RTT and clock offset."""
        if frame.type != FrameType.TIME_SYNC_REPLY or len(frame.body) < 24:
            raise ValueError("Not a valid TimeSyncReply frame")
        host_send_us, mcu_recv_us, mcu_send_us = struct.unpack("<QQQ", frame.body[:24])
        rtt_us = (host_recv_us - host_send_us) - (mcu_send_us - mcu_recv_us)
        # Standard NTP clock offset: ((T1 - T0) + (T2 - T3)) / 2
        clock_offset_us = ((mcu_recv_us - host_send_us) + (mcu_send_us - host_recv_us)) // 2
        return TimeSyncResult(
            host_send_us=host_send_us,
            mcu_recv_us=mcu_recv_us,
            mcu_send_us=mcu_send_us,
            host_recv_us=host_recv_us,
            rtt_us=rtt_us,
            clock_offset_us=clock_offset_us,
        )


class McuSession:
    """MCU side of a Link session (ADR-0002).

    Mirrors vk::link::Session in C++.
    """

    DEFAULT_PEER_TIMEOUT_US = 3_000_000

    def __init__(
        self,
        topics: list[TopicDescription] | None = None,
        peer_timeout_us: int = DEFAULT_PEER_TIMEOUT_US,
    ):
        self.topics = topics or []
        self.peer_timeout_us = peer_timeout_us
        self.writer = FrameWriter()
        self.version = 0
        self.handshake_done = False
        self.last_heard_us = 0
        self.frames_sent = 0

    def handle(self, frame: Frame, now_us: int) -> list[bytes]:
        """Handles an incoming frame; returns wire frames to send in reply."""
        if frame.type == FrameType.HELLO:
            replies = self._on_hello(frame.body)
            if self.handshake_done:
                self.last_heard_us = now_us
            return replies

        if not self.handshake_done:
            return []

        self.last_heard_us = now_us
        if frame.type == FrameType.TIME_SYNC_REQUEST:
            return self._on_time_sync(frame.body, now_us)
        return []

    def connected(self, now_us: int) -> bool:
        """Whether topic data may be sent at now_us."""
        return self.handshake_done and (now_us - self.last_heard_us <= self.peer_timeout_us)

    def send_topic(
        self, topic_id: int, instance: int, message: bytes, now_us: int
    ) -> bytes | None:
        """Sends one topic data frame if connected."""
        if not self.connected(now_us):
            return None
        body = struct.pack("<HB", topic_id, instance) + message
        wire = self.writer.write(FrameType.TOPIC_DATA, body)
        self.frames_sent += 1
        return wire

    def _on_hello(self, body: bytes) -> list[bytes]:
        if len(body) < 1:
            return []
        count = body[0]
        offered_versions = body[1 : 1 + count]
        chosen = 0
        for offered in offered_versions:
            if offered <= PROTOCOL_VERSION:
                chosen = max(chosen, offered)

        self.version = chosen
        self.handshake_done = chosen != 0

        out = []
        reply_body = struct.pack("<BH", chosen, len(self.topics) if self.handshake_done else 0)
        out.append(self.writer.write(FrameType.HELLO_REPLY, reply_body))
        self.frames_sent += 1

        if not self.handshake_done:
            return out

        for topic in self.topics:
            name_bytes = topic.name.encode("utf-8")
            info_body = struct.pack(
                "<HBQB", topic.id, topic.instances, topic.schema_hash, len(name_bytes)
            ) + name_bytes
            out.append(self.writer.write(FrameType.TOPIC_INFO, info_body))
            self.frames_sent += 1

        return out

    def _on_time_sync(self, body: bytes, now_us: int) -> list[bytes]:
        if len(body) < 8:
            return []
        host_send_us = struct.unpack("<Q", body[:8])[0]
        reply_body = struct.pack("<QQQ", host_send_us, now_us, now_us)
        wire = self.writer.write(FrameType.TIME_SYNC_REPLY, reply_body)
        self.frames_sent += 1
        return [wire]
