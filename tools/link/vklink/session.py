"""Link session management (ADR-0002).

Implements both Host-side Link session and MCU-side Link session for tools and tests.
"""

import enum
import struct
from dataclasses import dataclass

from .frame import PROTOCOL_VERSION, Frame, FrameType, FrameWriter


class ParamType(enum.IntEnum):
    """Parameter types on the wire (ADR-0012)."""

    INT32 = 1
    FLOAT = 2


# Why the MCU refused a parameter request; vk::core::ParamResult.
REJECT_REASONS = {
    1: "unknown name",
    2: "wrong type",
    3: "out of range",
    4: "locked while armed",
}


# Modes and the reasons of mode, arming and emergency stop replies; vk::core.
MODES = {0: "STANDBY", 1: "TEST"}
MODE_REJECT_REASONS = {1: "armed", 2: "unknown mode"}
ARMING_REJECT_REASONS = {
    1: "board not verified",
    2: "emergency stop engaged",
    3: "wrong mode",
    4: "not in control",
    5: "setpoints active",
    6: "bring-up running",
    7: "actuator power off",
    8: "actuators not started",
}
DISARM_REASONS = {
    0: "none",
    1: "request",
    2: "emergency stop",
    3: "actuator power lost",
    4: "setpoint timeout",
    5: "idle",
    6: "bring-up abort",
}
ACTUATOR_POWER_STATES = {0: "not available", 1: "unpowered", 2: "settling", 3: "powered"}
# Bits of COM_ARM_CHECK.
ARM_CHECK_BITS = {0: "ACT_POWER"}


# The bring-up cases, why a bringup_run was refused, and each output's result; vk::core and vk::actuators.
BRINGUP_CASES = {"motors": 1}
BRINGUP_REJECT_REASONS = {
    1: "not available",
    2: "unknown case",
    3: "wrong mode",
    4: "not armed",
    5: "not in control",
    6: "emergency stop engaged",
    7: "already running",
}
BRINGUP_RESULTS = {0: "pass", 1: "no encoder moved", 2: "reversed", 3: "another encoder moved"}


@dataclass(frozen=True)
class BringupOutput:
    result: int
    encoder: int  # 1 to 4, 0 for none
    sign: int

    @property
    def passed(self) -> bool:
        return self.result == 0


@dataclass(frozen=True)
class BringupReply:
    """A bringup_reject, or the bringup_result of a case that ended."""

    rejected: bool
    reason: int = 0              # bringup_reject
    bringup_case: int = 0
    aborted: bool = False
    outputs: tuple[BringupOutput, ...] = ()

    @property
    def reason_text(self) -> str:
        return BRINGUP_REJECT_REASONS.get(self.reason, f"reason {self.reason}")

    @classmethod
    def parse(cls, frame: Frame) -> "BringupReply | None":
        body = frame.body
        if frame.type == FrameType.BRINGUP_REJECT and body:
            return cls(rejected=True, reason=body[0])
        if frame.type != FrameType.BRINGUP_RESULT or len(body) < 3 or len(body) < 3 + 3 * body[2]:
            return None
        outputs = tuple(BringupOutput(body[3 + 3 * i], body[4 + 3 * i], struct.unpack("<b", body[5 + 3 * i:6 + 3 * i])[0])
                        for i in range(body[2]))
        return cls(rejected=False, bringup_case=body[0], aborted=body[1] == 3, outputs=outputs)


def check_names(mask: int) -> list[str]:
    """The arming checks a COM_ARM_CHECK-style mask names."""
    return [ARM_CHECK_BITS.get(bit, f"bit {bit}") for bit in range(32) if mask & (1 << bit)]


@dataclass(frozen=True)
class ControlReply:
    """A reply to a mode, arming or emergency stop request."""

    type: FrameType
    value: int        # the mode, armed or engaged now in effect; for a reject, the reason
    mode: int = 0     # mode_reject: the mode still in effect
    skipped: int = 0  # arming_reply: the arming checks skipped

    @property
    def refused(self) -> bool:
        return self.type in (FrameType.MODE_REJECT, FrameType.ARMING_REJECT)

    @property
    def reason_text(self) -> str:
        reasons = MODE_REJECT_REASONS if self.type == FrameType.MODE_REJECT else ARMING_REJECT_REASONS
        return reasons.get(self.value, f"reason {self.value}")

    @classmethod
    def parse(cls, frame: Frame) -> "ControlReply | None":
        body = frame.body
        if frame.type == FrameType.MODE_REJECT and len(body) >= 2:
            return cls(frame.type, body[0], mode=body[1])
        if frame.type == FrameType.ARMING_REPLY and len(body) >= 5:
            return cls(frame.type, body[0], skipped=struct.unpack("<I", body[1:5])[0])
        if frame.type in (FrameType.MODE_REPLY, FrameType.ARMING_REJECT, FrameType.ESTOP_REPLY) and body:
            return cls(frame.type, body[0])
        return None


def encode_param(param_type: ParamType, value: int | float) -> int:
    """The 32 bits of a value as the MCU stores it."""
    if param_type == ParamType.FLOAT:
        return struct.unpack("<I", struct.pack("<f", value))[0]
    return struct.unpack("<I", struct.pack("<i", int(value)))[0]


def decode_param(param_type: int, bits: int) -> int | float:
    if param_type == ParamType.FLOAT:
        return struct.unpack("<f", struct.pack("<I", bits))[0]
    return struct.unpack("<i", struct.pack("<I", bits))[0]


@dataclass(frozen=True)
class ParamValue:
    """A param_value frame: one parameter's current value."""

    index: int
    count: int
    type: int
    bits: int
    flags: int
    name: str

    @property
    def value(self) -> int | float:
        return decode_param(self.type, self.bits)

    @classmethod
    def parse(cls, body: bytes) -> "ParamValue | None":
        if len(body) < 11:
            return None
        index, count, param_type, bits, flags, name_len = struct.unpack("<HHBIBB", body[:11])
        if len(body) < 11 + name_len:
            return None
        return cls(index, count, param_type, bits, flags, body[11:11 + name_len].decode("ascii", errors="replace"))


@dataclass(frozen=True)
class ParamReject:
    """A param_reject frame: the MCU refused a request about one parameter."""

    reason: int
    name: str

    @property
    def reason_text(self) -> str:
        return REJECT_REASONS.get(self.reason, f"reason {self.reason}")

    @classmethod
    def parse(cls, body: bytes) -> "ParamReject | None":
        if len(body) < 2 or len(body) < 2 + body[1]:
            return None
        return cls(body[0], body[2:2 + body[1]].decode("ascii", errors="replace"))


def _name(name: str) -> bytes:
    raw = name.encode("ascii")
    return struct.pack("<B", len(raw)) + raw


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
        self.param_set_hash = 0
        self.param_count = 0

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

    def make_param_list_request(self, first: int = 0) -> bytes:
        """Asks for every parameter from index `first` on, as param_value frames."""
        return self.writer.write(FrameType.PARAM_LIST_REQUEST, struct.pack("<H", first))

    def make_param_get(self, name: str) -> bytes:
        return self.writer.write(FrameType.PARAM_GET, _name(name))

    def make_param_set(self, name: str, param_type: ParamType, value: int | float) -> bytes:
        body = _name(name) + struct.pack("<BI", int(param_type), encode_param(param_type, value))
        return self.writer.write(FrameType.PARAM_SET, body)

    def make_mode_request(self, mode: int) -> bytes:
        return self.writer.write(FrameType.MODE_REQUEST, struct.pack("<B", mode))

    def make_arming_request(self, arm: bool) -> bytes:
        return self.writer.write(FrameType.ARMING_REQUEST, struct.pack("<B", int(arm)))

    def make_estop_request(self, engage: bool) -> bytes:
        return self.writer.write(FrameType.ESTOP_REQUEST, struct.pack("<B", int(engage)))

    def make_bringup_run(self, bringup_case: int) -> bytes:
        return self.writer.write(FrameType.BRINGUP_RUN, struct.pack("<B", bringup_case))

    def make_sim_actuator_power(self, powered: bool) -> bytes:
        return self.writer.write(FrameType.SIM_ACTUATOR_POWER, struct.pack("<B", int(powered)))

    def make_topic_data(self, name: str, message: bytes, instance: int = 0) -> bytes:
        """Encodes an inbound TopicData frame for a topic the MCU announced at the handshake.

        Raises KeyError when the MCU did not announce the topic.
        """
        topic = self.peer_topics_by_name[name]
        return self.writer.write(FrameType.TOPIC_DATA, struct.pack("<HB", topic.id, instance) + message)

    def handle_frame(self, frame: Frame) -> None:
        """Consumes one frame received from the MCU."""
        if frame.type == FrameType.HELLO_REPLY:
            if len(frame.body) < 3:
                return
            chosen, topic_count = struct.unpack("<BH", frame.body[:3])
            if chosen == 0 or chosen not in self.supported_versions or len(frame.body) < 13:
                self.handshake_done = False
                self.version = 0
                return
            self.version = chosen
            self.expected_topic_count = topic_count
            self.param_set_hash, self.param_count = struct.unpack("<QH", frame.body[3:13])
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


@dataclass
class McuParam:
    """A parameter a McuSession offers: an int32 or float range check, as the MCU does."""

    name: str
    type: ParamType
    value: int | float
    minimum: int | float
    maximum: int | float


@dataclass
class McuControl:
    """The vehicle control a McuSession offers: it answers as told, like a fake commander."""

    mode: int = 0
    armed: bool = False
    emergency_stop: bool = False
    skipped: int = 0
    mode_result: int = 0    # vk::core::ModeResult
    arming_result: int = 0  # vk::core::ArmingResult
    sim_actuator_power: bool = True
    bringup_result: int = 0  # vk::core::BringupStart
    # What a case reports at once when it is accepted: (result, encoder, sign) per output.
    bringup_outputs: tuple[tuple[int, int, int], ...] = ((0, 1, 1),)


class McuSession:
    """MCU side of a Link session (ADR-0002).

    Mirrors vk::link::Session in C++, parameters included, except that a
    parameter list is answered at once rather than paced.
    """

    DEFAULT_PEER_TIMEOUT_US = 3_000_000

    def __init__(
        self,
        topics: list[TopicDescription] | None = None,
        peer_timeout_us: int = DEFAULT_PEER_TIMEOUT_US,
        param_set_hash: int = 0,
        params: list[McuParam] | None = None,
        control: McuControl | None = None,
    ):
        self.control = control
        self.topics = topics or []
        self.param_set_hash = param_set_hash
        self.params = sorted(params or [], key=lambda p: p.name)
        self.peer_timeout_us = peer_timeout_us
        self.writer = FrameWriter()
        self.version = 0
        self.handshake_done = False
        self.last_heard_us = 0
        self.frames_sent = 0
        # Inbound topic data, as (topic id, instance, message) in arrival order.
        self.topic_data: list[tuple[int, int, bytes]] = []

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
        if frame.type == FrameType.PARAM_LIST_REQUEST and len(frame.body) >= 2:
            first = struct.unpack("<H", frame.body[:2])[0]
            return [self._param_value(i) for i in range(first, len(self.params))]
        if frame.type in (FrameType.PARAM_GET, FrameType.PARAM_SET) and frame.body:
            return [self._on_param(frame)]
        if frame.type == FrameType.TOPIC_DATA and len(frame.body) >= 3:
            topic_id, instance = struct.unpack("<HB", frame.body[:3])
            self.topic_data.append((topic_id, instance, bytes(frame.body[3:])))
            return []
        if self.control is not None and frame.body:
            return self._on_control(frame)
        return []

    def _on_control(self, frame: Frame) -> list[bytes]:
        control, value = self.control, frame.body[0]
        if frame.type == FrameType.MODE_REQUEST:
            if value not in MODES:
                return [self._send(FrameType.MODE_REJECT, struct.pack("<BB", 2, control.mode))]
            if control.mode_result != 0:
                return [self._send(FrameType.MODE_REJECT, struct.pack("<BB", control.mode_result, control.mode))]
            control.mode = value
            return [self._send(FrameType.MODE_REPLY, struct.pack("<B", control.mode))]
        if frame.type == FrameType.ARMING_REQUEST:
            if value and control.arming_result != 0:
                return [self._send(FrameType.ARMING_REJECT, struct.pack("<B", control.arming_result))]
            control.armed = bool(value)
            return [self._send(FrameType.ARMING_REPLY, struct.pack("<BI", int(control.armed), control.skipped))]
        if frame.type == FrameType.ESTOP_REQUEST:
            control.emergency_stop = bool(value)
            return [self._send(FrameType.ESTOP_REPLY, struct.pack("<B", int(control.emergency_stop)))]
        if frame.type == FrameType.SIM_ACTUATOR_POWER:
            control.sim_actuator_power = bool(value)
        if frame.type == FrameType.BRINGUP_RUN:
            if control.bringup_result != 0:
                return [self._send(FrameType.BRINGUP_REJECT, struct.pack("<B", control.bringup_result))]
            body = struct.pack("<BBB", value, 2, len(control.bringup_outputs))
            for result, encoder, sign in control.bringup_outputs:
                body += struct.pack("<BBb", result, encoder, sign)
            return [self._send(FrameType.BRINGUP_RESULT, body)]
        return []

    def _send(self, frame_type: FrameType, body: bytes) -> bytes:
        self.frames_sent += 1
        return self.writer.write(frame_type, body)

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
        chosen = PROTOCOL_VERSION if PROTOCOL_VERSION in offered_versions else 0

        self.version = chosen
        self.handshake_done = chosen != 0

        out = []
        reply_body = struct.pack("<BHQH", chosen, len(self.topics) if self.handshake_done else 0,
                                 self.param_set_hash if self.handshake_done else 0,
                                 len(self.params) if self.handshake_done else 0)
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

    def _param_value(self, index: int) -> bytes:
        param = self.params[index]
        body = struct.pack("<HHBIB", index, len(self.params), int(param.type),
                           encode_param(param.type, param.value), 0) + _name(param.name)
        self.frames_sent += 1
        return self.writer.write(FrameType.PARAM_VALUE, body)

    def _reject(self, reason: int, name: str) -> bytes:
        self.frames_sent += 1
        return self.writer.write(FrameType.PARAM_REJECT, struct.pack("<B", reason) + _name(name))

    def _on_param(self, frame: Frame) -> bytes:
        name = frame.body[1:1 + frame.body[0]].decode("ascii", errors="replace")
        index = next((i for i, p in enumerate(self.params) if p.name == name), None)
        if index is None:
            return self._reject(1, name)
        if frame.type == FrameType.PARAM_SET:
            rest = frame.body[1 + frame.body[0]:]
            param_type, bits = struct.unpack("<BI", rest[:5])
            param = self.params[index]
            if param_type != param.type:
                return self._reject(2, name)
            value = decode_param(param_type, bits)
            if not param.minimum <= value <= param.maximum:
                return self._reject(3, name)
            param.value = value
        return self._param_value(index)
