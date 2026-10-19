"""Parameter access over the Link, protocol version 2 (spec 0005): the frames,
the Link monitor library, and its param command against a fake MCU."""

import json
import subprocess
import sys
from pathlib import Path

import pytest

from test_monitor import FakeMcu, messages
from vklink import FrameReader, HostSession, McuParam, McuSession, ParamReject, ParamType, ParamValue
from vklink.monitor import LinkMonitor, UdpConnection, format_param, parse_param_text
from vklink.session import decode_param, encode_param

MONITOR = Path(__file__).resolve().parents[1] / "link_monitor.py"
SET_HASH = 0x0123456789ABCDEF


def mcu_params():
    return [
        McuParam("T_GAIN", ParamType.FLOAT, 0.5, 0.0, 2.0),
        McuParam("T_MODE", ParamType.INT32, 0, 0, 1),
        McuParam("T_COUNT", ParamType.INT32, 3, -10, 10),
    ]


def exchange(mcu: McuSession, wire: bytes) -> list:
    replies = []
    for frame in FrameReader().feed(wire):
        for reply in mcu.handle(frame, now_us=2000):
            replies += FrameReader().feed(reply)
    return replies


def connected_pair():
    mcu = McuSession(params=mcu_params(), param_set_hash=SET_HASH)
    host = HostSession()
    for frame in exchange(mcu, host.make_hello()):
        host.handle_frame(frame)
    assert host.handshake_done
    return mcu, host


def test_values_travel_as_their_32_bits():
    assert encode_param(ParamType.FLOAT, 1.0) == 0x3F800000
    assert encode_param(ParamType.INT32, -1) == 0xFFFFFFFF
    assert decode_param(ParamType.INT32, 0xFFFFFFFF) == -1
    assert decode_param(ParamType.FLOAT, 0x3F800000) == 1.0


def test_hello_reply_carries_the_parameter_set_hash_and_count():
    _, host = connected_pair()
    assert host.param_set_hash == SET_HASH
    assert host.param_count == 3


def test_get_set_and_list_frames():
    mcu, host = connected_pair()
    [reply] = exchange(mcu, host.make_param_get("T_GAIN"))
    value = ParamValue.parse(reply.body)
    assert (value.name, value.index, value.count, value.type, value.value) == ("T_GAIN", 1, 3, ParamType.FLOAT, 0.5)

    [reply] = exchange(mcu, host.make_param_set("T_COUNT", ParamType.INT32, -4))
    assert ParamValue.parse(reply.body).value == -4

    replies = exchange(mcu, host.make_param_list_request(1))
    assert [ParamValue.parse(r.body).name for r in replies] == ["T_GAIN", "T_MODE"]


@pytest.mark.parametrize("request_args, reason", [
    (("T_NONE", ParamType.INT32, 1), "unknown name"),
    (("T_GAIN", ParamType.INT32, 1), "wrong type"),
    (("T_COUNT", ParamType.INT32, 11), "out of range"),
])
def test_rejects_carry_their_reason(request_args, reason):
    mcu, host = connected_pair()
    [reply] = exchange(mcu, host.make_param_set(*request_args))
    reject = ParamReject.parse(reply.body)
    assert reject.name == request_args[0]
    assert reject.reason_text == reason


def test_values_typed_by_a_user():
    meta = {"values": [{"name": "OFF", "value": 0}, {"name": "ON", "value": 1}]}
    assert parse_param_text("ON", ParamType.INT32, meta) == 1
    assert parse_param_text("0x10", ParamType.INT32, None) == 16
    assert parse_param_text("2.5", ParamType.FLOAT, None) == 2.5
    with pytest.raises(ValueError, match="OFF, ON"):
        parse_param_text("MAYBE", ParamType.INT32, meta)


def test_format_shows_metadata_and_enum_names():
    value = ParamValue(index=0, count=1, type=ParamType.INT32, bits=1, flags=0, name="T_MODE")
    meta = {"type": "INT32", "default": 0, "min": 0, "max": 1, "unit": None, "description": "A mode.",
            "reboot_required": True, "locked_when_armed": False,
            "values": [{"name": "OFF", "value": 0}, {"name": "ON", "value": 1}]}
    assert format_param(value, meta) == ('param name=T_MODE value=1 value_name=ON type=INT32 default=0 min=0 max=1 '
                                         'reboot_required=1 description="A mode."')
    assert format_param(value, None) == "param name=T_MODE value=1 type=INT32"


def test_monitor_lists_gets_and_sets_parameters():
    with FakeMcu(params=mcu_params(), param_set_hash=SET_HASH) as mcu:
        connection = UdpConnection("127.0.0.1", mcu.port)
        monitor = LinkMonitor(connection, messages())
        try:
            assert monitor.handshake(2.0)
            params = monitor.list_params(2.0)
            assert [p.name for p in params] == ["T_COUNT", "T_GAIN", "T_MODE"]
            assert monitor.get_param("T_GAIN", 2.0).value == 0.5
            assert monitor.set_param("T_GAIN", ParamType.FLOAT, 1.5, 2.0).value == 1.5
            assert monitor.get_param("T_GAIN", 2.0).value == 1.5
            reject = monitor.set_param("T_GAIN", ParamType.FLOAT, 3.0, 2.0)
            assert isinstance(reject, ParamReject) and reject.reason_text == "out of range"
        finally:
            connection.close()


VKMSG = '''
import struct

class Sample:
    NAME = "sample"
    TOPIC_ID = 0
    SCHEMA_HASH = 0x1122334455667788
    ENCODED_SIZE = 12
    FIELDS = ("timestamp", "value")

    def __init__(self, timestamp=0, value=0):
        self.timestamp, self.value = timestamp, value

    @classmethod
    def decode(cls, data):
        return cls(*struct.unpack("<QI", data))

TOPICS = {0: Sample}
BY_NAME = {"sample": Sample}
'''


def write_metadata(path: Path, set_hash: int) -> Path:
    document = {"set_hash": f"0x{set_hash:016X}", "parameters": [
        {"name": "T_GAIN", "type": "FLOAT", "default": 0.5, "min": 0.0, "max": 2.0, "unit": "1/s",
         "description": "A gain.", "reboot_required": False, "locked_when_armed": False},
        {"name": "T_MODE", "type": "INT32", "default": 0, "min": 0, "max": 1, "unit": None,
         "description": "A mode.", "reboot_required": False, "locked_when_armed": False,
         "values": [{"name": "OFF", "value": 0}, {"name": "ON", "value": 1}]},
    ]}
    path.write_text(json.dumps(document), encoding="utf-8")
    return path


def run_cli(port: int, tmp_path: Path, set_hash: int, *command: str) -> subprocess.CompletedProcess:
    (tmp_path / "vkmsg.py").write_text(VKMSG, encoding="utf-8")
    meta = write_metadata(tmp_path / "params.json", set_hash)
    return subprocess.run(
        [sys.executable, str(MONITOR), "--udp", f"127.0.0.1:{port}", "--msg-dir", str(tmp_path),
         "--param-meta", str(meta), "--handshake-timeout", "2", "param", *command],
        capture_output=True, text=True, timeout=30, check=False)


def test_cli_lists_with_metadata_and_sets_by_enum_name(tmp_path: Path):
    with FakeMcu(params=mcu_params(), param_set_hash=SET_HASH) as mcu:
        listed = run_cli(mcu.port, tmp_path, SET_HASH, "list")
        assert listed.returncode == 0, listed.stderr
        assert listed.stderr == ""
        lines = listed.stdout.splitlines()
        assert [line.split()[1] for line in lines] == ["name=T_COUNT", "name=T_GAIN", "name=T_MODE"]
        assert lines[1] == ('param name=T_GAIN value=0.5 type=FLOAT unit=1/s default=0.5 min=0 max=2 '
                            'description="A gain."')

        changed = run_cli(mcu.port, tmp_path, SET_HASH, "set", "T_MODE", "ON")
        assert changed.returncode == 0, changed.stderr
        assert changed.stdout.startswith("param name=T_MODE value=1 value_name=ON")


def test_cli_warns_when_the_metadata_does_not_match(tmp_path: Path):
    with FakeMcu(params=mcu_params(), param_set_hash=SET_HASH) as mcu:
        result = run_cli(mcu.port, tmp_path, 0xDEAD, "get", "T_GAIN")
    assert result.returncode == 0
    assert "parameter set hash 0x0123456789ABCDEF differs from 0x000000000000DEAD" in result.stderr
    assert result.stdout.startswith("param name=T_GAIN value=0.5")


def test_cli_fails_with_the_reason_on_a_refused_set(tmp_path: Path):
    with FakeMcu(params=mcu_params(), param_set_hash=SET_HASH) as mcu:
        out_of_range = run_cli(mcu.port, tmp_path, SET_HASH, "set", "T_GAIN", "5")
        unknown = run_cli(mcu.port, tmp_path, SET_HASH, "set", "T_NONE", "1")
    assert out_of_range.returncode == 1
    assert "FAIL: T_GAIN rejected: out of range" in out_of_range.stderr
    assert unknown.returncode == 1
    assert "FAIL: T_NONE rejected: unknown name" in unknown.stderr


def test_flags_show_pending_reboot_and_unsaved():
    value = ParamValue(index=0, count=1, type=ParamType.INT32, bits=4, flags=3, name="T_COUNT")
    assert format_param(value, None) == "param name=T_COUNT value=4 type=INT32 pending_reboot=1 unsaved=1"


def test_summary_and_live_view_show_parameter_status():
    import types

    from vklink.monitor import TopicStats
    from vklink.view import render

    connection = types.SimpleNamespace(send=lambda data: None, receive=lambda timeout: b"", close=lambda: None)
    monitor = LinkMonitor(connection, messages())
    status = types.SimpleNamespace(timestamp=1, parameter_count=5, active_sector=1, bytes_used=44,
                                   sector_size=16384, unsaved=1, pending_reboot=2, skipped_unknown=1,
                                   skipped_wrong_type=0, skipped_out_of_range=0, skipped_corrupt=1, failed=False)
    monitor.topics[("parameter_status", 0)] = TopicStats("parameter_status", 0)
    monitor.topics[("parameter_status", 0)].add(status)
    assert ("param_status count=5 sector=1 bytes_used=44 sector_size=16384 unsaved=1 pending_reboot=2 "
            "skipped_unknown=1 skipped_wrong_type=0 skipped_out_of_range=0 skipped_corrupt=1 failed=0") \
        in monitor.summary()
    view = render(monitor, "udp", width=200)
    assert any(line.startswith("parameters     5  sector 1 44/16384 B  unsaved 1  pending reboot 2  "
                               "skipped at boot 2") for line in view)
