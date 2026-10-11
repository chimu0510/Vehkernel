"""ahrs_bench 串口協議的主機端（對應 examples/demos/ahrs_bench/bench_proto.h 與 common/frame.h）。

分幀：負載 + CRC16（CCITT-FALSE，little endian）→ COBS → 0x00 結尾。
"""

import struct

PROTO_VERSION = 1
BAUD = 921600
PARAM_NAME_LEN = 14

MSG_HELLO, MSG_SAMPLE, MSG_PARAMS, MSG_TEXT = 0x01, 0x10, 0x20, 0x30
MSG_SET_PARAMS, MSG_CMD = 0x80, 0x81
CMDS = {"reset_ekf": 1, "reset_mahony": 2, "reset_both": 3, "mahony_cal": 4, "hello": 5}

SF_EKF_INIT, SF_MAH_INIT, SF_MAH_BIAS_SET, SF_STILL = 0x01, 0x02, 0x04, 0x08
SF_ACCEL_USED, SF_ZUPT_USED, SF_MAH_ACCEL_USED, SF_IMU_ERROR = 0x10, 0x20, 0x40, 0x80

HELLO = struct.Struct("<BBHBB16s")
SAMPLE = struct.Struct("<BBBBBI" + "f" * 28 + "HH")
SAMPLE_FIELDS = [("accel_g", 3), ("gyro_dps", 3), ("temp_c", 1), ("ekf_q", 4), ("ekf_b", 3),
                 ("ekf_sig_th", 3), ("ekf_sig_b", 3), ("ekf_nis", 1), ("mah_q", 4), ("mah_b", 3)]
PARAM_ENTRY = struct.Struct(f"<{PARAM_NAME_LEN}sf")


def _crc_table():
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
        table.append(crc & 0xFFFF)
    return table


_CRC = _crc_table()


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC[(crc >> 8) ^ b]
    return crc


def encode(payload: bytes) -> bytes:
    data = payload + struct.pack("<H", crc16(payload))
    out = bytearray([0])
    code_pos, code = 0, 1
    for b in data:
        if b:
            out.append(b)
            code += 1
        if not b or code == 0xFF:
            out[code_pos] = code
            code_pos = len(out)
            out.append(0)
            code = 1
    out[code_pos] = code
    out.append(0)
    return bytes(out)


def decode(frame: bytes):
    """解一個訊框（不含結尾 0x00）；錯誤回傳 None。"""
    out = bytearray()
    i, n = 0, len(frame)
    while i < n:
        code = frame[i]
        i += 1
        if code == 0 or i + code - 1 > n:
            return None
        out += frame[i:i + code - 1]
        i += code - 1
        if code != 0xFF and i < n:
            out.append(0)
    if len(out) < 2 or crc16(bytes(out[:-2])) != struct.unpack_from("<H", out, len(out) - 2)[0]:
        return None
    return bytes(out[:-2])


class Deframer:
    """把串口位元組流切成訊框；統計 CRC/格式錯誤。"""

    def __init__(self):
        self.buf = bytearray()
        self.errors = 0

    def feed(self, data: bytes):
        frames = []
        for b in data:
            if b:
                self.buf.append(b)
                if len(self.buf) > 4096:
                    self.buf.clear()
                    self.errors += 1
                continue
            if self.buf:
                payload = decode(bytes(self.buf))
                self.buf.clear()
                if payload is None:
                    self.errors += 1
                else:
                    frames.append(payload)
        return frames


def parse(payload: bytes):
    """回傳 (類型, 內容 dict)；未知或長度不符回傳 (None, None)。"""
    t = payload[0]
    if t == MSG_SAMPLE and len(payload) == SAMPLE.size:
        v = SAMPLE.unpack(payload)
        d = {"flags": v[1], "ekf_epoch": v[2], "mah_epoch": v[3], "param_rev": v[4], "seq": v[5]}
        i = 6
        for name, n in SAMPLE_FIELDS:
            d[name] = v[i] if n == 1 else list(v[i:i + n])
            i += n
        d["isr_us"], d["overruns"] = v[i], v[i + 1]
        return t, d
    if t == MSG_HELLO and len(payload) == HELLO.size:
        _, ver, rate, count, hw, name = HELLO.unpack(payload)
        return t, {"proto_version": ver, "rate_hz": rate, "param_count": count, "hw_version": hw,
                   "name": name.rstrip(b"\0").decode(errors="replace")}
    if t == MSG_PARAMS and len(payload) >= 3:
        rev, count = payload[1], payload[2]
        if len(payload) != 3 + count * PARAM_ENTRY.size:
            return None, None
        names, values = [], []
        for k in range(count):
            name, value = PARAM_ENTRY.unpack_from(payload, 3 + k * PARAM_ENTRY.size)
            names.append(name.rstrip(b"\0").decode())
            values.append(value)
        return t, {"rev": rev, "names": names, "values": values}
    if t == MSG_TEXT:
        return t, {"text": payload[1:].decode(errors="replace")}
    return None, None


def cmd(name: str) -> bytes:
    return encode(bytes([MSG_CMD, CMDS[name]]))


def set_params(values) -> bytes:
    return encode(bytes([MSG_SET_PARAMS, len(values)]) + struct.pack(f"<{len(values)}f", *values))
