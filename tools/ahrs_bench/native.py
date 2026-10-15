"""控制板濾波器（examples/common）的主機版本：以 gcc 編成共用函式庫，ctypes 呼叫。

主機與控制板執行的是同一份 C 代碼，差別只在浮點運算順序（控制板的編譯器會用 FMA），
所以副本與韌體輸出的差異應在 1e-4 度量級；差異明顯變大代表副本沒有吃到同樣的資料或參數。
"""

import ctypes
import subprocess
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
COMMON = HERE.parent.parent / "examples" / "common"
BUILD = HERE / ".build"
LIB = BUILD / "libbenchfilters.so"
SOURCES = [HERE / "native" / "shim.c", COMMON / "ekf_ahrs.c", COMMON / "ahrs.c", COMMON / "frame.c",
           COMMON / "ekf_ahrs.h", COMMON / "ahrs.h", COMMON / "frame.h"]

# bench_ekf_export 的欄位：(名稱, 形狀)
EKF_FIELDS = [
    ("q", (4,)), ("b", (3,)), ("P", (6, 6)), ("P_prior", (6, 6)), ("v", (3,)), ("a_unit", (3,)),
    ("y", (3,)), ("S", (3, 3)), ("K", (6, 3)), ("dx", (6,)), ("sigma_a", ()), ("nis", ()),
    ("y_zupt", (3,)), ("nis_zupt", ()), ("accel_used", ()), ("zupt_used", ()), ("still", ()),
]
MAH_FIELDS = [("q", (4,)), ("b", (3,)), ("e", (3,)), ("accel_used", ())]


def _size(fields):
    return sum(int(np.prod(s)) if s else 1 for _, s in fields)


EKF_EXPORT_LEN = _size(EKF_FIELDS)
MAH_EXPORT_LEN = _size(MAH_FIELDS)


def _unpack(fields, flat):
    out, i = {}, 0
    for name, shape in fields:
        n = int(np.prod(shape)) if shape else 1
        out[name] = flat[i:i + n].reshape(shape).copy() if shape else float(flat[i])
        i += n
    return out


def build():
    """原始碼比函式庫新時重新編譯。"""
    if LIB.exists() and all(LIB.stat().st_mtime >= s.stat().st_mtime for s in SOURCES):
        return
    PROTO_INC = HERE.parent.parent / "protocol" / "include"
    cmd = ["gcc", "-O2", "-shared", "-fPIC", "-std=gnu11", "-ffp-contract=off", f"-I{COMMON}", f"-I{PROTO_INC}",
           "-o", str(LIB), str(HERE / "native" / "shim.c"), "-lm"]
    subprocess.run(cmd, check=True)


build()
_lib = ctypes.CDLL(str(LIB))
_f = ctypes.POINTER(ctypes.c_float)
_u8 = ctypes.POINTER(ctypes.c_uint8)
for name, args, res in [
    ("bench_ekf_size", [], ctypes.c_size_t),
    ("bench_mah_size", [], ctypes.c_size_t),
    ("bench_ekf_init", [ctypes.c_void_p, _f, _f], None),
    ("bench_ekf_set_params", [ctypes.c_void_p, _f], None),
    ("bench_ekf_step", [ctypes.c_void_p, _f, _f, ctypes.c_float], None),
    ("bench_ekf_set_state", [ctypes.c_void_p, _f, _f], None),
    ("bench_ekf_export", [ctypes.c_void_p, _f], None),
    ("bench_ekf_run", [_f, ctypes.c_size_t, _f, _f, ctypes.c_float, _f], None),
    ("bench_mah_init", [ctypes.c_void_p, _f, _f], None),
    ("bench_mah_set_params", [ctypes.c_void_p, _f], None),
    ("bench_mah_update", [ctypes.c_void_p, _f, _f, ctypes.c_float], None),
    ("bench_mah_set_state", [ctypes.c_void_p, _f, _f], None),
    ("bench_mah_export", [ctypes.c_void_p, _f], None),
    ("bench_mah_run", [_f, _f, ctypes.c_size_t, _f, _f, ctypes.c_float, _f], None),
    ("bench_frame_encode", [_u8, ctypes.c_size_t, _u8], ctypes.c_size_t),
    ("bench_frame_decode", [_u8, ctypes.c_size_t], ctypes.c_int),
]:
    fn = getattr(_lib, name)
    fn.argtypes, fn.restype = args, res


def _fa(values):
    a = np.ascontiguousarray(values, dtype=np.float32)
    return a, a.ctypes.data_as(_f)


class Ekf:
    """ekf_ahrs 的一個實例。params 為 13 個 float，順序同 ekf_ahrs_params_t。"""

    def __init__(self):
        self._mem = ctypes.create_string_buffer(_lib.bench_ekf_size())
        self.ready = False

    def init(self, params, accel):
        p, pp = _fa(params)
        a, ap = _fa(accel)
        _lib.bench_ekf_init(self._mem, pp, ap)
        self.ready = True

    def set_params(self, params):
        p, pp = _fa(params)
        _lib.bench_ekf_set_params(self._mem, pp)

    def step(self, gyro_dps, accel_g, dt):
        g, gp = _fa(gyro_dps)
        a, ap = _fa(accel_g)
        _lib.bench_ekf_step(self._mem, gp, ap, ctypes.c_float(dt))

    def set_state(self, q, b):
        q, qp = _fa(q)
        b, bp = _fa(b)
        _lib.bench_ekf_set_state(self._mem, qp, bp)

    def export(self):
        out = np.zeros(EKF_EXPORT_LEN, dtype=np.float32)
        _lib.bench_ekf_export(self._mem, out.ctypes.data_as(_f))
        return _unpack(EKF_FIELDS, out.astype(np.float64))


class Mahony:
    """ahrs（Mahony）的一個實例。params 為 kp、ki、accel_gate_g。"""

    def __init__(self):
        self._mem = ctypes.create_string_buffer(_lib.bench_mah_size())
        self.ready = False

    def init(self, params, accel):
        p, pp = _fa(params)
        a, ap = _fa(accel)
        _lib.bench_mah_init(self._mem, pp, ap)
        self.ready = True

    def set_params(self, params):
        p, pp = _fa(params)
        _lib.bench_mah_set_params(self._mem, pp)

    def update(self, gyro_dps, accel_g, dt):
        g, gp = _fa(gyro_dps)
        a, ap = _fa(accel_g)
        _lib.bench_mah_update(self._mem, gp, ap, ctypes.c_float(dt))

    def set_bias(self, b):
        b, bp = _fa(b)
        _lib.bench_mah_set_state(self._mem, None, bp)

    def set_state(self, q, b):
        q, qp = _fa(q)
        b, bp = _fa(b)
        _lib.bench_mah_set_state(self._mem, qp, bp)

    def export(self):
        out = np.zeros(MAH_EXPORT_LEN, dtype=np.float32)
        _lib.bench_mah_export(self._mem, out.ctypes.data_as(_f))
        return _unpack(MAH_FIELDS, out.astype(np.float64))


def ekf_run(params, gyro_dps, accel_g, dt):
    """整段資料跑一次 EKF。回傳 dict：q (n,4)、b (n,3)、Pdiag (n,6)、nis (n,)、flags (n,)。"""
    g, gp = _fa(gyro_dps)
    a, ap = _fa(accel_g)
    p, pp = _fa(params)
    n = len(g)
    out = np.zeros((n, 15), dtype=np.float32)
    _lib.bench_ekf_run(pp, n, gp, ap, ctypes.c_float(dt), out.ctypes.data_as(_f))
    out = out.astype(np.float64)
    return {"q": out[:, 0:4], "b": out[:, 4:7], "Pdiag": out[:, 7:13], "nis": out[:, 13],
            "flags": out[:, 14].astype(int)}


def mah_run(params, gyro_dps, accel_g, dt, bias0=None):
    g, gp = _fa(gyro_dps)
    a, ap = _fa(accel_g)
    p, pp = _fa(params)
    b0 = None
    if bias0 is not None:
        b0, b0p = _fa(bias0)
    n = len(g)
    out = np.zeros((n, 7), dtype=np.float32)
    _lib.bench_mah_run(pp, b0p if b0 is not None else None, n, gp, ap, ctypes.c_float(dt),
                     out.ctypes.data_as(_f))
    out = out.astype(np.float64)
    return {"q": out[:, 0:4], "b": out[:, 4:7]}


def frame_encode(payload: bytes) -> bytes:
    src = (ctypes.c_uint8 * max(len(payload), 1)).from_buffer_copy(payload or b"\0")
    dst = (ctypes.c_uint8 * (len(payload) + 2 + (len(payload) + 2) // 254 + 2))()
    n = _lib.bench_frame_encode(src, len(payload), dst)
    return bytes(dst[:n])


def frame_decode(frame: bytes):
    buf = (ctypes.c_uint8 * max(len(frame), 1)).from_buffer_copy(frame or b"\0")
    n = _lib.bench_frame_decode(buf, len(frame))
    return None if n < 0 else bytes(buf[:n])
