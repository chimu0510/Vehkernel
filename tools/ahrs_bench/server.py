#!/usr/bin/env python3
"""AHRS Bench（姿態估測工作台）的主機端：與 ahrs_bench 韌體互動的 Web UI 伺服器。

  tools/ahrs_bench/server.py                    # 自動找 CH9102F（USART1）
  tools/ahrs_bench/server.py --port /dev/ttyUSB0
  tools/ahrs_bench/server.py --sim              # 不接控制板，用模擬器
然後以瀏覽器開啟 http://localhost:8008

資料流：資料來源（串口 / 模擬器 / 重播）→ 每一筆樣本送進兩組濾波器副本：
  A「目前」：串口時與控制板同參數、同重設時機，逐筆重現韌體的計算（可看內部量並檢查一致性）；
  B「候選」：使用者正在調的參數，在同一份資料上比較。
濾波器副本是 examples/common 的同一份 C 代碼（native.py）。
"""

import argparse
import json
import math
import queue
import sys
import threading
import time
from datetime import datetime
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

import analysis
import native
import params
import protocol
import sim

HERE = Path(__file__).resolve().parent
STATIC = HERE / "static"
RECORDINGS = HERE / "recordings"
sys.path.insert(0, str(HERE.parent))
D2R = math.pi / 180.0
R2D = 180.0 / math.pi
BATCH_PERIOD = 0.05
LATEST_PERIOD = 0.066


def _euler(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll * R2D, pitch * R2D, yaw * R2D


def _r(x, nd=5):
    if x is None:
        return None
    x = float(x)
    return round(x, nd) if math.isfinite(x) else None


def _clean(obj, sig=6):
    """把 numpy 型別轉成 JSON 可用的型別，浮點數保留 sig 位有效數字，NaN/inf → None。"""
    if isinstance(obj, dict):
        return {k: _clean(v, sig) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_clean(v, sig) for v in obj]
    if isinstance(obj, np.ndarray):
        return _clean(obj.tolist(), sig)
    if isinstance(obj, (bool, np.bool_)):
        return bool(obj)
    if isinstance(obj, (int, np.integer)):
        return int(obj)
    if isinstance(obj, (float, np.floating)):
        x = float(obj)
        if not math.isfinite(x):
            return None
        return float(f"{x:.{sig}g}")
    return obj


class FilterSet:
    """一組參數下的 EKF 與 Mahony。"""

    def __init__(self, values):
        self.values = dict(values)
        self.ekf = native.Ekf()
        self.mah = native.Mahony()

    def set_values(self, values):
        self.values = dict(values)
        if self.ekf.ready:
            self.ekf.set_params(params.ekf_array(self.values))
        if self.mah.ready:
            self.mah.set_params(params.mah_array(self.values))

    def init_ekf(self, accel):
        self.ekf.init(params.ekf_array(self.values), accel)

    def init_mah(self, accel):
        self.mah.init(params.mah_array(self.values), accel)


class Hub:
    def __init__(self):
        self.lock = threading.RLock()
        self.values = {"A": dict(params.DEFAULTS), "B": dict(params.DEFAULTS)}
        self.sensor = dict(params.SENSOR_DEFAULTS)
        self.sets = {"A": FilterSet(self.values["A"]), "B": FilterSet(self.values["B"])}
        self.source = None
        self.clients = []
        self.rows = []
        self.log = []
        self.recording = None
        self.dt = sim.DT
        self.reset_local()

    # ---- 狀態 ----

    def reset_local(self):
        self.t0_seq = None
        self.last_seq = None
        self.samples = 0
        self.gaps = 0
        self.fw_diff_max = 0.0
        self.fw_rev_params = {}
        self.fw_rev = None
        self.fw_hello = None
        self.synced = False
        self.pending = {"ekf": False, "mah": False, "B": False}
        self.cal = None
        self.cal_req = False
        self.latest = None
        self.rate_hist = []
        self.last_info = {}
        for s in self.sets.values():
            s.ekf.ready = s.mah.ready = False

    def say(self, text):
        stamp = datetime.now().strftime("%H:%M:%S")
        with self.lock:
            self.log.append(f"{stamp} {text}")
            self.log = self.log[-200:]

    def status(self):
        src = self.source
        rate = 0.0
        if len(self.rate_hist) >= 2:
            (t0, n0), (t1, n1) = self.rate_hist[0], self.rate_hist[-1]
            rate = (n1 - n0) / max(t1 - t0, 1e-6)
        st = {
            "source": src.describe() if src else None,
            "samples": self.samples, "rate": round(rate, 1), "gaps": self.gaps,
            "synced": self.synced, "fw_diff_max_deg": float(f"{self.fw_diff_max:.3g}"),
            "fw_rev": self.fw_rev, "fw_hello": self.fw_hello,
            "recording": None if self.recording is None else len(self.recording["accel"]),
            "log": self.log[-40:],
        }
        st.update(self.last_info)
        return st

    # ---- 參數 ----

    def set_values(self, tag, values):
        with self.lock:
            merged = dict(self.values[tag])
            merged.update({k: float(v) for k, v in values.items() if k in params.DEFAULTS})
            if tag == "A" and isinstance(self.source, SerialSource):
                # 目前參數以控制板為準：送出後等 MSG_PARAMS 回來才更新副本。
                self.source.send(protocol.set_params([merged[n] for n in params.NAMES]))
                self.say("已送出參數到控制板")
                return
            self.values[tag] = merged
            self.sets[tag].set_values(merged)

    def command(self, name):
        with self.lock:
            if name == "reset_B":
                self.pending["B"] = True
                return
            if isinstance(self.source, SerialSource):
                self.source.send(protocol.cmd(name))
                return
            if name in ("reset_ekf", "reset_both"):
                self.pending["ekf"] = True
            if name in ("reset_mahony", "reset_both"):
                self.pending["mah"] = True
            if name == "mahony_cal":
                self.cal_req = True
                self.say("Mahony 靜置校正：保持靜止 2 s")

    # ---- 來源 ----

    def set_source(self, src):
        old = self.source
        if old:
            old.stop()
        with self.lock:
            self.source = None
            self.reset_local()
            self.rows = []
            self.source = src
        src.start()

    def on_message(self, kind, d):
        with self.lock:
            if kind == protocol.MSG_HELLO:
                self.fw_hello = d
                if d["proto_version"] != protocol.PROTO_VERSION:
                    self.say(f"協議版本不符：控制板 {d['proto_version']}，主機 {protocol.PROTO_VERSION}")
            elif kind == protocol.MSG_PARAMS:
                if d["names"] != params.NAMES:
                    self.say("控制板的參數表與 params.py 不一致：" + ", ".join(d["names"]))
                    return
                vals = dict(zip(d["names"], d["values"]))
                self.fw_rev_params[d["rev"]] = vals
                self.values["A"] = vals
                self.say(f"控制板參數版本 {d['rev']}")
                if not self.synced and isinstance(self.source, SerialSource):
                    # 現在知道控制板的參數了，再要求一次重設，副本才能從同一個起點、同一組參數開始。
                    self.source.send(protocol.cmd("reset_both"))
            elif kind == protocol.MSG_TEXT:
                self.say("控制板：" + d["text"])

    def on_sample(self, s, fw=None, truth=None):
        """s：accel_g、gyro_dps、temp_c、flags（串口時由韌體決定重設時機）。"""
        with self.lock:
            self._process(s, fw, truth)

    def _process(self, s, fw, truth):
        A, B = self.sets["A"], self.sets["B"]
        flags = s.get("flags", 0)
        seq = s["seq"]
        if self.t0_seq is None:
            self.t0_seq = seq
        realign = False
        if self.last_seq is not None and seq != self.last_seq + 1 and fw is not None:
            # 掉了樣本，副本無法逐筆重現：這一筆照常計算後，以韌體的 q、b 覆蓋（P 沿用）。
            self.gaps += 1
            realign = True
            self.say(f"樣本序號不連續（{self.last_seq} → {seq}），副本已以韌體狀態重新對齊")
        self.last_seq = seq
        self.samples += 1
        now = time.monotonic()
        self.rate_hist.append((now, self.samples))
        while self.rate_hist and now - self.rate_hist[0][0] > 2.0:
            self.rate_hist.pop(0)

        if fw is not None:
            rev = s["param_rev"]
            if rev not in self.fw_rev_params:
                self.synced = False      # 還不知道這一筆用的參數，副本無法重現
                return
            if rev != self.fw_rev:
                self.fw_rev = rev
                A.set_values(self.fw_rev_params[rev])
            self.last_info = {"isr_us": s["isr_us"], "overruns": s["overruns"],
                              "temp_c": _r(s["temp_c"], 2)}
        if flags & protocol.SF_IMU_ERROR:
            self.say("IMU 讀取失敗（這一筆兩個濾波器都沒有執行）")
            return

        acc, gyr = s["accel_g"], s["gyro_dps"]
        if fw is None:
            # 模擬或重播：重設時機由主機決定。
            if not A.ekf.ready or self.pending["ekf"]:
                flags |= protocol.SF_EKF_INIT
            if not A.mah.ready or self.pending["mah"]:
                flags |= protocol.SF_MAH_INIT
            self.pending["ekf"] = self.pending["mah"] = False
            if self.cal_req:
                self.cal_req = False
                self.cal = {"n": 0, "sum": np.zeros(3), "sq": np.zeros(3)}
            if self.cal is not None and not flags & protocol.SF_MAH_INIT:
                c = self.cal
                if c["n"] < 400:
                    c["n"] += 1
                    c["sum"] += gyr
                    c["sq"] += np.square(gyr)
                else:
                    mean = c["sum"] / 400
                    std = np.sqrt(np.maximum(c["sq"] / 400 - mean * mean, 0)).max()
                    self.cal = None
                    flags |= protocol.SF_MAH_BIAS_SET
                    s = dict(s, mah_b=list(mean * D2R))
                    self.say(f"Mahony 校正：零偏 {mean[0]:+.3f} {mean[1]:+.3f} {mean[2]:+.3f} dps，"
                             f"最大標準差 {std:.3f} dps")
        elif flags & protocol.SF_EKF_INIT:
            self.synced = True
            self.fw_diff_max = 0.0

        if fw is not None and not self.synced:
            return    # 等控制板下一次重設，副本才能從同一個起點開始

        reset_b = self.pending["B"]
        self.pending["B"] = False
        for tag, fs in (("A", A), ("B", B)):
            if flags & protocol.SF_EKF_INIT or (tag == "B" and (reset_b or not fs.ekf.ready)):
                fs.init_ekf(acc)
            else:
                fs.ekf.step(gyr, acc, self.dt)
            if flags & protocol.SF_MAH_INIT or (tag == "B" and (reset_b or not fs.mah.ready)):
                fs.init_mah(acc)
            elif flags & protocol.SF_MAH_BIAS_SET:
                bias = s["mah_b"] if fw is None else fw["mah_b"]
                fs.mah.set_bias(np.array(bias))
            else:
                fs.mah.update(gyr, acc, self.dt)

        if realign:
            A.ekf.set_state(np.array(fw["ekf_q"]), np.array(fw["ekf_b"]))
            A.mah.set_state(np.array(fw["mah_q"]), np.array(fw["mah_b"]))
        ea, ma = A.ekf.export(), A.mah.export()
        eb, mb = B.ekf.export(), B.mah.export()
        t = (seq - self.t0_seq) * self.dt
        row = {"t": round(t, 4)}
        for i, k in enumerate("xyz"):
            row["g" + k] = _r(gyr[i], 4)
            row["a" + k] = _r(acc[i], 5)
        row["an"] = _r(math.sqrt(acc[0] ** 2 + acc[1] ** 2 + acc[2] ** 2), 5)
        row["temp"] = _r(s.get("temp_c"), 2)

        def put(prefix, ekf_x, mah_x):
            r, p, y = _euler(ekf_x["q"])
            row.update({prefix + "r": _r(r), prefix + "p": _r(p), prefix + "y": _r(y)})
            P = ekf_x["P"]
            for i, k in enumerate("xyz"):
                row[prefix + "s" + k] = _r(math.sqrt(max(P[i][i], 0)) * R2D, 6)
                row[prefix + "b" + k] = _r(ekf_x["b"][i] * R2D, 5)
                row[prefix + "sb" + k] = _r(math.sqrt(max(P[3 + i][3 + i], 0)) * R2D, 6)
            row[prefix + "nis"] = _r(ekf_x["nis"], 4) if ekf_x["accel_used"] else None
            row[prefix + "sa"] = _r(ekf_x["sigma_a"], 5)
            r, p, y = _euler(mah_x["q"])
            row.update({prefix + "mr": _r(r), prefix + "mp": _r(p), prefix + "my": _r(y)})
            for i, k in enumerate("xyz"):
                row[prefix + "mb" + k] = _r(mah_x["b"][i] * R2D, 5)

        put("A", ea, ma)
        put("B", eb, mb)
        row["still"] = int(ea["still"])
        row["accu"] = int(ea["accel_used"])
        row["zupt"] = int(ea["zupt_used"])
        row["maccu"] = int(ma["accel_used"])
        if fw is not None:
            fr, fp, fy = _euler(fw["ekf_q"])
            mr, mp, my = _euler(fw["mah_q"])
            row.update({"Fr": _r(fr), "Fp": _r(fp), "Fy": _r(fy), "Fmr": _r(mr), "Fmp": _r(mp), "Fmy": _r(my)})
            # 用未四捨五入的值比較
            ar, ap, ay = _euler(ea["q"])
            amr, amp, amy = _euler(ma["q"])
            diff = max(abs(fr - ar), abs(fp - ap), abs((fy - ay + 180) % 360 - 180),
                       abs(mr - amr), abs(mp - amp), abs((my - amy + 180) % 360 - 180))
            row["dfw"] = float(f"{diff:.3g}")
            self.fw_diff_max = max(self.fw_diff_max, diff)
        if truth is not None:
            tr, tp, ty = _euler(truth["q"])
            row.update({"Tr": _r(tr), "Tp": _r(tp), "Ty": _r(ty)})
            for i, k in enumerate("xyz"):
                row["Tb" + k] = _r(truth["b"][i] * R2D, 5)
            for prefix, x in (("A", ea), ("B", eb)):
                err = sim.att_error(np.array(x["q"]), np.array(truth["q"])) * R2D
                for i, k in enumerate("xyz"):
                    row[prefix + "e" + k] = _r(err[i], 5)
            for prefix, x in (("A", ma), ("B", mb)):
                err = sim.att_error(np.array(x["q"]), np.array(truth["q"])) * R2D
                for i, k in enumerate("xyz"):
                    row[prefix + "me" + k] = _r(err[i], 5)
        self.rows.append(row)
        self.latest = {"A": ea, "Am": ma, "Bq": eb["q"], "BP": eb["P"], "Bmq": mb["q"],
                       "Fq": fw["ekf_q"] if fw else None, "Tq": list(truth["q"]) if truth else None,
                       "accel": list(acc), "gyro": list(gyr), "t": t}

        if self.recording is not None:
            rec = self.recording
            rec["accel"].append(list(acc))
            rec["gyro"].append(list(gyr))
            rec["temp"].append(s.get("temp_c", 0.0))
            rec["flags"].append(flags)
            rec["seq"].append(seq)
            if truth is not None:
                rec["q_true"].append(list(truth["q"]))
                rec["b_true"].append(list(truth["b"]))

    # ---- 錄製 ----

    def record(self, action):
        with self.lock:
            if action == "start":
                self.recording = {k: [] for k in ("accel", "gyro", "temp", "flags", "seq", "q_true", "b_true")}
                self.recording_meta = {"params_A": dict(self.values["A"]),
                                       "source": self.source.describe() if self.source else None,
                                       "started": datetime.now().isoformat(timespec="seconds")}
                return None
            rec, self.recording = self.recording, None
        if not rec or not rec["accel"]:
            return None
        RECORDINGS.mkdir(exist_ok=True)
        name = datetime.now().strftime("%Y%m%d-%H%M%S") + ".npz"
        arrays = {k: np.array(v) for k, v in rec.items() if v}
        np.savez_compressed(RECORDINGS / name, meta=json.dumps(self.recording_meta), **arrays)
        self.say(f"已存檔 recordings/{name}（{len(rec['accel'])} 筆）")
        return name

    # ---- 推送 ----

    def broadcast_loop(self):
        last_latest = 0.0
        while True:
            time.sleep(BATCH_PERIOD)
            with self.lock:
                rows, self.rows = self.rows, []
                latest = None
                now = time.monotonic()
                if self.latest is not None and now - last_latest >= LATEST_PERIOD:
                    latest = self.latest
                    last_latest = now
                status = self.status()
            msg = {"status": status}
            if rows:
                keys = list(rows[-1].keys())
                msg["cols"] = {k: [r.get(k) for r in rows] for k in keys}
            if latest is not None:
                msg["latest"] = _clean(latest)
            data = json.dumps(msg, separators=(",", ":"))
            for q in list(self.clients):
                try:
                    q.put_nowait(data)
                except queue.Full:
                    pass


class SerialSource:
    def __init__(self, hub, port):
        import serial  # noqa: F401（確認 pyserial 可用）
        self.hub, self.port = hub, port
        self.ser = None
        self.running = False
        self.tx_lock = threading.Lock()
        self.deframer = protocol.Deframer()

    def describe(self):
        return {"type": "serial", "port": self.port, "baud": protocol.BAUD,
                "crc_errors": self.deframer.errors, "open": self.ser is not None}

    def start(self):
        import c50c
        self.ser = c50c.open_quiet(self.port)
        self.ser.baudrate = protocol.BAUD
        self.ser.timeout = 0.05
        self.running = True
        threading.Thread(target=self._run, daemon=True).start()
        self.send(protocol.cmd("hello"))
        self.send(protocol.cmd("reset_both"))
        self.hub.say(f"已開啟 {self.port}（{protocol.BAUD} baud），要求控制板重設濾波器以同步副本")

    def reset_board(self):
        import c50c
        c50c.pulse_reset(self.ser)
        self.ser.baudrate = protocol.BAUD

    def stop(self):
        self.running = False
        time.sleep(0.1)
        if self.ser:
            self.ser.close()
            self.ser = None

    def send(self, data):
        with self.tx_lock:
            if self.ser:
                self.ser.write(data)

    def _run(self):
        last_hello = time.monotonic()
        while self.running:
            try:
                data = self.ser.read(4096)
            except Exception as e:  # 拔線等
                self.hub.say(f"串口錯誤：{e}")
                self.running = False
                break
            if self.hub.fw_hello is None and time.monotonic() - last_hello > 1.0:
                last_hello = time.monotonic()
                self.send(protocol.cmd("hello"))
            if not data:
                continue
            for payload in self.deframer.feed(data):
                kind, d = protocol.parse(payload)
                if kind == protocol.MSG_SAMPLE:
                    sample = {"accel_g": np.array(d["accel_g"], dtype=np.float32),
                              "gyro_dps": np.array(d["gyro_dps"], dtype=np.float32), **d}
                    self.hub.on_sample(sample, fw=d)
                elif kind is not None:
                    self.hub.on_message(kind, d)


class SimSource:
    """以實際時間播放模擬情境；播完重新產生（新的雜訊種子）並重設濾波器。"""

    def __init__(self, hub, scenario, seed=1, speed=1.0, recording=None):
        self.hub, self.scenario, self.seed, self.speed = hub, scenario, seed, speed
        self.recording = recording
        self.running = False

    def describe(self):
        if self.recording:
            return {"type": "replay", "file": self.recording, "speed": self.speed}
        return {"type": "sim", "scenario": self.scenario, "seed": self.seed, "speed": self.speed}

    def _load(self):
        if self.recording:
            z = np.load(RECORDINGS / self.recording)
            d = {"accel_g": z["accel"], "gyro_dps": z["gyro"], "temp_c": z["temp"]}
            if "q_true" in z:
                d["q_true"], d["b_true"] = z["q_true"], z["b_true"]
            return d
        return sim.generate(self.scenario, self.hub.sensor, seed=self.seed)

    def start(self):
        self.running = True
        threading.Thread(target=self._run, daemon=True).start()

    def stop(self):
        self.running = False
        time.sleep(0.1)

    def _run(self):
        seq = 0
        while self.running:
            d = self._load()
            n = len(d["accel_g"])
            acc = d["accel_g"].astype(np.float32)
            gyr = d["gyro_dps"].astype(np.float32)
            self.hub.command("reset_both")
            t_start = time.monotonic()
            k = 0
            while self.running and k < n:
                due = int((time.monotonic() - t_start) * sim.RATE_HZ * self.speed)
                while k < min(due, n):
                    truth = {"q": d["q_true"][k], "b": d["b_true"][k]} if "q_true" in d else None
                    self.hub.on_sample({"accel_g": acc[k], "gyro_dps": gyr[k], "temp_c": float(d["temp_c"][k]),
                                        "seq": seq}, truth=truth)
                    k += 1
                    seq += 1
                time.sleep(0.01)
            self.seed += 1


def run_scenario(hub, req):
    """同一段模擬資料，A、B 兩組參數的 EKF 與 Mahony 一次跑完，與真值比較。"""
    with hub.lock:
        va, vb = dict(hub.values["A"]), dict(hub.values["B"])
        sensor = dict(hub.sensor)
    sensor.update({k: float(v) for k, v in req.get("sensor", {}).items() if k in sensor})
    name = req.get("scenario", "accel")
    d = sim.generate(name, sensor, seed=int(req.get("seed", 1)))
    q_true, b_true = d["q_true"], d["b_true"]
    bias0 = b_true[0] if req.get("mahony_cal", True) else None
    dec = slice(None, None, 4)
    out = {"t": d["t"][dec], "truth_deg": sim.euler_deg(q_true)[dec], "b_true_dps": np.degrees(b_true)[dec],
           "a_norm": np.linalg.norm(d["accel_g"], axis=1)[dec], "runs": {}}
    settle = d["t"] >= 2.0
    for tag, v in (("A", va), ("B", vb)):
        e = native.ekf_run(params.ekf_array(v), d["gyro_dps"], d["accel_g"], sim.DT)
        m = native.mah_run(params.mah_array(v), d["gyro_dps"], d["accel_g"], sim.DT, bias0=bias0)
        err_e = sim.att_error(e["q"], q_true) * R2D
        err_m = sim.att_error(m["q"], q_true) * R2D
        sig = np.sqrt(np.maximum(e["Pdiag"][:, :3], 0)) * R2D
        sigb = np.sqrt(np.maximum(e["Pdiag"][:, 3:], 0)) * R2D
        z = np.abs(err_e) / np.maximum(sig, 1e-9)
        used = (e["flags"] & 2) > 0
        out["runs"][tag] = {
            "ekf_err": err_e[dec], "ekf_sigma": sig[dec], "mah_err": err_m[dec],
            "ekf_b": np.degrees(e["b"])[dec], "ekf_sigb": sigb[dec], "mah_b": np.degrees(m["b"])[dec],
            "still": (e["flags"] & 1)[dec], "nis": np.where(used, e["nis"], np.nan)[dec],
            "metrics": {
                "ekf_rms": np.sqrt(np.mean(err_e[settle] ** 2, axis=0)),
                "ekf_max": np.max(np.abs(err_e[settle]), axis=0),
                "mah_rms": np.sqrt(np.mean(err_m[settle] ** 2, axis=0)),
                "mah_max": np.max(np.abs(err_m[settle]), axis=0),
                "within_1s": np.mean(z[settle] < 1, axis=0), "within_2s": np.mean(z[settle] < 2, axis=0),
                "nis_mean": float(np.nanmean(np.where(used, e["nis"], np.nan)[settle])) if used.any() else None,
                "accel_used": float(np.mean(used)),
                "still_frac": float(np.mean(e["flags"] & 1)),
                "bias_err_end": np.degrees(e["b"][-1] - b_true[-1]),
            },
        }
    return _clean(out, 5)


def make_handler(hub):
    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *a, **kw):
            super().__init__(*a, directory=str(STATIC), **kw)

        def log_message(self, *args):
            pass

        def _json(self, obj, code=200):
            data = json.dumps(_clean(obj), separators=(",", ":")).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _body(self):
            n = int(self.headers.get("Content-Length") or 0)
            return json.loads(self.rfile.read(n) or b"{}")

        def do_GET(self):
            if self.path == "/api/stream":
                return self._stream()
            if self.path == "/api/state":
                with hub.lock:
                    return self._json({"values": hub.values, "sensor": hub.sensor, "status": hub.status(),
                                       "meta": params.meta(), "sensor_meta": params.sensor_meta(),
                                       "scenarios": {k: v for k, v in sim.SCENARIOS.items()},
                                       "rate_hz": sim.RATE_HZ})
            if self.path == "/api/ports":
                from serial.tools import list_ports
                return self._json([{"device": p.device, "desc": p.description, "wch": p.vid == 0x1A86}
                                   for p in list_ports.comports()])
            if self.path == "/api/recordings":
                files = sorted(RECORDINGS.glob("*.npz")) if RECORDINGS.exists() else []
                return self._json([f.name for f in files])
            return super().do_GET()

        def do_POST(self):
            try:
                body = self._body()
                if self.path == "/api/params":
                    hub.set_values(body["set"], body["values"])
                    return self._json({"ok": True})
                if self.path == "/api/sensor":
                    with hub.lock:
                        hub.sensor.update({k: float(v) for k, v in body.items() if k in hub.sensor})
                    return self._json({"ok": True})
                if self.path == "/api/cmd":
                    if body["cmd"] == "reset_board" and isinstance(hub.source, SerialSource):
                        hub.source.reset_board()
                        hub.set_source(SerialSource(hub, hub.source.port))
                    else:
                        hub.command(body["cmd"])
                    return self._json({"ok": True})
                if self.path == "/api/source":
                    t = body["type"]
                    if t == "serial":
                        hub.set_source(SerialSource(hub, body["port"]))
                    elif t == "sim":
                        hub.set_source(SimSource(hub, body.get("scenario", "drive"), int(body.get("seed", 1)),
                                                 float(body.get("speed", 1.0))))
                    elif t == "replay":
                        hub.set_source(SimSource(hub, None, speed=float(body.get("speed", 1.0)),
                                                 recording=Path(body["file"]).name))
                    return self._json({"ok": True})
                if self.path == "/api/record":
                    return self._json({"file": hub.record(body["action"])})
                if self.path == "/api/analysis":
                    with hub.lock:
                        va, vb, sensor = dict(hub.values["A"]), dict(hub.values["B"]), dict(hub.sensor)
                    va.update(body.get("A", {}))
                    vb.update(body.get("B", {}))
                    sensor.update(body.get("sensor", {}))
                    return self._json(analysis.analyze(va, vb, sensor))
                if self.path == "/api/scenario":
                    return self._json(run_scenario(hub, body))
            except Exception as e:  # 回報給前端顯示
                hub.say(f"錯誤：{e}")
                return self._json({"error": str(e)}, 500)
            self.send_error(HTTPStatus.NOT_FOUND)

        def _stream(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            q = queue.Queue(maxsize=100)
            hub.clients.append(q)
            try:
                while True:
                    try:
                        data = q.get(timeout=1.0)
                        self.wfile.write(b"data: " + data.encode() + b"\n\n")
                    except queue.Empty:
                        self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                hub.clients.remove(q)

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", help="USART1 的串口（預設：唯一的 CH9102F）")
    ap.add_argument("--sim", nargs="?", const="drive", metavar="SCENARIO",
                    help="不接控制板，播放模擬情境：" + ", ".join(sim.SCENARIOS))
    ap.add_argument("--http", type=int, default=8008, help="HTTP 埠（預設 8008）")
    args = ap.parse_args()

    hub = Hub()
    threading.Thread(target=hub.broadcast_loop, daemon=True).start()
    if args.sim:
        hub.set_source(SimSource(hub, args.sim))
    else:
        import c50c
        try:
            port = args.port or c50c.resolve_port(None)
            hub.set_source(SerialSource(hub, port))
        except SystemExit as e:
            print(f"{e}; starting with the simulator instead", file=sys.stderr)
            hub.set_source(SimSource(hub, "drive"))

    server = ThreadingHTTPServer(("127.0.0.1", args.http), make_handler(hub))
    server.daemon_threads = True
    print(f"AHRS Bench: http://localhost:{args.http}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
