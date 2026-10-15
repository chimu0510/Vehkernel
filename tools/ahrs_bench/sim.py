"""IMU 模擬器：產生已知真值的運動情境與感測器讀值，用來檢驗濾波器與參數。

座標與控制板相同（REP-103）：世界座標 z 朝上，四元數 q 為本體到世界，[w, x, y, z]。
加速度計量的是比力 f = Rᵀ(a_lin + g·e_z)，以 g 為單位；靜止水平時讀 (0, 0, +1)。
雜訊為白雜訊（實際 IMU 經 DLPF 後是有色的），零偏為隨機遊走。
"""

import numpy as np

RATE_HZ = 200
DT = 1.0 / RATE_HZ
D2R = np.pi / 180.0


# ---- 四元數 ----

def qmul(a, b):
    aw, ax, ay, az = np.moveaxis(np.asarray(a), -1, 0)
    bw, bx, by, bz = np.moveaxis(np.asarray(b), -1, 0)
    return np.stack([aw * bw - ax * bx - ay * by - az * bz,
                     aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw], axis=-1)


def qconj(q):
    q = np.asarray(q)
    return q * np.array([1.0, -1.0, -1.0, -1.0])


def qexp(rv):
    """旋轉向量 → 四元數。"""
    rv = np.asarray(rv, dtype=float)
    ang = np.linalg.norm(rv, axis=-1, keepdims=True)
    half = 0.5 * ang
    s = np.where(ang > 1e-12, np.sin(half) / np.where(ang > 1e-12, ang, 1.0), 0.5)
    return np.concatenate([np.cos(half), rv * s], axis=-1)


def qrot_t(q, v):
    """Rᵀ v：世界向量轉到本體座標。"""
    qv = np.concatenate([np.zeros(np.shape(v)[:-1] + (1,)), v], axis=-1)
    return qmul(qmul(qconj(q), qv), q)[..., 1:]


def euler_deg(q):
    """ZYX 尤拉角（roll, pitch, yaw），度；與韌體 ekf_ahrs_euler 相同公式。"""
    q = np.asarray(q)
    w, x, y, z = np.moveaxis(q, -1, 0)
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.degrees(np.stack([roll, pitch, yaw], axis=-1))


def att_error(q_est, q_true):
    """姿態誤差 δθ（本體座標，弧度），定義同 EKF：q_true = q_est ⊗ exp(δθ)。"""
    dq = qmul(qconj(q_est), q_true)
    dq = dq * np.sign(dq[..., :1] + 1e-300)
    return 2.0 * dq[..., 1:]


# ---- 情境 ----

def _smooth_pulse(t, t0, t1, ramp=0.25):
    """t0..t1 之間為 1，前後以 cosine 斜坡過渡。"""
    up = np.clip((t - t0) / ramp, 0, 1)
    down = np.clip((t1 - t) / ramp, 0, 1)
    return 0.5 * (1 - np.cos(np.pi * np.minimum(up, down)))


def _rate_move(t, t0, dur, angle_deg):
    """t0 起 dur 秒轉 angle_deg 度的角速度（升餘弦，積分剛好等於角度），rad/s。"""
    tau = (t - t0) / dur
    inside = (tau >= 0) & (tau <= 1)
    return np.where(inside, angle_deg * D2R / dur * (1 - np.cos(2 * np.pi * tau)), 0.0)


SCENARIOS = {
    "still": ("靜止", 60.0, "完全靜止；看零偏與 σ 的收斂、ZUPT 的效果與 NIS 的統計。"),
    "tilt": ("傾斜", 30.0, "roll +30° 停 4 s 回水平，再 pitch +20° 停 4 s 回水平；看陀螺儀與加速度修正的分工。"),
    "accel": ("加減速", 25.0, "水平時 +0.3 g 加速 2 s、等速、−0.3 g 煞車 2 s，車體不轉；"
              "加速度計把線性加速度誤認為傾斜。"),
    "turn": ("原地旋轉", 30.0, "+90 dps 轉一圈、停、−45 dps 轉回；看 yaw 與 z 軸零偏、靜止偵測。"),
    "drive": ("行駛", 45.0, "UGV 行駛：起步、轉彎（向心加速度）、路面顛簸、停車，重複兩次。"),
}


def _profile(name, t):
    """回傳 ω_body (n,3) rad/s 與 a_lin_world (n,3) g。"""
    n = len(t)
    w = np.zeros((n, 3))
    a = np.zeros((n, 3))
    if name == "tilt":
        w[:, 0] = _rate_move(t, 5, 1.0, 30) - _rate_move(t, 10, 1.0, 30)
        w[:, 1] = _rate_move(t, 15, 1.0, 20) - _rate_move(t, 20, 1.0, 20)
    elif name == "accel":
        a[:, 0] = 0.3 * _smooth_pulse(t, 5, 7) - 0.3 * _smooth_pulse(t, 12, 14)
    elif name == "turn":
        w[:, 2] = _rate_move(t, 5, 8, 360) - _rate_move(t, 14, 16, 360)
    elif name == "drive":
        acc = np.zeros(n)
        yaw_rate = np.zeros(n)
        for t0 in (3.0, 24.0):
            acc += 0.1 * _smooth_pulse(t, t0, t0 + 2) - 0.1 * _smooth_pulse(t, t0 + 15, t0 + 17)
            yaw_rate += 0.5 * _smooth_pulse(t, t0 + 4, t0 + 7) - 0.5 * _smooth_pulse(t, t0 + 9, t0 + 12)
        v = np.cumsum(acc * 9.80665) * DT
        heading = np.cumsum(yaw_rate) * DT
        a[:, 0] = (acc * 9.80665 * np.cos(heading) - v * yaw_rate * np.sin(heading)) / 9.80665
        a[:, 1] = (acc * 9.80665 * np.sin(heading) + v * yaw_rate * np.cos(heading)) / 9.80665
        moving = np.clip(v / 1.0, 0, 1)
        rng = np.random.default_rng(7)
        bump = np.convolve(rng.normal(0, 1, n), np.ones(10) / 10, mode="same")
        a[:, 2] = 0.08 * moving * bump / (np.std(bump) + 1e-12)
        # 懸吊晃動：約 2 Hz、±1.5° 的 roll/pitch 擺動
        w[:, 0] = moving * 1.5 * D2R * 2 * np.pi * 2.0 * np.cos(2 * np.pi * 2.0 * t)
        w[:, 1] = moving * 1.0 * D2R * 2 * np.pi * 1.7 * np.cos(2 * np.pi * 1.7 * t + 1.0)
        w[:, 2] = yaw_rate
    return w, a


def generate(name, sensor, seed=1, duration=None):
    """產生一段情境。sensor 為 params.SENSOR 的值（dict）。"""
    dur = duration or SCENARIOS[name][1]
    n = int(round(dur * RATE_HZ))
    t = np.arange(n) * DT
    w_true, a_lin = _profile(name, t)
    rng = np.random.default_rng(seed)

    q = np.zeros((n, 4))
    q[0] = [1, 0, 0, 0]
    steps = qexp(w_true * DT)
    for k in range(1, n):
        q[k] = qmul(q[k - 1], steps[k])
    q /= np.linalg.norm(q, axis=1, keepdims=True)

    b0 = np.array([sensor["gyro_bias_x"], sensor["gyro_bias_y"], sensor["gyro_bias_z"]]) * D2R
    walk = rng.normal(0, 1, (n, 3)) * sensor["gyro_bias_walk"] * D2R * np.sqrt(DT)
    walk[0] = 0
    b = b0 + np.cumsum(walk, axis=0)
    gyro = w_true + b + rng.normal(0, 1, (n, 3)) * sensor["gyro_arw"] * D2R * np.sqrt(RATE_HZ)

    f = qrot_t(q, a_lin + np.array([0, 0, 1.0]))
    f += rng.normal(0, 1, (n, 3)) * sensor["accel_noise"] * 1e-3
    if sensor["vibration"] > 0:
        spec = np.fft.rfft(rng.normal(0, 1, (n, 3)), axis=0)
        freq = np.fft.rfftfreq(n, DT)
        spec[(freq < 30) | (freq > 80)] = 0
        vib = np.fft.irfft(spec, n, axis=0)
        f += vib / (vib.std(axis=0) + 1e-12) * sensor["vibration"]

    return {"t": t, "gyro_dps": np.degrees(gyro), "accel_g": f, "q_true": q, "b_true": b,
            "w_true": w_true, "a_lin": a_lin, "temp_c": np.full(n, 30.0)}
