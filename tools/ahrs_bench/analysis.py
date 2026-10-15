"""濾波器參數的線性化分析（單軸傾角）。

模型（小角度、角速度約 0、|a| ≈ 1 g，所以三軸解耦，每個 roll/pitch 軸都是同一個二維問題）：
  狀態 x = [θ, b]，陀螺儀 ω_m = θ̇ + b + n_g，零偏 b_k = b_{k−1} + n_b，
  加速度給出的傾角 z = θ + n_a（重力方向單位向量的切向分量），靜止時另有 ZUPT 量測 ω_m = b + n_g。
在這個模型下，EKF 的穩態就是一個固定增益的線性濾波器，和 Mahony 一樣可以寫成
  x̂_k = A x̂_{k−1} + B_ω ω_k + B_z z_k，
所以兩者可以放在同一個框架比較：頻率響應（互補濾波）、各雜訊源造成的誤差、對線性加速度的反應。

注意：
  - 這裡用的是 EKF 的穩態增益；實際 EKF 的增益隨 P 變化，且自適應 R 讓增益隨 |a| 改變。
    非線性與自適應的效果請看離線模擬（同一份 C 代碼在模擬資料上執行）。
  - 「濾波器自認的 σ」來自 Riccati 方程（假設模型正確）；「實際 σ」用真實雜訊（模擬器的感測器模型）
    求 Lyapunov 方程，包含 ZUPT 與預測共用同一筆陀螺儀雜訊的相關性。兩者不同就是模型不符。
"""

import numpy as np
import scipy.linalg as sla
from scipy.signal import lfilter, ss2tf

D2R = np.pi / 180.0
R2D = 180.0 / np.pi
DT = 1.0 / 200.0
E2 = np.array([0.0, 1.0])


def _pe(p):
    return {k: float(v) for k, v in p.items()}


# ---- 穩態卡爾曼濾波 ----

def kf_matrices(p, still, dt=DT, sigma_a=None):
    sg, sb = p["ekf.gyro_noise"] * D2R, p["ekf.bias_walk"] * D2R
    sa = p["ekf.acc_noise"] if sigma_a is None else sigma_a
    F = np.array([[1.0, -dt], [0.0, 1.0]])
    Q = np.diag([sg * sg * dt, sb * sb * dt])
    if still and p["ekf.zupt_on"] != 0:
        H = np.eye(2)
        R = np.diag([sa * sa, (p["ekf.zupt_noise"] * D2R) ** 2])
    else:
        H = np.array([[1.0, 0.0]])
        R = np.array([[sa * sa]])
    return F, Q, H, R


def steady_kf(F, Q, H, R, P0=None):
    """回傳 (P⁻, K, P⁺, 是否收斂)。DARE 失敗（例如 σ_b = 0）時改以迭代逼近。"""
    try:
        Pm = sla.solve_discrete_are(F.T, H.T, Q, R)
        if np.all(np.isfinite(Pm)):
            K = Pm @ H.T @ np.linalg.inv(H @ Pm @ H.T + R)
            return Pm, K, (np.eye(2) - K @ H) @ Pm, True
    except (np.linalg.LinAlgError, ValueError):
        pass
    P = np.diag([1e-2, 1e-4]) if P0 is None else P0
    converged = False
    for _ in range(200000):
        Pm = F @ P @ F.T + Q
        K = Pm @ H.T @ np.linalg.inv(H @ Pm @ H.T + R)
        Pn = (np.eye(2) - K @ H) @ Pm
        if np.max(np.abs(Pn - P)) < 1e-18:
            converged = True
            P = Pn
            break
        P = Pn
    return Pm, K, P, converged


# ---- 統一的線性濾波器表示：x̂_k = A x̂_{k−1} + B_ω ω_k + B_z z_k ----

def kf_linear(K, H, F, still, dt=DT):
    G = np.array([dt, 0.0])
    IKH = np.eye(2) - K @ H
    A = IKH @ F
    Bw = IKH @ G
    if still and H.shape[0] == 2:
        Bw = Bw + K[:, 1]
    return A, Bw, K[:, 0].copy()


def mahony_linear(kp, ki, dt=DT):
    A = np.array([[1.0 - kp * dt, -dt], [ki * dt, 1.0]])
    return A, np.array([dt, 0.0]), np.array([kp * dt, -ki * dt])


def _noise(sensor, dt=DT):
    """每筆雜訊的變異數：陀螺儀（rad/s）²、零偏增量（rad/s）²、傾角量測（rad）²。"""
    return {
        "gyro": (sensor["gyro_arw"] * D2R) ** 2 / dt,
        "bias_walk": (sensor["gyro_bias_walk"] * D2R) ** 2 * dt,
        "accel": (sensor["accel_noise"] * 1e-3) ** 2 + sensor.get("vibration", 0.0) ** 2,
    }


def _lyap(A, W):
    """穩態誤差共變異數；不收斂（例如 Ki = 0 時零偏誤差不會被修正）的項回傳 inf。"""
    rho = np.max(np.abs(np.linalg.eigvals(A)))
    if rho < 1.0 - 1e-12:
        return sla.solve_discrete_lyapunov(A, W)
    S, Ak = W.copy(), A.copy()
    hist = []
    for _ in range(26):
        S = S + Ak @ S @ Ak.T
        Ak = Ak @ Ak
        hist.append(np.diag(S).copy())
    d = hist[-1]
    grow = hist[-1] > hist[-2] * 1.01 + 1e-30
    return np.diag(np.where(grow, np.inf, d))


def error_cov(A, Bw, Bz, noise):
    """各雜訊源造成的穩態誤差變異數（θ 與 b），以及總和。"""
    cols = {"gyro": Bw, "bias_walk": -E2, "accel": Bz}
    out = {}
    total = np.zeros((2, 2))
    for k, col in cols.items():
        S = _lyap(A, np.outer(col, col) * noise[k])
        out[k] = np.diag(S)
        total = total + np.where(np.isfinite(S), S, 0) + np.diag(np.where(np.isfinite(np.diag(S)), 0, np.inf))
    out["total"] = np.diag(total)
    return out


def _resolvent_row0(A, z):
    """(I − A z⁻¹)⁻¹ 的第一列，對每個 z 向量化計算（2×2 的解析逆矩陣）。"""
    m11, m12 = 1 - A[0, 0] / z, -A[0, 1] / z
    m21, m22 = -A[1, 0] / z, 1 - A[1, 1] / z
    det = m11 * m22 - m12 * m21
    return np.stack([m22 / det, -m12 / det], axis=-1)


def bode(A, Bw, Bz, f, dt=DT):
    """加速度路徑 T_z = θ̂/z 與陀螺儀路徑 T_g = θ̂/θ（ω = θ̇）；兩者相加為 1（互補）。"""
    z = np.exp(1j * 2 * np.pi * f * dt)
    r = _resolvent_row0(A, z)
    Tz = r @ Bz
    Tg = (r @ Bw) * (1 - 1 / z) / dt
    return Tz, Tg


def error_psd(A, Bw, Bz, noise, f, dt=DT):
    """θ 誤差的單邊功率譜密度，按雜訊源分開，deg²/Hz。"""
    r = _resolvent_row0(A, np.exp(1j * 2 * np.pi * f * dt))
    out = {"gyro": np.abs(r @ Bw) ** 2 * noise["gyro"],
           "bias_walk": np.abs(r @ (-E2)) ** 2 * noise["bias_walk"],
           "accel": np.abs(r @ Bz) ** 2 * noise["accel"]}
    return {k: v * 2 * dt * R2D * R2D for k, v in out.items()}


def simulate(A, Bw, Bz, w, z):
    """θ̂ 對輸入序列的響應（初值 0）。以轉移函數 + lfilter 計算。"""
    C = np.array([1.0, 0.0])
    y = np.zeros(len(w))
    for B, u in ((Bw, w), (Bz, z)):
        if not np.any(u):
            continue
        # x_k = A x_{k−1} + B u_k → 以 s_k = x_{k−1} 寫成標準形：y_k = C A s_k + C B u_k
        num, den = ss2tf(A, B.reshape(2, 1), (C @ A).reshape(1, 2), np.array([[C @ B]]))
        y += lfilter(num[0], den, u)
    return y


def _bandwidth(A, Bw, Bz):
    """加速度路徑 |T_z| 降到 DC 值 1/√2 的頻率（Hz）。"""
    f = np.logspace(-4, np.log10(99), 400)
    Tz, _ = bode(A, Bw, Bz, f)
    mag = np.abs(Tz)
    below = np.nonzero(mag < mag[0] / np.sqrt(2))[0]
    return float(f[below[0]]) if len(below) else None


def _pulse(t, a_h, t0=1.0, dur=2.0):
    return np.where((t >= t0) & (t < t0 + dur), np.arctan(a_h), 0.0)


def _peak_pulse(A, Bw, Bz, a_h=0.2):
    t = np.arange(0, 8, DT)
    return float(np.max(np.abs(simulate(A, Bw, Bz, np.zeros_like(t), _pulse(t, a_h))))) * R2D


def _fin(x):
    x = np.asarray(x, dtype=float)
    if x.ndim == 0:
        return float(x) if np.isfinite(x) else None
    return [float(v) if np.isfinite(v) else None for v in x]


def _describe_kf(p, still, noise):
    F, Q, H, R = kf_matrices(p, still)
    Pm, K, Pp, ok = steady_kf(F, Q, H, R)
    A, Bw, Bz = kf_linear(K, H, F, still)
    cov = error_cov(A, Bw, Bz, noise)
    kp_eq, ki_eq = K[0, 0] / DT, -K[1, 0] / DT
    d = {
        "converged": ok,
        "K": K.tolist(),
        "P_prior": Pm.tolist(), "P_post": Pp.tolist(),
        "sigma_theta_filter_deg": float(np.sqrt(max(Pp[0, 0], 0.0)) * R2D),
        "sigma_b_filter_dps": float(np.sqrt(max(Pp[1, 1], 0.0)) * R2D),
        "sigma_theta_true_deg": _fin(np.sqrt(cov["total"][0]) * R2D),
        "sigma_b_true_dps": _fin(np.sqrt(cov["total"][1]) * R2D),
        "contrib_theta_deg2": {k: _fin(v[0] * R2D * R2D) for k, v in cov.items() if k != "total"},
        "kp_eq": float(kp_eq), "ki_eq": float(ki_eq),
        "eig": [abs(e) for e in np.linalg.eigvals(A)],
    }
    if ki_eq > 0:
        d["wn"] = float(np.sqrt(ki_eq))
        d["zeta"] = float(kp_eq / (2 * np.sqrt(ki_eq)))
    return d, (A, Bw, Bz)


def _describe_mahony(p, noise, bias_dps):
    kp, ki = p["mah.kp"], p["mah.ki"]
    A, Bw, Bz = mahony_linear(kp, ki)
    cov = error_cov(A, Bw, Bz, noise)
    d = {
        "sigma_theta_true_deg": _fin(np.sqrt(cov["total"][0]) * R2D),
        "sigma_b_true_dps": _fin(np.sqrt(cov["total"][1]) * R2D),
        "contrib_theta_deg2": {k: _fin(v[0] * R2D * R2D) for k, v in cov.items() if k != "total"},
        "sigma_theta_white_deg": _fin(np.sqrt(cov["gyro"][0] + cov["accel"][0]) * R2D),
        "kp": kp, "ki": ki,
        "eig": [abs(e) for e in np.linalg.eigvals(A)],
    }
    if ki > 0:
        d["wn"] = float(np.sqrt(ki))
        d["zeta"] = float(kp / (2 * np.sqrt(ki)))
        d["bias_offset_deg"] = 0.0
    else:
        # Ki = 0：常數零偏 b 讓穩態傾角誤差 = b / Kp（x、y 軸取較大者）。
        d["bias_offset_deg"] = float(max(abs(bias_dps[0]), abs(bias_dps[1])) / kp) if kp > 0 else None
    return d, (A, Bw, Bz)


def analyze(pa, pb, sensor):
    """pa、pb：兩組參數（目前 / 候選），sensor：真實雜訊模型。"""
    pa, pb, sensor = _pe(pa), _pe(pb), _pe(sensor)
    noise = _noise(sensor)
    bias_dps = [sensor["gyro_bias_x"], sensor["gyro_bias_y"], sensor["gyro_bias_z"]]
    f = np.logspace(-3, np.log10(99.0), 240)
    t = np.arange(0, 20, DT)
    dec = slice(None, None, 4)
    out = {"freq": f.tolist(), "t": t[dec].tolist(), "sets": {}}

    for tag, p in (("A", pa), ("B", pb)):
        kf_m, lin_m = _describe_kf(p, False, noise)
        kf_s, lin_s = _describe_kf(p, True, noise)
        mah, lin_h = _describe_mahony(p, noise, bias_dps)
        s = {"kf_moving": kf_m, "kf_still": kf_s, "mahony": mah, "bode": {}, "psd": {}, "resp": {}}
        for name, lin in (("kf", lin_m), ("mahony", lin_h)):
            Tz, Tg = bode(*lin, f)
            s["bode"][name] = {"acc_db": (20 * np.log10(np.abs(Tz) + 1e-300)).tolist(),
                               "gyro_db": (20 * np.log10(np.abs(Tg) + 1e-300)).tolist(),
                               "acc_phase": np.degrees(np.angle(Tz)).tolist(),
                               "gyro_phase": np.degrees(np.angle(Tg)).tolist(),
                               "bandwidth_hz": _bandwidth(*lin)}
            s["psd"][name] = {k: np.sqrt(v).tolist() for k, v in error_psd(*lin, noise, f).items()}
            s["resp"][name] = {
                "pulse_deg": (simulate(*lin, np.zeros_like(t), _pulse(t, 0.2)) * R2D)[dec].tolist(),
                "bias_step_deg": (simulate(*lin, np.where(t >= 1, 0.5 * D2R, 0), np.zeros_like(t)) * R2D)[dec].tolist(),
            }
        s["resp"]["kf_still"] = {
            "bias_step_deg": (simulate(*lin_s, np.where(t >= 1, 0.5 * D2R, 0), np.zeros_like(t)) * R2D)[dec].tolist()}
        s["adaptive"] = _adaptive(p)
        s["yaw"] = _yaw(p, sensor, noise)
        out["sets"][tag] = s

    out["tradeoff"] = _tradeoff(pa, pb, noise)
    return out


def _adaptive(p):
    """水平與垂直加速度對 σ_a、增益與視傾角的影響（EKF 的 |a| 自適應）。"""
    a = np.linspace(0, 0.6, 61)
    res = {"a_g": a.tolist(), "apparent_tilt_deg": np.degrees(np.arctan(a)).tolist()}
    for kind, dev in (("horizontal", np.sqrt(1 + a * a) - 1), ("vertical", a)):
        sig = p["ekf.acc_noise"] + p["ekf.acc_adapt"] * dev
        rejected = dev > p["ekf.acc_reject"]
        kp = []
        for s_i, r in zip(sig, rejected):
            if r:
                kp.append(0.0)
                continue
            F, Q, H, R = kf_matrices(p, False, sigma_a=float(s_i))
            _, K, _, _ = steady_kf(F, Q, H, R)
            kp.append(K[0, 0] / DT)
        res[kind] = {"dev_g": dev.tolist(), "sigma": sig.tolist(), "rejected": rejected.tolist(),
                     "kp_eq": kp, "mahony_used": (dev < p["mah.acc_gate"]).tolist()}
    return res


def _yaw(p, sensor, noise):
    """yaw 沒有絕對量測：靜止時 ZUPT 把 z 軸零偏收斂，開始運動後誤差隨時間成長。"""
    t = np.linspace(0, 600, 301)
    sg = p["ekf.gyro_noise"] * D2R
    sb = p["ekf.bias_walk"] * D2R
    # z 軸零偏只有 ZUPT 可觀測（yaw 本身不可觀測），所以零偏部分是一個純量問題：
    #   b 隨機遊走（每筆變異數 q），量測 ω_m = b + n_g（每筆變異數 r_z）。
    zupt = p["ekf.zupt_on"] != 0
    if zupt and sb == 0:
        sb_filter = sb_true = 0.0   # 零偏不變時無限長的靜止讓它完全收斂
    elif zupt:
        q = sb * sb * DT
        r_z = (p["ekf.zupt_noise"] * D2R) ** 2
        # 純量 Riccati：P⁻ = P⁺ + q，P⁺ = P⁻ r_z / (P⁻ + r_z)
        pm = (q + np.sqrt(q * q + 4 * q * r_z)) / 2
        k = pm / (pm + r_z)
        sb_filter = np.sqrt(pm * (1 - k))
        # 實際：e_b ← (1 − k)(e_b − n_b) + k·n_g，n_g 是真實的每筆陀螺儀雜訊。
        a = (1 - k) ** 2
        sb_true = np.sqrt((a * noise["bias_walk"] + k * k * noise["gyro"]) / (1 - a))
    else:
        sb_filter = p["ekf.init_bias"] * D2R
        sb_true = abs(sensor["gyro_bias_z"]) * D2R
    ng_t = sensor["gyro_arw"] * D2R
    kb_t = sensor["gyro_bias_walk"] * D2R
    filt = np.sqrt(sb_filter ** 2 * t ** 2 + sg ** 2 * t + sb ** 2 * t ** 3 / 3) * R2D
    true = np.sqrt(sb_true ** 2 * t ** 2 + ng_t ** 2 * t + kb_t ** 2 * t ** 3 / 3) * R2D
    mah = abs(sensor["gyro_bias_z"]) * t
    return {"t": t.tolist(), "ekf_filter_deg": filt.tolist(), "ekf_true_deg": true.tolist(),
            "mahony_uncal_deg": mah.tolist(), "sb_filter_dps": float(sb_filter * R2D),
            "sb_true_dps": float(sb_true * R2D)}


def _tradeoff(pa, pb, noise):
    """以等效 Kp（加速度修正的頻寬）為橫軸：雜訊 σθ 與「0.2 g、2 s 水平加速度」最大誤差的取捨。"""
    def point(kp, A, Bw, Bz, skip_bias=False):
        cov = error_cov(A, Bw, Bz, noise)
        var = cov["gyro"][0] + cov["accel"][0] + (0 if skip_bias else cov["bias_walk"][0])
        return [float(kp), _fin(np.sqrt(var) * R2D), _peak_pulse(A, Bw, Bz)]

    def kf_point(p, sigma_a=None):
        F, Q, H, R = kf_matrices(p, False, sigma_a=sigma_a)
        _, K, _, _ = steady_kf(F, Q, H, R)
        return point(K[0, 0] / DT, *kf_linear(K, H, F, False))

    fam = {"kf": [], "mahony_zeta1": [], "mahony_ki0": []}
    for sa in np.logspace(0, -3.5, 36):
        fam["kf"].append(kf_point(pa, float(sa)) + [float(sa)])
    for kp in np.logspace(-2.5, 1.7, 36):
        fam["mahony_zeta1"].append(point(kp, *mahony_linear(kp, kp * kp / 4)))
        fam["mahony_ki0"].append(point(kp, *mahony_linear(kp, 0.0), skip_bias=True))
    marks = {}
    for tag, p in (("A", pa), ("B", pb)):
        marks[f"kf_{tag}"] = kf_point(p)
        marks[f"mahony_{tag}"] = point(p["mah.kp"], *mahony_linear(p["mah.kp"], p["mah.ki"]), skip_bias=p["mah.ki"] == 0)
    # 用真實雜訊當濾波器參數的卡爾曼濾波：純雜訊意義下的最小 σθ（理論下限）。
    truth = dict(pa)
    truth["ekf.gyro_noise"] = np.sqrt(noise["gyro"] * DT) * R2D
    truth["ekf.bias_walk"] = max(np.sqrt(noise["bias_walk"] / DT) * R2D, 1e-9)
    truth["ekf.acc_noise"] = np.sqrt(noise["accel"])
    marks["kf_opt"] = kf_point(truth)
    return {"families": fam, "marks": marks}
