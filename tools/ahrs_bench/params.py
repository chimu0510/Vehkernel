"""濾波器參數的說明（名稱與順序對應 examples/demos/ahrs_bench/main.c 的 k_params）。

ekf.* 的順序就是 ekf_ahrs_params_t 的欄位順序，mah.* 是 ahrs_params_t 的順序，
native.py 以此順序把參數陣列直接 memcpy 進結構。
"""

P = [
    # name, symbol (LaTeX), unit, default, min, max, log, group, label, description
    ("ekf.gyro_noise", r"\sigma_g", "dps/√Hz", 0.05, 0.001, 1.0, True, "Q：過程雜訊", "陀螺儀角度隨機遊走",
     "陀螺儀白雜訊的密度（ARW）。預測時加到姿態誤差的變異數 Q_θ = σ_g²·Δt。調大：濾波器較不信任陀螺儀積分，"
     "加速度修正的頻寬變高，穩態 σ_θ 變大，對線性加速度更敏感。真實值由 Allan 方差 τ = 1 s 處讀出"
     "（斜率 −½ 段）；ICM-20948 規格約 0.015 dps/√Hz，預設值留了震動餘量。"),
    ("ekf.bias_walk", r"\sigma_b", "dps/s/√Hz", 0.002, 1e-5, 0.1, True, "Q：過程雜訊", "零偏隨機遊走",
     "零偏隨時間漂移的密度，Q_b = σ_b²·Δt。調大：零偏估計跟得快（溫度變化時有用），但估計值較吵、"
     "等效 Ki 變大；調小：σ_b 收斂得很小，零偏真的改變時濾波器會過度自信而跟不上。"),
    ("ekf.acc_noise", r"\sigma_a", "單位向量", 0.02, 1e-4, 0.5, True, "R：加速度量測", "重力方向量測雜訊",
     "每筆加速度量測（正規化成單位向量）的標準差，R = σ²·I₃；小角度時約等於弧度。應包含感測器雜訊、"
     "振動與沒有建模的線性加速度。調小：更信任加速度計，roll/pitch 收斂快但加減速時誤差大。"
     "感測器本身每筆雜訊約 0.002（DLPF 51 Hz）。"),
    ("ekf.acc_adapt", r"\kappa", "1/g", 5.0, 0.0, 100.0, False, "R：加速度量測", "自適應斜率",
     "σ = σ_a + κ·||a| − 1 g|：|a| 偏離 1 g 時加大量測雜訊。垂直加速度（顛簸）是一階；水平加速度 a_h 只讓 |a| "
     "變化約 a_h²/2（二階）：0.1 g 只變 0.005 g，幾乎不被降權，0.3 g 變 0.044 g 才明顯。"),
    ("ekf.acc_reject", r"g_{\mathrm{rej}}", "g", 0.3, 0.01, 2.0, True, "R：加速度量測", "拒絕門檻",
     "||a| − 1 g| 超過此值時完全不採用加速度量測（只靠陀螺儀積分）。"),
    ("ekf.zupt_noise", r"\sigma_z", "dps", 0.1, 0.001, 5.0, True, "ZUPT：零角速度量測", "零角速度量測雜訊",
     "靜止時以「真實角速度 = 0」為量測：y = ω_量測 − b，R = σ_z²·I₃（每筆）。合理值是陀螺儀每筆雜訊"
     "的標準差（σ_g·√f_s 量級）。這個量測讓 z 軸零偏可觀測。模型假設它與預測用的陀螺儀雜訊獨立，"
     "實際上是同一筆讀值，所以 σ_z 太小時濾波器會過度自信。"),
    ("ekf.init_angle", r"\sigma_{\theta,0}", "deg", 5.0, 0.1, 90.0, True, "初始化", "初始 roll/pitch 不確定度",
     "初始化時 P 的姿態對角項；只在重設時使用。初始姿態由第一筆加速度決定，yaw 定義為 0。"),
    ("ekf.init_bias", r"\sigma_{b,0}", "dps", 1.0, 0.01, 10.0, True, "初始化", "初始零偏不確定度",
     "初始化時 P 的零偏對角項；越大，第一次靜止時零偏收斂越快（增益大）。只在重設時使用。"),
    ("ekf.still_std", r"s_\omega", "dps", 0.3, 0.01, 5.0, True, "靜止偵測", "角速度標準差上限",
     "原始角速度的移動標準差（約 0.1 s 視窗）低於此值才可能是靜止；不需要先知道零偏。"),
    ("ekf.still_rate", r"\omega_{\mathrm{still}}", "dps", 3.0, 0.1, 30.0, True, "靜止偵測", "角速度上限",
     "扣掉估計零偏後的角速度低於此值。比它慢的等速轉動會被當成零偏吃掉（ZUPT 的根本限制）。"),
    ("ekf.still_acc", r"a_{\mathrm{still}}", "g", 0.05, 0.001, 0.5, True, "靜止偵測", "||a|−1 g| 上限", ""),
    ("ekf.still_time", r"T_{\mathrm{still}}", "s", 0.5, 0.0, 5.0, False, "靜止偵測", "需持續時間", ""),
    ("ekf.zupt_on", "", "0/1", 1.0, 0.0, 1.0, False, "ZUPT：零角速度量測", "啟用 ZUPT",
     "0 時完全不做零角速度量測：z 軸零偏不可觀測，yaw 會以零偏誤差持續漂移。"),
    ("mah.kp", r"K_p", "1/s", 1.0, 0.001, 50.0, True, "Mahony", "比例增益",
     "ω ← ω − b + K_p·e，e = â × v。小角度時 roll/pitch 的加速度修正是一階低通，時間常數 1/K_p；"
     "Ki = 0 時互補濾波的交越頻率 f_c = K_p / 2π。"),
    ("mah.ki", r"K_i", "1/s²", 0.0, 0.0, 5.0, False, "Mahony", "積分增益（零偏）",
     "b ← b − K_i·e·Δt。K_i > 0 時誤差動態為二階：s² + K_p s + K_i，ω_n = √K_i、ζ = K_p / (2√K_i)。"
     "e 永遠垂直於重力，所以水平時 z 軸零偏不會被修正。K_i = 0 時常數零偏 b 造成穩態傾角誤差 b / K_p。"),
    ("mah.acc_gate", r"g_{\mathrm{gate}}", "g", 0.15, 0.01, 2.0, True, "Mahony", "加速度門檻",
     "||a| − 1 g| 超過此值時不做加速度修正。"),
]

NAMES = [p[0] for p in P]
EKF_ORDER = [n for n in NAMES if n.startswith("ekf.")]
MAH_ORDER = [n for n in NAMES if n.startswith("mah.")]
DEFAULTS = {p[0]: p[3] for p in P}


def meta():
    keys = ["name", "symbol", "unit", "default", "min", "max", "log", "group", "label", "desc"]
    return [dict(zip(keys, p)) for p in P]


def ekf_array(values: dict):
    return [float(values[n]) for n in EKF_ORDER]


def mah_array(values: dict):
    return [float(values[n]) for n in MAH_ORDER]


# 模擬器的感測器模型（真實雜訊）。與濾波器假設的雜訊分開，才能看出「模型不符」的效果。
SENSOR = [
    # name, unit, default, min, max, log, label, description
    ("gyro_arw", "dps/√Hz", 0.015, 0.0, 1.0, False, "陀螺儀白雜訊密度",
     "每筆標準差 = 密度·√f_s。ICM-20948 規格 0.015。"),
    ("gyro_bias_walk", "dps/s/√Hz", 0.0005, 0.0, 0.05, False, "零偏隨機遊走", ""),
    ("gyro_bias_x", "dps", 0.4, -5.0, 5.0, False, "初始零偏 x", ""),
    ("gyro_bias_y", "dps", -0.6, -5.0, 5.0, False, "初始零偏 y", ""),
    ("gyro_bias_z", "dps", 0.8, -5.0, 5.0, False, "初始零偏 z", ""),
    ("accel_noise", "mg", 2.0, 0.0, 100.0, False, "加速度每筆雜訊",
     "每軸每筆標準差。230 µg/√Hz 經 51 Hz DLPF 約 2 mg。"),
    ("vibration", "g", 0.0, 0.0, 1.0, False, "振動加速度（RMS）",
     "30–80 Hz 的隨機振動疊加在三軸加速度上，模擬馬達與路面。"),
]
SENSOR_DEFAULTS = {s[0]: s[2] for s in SENSOR}


def sensor_meta():
    keys = ["name", "unit", "default", "min", "max", "log", "label", "desc"]
    return [dict(zip(keys, s)) for s in SENSOR]
