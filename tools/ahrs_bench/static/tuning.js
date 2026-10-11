// 參數分析：A（目前）/ B（候選）參數表、真實雜訊模型、線性化分析圖。
/* global katex */
import { api, refreshState, state } from './app.js';
import { COLORS, chartCard, fmt, makeChart, section } from './charts.js';

let timer = null, inited = false;
const C = {};

function sliderPos(m, v) {
  if (m.log) {
    const lo = Math.log10(Math.max(m.min, 1e-6)), hi = Math.log10(m.max);
    return (Math.log10(Math.max(v, 1e-9)) - lo) / (hi - lo);
  }
  return (v - m.min) / (m.max - m.min);
}
function sliderVal(m, p) {
  if (m.log) {
    const lo = Math.log10(Math.max(m.min, 1e-6)), hi = Math.log10(m.max);
    return 10 ** (lo + p * (hi - lo));
  }
  return m.min + p * (m.max - m.min);
}
const round = (v) => +(+v).toPrecision(4);

function renderParams() {
  const rows = [];
  let grp = null;
  rows.push('<table class="pt"><tr><th>參數</th><th style="text-align:right" class="ca">A</th><th class="cb">B（可調）</th></tr>');
  for (const m of state.meta) {
    if (m.group !== grp) {
      grp = m.group;
      rows.push(`<tr class="grp"><td colspan="3">${grp}</td></tr>`);
    }
    const sym = m.symbol ? `<span class="sym" data-tex="${m.symbol}"></span> ` : '';
    rows.push(`<tr data-n="${m.name}"><td class="name" title="${m.desc.replace(/"/g, '&quot;')}">${sym}${m.label} <span class="unit">${m.unit}</span></td>`
      + `<td class="a"></td><td class="b"><input type="number" step="any"> <input type="range" min="0" max="1" step="0.001"></td></tr>`);
    if (m.desc) rows.push(`<tr class="desc" data-d="${m.name}" style="display:none"><td colspan="3">${m.desc}</td></tr>`);
  }
  rows.push('</table><div class="hint" style="margin-top:6px">點參數名稱展開說明。A 是控制板目前使用的值（接控制板時由控制板回報）；'
    + 'B 只在主機副本上執行，可以安全地試。按「B → A 套用」才會寫入控制板（不存 Flash，重新開機恢復預設）。</div>');
  const host = document.getElementById('ptable');
  host.innerHTML = rows.join('');
  for (const el of host.querySelectorAll('[data-tex]')) katex.render(el.dataset.tex, el, { throwOnError: false });
  for (const tr of host.querySelectorAll('tr[data-n]')) {
    const m = state.meta.find((x) => x.name === tr.dataset.n);
    const [num, rng] = tr.querySelectorAll('input');
    num.onchange = () => setB(m.name, +num.value);
    rng.oninput = () => { const v = round(sliderVal(m, +rng.value)); num.value = v; setB(m.name, v, true); };
    rng.onchange = () => setB(m.name, +num.value);
    tr.querySelector('td.name').onclick = () => {
      const d = host.querySelector(`tr[data-d="${m.name}"]`);
      if (d) d.style.display = d.style.display === 'none' ? '' : 'none';
    };
  }
  fillParams();
}

function fillParams() {
  const host = document.getElementById('ptable');
  for (const tr of host.querySelectorAll('tr[data-n]')) {
    const n = tr.dataset.n, m = state.meta.find((x) => x.name === n);
    const a = state.values.A[n], b = state.values.B[n];
    tr.querySelector('td.a').textContent = fmt(a);
    const [num, rng] = tr.querySelectorAll('input');
    if (document.activeElement !== num) num.value = round(b);
    if (document.activeElement !== rng) rng.value = sliderPos(m, b);
    tr.classList.toggle('diff', Math.abs(a - b) > 1e-9 * Math.max(1, Math.abs(a)));
  }
}

function renderSensor() {
  const rows = ['<table class="pt"><tr><th>真實感測器</th><th>值</th></tr>'];
  for (const m of state.sensorMeta) {
    rows.push(`<tr data-s="${m.name}"><td class="name" title="${m.desc}">${m.label} <span class="unit">${m.unit}</span></td>`
      + `<td class="b"><input type="number" step="any" value="${state.sensor[m.name]}"></td></tr>`);
  }
  rows.push('</table><div class="hint" style="margin-top:6px">濾波器參數是「濾波器以為的雜訊」，這裡是「實際的雜訊」：'
    + '模擬器用它產生資料，分析用它算「實際 σ」。接控制板時應填實測值（例如靜置錄製後算 Allan 方差）。</div>');
  const host = document.getElementById('stable');
  host.innerHTML = rows.join('');
  for (const tr of host.querySelectorAll('tr[data-s]')) {
    tr.querySelector('input').onchange = async (ev) => {
      await api('/api/sensor', { [tr.dataset.s]: +ev.target.value });
      await refreshState();
      schedule();
    };
  }
}

let pendingB = {};
async function setB(name, v, live = false) {
  state.values.B[name] = v;
  pendingB[name] = v;
  fillParams();
  if (live) { schedule(150); return; }
  const body = pendingB;
  pendingB = {};
  await api('/api/params', { set: 'B', values: body });
  schedule();
}

export function onValuesChanged() {
  if (!inited) return;
  fillParams();
  schedule();
}

function schedule(ms = 250) {
  clearTimeout(timer);
  timer = setTimeout(runAnalysis, ms);
}

// ---- 分析 ----

const S = () => ({ ...state.sensor });

function sumTable(r) {
  const a = r.sets.A, b = r.sets.B;
  const val = (x, d = 4) => (x == null ? '<span class="inf">∞</span>' : fmt(x));
  const row = (label, fa, fb, cls = ['ca', 'cb']) => `<tr><td>${label}</td><td class="${cls[0]}">${fa}</td><td class="${cls[1]}">${fb}</td></tr>`;
  const kf = (k, f) => row(k, val(f(a)), val(f(b)));
  const mh = (k, f) => row(k, val(f(a)), val(f(b)), ['cm', 'cbm']);
  const opt = (x) => (x == null ? '—（K<sub>i</sub> = 0，一階）' : fmt(x));
  return `<table class="sum"><tr><th></th><th>A</th><th>B</th></tr>
    <tr class="sec"><td colspan="3">EKF，運動中（只有加速度量測）</td></tr>
    ${kf('σθ 濾波器自認（°）', (s) => s.kf_moving.sigma_theta_filter_deg)}
    ${kf('σθ 實際（°）', (s) => s.kf_moving.sigma_theta_true_deg)}
    ${kf('σb 濾波器自認（dps）', (s) => s.kf_moving.sigma_b_filter_dps)}
    ${kf('σb 實際（dps）', (s) => s.kf_moving.sigma_b_true_dps)}
    ${kf('穩態 K<sub>θ</sub>（每步）', (s) => s.kf_moving.K[0][0])}
    ${kf('等效 K<sub>p</sub> = K<sub>θ</sub>/Δt（1/s）', (s) => s.kf_moving.kp_eq)}
    ${kf('等效 K<sub>i</sub> = −K<sub>b</sub>/Δt（1/s²）', (s) => s.kf_moving.ki_eq)}
    ${kf('ω<sub>n</sub> = √K<sub>i</sub>（rad/s）', (s) => s.kf_moving.wn)}
    ${kf('ζ = K<sub>p</sub>/(2ω<sub>n</sub>)', (s) => s.kf_moving.zeta)}
    ${kf('加速度路徑 −3 dB（Hz）', (s) => s.bode.kf.bandwidth_hz)}
    <tr class="sec"><td colspan="3">EKF，靜止（加速度 + ZUPT）</td></tr>
    ${kf('σθ 濾波器自認（°）', (s) => s.kf_still.sigma_theta_filter_deg)}
    ${kf('σθ 實際（°）', (s) => s.kf_still.sigma_theta_true_deg)}
    ${kf('σb 濾波器自認（dps）', (s) => s.kf_still.sigma_b_filter_dps)}
    ${kf('σb 實際（dps）', (s) => s.kf_still.sigma_b_true_dps)}
    ${kf('z 軸零偏 σ 自認（dps）', (s) => s.yaw.sb_filter_dps)}
    ${kf('z 軸零偏 σ 實際（dps）', (s) => s.yaw.sb_true_dps)}
    <tr class="sec"><td colspan="3">Mahony（K<sub>p</sub>、K<sub>i</sub> 為參數值）</td></tr>
    ${mh('σθ 實際：白雜訊部分（°）', (s) => s.mahony.sigma_theta_white_deg)}
    ${mh('σθ 實際：含零偏遊走（°）', (s) => s.mahony.sigma_theta_true_deg)}
    ${mh('常數零偏造成的傾角偏差（°）', (s) => s.mahony.bias_offset_deg)}
    ${row('ω<sub>n</sub>（rad/s）', opt(a.mahony.wn), opt(b.mahony.wn), ['cm', 'cbm'])}
    ${row('ζ', opt(a.mahony.zeta), opt(b.mahony.zeta), ['cm', 'cbm'])}
    ${mh('加速度路徑 −3 dB（Hz）', (s) => s.bode.mahony.bandwidth_hz)}
    </table>`;
}

function contribChart(body, r) {
  const cats = ['EKF A 運動', 'EKF B 運動', 'EKF A 靜止', 'EKF B 靜止', 'Mahony A', 'Mahony B'];
  const src = [r.sets.A.kf_moving, r.sets.B.kf_moving, r.sets.A.kf_still, r.sets.B.kf_still, r.sets.A.mahony, r.sets.B.mahony];
  const sq = (x) => (x == null ? null : Math.sqrt(x));
  const html = ['<table class="sum"><tr><th></th><th>陀螺儀白雜訊</th><th>零偏遊走</th><th>加速度雜訊</th><th>總和</th></tr>'];
  src.forEach((s, i) => {
    const c = s.contrib_theta_deg2;
    const tot = s.sigma_theta_true_deg;
    const f = (x) => (x == null ? '<span class="inf">∞</span>' : fmt(sq(x)));
    html.push(`<tr><td>${cats[i]}</td><td>${f(c.gyro)}</td><td>${f(c.bias_walk)}</td><td>${f(c.accel)}</td><td>${tot == null ? '<span class="inf">∞</span>' : fmt(tot)}</td></tr>`);
  });
  html.push('</table>');
  body.innerHTML = html.join('');
}

function buildCharts() {
  const root = document.getElementById('analysis');
  root.innerHTML = '';
  let host = section(root, '穩態', '單軸傾角線性化：小角度、ω ≈ 0、|a| ≈ 1 g，EKF 取穩態增益');
  C.sum = chartCard(host, '穩態摘要',
    '「濾波器自認」由 Riccati 方程得到，假設濾波器的雜訊參數就是真的；「實際」用左側「真實雜訊模型」解 Lyapunov 方程，'
    + '並包含 ZUPT 與預測共用同一筆陀螺儀雜訊的相關性。<br>兩者差很多就是模型不符：自認 ≫ 實際 → 保守（修正過慢）；自認 ≪ 實際 → 過度自信。'
    + '<br>Mahony 在 K<sub>i</sub> = 0 時不估零偏：常數零偏留下 b/K<sub>p</sub> 的偏差（取模擬器 x、y 零偏的較大者），零偏隨機遊走造成的誤差無界（∞）。', { wide: true });
  C.contrib = chartCard(host, 'σθ 的來源分解（實際雜訊，度）',
    '每個雜訊源單獨作用時的穩態 σθ，總變異數是三者的平方和。陀螺儀白雜訊走「高通」路徑、加速度雜訊走「低通」路徑：'
    + '頻寬越高，加速度雜訊占比越高、陀螺儀占比越低；卡爾曼的最佳頻寬就是兩者平衡的位置。', { wide: true });
  host = section(root, '頻率域', '穩態 EKF 與 Mahony 都可以寫成二階互補濾波器');

  const bode = chartCard(host, '互補濾波的頻率響應 |T|',
    'θ̂ = T<sub>acc</sub>·θ<sub>acc</sub> + T<sub>gyro</sub>·θ<sub>gyro</sub>，且 T<sub>acc</sub> + T<sub>gyro</sub> = 1：'
    + '低頻相信加速度（沒有漂移），高頻相信陀螺儀積分（沒有雜訊與線性加速度的干擾）。交越頻率以下的線性加速度會完全變成傾角誤差。'
    + '穩態 EKF 就是一個二階互補濾波器，等效 K<sub>p</sub>、K<sub>i</sub> 見摘要。'
    + '（Mahony 用修正前的姿態算 e，與互補形式差一步延遲，高頻差約 K<sub>p</sub>Δt。）');
  C.bode = makeChart(bode, {
    logX: true, xLabel: 'Hz', yLabel: 'dB', height: 230, yRange: [-60, 10],
    series: [
      { label: 'T_acc EKF A', color: COLORS.a, width: 2 }, { label: 'T_gyro EKF A', color: COLORS.a, dash: [6, 3] },
      { label: 'T_acc EKF B', color: COLORS.b, width: 2 }, { label: 'T_gyro EKF B', color: COLORS.b, dash: [6, 3] },
      { label: 'T_acc Mahony A', color: COLORS.m, width: 2 }, { label: 'T_gyro Mahony A', color: COLORS.m, dash: [6, 3] },
      { label: 'T_acc Mahony B', color: COLORS.bm, width: 2 }, { label: 'T_gyro Mahony B', color: COLORS.bm, dash: [6, 3] },
    ],
  });

  const psd = chartCard(host, '傾角誤差的頻譜密度（實際雜訊）',
    '|H<sub>i</sub>(f)|·√(雜訊 PSD)：每個雜訊源經過濾波器後在各頻率留下的誤差。曲線下的面積（平方後）就是上表的變異數。'
    + '加速度雜訊集中在低頻（被低通留下），陀螺儀白雜訊被積分後在交越頻率附近最大，零偏遊走只在極低頻。');
  C.psdSel = document.createElement('select');
  for (const [v, l] of [['A.kf', 'EKF A'], ['B.kf', 'EKF B'], ['A.mahony', 'Mahony A'], ['B.mahony', 'Mahony B']]) C.psdSel.add(new Option(l, v));
  C.psdSel.onchange = () => C.last && drawPsd(C.last);
  psd.parentElement.querySelector('.card-h').appendChild(C.psdSel);
  C.psd = makeChart(psd, {
    logX: true, logY: true, xLabel: 'Hz', yLabel: 'deg/√Hz', height: 220,
    series: [{ label: '陀螺儀白雜訊', color: COLORS.x }, { label: '零偏遊走', color: COLORS.z }, { label: '加速度雜訊', color: COLORS.y }],
  });

  host = section(root, '時域', '固定增益的線性響應；非線性與自適應的效果看「離線模擬」');
  const resp = chartCard(host, '0.2 g 水平加速度持續 2 s（t = 1–3 s）的傾角誤差',
    '加速度計把水平加速度當成 atan(0.2) = 11.3° 的傾斜。誤差上升的速度由加速度路徑的頻寬決定：頻寬越低越不受影響，'
    + '但真正傾斜時也收斂得越慢。這是線性模型（固定增益），沒有含 EKF 的 |a| 自適應：0.2 g 水平加速度讓 |a| 變 0.02 g，'
    + '預設參數下 σ 約 ×6，實際 EKF 的誤差會比這裡小很多（見「離線模擬」的「加減速」）；Mahony 的門檻 0.15 g 對它完全不起作用。');
  C.pulse = makeChart(resp, {
    xLabel: 's', yLabel: 'deg', height: 200,
    series: [{ label: 'EKF A', color: COLORS.a, width: 2 }, { label: 'EKF B', color: COLORS.b, width: 2 },
      { label: 'Mahony A', color: COLORS.m }, { label: 'Mahony B', color: COLORS.bm }],
  });
  const bstep = chartCard(host, '零偏在 t = 1 s 突增 0.5 dps 的傾角誤差',
    '零偏改變（例如溫度）時，誤差先以 0.5°/s 成長，直到零偏估計跟上。K<sub>i</sub> = 0 的 Mahony 留下 b/K<sub>p</sub> 的穩態誤差。'
    + 'EKF 跟上的速度由穩態的零偏增益決定，而增益取決於 σ<sub>b</sub>（零偏遊走）：σ<sub>b</sub> 越小，濾波器越相信「零偏不會變」，'
    + '穩態 σ<sub>b</sub> 越小、增益越小，突然的零偏變化就要越久才修正——即使靜止時 ZUPT 每一筆都直接量到零偏也一樣。');
  C.bstep = makeChart(bstep, {
    xLabel: 's', yLabel: 'deg', height: 200,
    series: [{ label: 'EKF A 運動', color: COLORS.a, width: 2 }, { label: 'EKF B 運動', color: COLORS.b, width: 2 },
      { label: 'EKF A 靜止(ZUPT)', color: COLORS.a, dash: [2, 3] }, { label: 'EKF B 靜止(ZUPT)', color: COLORS.b, dash: [2, 3] },
      { label: 'Mahony A', color: COLORS.m }, { label: 'Mahony B', color: COLORS.bm }],
  });

  host = section(root, '取捨與限制', '線性加速度、|a| 自適應與 yaw 的根本限制');
  const trade = chartCard(host, '增益的取捨：雜訊 σθ（實線，左軸）與加速度誤差（虛線，右軸）',
    '橫軸是加速度修正的增益（約等於頻寬 ×2π）。增益越高，線性加速度造成的傾角誤差越大（虛線，0.2 g × 2 s 的最大誤差）；'
    + '雜訊 σθ（實線）則是 U 形：太低時陀螺儀雜訊與零偏遊走累積，太高時加速度雜訊直接進來。'
    + '「最佳」點是把濾波器的雜訊參數設成真實雜訊的卡爾曼濾波——純雜訊意義下的下限，但真實模型裡沒有線性加速度，'
    + '所以它對加減速一點也不保守；實務上把 σ<sub>a</sub> 設得比感測器雜訊大，就是用一點雜訊換對加速度的耐受度。'
    + 'EKF 曲線是掃 σ<sub>a</sub>（其餘用 A 的參數）；K<sub>i</sub> = 0 的 Mahony 不計零偏遊走（否則 ∞）。', { wide: true });
  C.trade = makeChart(trade, {
    logX: true, logY: true, xLabel: '等效 Kp（1/s）', yLabel: 'σθ（°）', y2: '最大誤差（°）', height: 260,
    series: [
      { label: 'EKF σθ', color: COLORS.a, width: 1.8, spanGaps: true },
      { label: 'Mahony ζ=1 σθ', color: COLORS.m, width: 1.5, spanGaps: true },
      { label: 'Mahony Ki=0 σθ', color: COLORS.bm, width: 1.5, spanGaps: true },
      { label: 'EKF 誤差', color: COLORS.a, dash: [5, 3], scale: 'y2', spanGaps: true },
      { label: 'Mahony ζ=1 誤差', color: COLORS.m, dash: [5, 3], scale: 'y2', spanGaps: true },
      { label: 'Mahony Ki=0 誤差', color: COLORS.bm, dash: [5, 3], scale: 'y2', spanGaps: true },
      { label: 'EKF A', color: COLORS.a, points: true }, { label: 'EKF B', color: COLORS.b, points: true },
      { label: 'Mahony A', color: COLORS.m, points: true }, { label: 'Mahony B', color: COLORS.bm, points: true },
      { label: '最佳（真實雜訊 KF）', color: COLORS.t, points: true },
    ],
  });

  const adapt = chartCard(host, 'EKF 的 |a| 自適應：σ（左軸）與等效 K<sub>p</sub>（右軸）',
    '垂直加速度（顛簸）直接改變 |a|，是一階；水平加速度 a<sub>h</sub> 只讓 |a| 變 √(1+a<sub>h</sub>²) − 1 ≈ a<sub>h</sub>²/2，是二階：'
    + '以預設 κ = 5，0.1 g 只讓 |a| 變 0.005 g（σ 約 ×2），0.3 g 變 0.044 g（σ 約 ×12）。'
    + '所以小的加減速幾乎不被降權，卻照樣造成 atan(a<sub>h</sub>) 的視傾角；大的加減速才會被明顯壓低權重。'
    + '要可靠處理加減速，需要由輪速或指令推算線性加速度並從量測中扣掉。');
  C.adapt = makeChart(adapt, {
    xLabel: '加速度（g）', yLabel: 'σ', y2: 'Kp 等效（1/s）', height: 220,
    series: [
      { label: 'σ 水平 A', color: COLORS.a }, { label: 'σ 垂直 A', color: COLORS.a, dash: [6, 3] },
      { label: 'σ 水平 B', color: COLORS.b }, { label: 'σ 垂直 B', color: COLORS.b, dash: [6, 3] },
      { label: 'Kp 水平 A', color: COLORS.t, scale: 'y2' }, { label: 'Kp 垂直 A', color: COLORS.t, dash: [6, 3], scale: 'y2' },
    ],
  });

  const yaw = chartCard(host, '開始運動後 yaw 誤差的成長（1σ）',
    '靜止時 ZUPT 把 z 軸零偏收斂到 σ<sub>bz</sub>，開始運動後沒有任何量測：'
    + 'σ<sub>ψ</sub>²(t) = σ<sub>bz</sub>²t² + σ<sub>g</sub>²t + σ<sub>b</sub>²t³/3（零偏誤差、角度隨機遊走、零偏遊走）。'
    + '「自認」用濾波器參數，「實際」用真實雜訊；未校正的 Mahony 直接以真實 b<sub>z</sub> 漂移。');
  C.yaw = makeChart(yaw, {
    xLabel: 's', yLabel: 'deg', height: 220, logY: true,
    series: [{ label: 'EKF A 自認', color: COLORS.a, dash: [6, 3] }, { label: 'EKF A 實際', color: COLORS.a, width: 2 },
      { label: 'EKF B 自認', color: COLORS.b, dash: [6, 3] }, { label: 'EKF B 實際', color: COLORS.b, width: 2 },
      { label: 'Mahony 未校正 b_z', color: COLORS.m }],
  });
}

function drawPsd(r) {
  const [tag, f] = C.psdSel.value.split('.');
  const p = r.sets[tag].psd[f];
  const pos = (a) => a.map((x) => (x > 0 ? x : null));
  C.psd.set([r.freq, pos(p.gyro), pos(p.bias_walk), pos(p.accel)]);
}

async function runAnalysis() {
  let r;
  try {
    r = await api('/api/analysis', { B: state.values.B, sensor: S() });
  } catch (e) {
    C.sum.innerHTML = `<div class="inf">分析失敗：${e.message}</div>`;
    return;
  }
  C.last = r;
  C.sum.innerHTML = sumTable(r);
  contribChart(C.contrib, r);
  const A = r.sets.A, B = r.sets.B;
  C.bode.set([r.freq, A.bode.kf.acc_db, A.bode.kf.gyro_db, B.bode.kf.acc_db, B.bode.kf.gyro_db,
    A.bode.mahony.acc_db, A.bode.mahony.gyro_db, B.bode.mahony.acc_db, B.bode.mahony.gyro_db]);
  drawPsd(r);
  C.pulse.set([r.t, A.resp.kf.pulse_deg, B.resp.kf.pulse_deg, A.resp.mahony.pulse_deg, B.resp.mahony.pulse_deg]);
  C.bstep.set([r.t, A.resp.kf.bias_step_deg, B.resp.kf.bias_step_deg, A.resp.kf_still.bias_step_deg,
    B.resp.kf_still.bias_step_deg, A.resp.mahony.bias_step_deg, B.resp.mahony.bias_step_deg]);

  // 取捨圖：各條曲線的 x（Kp）不同，合併成一個 x 陣列，其他位置填 null
  const fam = r.tradeoff.families, mk = r.tradeoff.marks;
  const series = [ // [點列表, 取第幾個值（1 = σθ、2 = 最大誤差）]
    [fam.kf, 1], [fam.mahony_zeta1, 1], [fam.mahony_ki0, 1], [fam.kf, 2], [fam.mahony_zeta1, 2], [fam.mahony_ki0, 2],
    [[mk.kf_A], 1], [[mk.kf_B], 1], [[mk.mahony_A], 1], [[mk.mahony_B], 1], [[mk.kf_opt], 1]];
  const xs = [...new Set(series.flatMap(([pts]) => pts.map((p) => p[0])).filter((x) => x > 0))].sort((p, q) => p - q);
  const data = [xs];
  for (const [pts, j] of series) {
    const m = new Map(pts.map((p) => [p[0], p[j]]));
    data.push(xs.map((x) => (m.has(x) && m.get(x) > 0 ? m.get(x) : null)));
  }
  C.trade.set(data);

  const ad = A.adaptive, bd = B.adaptive;
  C.adapt.set([ad.a_g, ad.horizontal.sigma, ad.vertical.sigma, bd.horizontal.sigma, bd.vertical.sigma, ad.horizontal.kp_eq, ad.vertical.kp_eq]);
  C.yaw.set([A.yaw.t.map((t) => t || null), A.yaw.ekf_filter_deg, A.yaw.ekf_true_deg, B.yaw.ekf_filter_deg, B.yaw.ekf_true_deg, A.yaw.mahony_uncal_deg]
    .map((arr, i) => (i === 0 ? A.yaw.t : arr.map((x) => (x > 0 ? x : null)))));
}

export function initTuning() {
  renderParams();
  renderSensor();
  buildCharts();
  inited = true;
  document.getElementById('b-to-a').onclick = async () => {
    await api('/api/params', { set: 'A', values: state.values.B });
    setTimeout(refreshState, 300);
  };
  document.getElementById('a-to-b').onclick = async () => {
    await api('/api/params', { set: 'B', values: state.values.A });
    await refreshState();
    fillParams();
    schedule();
  };
  document.getElementById('def-to-b').onclick = async () => {
    const d = Object.fromEntries(state.meta.map((m) => [m.name, m.default]));
    await api('/api/params', { set: 'B', values: d });
    await refreshState();
    fillParams();
    schedule();
  };
  schedule(10);
}
