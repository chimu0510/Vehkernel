// 離線模擬：同一段有真值的模擬資料，A、B 兩組參數一次跑完比較。
import { api, state } from './app.js';
import { COLORS, chartCard, fmt, makeChart, section } from './charts.js';

let charts = null;

function build() {
  const host = document.getElementById('scen-out');
  host.innerHTML = '';
  const sumGrid = section(host, '統計', '同一份 C 代碼在模擬資料上執行，包含非線性、自適應 σ、拒絕門檻與靜止偵測');
  const metrics = chartCard(sumGrid, '誤差統計（t ≥ 2 s，x / y / z 軸）',
    '|誤差| &lt; kσ 的比例逐軸計算（σ 取 P 的對角線）；「期望」是誤差為高斯且 σ 正確時的值。'
    + '相鄰時間點高度相關，比例會在期望附近大幅擺動，請看趨勢而不是小數點。', { wide: true });
  metrics.classList.add('metrics');
  let grid = section(host, '姿態誤差', 'EKF 的 ±2σ 是濾波器自認的不確定度；Mahony 沒有 σ');
  const err = (axis, label) => {
    const body = chartCard(grid, `${label}：δθ${axis}（度）`,
      axis === 'z' ? 'yaw 不可觀測：σ 只增不減，誤差在靜止時（ZUPT 把零偏拉回）才停止成長。'
        : '誤差應約 95% 的時間落在 ±2σ 內。超出很多 → σ 太樂觀（濾波器過度自信）；誤差遠小於 σ → 太保守。');
    return makeChart(body, {
      xLabel: 's', yLabel: 'deg', height: 210,
      series: [
        { label: 'EKF A', color: COLORS.a, width: 1.8 }, { label: '+2σ A', color: COLORS.a, dash: [3, 3], width: 1 },
        { label: '−2σ A', color: COLORS.a, dash: [3, 3], width: 1 },
        { label: 'EKF B', color: COLORS.b, width: 1.5 }, { label: '+2σ B', color: COLORS.b, dash: [3, 3], width: 1 },
        { label: 'Mahony A', color: COLORS.m }, { label: 'Mahony B', color: COLORS.bm },
      ],
    });
  };
  charts = {
    metrics,
    ex: err('x', 'Roll'), ey: err('y', 'Pitch'), ez: err('z', 'Yaw'),
  };
  grid = section(host, '情境與濾波器內部', '');
  const truth = chartCard(grid, '真值姿態（度）與 |a|（g）', '情境本身：哪裡在轉、哪裡有線性加速度（|a| ≠ 1 g）。');
  charts.truth = makeChart(truth, {
    xLabel: 's', yLabel: 'deg', y2: '|a| g', height: 210,
    series: [{ label: 'roll', color: COLORS.x }, { label: 'pitch', color: COLORS.y }, { label: 'yaw', color: COLORS.z },
      { label: '|a|', color: '#ddd', scale: 'y2', width: 1 }],
  });
  const bias = chartCard(grid, '零偏估計 vs 真值（dps）', 'EKF 的 b<sub>z</sub> 只在靜止（ZUPT）時移動；Mahony 的零偏只靠 K<sub>i</sub>（水平時 z 軸修不到）。');
  charts.bias = makeChart(bias, {
    xLabel: 's', yLabel: 'dps', height: 210,
    series: [
      { label: 'bx 真', color: COLORS.x, width: 3 }, { label: 'by 真', color: COLORS.y, width: 3 }, { label: 'bz 真', color: COLORS.z, width: 3 },
      { label: 'bx A', color: COLORS.a }, { label: 'by A', color: COLORS.a, dash: [4, 2] }, { label: 'bz A', color: COLORS.a, dash: [1, 2] },
      { label: 'bx B', color: COLORS.b }, { label: 'by B', color: COLORS.b, dash: [4, 2] }, { label: 'bz B', color: COLORS.b, dash: [1, 2] },
      { label: 'bz Mahony A', color: COLORS.m },
    ],
  });
  const nis = chartCard(grid, 'NIS（加速度修正，對數）與靜止旗標',
    '模型正確時 NIS ~ χ²₃（平均 3）。加減速時 NIS 暴增 = 加速度計違反了「只量到重力」的假設；自適應 σ 會把它壓回來到什麼程度，看這裡。');
  charts.nis = makeChart(nis, {
    xLabel: 's', yLabel: 'NIS', y2: '靜止', y2Range: [-0.1, 1.1], height: 210, logY: true,
    series: [{ label: 'NIS A', color: COLORS.a }, { label: 'NIS B', color: COLORS.b, width: 1 },
      { label: '靜止 A', color: COLORS.t, scale: 'y2', width: 1 }],
  });
}

function metricsTable(r) {
  const m = { A: r.runs.A.metrics, B: r.runs.B.metrics };
  const v3 = (a, d) => a.map((x) => (x == null ? '—' : x.toFixed(d))).join(' / ');
  const pct = (a) => a.map((x) => `${(x * 100).toFixed(0)}%`).join(' / ');
  return `<table class="sum">
    <tr><th></th><th>A</th><th>B</th><th>期望</th></tr>
    <tr><td>EKF RMS 誤差（°）</td><td class="ca">${v3(m.A.ekf_rms, 3)}</td><td class="cb">${v3(m.B.ekf_rms, 3)}</td><td></td></tr>
    <tr><td>EKF 最大誤差（°）</td><td class="ca">${v3(m.A.ekf_max, 3)}</td><td class="cb">${v3(m.B.ekf_max, 3)}</td><td></td></tr>
    <tr><td>Mahony RMS 誤差（°）</td><td class="cm">${v3(m.A.mah_rms, 3)}</td><td class="cbm">${v3(m.B.mah_rms, 3)}</td><td></td></tr>
    <tr><td>Mahony 最大誤差（°）</td><td class="cm">${v3(m.A.mah_max, 3)}</td><td class="cbm">${v3(m.B.mah_max, 3)}</td><td></td></tr>
    <tr><td>EKF |誤差| &lt; 1σ 的比例</td><td class="ca">${pct(m.A.within_1s)}</td><td class="cb">${pct(m.B.within_1s)}</td><td>68%</td></tr>
    <tr><td>EKF |誤差| &lt; 2σ 的比例</td><td class="ca">${pct(m.A.within_2s)}</td><td class="cb">${pct(m.B.within_2s)}</td><td>95%</td></tr>
    <tr><td>NIS 平均（有加速度修正時）</td><td class="ca">${fmt(m.A.nis_mean)}</td><td class="cb">${fmt(m.B.nis_mean)}</td><td>3</td></tr>
    <tr><td>加速度採用率／靜止比例</td><td class="ca">${(m.A.accel_used * 100).toFixed(0)}% / ${(m.A.still_frac * 100).toFixed(0)}%</td>
      <td class="cb">${(m.B.accel_used * 100).toFixed(0)}% / ${(m.B.still_frac * 100).toFixed(0)}%</td><td></td></tr>
    <tr><td>結束時零偏誤差（dps）</td><td class="ca">${v3(m.A.bias_err_end, 4)}</td><td class="cb">${v3(m.B.bias_err_end, 4)}</td><td>0</td></tr>
    </table>`;
}

async function run() {
  const btn = document.getElementById('scen-run');
  btn.disabled = true;
  btn.textContent = '執行中…';
  try {
    const r = await api('/api/scenario', {
      scenario: document.getElementById('scen').value,
      seed: +document.getElementById('seed').value,
      mahony_cal: document.getElementById('mcal').checked,
    });
    if (!charts) build();
    charts.metrics.innerHTML = metricsTable(r);
    const A = r.runs.A, B = r.runs.B;
    const col = (m, i) => m.map((x) => x[i]);
    ['x', 'y', 'z'].forEach((ax, i) => {
      const sa = col(A.ekf_sigma, i), sb = col(B.ekf_sigma, i);
      charts['e' + ax].set([r.t, col(A.ekf_err, i), sa.map((s) => 2 * s), sa.map((s) => -2 * s),
        col(B.ekf_err, i), sb.map((s) => 2 * s), col(A.mah_err, i), col(B.mah_err, i)]);
    });
    charts.truth.set([r.t, col(r.truth_deg, 0), col(r.truth_deg, 1), col(r.truth_deg, 2), r.a_norm]);
    charts.bias.set([r.t, col(r.b_true_dps, 0), col(r.b_true_dps, 1), col(r.b_true_dps, 2),
      col(A.ekf_b, 0), col(A.ekf_b, 1), col(A.ekf_b, 2), col(B.ekf_b, 0), col(B.ekf_b, 1), col(B.ekf_b, 2), col(A.mah_b, 2)]);
    charts.nis.set([r.t, A.nis.map((x) => (x > 0 ? x : null)), B.nis.map((x) => (x > 0 ? x : null)), A.still]);
  } catch (e) {
    document.getElementById('scen-out').innerHTML = `<div class="card inf">執行失敗：${e.message}</div>`;
    charts = null;
  } finally {
    btn.disabled = false;
    btn.textContent = '執行 A、B';
  }
}

export function initScenario() {
  const sel = document.getElementById('scen');
  for (const [k, v] of Object.entries(state.scenarios)) sel.add(new Option(`${v[0]}（${v[1]} s）`, k));
  sel.value = 'accel';
  const desc = () => { document.getElementById('scen-desc').textContent = state.scenarios[sel.value][2]; };
  sel.onchange = desc;
  desc();
  document.getElementById('scen-run').onclick = run;
  document.getElementById('scen-live').onclick = async () => {
    await api('/api/source', { type: 'sim', scenario: sel.value, seed: +document.getElementById('seed').value });
    document.querySelector('nav.tabs > button[data-tab="live"]').click();
  };
}
