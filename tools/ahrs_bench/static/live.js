// 即時監看：3D 姿態、狀態面板、分區的時間序列圖。
import { state } from './app.js';
import { COLORS, chartCard, fmt, makeChart, section } from './charts.js';
import { board, makeScene, setQuat, axes, THREE } from './scene3d.js';

let view, boards = {};
const charts = [];

// 圖的分區：[標題, 一句話說明]
const SECTIONS = {
  att: ['姿態', '估測結果與（模擬時）與真值的誤差'],
  inner: ['濾波器內部', '濾波器自認的不確定度、零偏估計、量測是否與模型一致'],
  raw: ['感測器原始讀值', ''],
  link: ['連線', '主機副本是否正確重現控制板'],
};

// 每張圖：分區、標題、y 軸、序列 [buffer key, 標籤, 顏色, 樣式]、說明（預設收起）。
function chartDefs() {
  const C = COLORS;
  const solid = { width: 1.6 }, dashM = { dash: [6, 3] }, dotB = { dash: [2, 3], width: 1.6 }, tru = { width: 3 };
  const env = { dash: [3, 3], width: 1 };
  return [
    {
      sec: 'att', title: 'Roll / Pitch', y: '度', series: [
        ['Ar', 'roll', C.x, solid], ['Ap', 'pitch', C.y, solid],
        ['Amr', 'roll Mahony', C.x, dashM], ['Amp', 'pitch Mahony', C.y, dashM],
        ['Br', 'roll B', C.b, dotB], ['Bp', 'pitch B', C.bm, dotB],
        ['Tr', 'roll 真值', C.t, tru], ['Tp', 'pitch 真值', C.t, tru],
      ],
      note: 'ZYX 尤拉角（REP-103）。實線 EKF（A）、虛線 Mahony、點線 B。roll/pitch 有重力當絕對參考，誤差不會無界成長。',
    },
    {
      sec: 'att', title: 'Yaw', y: '度', series: [
        ['Ay', 'yaw', C.a, solid], ['Amy', 'Mahony', C.m, dashM], ['By', 'B', C.b, dotB], ['Ty', '真值', C.t, tru],
      ],
      note: '沒有磁力計，yaw 只靠陀螺儀積分；誤差 = ∫(零偏誤差 + 雜訊) dt，會無界成長。'
        + 'EKF 靠靜止時的 ZUPT 估 z 軸零偏，所以停下來時漂移會變慢。',
    },
    {
      sec: 'att', title: '傾角誤差 δθx、δθy 與 ±2σ（模擬）', y: '度', series: [
        ['Aex', 'δθx', C.x, solid], ['Aey', 'δθy', C.y, solid],
        ['_sx', '+2σx', C.x, env], ['_sxn', '−2σx', C.x, env],
        ['Amex', 'δθx Mahony', C.x, dashM], ['Amey', 'δθy Mahony', C.y, dashM],
        ['Bex', 'δθx B', C.b, dotB], ['Bey', 'δθy B', C.bm, dotB],
      ],
      note: 'δθ 的定義同 EKF：q<sub>真</sub> = q<sub>估</sub> ⊗ Exp(δθ)，本體座標。'
        + '若 σ 正確，誤差約 95% 的時間落在 ±2σ 內；常常超出代表濾波器過度自信。',
    },
    {
      sec: 'att', title: 'Yaw 誤差 δθz 與 ±2σ（模擬）', y: '度', series: [
        ['Aez', 'δθz', C.z, solid], ['_sz', '+2σz', C.z, env], ['_szn', '−2σz', C.z, env],
        ['Amez', 'Mahony', C.m, dashM], ['Bez', 'B', C.b, dotB],
      ],
      note: 'yaw 不可觀測：σz 只增不減（靜止時 ZUPT 讓它增加得比較慢）。',
    },
    {
      sec: 'inner', title: '姿態 1σ', y: '度', logY: true, series: [
        ['Asx', 'σθx', C.x, solid], ['Asy', 'σθy', C.y, solid], ['Asz', 'σθz', C.z, solid],
        ['Bsx', 'σθx B', C.x, dotB], ['Bsy', 'σθy B', C.y, dotB], ['Bsz', 'σθz B', C.z, dotB],
      ],
      note: '√diag(P) 的姿態部分：濾波器「自認」的誤差，不是實際誤差（實際誤差看模擬的真值比較，或看 NIS）。'
        + '接近水平時 σθx、σθy、σθz 約等於 roll、pitch、yaw 的 σ。',
    },
    {
      sec: 'inner', title: '陀螺儀零偏估計', y: 'dps', series: [
        ['Abx', 'bx', C.x, solid], ['Aby', 'by', C.y, solid], ['Abz', 'bz', C.z, solid],
        ['Ambx', 'bx Mahony', C.x, dashM], ['Amby', 'by Mahony', C.y, dashM], ['Ambz', 'bz Mahony', C.z, dashM],
        ['Bbx', 'bx B', C.x, dotB], ['Bby', 'by B', C.y, dotB], ['Bbz', 'bz B', C.z, dotB],
        ['Tbx', 'bx 真值', C.t, tru], ['Tby', 'by 真值', C.t, tru], ['Tbz', 'bz 真值', C.t, tru],
      ],
      note: 'EKF：bz 只在靜止（ZUPT）時更新；bx、by 也能透過重力方向修正，但要等傾角誤差累積出來才看得到，所以慢。'
        + 'Mahony：只有 K<sub>i</sub> > 0 時才估零偏，水平時 bz 修不到。',
    },
    {
      sec: 'inner', title: 'NIS（加速度修正的一致性檢定）', y: '', logY: true, series: [
        ['Anis', 'NIS', C.a, {}], ['Bnis', 'NIS B', C.b, { width: 1 }],
        ['_lo', '2.5%', C.muted, { dash: [4, 4], width: 1 }], ['_hi', '97.5%', C.muted, { dash: [4, 4], width: 1 }],
        ['_m', '平均 3', C.muted, { dash: [1, 3], width: 1 }],
      ],
      note: 'NIS = yᵀS⁻¹y。模型正確時服從自由度 3 的 χ²：95% 落在 [0.216, 9.35]，平均 3。'
        + '<br>長期偏高 → 過度自信（σ<sub>a</sub> 或 Q 太小，或有未建模的加速度）；長期偏低 → 太保守，修正被拖慢。'
        + '<br>注意：y 的徑向分量與 δθ 無關（H 的值域垂直於 v），但 S 仍以 σ²I 計入，所以這不是純傾角的檢定。',
    },
    {
      sec: 'inner', title: '採用的加速度雜訊 σ 與旗標', y: 'σ', y2: '旗標', series: [
        ['Asa', 'σ', C.a, {}], ['Bsa', 'σ B', C.b, dotB],
        ['still', '靜止', C.t, { scale: 'y2', width: 1 }], ['zupt', 'ZUPT', C.f, { scale: 'y2', width: 1, dash: [3, 2] }],
        ['accu', '加速度採用', C.m, { scale: 'y2', width: 1, dash: [1, 2] }],
      ],
      note: 'σ = σ<sub>a</sub> + κ·||a| − 1 g|：|a| 偏離 1 g 時加大。旗標（0/1）用右軸。',
    },
    {
      sec: 'raw', title: '陀螺儀', y: 'dps', series: [
        ['gx', 'ωx', C.x, {}], ['gy', 'ωy', C.y, {}], ['gz', 'ωz', C.z, {}],
      ],
    },
    {
      sec: 'raw', title: '加速度計與 |a|', y: 'g', series: [
        ['ax', 'ax', C.x, {}], ['ay', 'ay', C.y, {}], ['az', 'az', C.z, {}], ['an', '|a|', '#ddd', { width: 1.2 }],
      ],
    },
    {
      sec: 'link', title: '主機副本與韌體的差', y: '度', logY: true, series: [['dfw', 'max |Δ|', C.f, {}]],
      note: '同一份 C 代碼、同樣的輸入與重設時機，差異只來自浮點運算順序（控制板的編譯器會用 FMA），應遠小於 0.01°。'
        + '持續增大代表主機沒收到完整資料（掉樣本）或參數不同步。',
    },
  ];
}

const DERIVED = {
  _lo: () => 0.216, _hi: () => 9.348, _m: () => 3,
  _sx: (b, i) => (b.Asx[i] == null ? null : 2 * b.Asx[i]), _sxn: (b, i) => (b.Asx[i] == null ? null : -2 * b.Asx[i]),
  _sz: (b, i) => (b.Asz[i] == null ? null : 2 * b.Asz[i]), _szn: (b, i) => (b.Asz[i] == null ? null : -2 * b.Asz[i]),
};

// ---- 狀態面板 ----

function initPanel() {
  document.getElementById('tiles').innerHTML = `
    <table class="pnl">
      <tr><th>姿態（度）</th><th class="ca">EKF A　±1σ</th><th class="cm">Mahony</th><th class="cb">B</th><th class="ct">真值</th></tr>
      ${['roll', 'pitch', 'yaw'].map((n, i) => `<tr><td>${n}</td><td class="ca big" id="p-a${i}"></td><td class="cm" id="p-m${i}"></td>`
        + `<td class="cb" id="p-b${i}"></td><td class="ct" id="p-t${i}"></td></tr>`).join('')}
      <tr class="gap"><th>零偏（dps）</th><th class="ca">EKF A　±1σ</th><th class="cm">Mahony</th><th class="cb">B</th><th class="ct">真值</th></tr>
      ${['x', 'y', 'z'].map((n) => `<tr><td>b${n}</td><td class="ca" id="p-ab${n}"></td><td class="cm" id="p-mb${n}"></td>`
        + `<td class="cb" id="p-bb${n}"></td><td class="ct" id="p-tb${n}"></td></tr>`).join('')}
    </table>
    <div class="pnl-row" id="p-flags"></div>
    <div class="pnl-row"><span class="lbl">NIS 一致性</span><span id="p-nis"></span></div>
    <div class="pnl-row"><span class="lbl">加速度</span><span id="p-acc"></span></div>
    <div class="pnl-row" id="p-sys"></div>
    <details class="log"><summary>訊息</summary><pre id="log"></pre></details>`;
}

function last(b, k) {
  const a = b[k];
  if (!a) return null;
  for (let i = a.length - 1; i >= Math.max(0, a.length - 5); i--) if (a[i] != null) return a[i];
  return null;
}

const s2 = (v) => (v == null ? '—' : (+v.toPrecision(2)).toString());
const sg = (v, d = 2) => (v == null ? '—' : (v >= 0 ? '+' : '−') + Math.abs(v).toFixed(d));

function updatePanel() {
  const b = state.buf;
  if (!b.t || !b.t.length) return;
  const set = (id, html) => { const e = document.getElementById(id); if (e) e.innerHTML = html; };
  ['r', 'p', 'y'].forEach((k, i) => {
    const ax = 'xyz'[i];
    set(`p-a${i}`, `${sg(last(b, 'A' + k))} <small>±${s2(last(b, 'As' + ax))}</small>`);
    set(`p-m${i}`, sg(last(b, 'Am' + k)));
    set(`p-b${i}`, sg(last(b, 'B' + k)));
    set(`p-t${i}`, b['T' + k] ? sg(last(b, 'T' + k)) : '—');
  });
  for (const ax of 'xyz') {
    set(`p-ab${ax}`, `${sg(last(b, 'Ab' + ax), 3)} <small>±${s2(last(b, 'Asb' + ax))}</small>`);
    set(`p-mb${ax}`, sg(last(b, 'Amb' + ax), 3));
    set(`p-bb${ax}`, sg(last(b, 'Bb' + ax), 3));
    set(`p-tb${ax}`, b['Tb' + ax] ? sg(last(b, 'Tb' + ax), 3) : '—');
  }
  const dot = (on, label) => `<span class="flag ${on ? 'on' : ''}"><span class="dot"></span>${label}</span>`;
  set('p-flags', [dot(last(b, 'still'), '靜止'), dot(last(b, 'zupt'), 'ZUPT'), dot(last(b, 'accu'), 'EKF 採用加速度'),
    dot(last(b, 'maccu'), 'Mahony 採用加速度')].join(''));
  // N 筆獨立 χ²₃ 的平均約為常態：平均 3、標準差 √(6/N)
  const n = Math.min(b.t.length, 2 * state.rate);
  let s = 0, c = 0;
  for (let i = b.t.length - n; i < b.t.length; i++) if (b.Anis[i] != null) { s += b.Anis[i]; c++; }
  if (c) {
    const m = s / c, half = 1.96 * Math.sqrt(6 / c);
    const verdict = m > 3 + half ? ['bad', '偏高：濾波器過度自信（R、Q 太小或有未建模的加速度）']
      : m < 3 - half ? ['warn', '偏低：設定的雜訊比實際大，修正偏慢'] : ['good', '與模型一致'];
    set('p-nis', `近 2 s 平均 <b>${m.toFixed(2)}</b>（期望 3 ± ${half.toFixed(2)}）<span class="${verdict[0]}">${verdict[1]}</span>`);
  } else set('p-nis', '這段時間沒有加速度修正');
  set('p-acc', `|a| = <b>${fmt(last(b, 'an'))}</b> g　採用的 σ：A ${fmt(last(b, 'Asa'))}、B ${fmt(last(b, 'Bsa'))}`);
  const st = state.status;
  set('p-sys', [`<span class="lbl">系統</span>溫度 ${fmt(last(b, 'temp'))} °C`,
    st.isr_us != null ? `中斷 ${st.isr_us} µs / 5000 µs，逾時 ${st.overruns}` : ''].filter(Boolean).join('　'));
}

// ---- 3D ----

function init3d() {
  const el = document.getElementById('live3d');
  view = makeScene(el, { distance: 2.6 });
  const grid = new THREE.GridHelper(3, 12, 0x2a313b, 0x20262e);
  grid.rotation.x = Math.PI / 2;
  view.scene.add(grid);
  view.scene.add(axes(1.3, 0.5));
  view.label('東 x', '#ef5b5b').pos.set(1.4, 0, 0);
  view.label('北 y', '#5fd35f').pos.set(0, 1.4, 0);
  view.label('上 z', '#5b8def').pos.set(0, 0, 1.4);
  boards.A = board(0x2d6a4f);
  boards.M = board(0xf2a33a, { wire: true, opacity: 0.9 });
  boards.B = board(0xc678dd, { wire: true, opacity: 0.9 });
  boards.T = board(0x98c379, { wire: true, opacity: 0.6 });
  for (const g of Object.values(boards)) view.scene.add(g);
}

export function initLive() {
  initPanel();
  init3d();
  const host = document.getElementById('live-charts');
  const grids = {};
  for (const d of chartDefs()) {
    if (!grids[d.sec]) grids[d.sec] = { grid: section(host, ...SECTIONS[d.sec]), cards: [] };
    const body = chartCard(grids[d.sec].grid, d.title, d.note);
    const c = makeChart(body, {
      series: d.series.map(([, label, color, st]) => ({ label, color, ...st })),
      yLabel: d.y, logY: d.logY, y2: d.y2, y2Range: d.y2 ? [-0.1, 1.1] : undefined, height: 200,
    });
    const entry = { def: d, chart: c, card: body.parentElement, sec: grids[d.sec] };
    grids[d.sec].cards.push(entry);
    charts.push(entry);
  }
  document.getElementById('win').onchange = () => updateLive();
  const loop = () => {
    const L = state.latest;
    if (L && document.getElementById('tab-live').classList.contains('on')) {
      setQuat(boards.A, L.A.q);
      setQuat(boards.M, L.Am.q);
      setQuat(boards.B, L.Bq);
      boards.M.visible = document.getElementById('v-mah').checked;
      boards.B.visible = document.getElementById('v-b').checked;
      boards.T.visible = !!L.Tq && document.getElementById('v-t').checked;
      if (L.Tq) setQuat(boards.T, L.Tq);
    }
    view.render();
    requestAnimationFrame(loop);
  };
  requestAnimationFrame(loop);
}

export function updateLive() {
  const b = state.buf;
  if (!b.t || !b.t.length) return;
  updatePanel();
  const win = +document.getElementById('win').value;
  const n = b.t.length, tEnd = b.t[n - 1];
  let i0 = n - 1;
  while (i0 > 0 && b.t[i0 - 1] >= tEnd - win) i0--;
  const stride = Math.max(1, Math.ceil((n - i0) / 1500));
  const idx = [];
  for (let i = i0; i < n; i += stride) idx.push(i);
  for (const { def, chart, card } of charts) {
    const keys = def.series.map((s) => s[0]);
    const has = keys.some((k) => b[k] && !k.startsWith('_'));
    card.style.display = has ? '' : 'none';
    if (!has) continue;
    const data = [idx.map((i) => b.t[i])];
    for (const k of keys) {
      if (DERIVED[k]) data.push(idx.map((i) => (b.Asx ? DERIVED[k](b, i) : null)));
      else if (b[k]) data.push(idx.map((i) => (def.logY && b[k][i] != null && b[k][i] <= 0 ? null : b[k][i])));
      else data.push(idx.map(() => null));
    }
    chart.set(data);
  }
  // 整個分區都沒有資料（例如接控制板時的「連線」以外、模擬時的「連線」）就隱藏標題
  for (const h of document.querySelectorAll('#live-charts .charts')) {
    const any = [...h.children].some((c) => c.style.display !== 'none');
    h.style.display = any ? '' : 'none';
    h.previousElementSibling.style.display = any ? '' : 'none';
  }
}
