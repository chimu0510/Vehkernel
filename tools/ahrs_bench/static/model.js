// 模型檢視：EKF 每一步的公式、即時數值與本體座標中的幾何。
/* global katex */
import { state } from './app.js';
import { fmt } from './charts.js';
import {
  THREE, arrow, axes, ellipse, ellipsoid, eigSym3, makeScene, mat, sphere, vec,
} from './scene3d.js';

const D = 180 / Math.PI;
const DT = 1 / 200;
let view, G = {}, active = null, frozen = null, lastText = 0;

const EQS = [
  {
    id: 'state', title: '1　狀態與誤差狀態',
    tex: String.raw`\begin{gathered}
      \mathbf{x}=(\mathbf{q},\,\mathbf{b}),\qquad
      \mathbf{q}_{\text{true}}=\mathbf{q}\otimes\operatorname{Exp}(\delta\boldsymbol\theta)\\
      \delta\mathbf{x}=\begin{bmatrix}\delta\boldsymbol\theta\\ \delta\mathbf{b}\end{bmatrix}\in\mathbb{R}^6,\qquad
      \mathbf{P}=\mathrm{E}\big[\delta\mathbf{x}\,\delta\mathbf{x}^{\top}\big]
      \end{gathered}`,
    points: [
      '<b>名義狀態</b>：四元數 q（本體 → 世界；世界為 ENU、z 朝上，REP-103）與陀螺儀零偏 b。',
      '<b>誤差狀態</b>：本體座標的小角度向量 δθ 與零偏誤差 δb，P 是它們的 6×6 共變異數。',
      '<b>為什麼不直接濾 q</b>：四元數有單位長度約束，只有 3 個自由度；對 4 維的 q 做 EKF 會讓共變異數奇異。每次修正後把 δθ 併回 q、δx 歸零（MEKF）。',
    ],
    view: '綠色橢球是 P<sub>θθ</sub> 的 1σ（旋轉向量空間，本體座標，自動縮放）。',
    show: ['body', 'ellP'],
  },
  {
    id: 'predict', title: '2　預測：陀螺儀積分與共變異數傳播',
    tex: String.raw`\begin{gathered}
      \hat{\boldsymbol\omega}=\boldsymbol\omega_m-\mathbf{b},\qquad
      \mathbf{q}\leftarrow\mathbf{q}\otimes\begin{bmatrix}1\\ \tfrac12\hat{\boldsymbol\omega}\Delta t\end{bmatrix}\\
      \mathbf{F}=\begin{bmatrix}\mathbf{I}-[\hat{\boldsymbol\omega}]_\times\Delta t & -\mathbf{I}\Delta t\\ \mathbf{0} & \mathbf{I}\end{bmatrix},\qquad
      \mathbf{Q}=\begin{bmatrix}\sigma_g^2\Delta t\,\mathbf{I}&\mathbf{0}\\ \mathbf{0}&\sigma_b^2\Delta t\,\mathbf{I}\end{bmatrix}\\
      \mathbf{P}\leftarrow\mathbf{F}\mathbf{P}\mathbf{F}^{\top}+\mathbf{Q}
      \end{gathered}`,
    points: [
      '<b>誤差動態</b>：δθ̇ = −[ω̂]<sub>×</sub>δθ − δb − n<sub>g</sub>。',
      '<b>F 右上角的 −IΔt</b>：零偏誤差每一步都轉成姿態誤差，P 的 θ–b 交叉項由此產生；之後加速度量測能修正零偏，靠的就是這個交叉項。',
      '<b>Q 的離散化</b>：連續時間雜訊密度 × Δt。σ<sub>g</sub> 的單位是 dps/√Hz，只靠陀螺儀時角度誤差以 σ<sub>g</sub>√t 成長。',
      '<b>近似</b>：q 的更新是一階小角度近似再正規化，每步截斷誤差約 (ωΔt)³/24。',
    ],
    view: '淡色橢球是預測後（修正前）的 P<sub>θθ</sub>，實色是修正後；兩者的差就是這一步量測帶來的資訊。',
    show: ['body', 'ellP', 'ellPprior'],
  },
  {
    id: 'meas', title: '3　量測模型：加速度計看到的重力方向',
    tex: String.raw`\begin{gathered}
      \mathbf{v}=\mathbf{h}(\mathbf{q})=\mathbf{R}(\mathbf{q})^{\top}\mathbf{e}_z,\qquad
      \hat{\mathbf{a}}=\frac{\mathbf{a}}{\lVert\mathbf{a}\rVert},\qquad
      \mathbf{y}=\hat{\mathbf{a}}-\mathbf{v}\\
      \mathbf{H}=\big[\,[\mathbf{v}]_\times\;\;\mathbf{0}\,\big],\qquad
      \mathbf{S}=\mathbf{H}\mathbf{P}\mathbf{H}^{\top}+\sigma^2\mathbf{I},\qquad
      \sigma=\sigma_a+\kappa\,\big|\lVert\mathbf{a}\rVert-1\big|
      \end{gathered}`,
    points: [
      '<b>假設</b>：加速度計只量到重力（靜止或等速），所以量到的方向應等於 v = Rᵀe<sub>z</sub>。',
      '<b>H = [v]<sub>×</sub></b>：δθ 讓預測方向變成 v + v×δθ，變化永遠垂直於 v。因此繞 v 的轉動（水平時就是 yaw）不改變量測 → 不可觀測；y 沿 v 的徑向分量與 δθ 無關，只能由 R 解釋（也計入 NIS）。',
      '<b>自適應 σ 只看 |a|</b>：垂直加速度是一階；水平加速度 a<sub>h</sub> 只讓 |a| 變 √(1+a<sub>h</sub>²) − 1 ≈ a<sub>h</sub>²/2。0.1 g 的加減速只讓 |a| 變 0.005 g，幾乎不被降權，卻會被當成 atan(0.1) = 5.7° 的傾斜。',
    ],
    view: 'v（藍）、â 以「v + 倍率×y」顯示（白點）、y（紅箭頭）、S 的 1σ 橢球（紅）、HPHᵀ（青色橢圓，在切平面上）。',
    show: ['body', 'sphere', 'v', 'a', 'y', 'ellS', 'ellHPH'],
  },
  {
    id: 'update', title: '4　卡爾曼增益與修正',
    tex: String.raw`\begin{gathered}
      \mathbf{K}=\mathbf{P}\mathbf{H}^{\top}\mathbf{S}^{-1}=\begin{bmatrix}\mathbf{K}_\theta\\ \mathbf{K}_b\end{bmatrix},\qquad
      \begin{bmatrix}\delta\boldsymbol\theta\\ \delta\mathbf{b}\end{bmatrix}=\mathbf{K}\mathbf{y}\\
      \mathbf{q}\leftarrow\mathbf{q}\otimes\operatorname{Exp}(\delta\boldsymbol\theta),\qquad
      \mathbf{b}\leftarrow\mathbf{b}+\delta\mathbf{b},\qquad
      \mathbf{P}\leftarrow(\mathbf{I}-\mathbf{K}\mathbf{H})\mathbf{P}
      \end{gathered}`,
    points: [
      '<b>K<sub>θ</sub> 修姿態、K<sub>b</sub> 修零偏</b>：K<sub>b</sub> = P<sub>bθ</sub>[v]<sub>×</sub>ᵀS⁻¹，只有交叉項不為 0 時零偏才會被加速度修正，而且只修得到垂直於 v 的分量（水平時是 x、y）。',
      '<b>修正多少</b>：預測方向移動 v×δθ，只是 y 切向分量的一小部分，比例約 HPHᵀ/S——目前姿態越準、感測器越吵，移得越少。',
      '<b>實作</b>：韌體用 P ← P − K(PHᵀ)ᵀ 再對稱化（不是 Joseph form，單精度下靠對稱化維持正定）。',
    ],
    view: 'EKF 的修正旋轉 δθ（綠箭頭）與修正後的 v⁺（綠點，倍率同 y）。',
    show: ['body', 'sphere', 'v', 'a', 'y', 'dth', 'vplus'],
  },
  {
    id: 'zupt', title: '5　零角速度量測（ZUPT）',
    tex: String.raw`\text{static: }\;\mathbf{y}_z=\boldsymbol\omega_m-\mathbf{b},\qquad
      \mathbf{H}_z=\big[\,\mathbf{0}\;\;\mathbf{I}\,\big],\qquad \mathbf{R}_z=\sigma_z^2\mathbf{I}`,
    points: [
      '<b>原理</b>：靜止時真實角速度為 0，陀螺儀讀值就是零偏的直接量測；這是 b<sub>z</sub> 唯一的資訊來源。',
      '<b>靜止判斷</b>：原始角速度的移動標準差、扣零偏後的角速度、||a| − 1 g| 都夠小，且持續 T<sub>still</sub>。',
      '<b>限制 1</b>：比 ω<sub>still</sub> 慢的等速轉動會被當成零偏吃掉。',
      '<b>限制 2</b>：同一筆 ω<sub>m</sub> 同時用在預測與這個量測，雜訊相關，違反 EKF 的獨立假設，σ<sub>b</sub> 因此偏樂觀（「參數分析」的「實際 σ」有算進去）。',
    ],
    view: 'P<sub>θθ</sub> 橢球：靜止時 ZUPT 讓沿 v 的方向成長變慢。',
    show: ['body', 'sphere', 'v', 'ellP'],
  },
  {
    id: 'mahony', title: '6　Mahony 與 EKF 的關係',
    tex: String.raw`\begin{gathered}
      \mathbf{e}=\hat{\mathbf{a}}\times\mathbf{v},\qquad
      \hat{\boldsymbol\omega}=\boldsymbol\omega_m-\mathbf{b}+K_p\mathbf{e},\qquad
      \mathbf{b}\leftarrow\mathbf{b}-K_i\,\mathbf{e}\,\Delta t\\[4pt]
      [\mathbf{v}]_\times^{\top}\mathbf{y}=-\mathbf{v}\times(\hat{\mathbf{a}}-\mathbf{v})=\hat{\mathbf{a}}\times\mathbf{v}=\mathbf{e}\\[4pt]
      \mathbf{P}_{\theta\theta}=p\mathbf{I},\ \mathbf{S}=s\mathbf{I}\;\Longrightarrow\;
      \delta\boldsymbol\theta=\frac{p}{s}\,\mathbf{e},\qquad K_p^{\text{eq}}=\frac{p}{s\,\Delta t}
      \end{gathered}`,
    points: [
      '<b>關鍵等式</b>：Mahony 的誤差向量 e 正好是 Hᵀy 的姿態部分。P<sub>θθ</sub> 與 S 都等向且固定時，EKF 的姿態修正就是 Mahony 的比例項。',
      '<b>EKF 多出來的</b>：(1) 增益隨 P 變化（開機時大、收斂後小）；(2) P 不等向，沿 v（yaw）大；(3) 零偏修正來自交叉項，相當於 K<sub>i</sub>，z 軸另有 ZUPT；(4) R 隨 |a| 自適應。',
      '<b>共同限制</b>：e 也垂直於 v，所以 Mahony 的 K<sub>i</sub> 在水平時同樣修不到 b<sub>z</sub>。',
    ],
    view: '同一縮放下比較每一步的修正旋轉：EKF δθ（綠）與 Mahony K<sub>p</sub>eΔt（橘）。',
    show: ['body', 'sphere', 'v', 'a', 'y', 'dth', 'emah'],
  },
  {
    id: 'obs', title: '7　不確定度的形狀與可觀測性',
    tex: String.raw`\begin{gathered}
      \mathbf{H}\mathbf{P}\mathbf{H}^{\top}=[\mathbf{v}]_\times\mathbf{P}_{\theta\theta}[\mathbf{v}]_\times^{\top}\\[4pt]
      \sigma_{\parallel}=\sqrt{\mathbf{v}^{\top}\mathbf{P}_{\theta\theta}\mathbf{v}}\ \ (\text{about gravity}),\qquad
      \sigma_{\perp}:\ \mathbf{P}_{\theta\theta}\ \text{restricted to}\ \mathbf{v}^{\perp}
      \end{gathered}`,
    points: [
      '<b>σ<sub>∥</sub></b>：P<sub>θθ</sub> 沿 v 的分量 = 繞重力轉動（yaw）的不確定度，加速度修不到，只會成長。',
      '<b>σ<sub>⊥</sub></b>：垂直於 v 的兩個方向是傾角，被重力約束在很小的值，所以 P<sub>θθ</sub> 的橢球沿 v 拉成細長的針。',
      '<b>取決於重力方向</b>：控制板側放時 v 不再是本體 z，被拉長的方向跟著變；可觀測性看的是重力方向，不是本體的哪一軸。',
    ],
    view: 'P<sub>θθ</sub> 橢球與 v：橢球的長軸應沿 v。',
    show: ['body', 'sphere', 'v', 'ellP'],
  },
];

const LEG = {
  v: ['#4fa3ff', 'v 預測重力方向'], a: ['#ffffff', 'â（v + 放大×y）'], y: ['#ef5b5b', 'y 創新量（放大）'],
  ellS: ['#ef5b5b', 'S 1σ'], ellHPH: ['#56b6c2', 'HPHᵀ 1σ'], ellP: ['#98c379', 'P_θθ 1σ（修正後）'],
  ellPprior: ['#98c379', 'P_θθ 1σ（預測後，虛）'], dth: ['#5fd35f', 'EKF δθ'], vplus: ['#5fd35f', 'v⁺'],
  emah: ['#f2a33a', 'Mahony Kp·e·Δt'],
};

function initScene() {
  const el = document.getElementById('model3d');
  view = makeScene(el, { distance: 3.4 });
  view.camera.position.set(2.6, -3.0, 2.0);
  view.controls.target.set(0, 0, 0.45);
  G.body = axes(1.25, 0.8);
  view.label('x_b', '#ef5b5b').pos.set(1.35, 0, 0);
  view.label('y_b', '#5fd35f').pos.set(0, 1.35, 0);
  view.label('z_b', '#5b8def').pos.set(0, 0, 1.35);
  G.sphere = sphere(1);
  G.v = arrow(0x4fa3ff, 0.08);
  G.a = new THREE.Mesh(new THREE.SphereGeometry(0.025), new THREE.MeshBasicMaterial({ color: 0xffffff }));
  G.y = arrow(0xef5b5b, 0.05);
  G.ellS = ellipsoid(0xef5b5b, 0.12);
  G.ellHPH = ellipse(0x56b6c2);
  G.ellP = ellipsoid(0x98c379, 0.18);
  G.ellPprior = ellipsoid(0x98c379, 0.05);
  G.dth = arrow(0x5fd35f, 0.06);
  G.vplus = new THREE.Mesh(new THREE.SphereGeometry(0.022), new THREE.MeshBasicMaterial({ color: 0x5fd35f }));
  G.emah = arrow(0xf2a33a, 0.06);
  for (const o of Object.values(G)) view.scene.add(o);
  G.lblv = view.label('v', '#4fa3ff');
  G.lbla = view.label('â', '#ffffff');
  const loop = () => { view.render(); requestAnimationFrame(loop); };
  requestAnimationFrame(loop);
}

function renderEqs() {
  const host = document.getElementById('eqs');
  for (const e of EQS) {
    const card = document.createElement('div');
    card.className = 'eq-card';
    card.innerHTML = `<h3>${e.title}</h3><div class="tex"></div>`
      + `<ul>${e.points.map((p) => `<li>${p}</li>`).join('')}</ul>`
      + `<div class="view3d-note">${e.view}</div>`
      + `<details open><summary>即時數值</summary><div class="live" id="live-${e.id}"></div></details>`;
    card.querySelector('details').onclick = (ev) => ev.stopPropagation();
    katex.render(e.tex, card.querySelector('.tex'), { displayMode: true, throwOnError: false, strict: false });
    card.onclick = () => { active = active === e.id ? null : e.id; highlight(); };
    host.appendChild(card);
  }
}

function highlight() {
  for (const c of document.querySelectorAll('.eq-card')) c.classList.toggle('on', false);
  const eq = EQS.find((e) => e.id === active);
  if (eq) document.querySelectorAll('.eq-card')[EQS.indexOf(eq)].classList.add('on');
  const show = eq ? new Set(eq.show) : new Set(['body', 'sphere', 'v', 'a', 'y', 'ellS', 'dth', 'emah', 'vplus']);
  for (const [k, o] of Object.entries(G)) {
    if (k.startsWith('lbl')) continue;
    o.userData.enabled = show.has(k);
  }
  document.getElementById('geo-legend').innerHTML = [...show].filter((k) => LEG[k])
    .map((k) => `<span style="--c:${LEG[k][0]}">${LEG[k][1]}</span>`).join('')
    + '<span style="--c:transparent">拖曳旋轉、滾輪縮放</span>';
}

export function initModel() {
  renderEqs();
  initScene();
  highlight();
  const mag = document.getElementById('mag');
  const show = () => { document.getElementById('mag-v').textContent = `自動 ×${fmt(10 ** (+mag.value - 2))}`; };
  mag.oninput = show;
  show();
  document.getElementById('freeze').onchange = (ev) => { frozen = ev.target.checked ? state.latest : null; };
}

function setVis(k, on) { G[k].visible = on && G[k].userData.enabled; }

const v3 = (a, d = 4) => `[${a.map((x) => (x >= 0 ? ' ' : '') + x.toFixed(d)).join(', ')}]`;
const deg3 = (a, d = 4) => v3(a.map((x) => x * D), d);

export function updateModel() {
  const L = frozen || state.latest;
  if (!L || !view) return;
  const A = L.A, M = L.Am;
  const P = A.P, Pp = A.P_prior;
  const Ptt = mat.sub3(P, 0, 0), Ptt0 = mat.sub3(Pp, 0, 0);
  const v = A.v, a = A.a_unit, y = A.y;
  const used = A.accel_used > 0.5;
  // y、S 的顯示倍率：自動時讓 max(|y|, S 的最大 1σ) 顯示成 0.4，再乘上滑桿的微調
  const sMaxS = Math.sqrt(Math.max(...eigSym3(A.S).vals, 1e-30));
  const auto = 0.4 / Math.max(vec.norm(y), sMaxS, 1e-9);
  const m = auto * 10 ** (+document.getElementById('mag').value - 2);

  setVis('body', true);
  setVis('sphere', true);
  G.v.update([0, 0, 0], v, 0.08);
  setVis('v', true);
  G.lblv.pos.set(...vec.scale(v, 1.1));
  const aDisp = vec.add(v, vec.scale(y, m));
  G.a.position.set(...aDisp);
  setVis('a', used);
  G.lbla.pos.set(...vec.scale(aDisp, 1.08));
  G.lbla.visible = used && G.a.userData.enabled;
  G.lblv.visible = G.v.userData.enabled;
  G.y.update(v, vec.scale(y, m), 0.05);
  setVis('y', used);
  G.ellS.update(v, A.S, m);
  setVis('ellS', used);

  // HPHᵀ 在切平面上：取切平面基底 u、w
  const ref = Math.abs(v[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
  const u = vec.scale(vec.cross(v, ref), 1 / vec.norm(vec.cross(v, ref)));
  const w = vec.cross(v, u);
  const Hm = mat.skew(v);
  const HPH = mat.mul(mat.mul(Hm, Ptt0), mat.T(Hm));
  const proj = (p, q) => vec.dot(p, mat.mv(HPH, q));
  G.ellHPH.update(v, u, w, [[proj(u, u), proj(u, w)], [proj(u, w), proj(w, w)]], m);
  setVis('ellHPH', used);

  // P_θθ 橢球：自動縮放到最長軸 0.9
  const ev = eigSym3(Ptt0).vals.concat(eigSym3(Ptt).vals);
  const sMax = Math.sqrt(Math.max(...ev, 1e-30));
  const kP = 0.9 / sMax;
  G.ellP.update([0, 0, 0], Ptt, kP);
  G.ellPprior.update([0, 0, 0], Ptt0, kP);
  setVis('ellP', true);
  setVis('ellPprior', true);

  // 修正：EKF δθ 與 Mahony Kp e Δt 用同一縮放
  const dth = A.dx.slice(0, 3);
  const kp = state.values.A['mah.kp'] ?? 1;
  const emah = vec.scale(M.e, kp * DT);
  const cMax = Math.max(vec.norm(dth), vec.norm(emah), 1e-30);
  const kC = 0.7 / cMax;
  G.dth.update([0, 0, 0], vec.scale(dth, kC), 0.06);
  G.emah.update([0, 0, 0], vec.scale(emah, kC), 0.06);
  setVis('dth', used);
  setVis('emah', M.accel_used > 0.5);
  const vplus = vec.add(v, vec.scale(vec.cross(v, dth), m));
  G.vplus.position.set(...vplus);
  setVis('vplus', used);

  const now = performance.now();
  if (now - lastText < 120) return;
  lastText = now;
  renderLive(L, { Ptt, Ptt0, dth, emah, kP, kC, u, w, HPH, m });
  drawMatrices(A);
}

function renderLive(L, g) {
  const A = L.A, M = L.Am;
  const set = (id, s) => { const e = document.getElementById(`live-${id}`); if (e) e.textContent = s; };
  const sig = (Pm, i) => Math.sqrt(Math.max(Pm[i][i], 0));
  const pv = A.P, pp = A.P_prior;
  set('state', [
    `q        = ${v3(A.q, 5)}   (w, x, y, z)`,
    `b        = ${deg3(A.b, 4)} dps`,
    `σ_θ(後)  = ${deg3([0, 1, 2].map((i) => sig(pv, i)), 4)} °`,
    `σ_b(後)  = ${deg3([3, 4, 5].map((i) => sig(pv, i)), 5)} dps`,
    `P_θθ 橢球顯示倍率 ×${fmt(g.kP)}`,
  ].join('\n'));
  const w = vec.sub(L.gyro.map((x) => x / D), A.b);
  set('predict', [
    `ω_m      = ${v3(L.gyro, 3)} dps`,
    `ω̂        = ${deg3(w, 3)} dps   |ω̂|Δt = ${(vec.norm(w) * DT * D).toExponential(2)}°/步`,
    `σ_θ 預測後 = ${deg3([0, 1, 2].map((i) => sig(pp, i)), 5)} °`,
    `σ_θ 修正後 = ${deg3([0, 1, 2].map((i) => sig(pv, i)), 5)} °`,
    `P_θb 交叉相關 ρ(δθx,δbx)=${(pv[0][3] / Math.sqrt(pv[0][0] * pv[3][3])).toFixed(3)}  ρ(δθy,δby)=${(pv[1][4] / Math.sqrt(pv[1][1] * pv[4][4])).toFixed(3)}`,
  ].join('\n'));
  const v = A.v, y = A.y;
  const yr = vec.dot(y, v), yt = vec.norm(vec.sub(y, vec.scale(v, yr)));
  set('meas', A.accel_used > 0.5 ? [
    `v        = ${v3(v, 5)}`,
    `â        = ${v3(A.a_unit, 5)}`,
    `y        = ${v3(y, 5)}   切向 ${yt.toExponential(2)}（≈${(Math.asin(Math.min(yt, 1)) * D).toFixed(3)}°）徑向 ${yr.toExponential(2)}`,
    `σ        = ${A.sigma_a.toFixed(4)}   √diag S = ${v3([0, 1, 2].map((i) => Math.sqrt(A.S[i][i])), 5)}`,
    `NIS      = ${A.nis.toFixed(3)}   （χ²₃：平均 3，95% 在 0.216–9.35）`,
    `y、S 顯示倍率 ×${fmt(g.m)}`,
  ].join('\n') : '這一步沒有採用加速度（||a| − 1 g| 超過拒絕門檻）');
  const dth = g.dth, db = A.dx.slice(3, 6);
  const moved = vec.norm(vec.cross(v, dth));
  set('update', A.accel_used > 0.5 ? [
    `δθ       = ${deg3(dth, 6)} °`,
    `δb       = ${deg3(db, 7)} dps`,
    `|v×δθ| / |y 切向| = ${(moved / Math.max(yt, 1e-30)).toExponential(3)}   （這一步把預測重力方向往量測拉的比例）`,
    `K_θ 對角 = ${v3([0, 1, 2].map((i) => A.K[i][i]), 5)}   K_b 最大 |元素| = ${Math.max(...A.K.slice(3).flat().map(Math.abs)).toExponential(2)}`,
    `修正量顯示倍率 ×${fmt(g.kC)}`,
  ].join('\n') : '沒有加速度修正');
  set('zupt', [
    `靜止：${A.still > 0.5 ? '是' : '否'}   ZUPT 這一步${A.zupt_used > 0.5 ? '有' : '沒有'}修正`,
    A.zupt_used > 0.5 ? `y_z      = ${deg3(A.y_zupt, 4)} dps   NIS_z = ${A.nis_zupt.toFixed(3)}` : '',
    `σ_bz     = ${(sig(pv, 5) * D).toExponential(3)} dps`,
  ].filter(Boolean).join('\n'));
  const e = M.e, kp = state.values.A['mah.kp'];
  // 用 EKF 自己的 v 算 e，才能和 EKF 的 δθ 比較（Mahony 的 v 來自它自己的姿態）
  const eE = vec.cross(A.a_unit, v);
  const dthPerp = vec.norm(vec.sub(dth, vec.scale(v, vec.dot(dth, v))));
  const cos = vec.dot(dth, eE) / Math.max(vec.norm(dth) * vec.norm(eE), 1e-30);
  set('mahony', [
    `Mahony：e     = ${v3(e, 5)}   Kp·e·Δt = ${deg3(g.emah, 6)} °/步 (Kp = ${kp})`,
    `EKF：â×v      = ${v3(eE, 5)}`,
    `EKF：δθ       = ${deg3(dth, 6)} °/步`,
    `δθ 與 â×v 的夾角 = ${(Math.acos(Math.max(-1, Math.min(1, cos))) * D).toFixed(1)}°   （P、S 等向時為 0）`,
    `EKF 等效 Kp = |δθ⊥| / (|â×v|Δt) = ${(dthPerp / Math.max(vec.norm(eE) * DT, 1e-30)).toFixed(4)} 1/s   （對照 Mahony Kp = ${kp}）`,
  ].join('\n'));
  const Pt = g.Ptt;
  const sPar = Math.sqrt(Math.max(vec.dot(v, mat.mv(Pt, v)), 0));
  const tr = vec.dot(g.u, mat.mv(Pt, g.u)), tw = vec.dot(g.w, mat.mv(Pt, g.w)), tuw = vec.dot(g.u, mat.mv(Pt, g.w));
  const disc = Math.sqrt(Math.max(((tr - tw) / 2) ** 2 + tuw * tuw, 0));
  set('obs', [
    `σ∥（繞重力，≈yaw）= ${(sPar * D).toFixed(4)}°`,
    `σ⊥（傾角兩個主軸）= ${(Math.sqrt(Math.max((tr + tw) / 2 + disc, 0)) * D).toFixed(5)}°, ${(Math.sqrt(Math.max((tr + tw) / 2 - disc, 0)) * D).toFixed(5)}°`,
    `比值 σ∥/σ⊥ = ${(sPar / Math.sqrt(Math.max((tr + tw) / 2 + disc, 1e-30))).toFixed(1)}`,
  ].join('\n'));
}

const LBL = ['δθx', 'δθy', 'δθz', 'δbx', 'δby', 'δbz'];

function heat(ctx, x, y, w, h, val) {
  // val ∈ [-1, 1]：藍（負）→ 深灰（0）→ 紅（正）
  const t = Math.max(-1, Math.min(1, val));
  const r = t > 0 ? 40 + 200 * t : 40, b = t < 0 ? 40 - 200 * t : 40, gg = 44;
  ctx.fillStyle = `rgb(${r | 0},${gg},${b | 0})`;
  ctx.fillRect(x, y, w, h);
}

function drawMatrices(A) {
  const pc = document.getElementById('pmat'), kc = document.getElementById('kmat');
  if (!pc.offsetParent) return;
  const P = A.P;
  let ctx = pc.getContext('2d');
  const o = 36, c = (pc.width - o) / 6;
  ctx.clearRect(0, 0, pc.width, pc.height);
  ctx.font = '11px system-ui';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  for (let i = 0; i < 6; i++) {
    ctx.fillStyle = '#8b95a3';
    ctx.fillText(LBL[i], o + c * i + c / 2, o / 2);
    ctx.fillText(LBL[i], o / 2, o + c * i + c / 2);
    for (let j = 0; j < 6; j++) {
      const rho = P[i][j] / Math.sqrt(Math.max(P[i][i] * P[j][j], 1e-300));
      heat(ctx, o + c * j + 1, o + c * i + 1, c - 2, c - 2, rho);
      ctx.fillStyle = '#e0e0e0';
      const txt = i === j ? (i < 3 ? `${fmt(Math.sqrt(P[i][i]) * D)}°` : fmt(Math.sqrt(P[i][i]) * D)) : rho.toFixed(2);
      ctx.fillText(txt, o + c * j + c / 2, o + c * i + c / 2);
    }
  }
  ctx = kc.getContext('2d');
  const ck = (kc.width - o) / 3, rk = (kc.height - o) / 6;
  ctx.clearRect(0, 0, kc.width, kc.height);
  ctx.font = '10px system-ui';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  const maxT = Math.max(...A.K.slice(0, 3).flat().map(Math.abs), 1e-30);
  const maxB = Math.max(...A.K.slice(3).flat().map(Math.abs), 1e-30);
  for (let j = 0; j < 3; j++) { ctx.fillStyle = '#8b95a3'; ctx.fillText(`y${'xyz'[j]}`, o + ck * j + ck / 2, o / 2); }
  for (let i = 0; i < 6; i++) {
    ctx.fillStyle = '#8b95a3';
    ctx.fillText(LBL[i], o / 2, o + rk * i + rk / 2);
    for (let j = 0; j < 3; j++) {
      const k = A.K[i][j];
      heat(ctx, o + ck * j + 1, o + rk * i + 1, ck - 2, rk - 2, k / (i < 3 ? maxT : maxB));
      ctx.fillStyle = '#e0e0e0';
      ctx.fillText(k.toExponential(1), o + ck * j + ck / 2, o + rk * i + rk / 2);
    }
  }
  document.getElementById('mat-note').innerHTML =
    '<b>左</b>：相關係數 ρ<sub>ij</sub> = P<sub>ij</sub>/√(P<sub>ii</sub>P<sub>jj</sub>)，對角線是 σ（姿態為度、零偏為 dps）。'
    + '同軸的 δθ–δb（例如 δθx–δbx）通常為負，來自預測的 −IΔt：零偏估太大時，扣掉後的角速度偏小，估計角度也跟著偏小。'
    + '<br><b>右</b>：卡爾曼增益 K（6×3，上下兩塊各自正規化上色）。水平時 δθz、δbz 兩列約為 0：重力看不到繞 z 的轉動。';
}
