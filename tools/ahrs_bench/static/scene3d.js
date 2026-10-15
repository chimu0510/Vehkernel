// three.js 共用工具：z 朝上的場景、座標軸、箭頭、橢球、文字標籤。
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

export { THREE };

export function makeScene(el, { distance = 3.2 } = {}) {
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(40, 1, 0.01, 100);
  camera.up.set(0, 0, 1);
  camera.position.set(distance * 0.75, -distance * 0.95, distance * 0.6);
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  el.appendChild(renderer.domElement);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  scene.add(new THREE.AmbientLight(0xffffff, 0.7));
  const light = new THREE.DirectionalLight(0xffffff, 0.9);
  light.position.set(2, -3, 5);
  scene.add(light);

  const labels = [];
  function label(text, color = '#8b95a3') {
    const d = document.createElement('div');
    d.className = 'axis-label';
    d.style.color = color;
    d.textContent = text;
    el.appendChild(d);
    const obj = { div: d, pos: new THREE.Vector3(), visible: true };
    labels.push(obj);
    return obj;
  }

  function resize() {
    const w = el.clientWidth, h = el.clientHeight;
    if (!w || !h) return;
    renderer.setSize(w, h, false);
    renderer.domElement.style.width = `${w}px`;
    renderer.domElement.style.height = `${h}px`;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
  new ResizeObserver(resize).observe(el);
  resize();

  function render() {
    if (!el.offsetParent) return;   // 分頁沒顯示
    controls.update();
    renderer.render(scene, camera);
    const w = el.clientWidth, h = el.clientHeight;
    for (const l of labels) {
      const p = l.pos.clone().project(camera);
      const show = l.visible && p.z < 1;
      l.div.style.display = show ? '' : 'none';
      if (show) {
        l.div.style.left = `${(p.x * 0.5 + 0.5) * w}px`;
        l.div.style.top = `${(-p.y * 0.5 + 0.5) * h}px`;
      }
    }
  }
  return { scene, camera, renderer, controls, render, label };
}

export function line(points, color, { dashed = false, opacity = 1 } = {}) {
  const g = new THREE.BufferGeometry().setFromPoints(points.map((p) => new THREE.Vector3(...p)));
  const m = dashed
    ? new THREE.LineDashedMaterial({ color, dashSize: 0.05, gapSize: 0.04, transparent: opacity < 1, opacity })
    : new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity });
  const l = new THREE.Line(g, m);
  if (dashed) l.computeLineDistances();
  return l;
}

/** 可更新的箭頭（ArrowHelper 的包裝）。 */
export function arrow(color, headLen = 0.08) {
  const a = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 1, color, headLen, headLen * 0.5);
  a.update = (from, vec, maxHead = headLen) => {
    const v = new THREE.Vector3(...vec);
    const len = v.length();
    a.visible = len > 1e-6;
    if (!a.visible) return;
    a.position.set(...from);
    a.setDirection(v.clone().normalize());
    const h = Math.min(maxHead, len * 0.35);
    a.setLength(len, h, h * 0.5);
  };
  return a;
}

export function axes(len = 1, opacity = 1) {
  const g = new THREE.Group();
  const cols = [0xef5b5b, 0x5fd35f, 0x5b8def];
  for (let i = 0; i < 3; i++) {
    const d = [0, 0, 0];
    d[i] = len;
    g.add(line([[0, 0, 0], d], cols[i], { opacity }));
  }
  return g;
}

/** 控制板模型：PCB 平板 + 本體座標軸。 */
export function board(color = 0x2d6a4f, { wire = false, opacity = 1 } = {}) {
  const g = new THREE.Group();
  const geo = new THREE.BoxGeometry(1.0, 0.7, 0.05);
  if (wire) {
    g.add(new THREE.LineSegments(new THREE.EdgesGeometry(geo), new THREE.LineBasicMaterial({ color, transparent: true, opacity })));
  } else {
    g.add(new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color, roughness: 0.8, transparent: opacity < 1, opacity })));
    // 元件面標記（+z 面上的 IMU 小方塊），看得出正反面
    const chip = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.12, 0.03), new THREE.MeshStandardMaterial({ color: 0x222222 }));
    chip.position.set(0, 0, 0.04);
    g.add(chip);
  }
  const ax = axes(wire ? 0.6 : 0.8, wire ? opacity : 1);
  g.add(ax);
  return g;
}

export function setQuat(obj, q) {
  // 韌體四元數 [w, x, y, z]（本體到世界）
  obj.quaternion.set(q[1], q[2], q[3], q[0]);
}

/** 單位線框球。 */
export function sphere(r = 1, color = 0x3a4250) {
  const g = new THREE.Group();
  const m = new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.5 });
  const seg = 72;
  for (let k = -2; k <= 2; k++) {            // 緯線
    const lat = (k * 30 * Math.PI) / 180;
    const pts = [];
    for (let i = 0; i <= seg; i++) {
      const lo = (i / seg) * 2 * Math.PI;
      pts.push(new THREE.Vector3(r * Math.cos(lat) * Math.cos(lo), r * Math.cos(lat) * Math.sin(lo), r * Math.sin(lat)));
    }
    g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), m));
  }
  for (let k = 0; k < 6; k++) {              // 經線
    const lo = (k * 30 * Math.PI) / 180;
    const pts = [];
    for (let i = 0; i <= seg; i++) {
      const lat = (i / seg) * 2 * Math.PI;
      pts.push(new THREE.Vector3(r * Math.cos(lat) * Math.cos(lo), r * Math.cos(lat) * Math.sin(lo), r * Math.sin(lat)));
    }
    g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), m));
  }
  return g;
}

/**
 * 3×3 共變異數的 1σ 橢球：單位球經 L（Σ = L Lᵀ，取特徵分解）變換。
 * 回傳的物件有 update(center, Sigma, scale)。
 */
export function ellipsoid(color, opacity = 0.25) {
  const mesh = new THREE.Mesh(
    new THREE.SphereGeometry(1, 32, 16),
    new THREE.MeshStandardMaterial({ color, transparent: true, opacity, depthWrite: false, side: THREE.DoubleSide }),
  );
  const wire = new THREE.LineSegments(
    new THREE.WireframeGeometry(new THREE.SphereGeometry(1, 16, 8)),
    new THREE.LineBasicMaterial({ color, transparent: true, opacity: Math.min(1, opacity * 2) }),
  );
  const g = new THREE.Group();
  g.add(mesh, wire);
  g.matrixAutoUpdate = false;
  g.update = (center, S, scale = 1) => {
    const { vals, vecs } = eigSym3(S);
    const m = new THREE.Matrix4();
    const s = vals.map((v) => Math.sqrt(Math.max(v, 0)) * scale);
    m.set(
      vecs[0][0] * s[0], vecs[0][1] * s[1], vecs[0][2] * s[2], center[0],
      vecs[1][0] * s[0], vecs[1][1] * s[1], vecs[1][2] * s[2], center[1],
      vecs[2][0] * s[0], vecs[2][1] * s[1], vecs[2][2] * s[2], center[2],
      0, 0, 0, 1,
    );
    g.matrix.copy(m);
    g.visible = s.every(isFinite);
  };
  return g;
}

/** 對稱 3×3 的特徵分解（Jacobi）。vecs 的第 j 行是第 j 個特徵向量。 */
export function eigSym3(S) {
  const a = S.map((r) => r.slice());
  const v = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  for (let sweep = 0; sweep < 30; sweep++) {
    let off = 0;
    for (let p = 0; p < 3; p++) for (let q = p + 1; q < 3; q++) off += a[p][q] * a[p][q];
    if (off < 1e-30) break;
    for (let p = 0; p < 3; p++) for (let q = p + 1; q < 3; q++) {
      if (Math.abs(a[p][q]) < 1e-300) continue;
      const th = (a[q][q] - a[p][p]) / (2 * a[p][q]);
      const t = Math.sign(th || 1) / (Math.abs(th) + Math.sqrt(th * th + 1));
      const c = 1 / Math.sqrt(t * t + 1), s = t * c;
      for (let k = 0; k < 3; k++) {
        const akp = a[k][p], akq = a[k][q];
        a[k][p] = c * akp - s * akq;
        a[k][q] = s * akp + c * akq;
      }
      for (let k = 0; k < 3; k++) {
        const apk = a[p][k], aqk = a[q][k];
        a[p][k] = c * apk - s * aqk;
        a[q][k] = s * apk + c * aqk;
      }
      for (let k = 0; k < 3; k++) {
        const vkp = v[k][p], vkq = v[k][q];
        v[k][p] = c * vkp - s * vkq;
        v[k][q] = s * vkp + c * vkq;
      }
    }
  }
  return { vals: [a[0][0], a[1][1], a[2][2]], vecs: v };
}

/** 平面上的 1σ 橢圓（以兩個正交方向與 2×2 共變異數），update(center, u, v, C2, scale)。 */
export function ellipse(color) {
  const seg = 64;
  const geo = new THREE.BufferGeometry().setFromPoints(new Array(seg + 1).fill(0).map(() => new THREE.Vector3()));
  const l = new THREE.Line(geo, new THREE.LineBasicMaterial({ color }));
  l.update = (c, u, v, C, scale) => {
    // C = [[cuu, cuv], [cuv, cvv]] → 主軸
    const tr = C[0][0] + C[1][1], det = C[0][0] * C[1][1] - C[0][1] * C[0][1];
    const disc = Math.sqrt(Math.max(tr * tr / 4 - det, 0));
    const l1 = tr / 2 + disc, l2 = tr / 2 - disc;
    const ang = 0.5 * Math.atan2(2 * C[0][1], C[0][0] - C[1][1]);
    const a = Math.sqrt(Math.max(l1, 0)) * scale, b = Math.sqrt(Math.max(l2, 0)) * scale;
    const pos = geo.attributes.position;
    for (let i = 0; i <= seg; i++) {
      const t = (i / seg) * 2 * Math.PI;
      const x = a * Math.cos(t), y = b * Math.sin(t);
      const cu = x * Math.cos(ang) - y * Math.sin(ang), cv = x * Math.sin(ang) + y * Math.cos(ang);
      pos.setXYZ(i, c[0] + cu * u[0] + cv * v[0], c[1] + cu * u[1] + cv * v[1], c[2] + cu * u[2] + cv * v[2]);
    }
    pos.needsUpdate = true;
    geo.computeBoundingSphere();
  };
  return l;
}

// ---- 小型線性代數 ----
export const vec = {
  add: (a, b) => a.map((x, i) => x + b[i]),
  sub: (a, b) => a.map((x, i) => x - b[i]),
  scale: (a, s) => a.map((x) => x * s),
  dot: (a, b) => a.reduce((s, x, i) => s + x * b[i], 0),
  norm: (a) => Math.hypot(...a),
  cross: (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]],
};

/** 四元數 [w,x,y,z] → 旋轉矩陣 R（本體到世界）。 */
export function quatToR(q) {
  const [w, x, y, z] = q;
  return [
    [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
    [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
    [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
  ];
}

export const mat = {
  mul: (A, B) => A.map((r) => B[0].map((_, j) => r.reduce((s, x, k) => s + x * B[k][j], 0))),
  T: (A) => A[0].map((_, j) => A.map((r) => r[j])),
  mv: (A, v) => A.map((r) => r.reduce((s, x, k) => s + x * v[k], 0)),
  skew: (v) => [[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]],
  sub3: (P, i0, j0) => [0, 1, 2].map((i) => [0, 1, 2].map((j) => P[i0 + i][j0 + j])),
};
