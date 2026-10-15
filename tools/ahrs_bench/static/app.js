// 主程式：與伺服器的連線（SSE + REST）、資料緩衝、分頁與標頭。
import { initLive, updateLive } from './live.js';
import { initModel, updateModel } from './model.js';
import { initTuning, onValuesChanged } from './tuning.js';
import { initScenario } from './scenario.js';

export const state = {
  meta: [], sensorMeta: [], values: { A: {}, B: {} }, sensor: {}, scenarios: {},
  status: {}, latest: null, buf: {}, paused: false, tab: 'live', rate: 200,
};
const MAX_SECONDS = 130;

export async function api(path, body) {
  const r = await fetch(path, body === undefined ? {} : {
    method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body),
  });
  const j = await r.json();
  if (j && j.error) throw new Error(j.error);
  return j;
}

export async function refreshState() {
  const s = await api('/api/state');
  const changed = JSON.stringify(s.values) !== JSON.stringify(state.values);
  Object.assign(state, { meta: s.meta, sensorMeta: s.sensor_meta, values: s.values, sensor: s.sensor,
    scenarios: s.scenarios, status: s.status, rate: s.rate_hz });
  if (changed) onValuesChanged();
  return s;
}

function appendCols(cols) {
  const b = state.buf;
  const n = cols.t.length;
  if (b.t && b.t.length && cols.t[0] < b.t[b.t.length - 1]) {
    // 來源重新開始（時間倒退）：清空緩衝。
    for (const k of Object.keys(b)) delete b[k];
  }
  for (const [k, v] of Object.entries(cols)) {
    if (!b[k]) b[k] = new Array((b.t ? b.t.length : 0)).fill(null);
    b[k].push(...v);
  }
  // 這一批沒有的欄位補 null，維持等長。
  const len = b.t.length;
  for (const k of Object.keys(b)) if (b[k].length < len) b[k].push(...new Array(len - b[k].length).fill(null));
  const tEnd = b.t[len - 1];
  let cut = 0;
  while (cut < len && b.t[cut] < tEnd - MAX_SECONDS) cut++;
  if (cut > 2000 || cut > len / 4) for (const k of Object.keys(b)) b[k].splice(0, cut);
  return n;
}

// ---- 標頭 ----

function chip(label, value, cls = '') {
  return `<span class="chip ${cls}">${label} <b>${value}</b></span>`;
}

function renderChips() {
  const s = state.status;
  const src = s.source || {};
  const out = [];
  if (src.type === 'serial') out.push(chip('串口', `${src.port} ${src.baud}`, src.open ? 'good' : 'bad'));
  else if (src.type === 'sim') out.push(chip('模擬', `${state.scenarios[src.scenario]?.[0] || src.scenario} #${src.seed}`));
  else if (src.type === 'replay') out.push(chip('重播', src.file));
  out.push(chip('取樣率', `${s.rate ?? 0} Hz`, Math.abs((s.rate || 0) - state.rate) < 5 ? 'good' : 'warn'));
  if (src.type === 'serial') {
    out.push(chip('副本', s.synced ? '已同步' : '等待重設', s.synced ? 'good' : 'warn'));
    const d = s.fw_diff_max_deg;
    out.push(chip('副本誤差', d == null ? '—' : `${d.toExponential(1)}°`, d != null && d > 0.01 ? 'bad' : 'good'));
    out.push(chip('掉樣本', s.gaps, s.gaps ? 'warn' : ''));
    out.push(chip('CRC 錯誤', src.crc_errors, src.crc_errors ? 'warn' : ''));
    if (s.isr_us != null) out.push(chip('中斷', `${s.isr_us} µs`, s.overruns ? 'bad' : ''));
    if (s.fw_rev != null) out.push(chip('參數版本', s.fw_rev));
  }
  if (s.recording != null) out.push(chip('錄製中', `${s.recording} 筆`, 'bad'));
  document.getElementById('chips').innerHTML = out.join('');
  document.getElementById('rec').classList.toggle('on', s.recording != null);
}

async function fillSourceArgs() {
  const type = document.getElementById('src-type').value;
  const sel = document.getElementById('src-arg');
  sel.innerHTML = '';
  if (type === 'serial') {
    const ports = await api('/api/ports');
    ports.sort((a, b) => b.wch - a.wch);
    for (const p of ports) sel.add(new Option(`${p.device} ${p.wch ? '(CH9102F)' : ''}`, p.device));
    if (!ports.length) sel.add(new Option('找不到串口', ''));
  } else if (type === 'sim') {
    for (const [k, v] of Object.entries(state.scenarios)) sel.add(new Option(`${v[0]}（${v[1]} s）`, k));
  } else {
    const files = await api('/api/recordings');
    for (const f of files.reverse()) sel.add(new Option(f, f));
    if (!files.length) sel.add(new Option('尚無錄製檔', ''));
  }
  const src = state.status.source;
  if (src && src.type === type) sel.value = src.port || src.scenario || src.file || sel.value;
}

function initHeader() {
  const type = document.getElementById('src-type');
  type.onchange = fillSourceArgs;
  document.getElementById('src-go').onclick = async () => {
    const arg = document.getElementById('src-arg').value;
    if (!arg) return;
    const t = type.value;
    await api('/api/source', t === 'serial' ? { type: t, port: arg } : t === 'sim' ? { type: t, scenario: arg } : { type: t, file: arg });
    for (const k of Object.keys(state.buf)) delete state.buf[k];
    setTimeout(refreshState, 500);
  };
  for (const b of document.querySelectorAll('[data-cmd]')) {
    b.onclick = () => api('/api/cmd', { cmd: b.dataset.cmd });
  }
  document.getElementById('rec').onclick = async () => {
    const on = state.status.recording != null;
    const r = await api('/api/record', { action: on ? 'stop' : 'start' });
    if (r.file) alert(`已存檔 recordings/${r.file}`);
  };
  const notes = document.getElementById('notes');
  try { notes.checked = localStorage.getItem('ahrs_bench_notes') === '1'; } catch (e) { /* 無痕模式 */ }
  const applyNotes = () => {
    document.body.classList.toggle('show-notes', notes.checked);
    try { localStorage.setItem('ahrs_bench_notes', notes.checked ? '1' : '0'); } catch (e) { /* 無痕模式 */ }
  };
  notes.onchange = applyNotes;
  applyNotes();
  const pause = document.getElementById('pause');
  pause.onclick = () => {
    state.paused = !state.paused;
    pause.classList.toggle('on', state.paused);
    pause.textContent = state.paused ? '繼續畫面' : '暫停畫面';
  };
  for (const b of document.querySelectorAll('nav.tabs > button')) {
    b.onclick = () => {
      state.tab = b.dataset.tab;
      for (const x of document.querySelectorAll('nav.tabs > button')) x.classList.toggle('on', x === b);
      for (const s of document.querySelectorAll('.tab')) s.classList.toggle('on', s.id === `tab-${state.tab}`);
      window.dispatchEvent(new Event('resize'));
    };
  }
}

// ---- 主迴圈 ----

let dirty = false;
function frame() {
  if (dirty && !state.paused) {
    dirty = false;
    if (state.tab === 'live') updateLive();
  }
  if (state.tab === 'model') updateModel();
  requestAnimationFrame(frame);
}

let lastLog = '';
function connect() {
  const es = new EventSource('/api/stream');
  es.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    state.status = msg.status;
    if (msg.cols && !state.paused) { appendCols(msg.cols); dirty = true; }
    if (msg.latest) state.latest = msg.latest;
    renderChips();
    const log = msg.status.log || [];
    const tail = log.length ? log[log.length - 1] : '';
    if (tail !== lastLog) {
      lastLog = tail;
      // 控制板回報參數版本時重新讀取 A。
      if (log.length && /參數版本|已送出參數/.test(log[log.length - 1])) refreshState();
      const el = document.getElementById('log');
      if (el) el.textContent = log.slice(-20).reverse().join('\n');
    }
  };
  es.onerror = () => { document.getElementById('chips').innerHTML = chip('伺服器', '連線中斷', 'bad'); };
}

async function main() {
  await refreshState();
  initHeader();
  document.getElementById('src-type').value = state.status.source?.type || 'sim';
  await fillSourceArgs();
  initLive();
  initModel();
  initTuning();
  initScenario();
  connect();
  requestAnimationFrame(frame);
}

main();
