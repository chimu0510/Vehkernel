// uPlot 的薄包裝：深色主題、隨容器寬度縮放、可選對數 x 軸。
/* global uPlot */

const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
export const COLORS = {
  a: css('--c-a'), m: css('--c-m'), b: css('--c-b'), bm: css('--c-bm'), t: css('--c-t'), f: css('--c-f'),
  x: css('--x'), y: css('--y'), z: css('--z'), muted: css('--muted'), warn: css('--warn'),
};

const AXIS = {
  stroke: '#8b95a3',
  grid: { stroke: '#232a33', width: 1 },
  ticks: { stroke: '#2a313b', width: 1 },
  font: '11px system-ui',
};

// 軸刻度：uPlot 的對數軸會對次要刻度傳 null，留白；對數軸只標 10 的次方。
function tick(log) {
  return (u, vals) => vals.map((v) => {
    if (v == null) return '';
    if (log) {
      const e = Math.log10(Math.abs(v));
      if (Math.abs(e - Math.round(e)) > 1e-6) return '';
      return Math.abs(e) <= 3 ? String(+v.toPrecision(1)) : `1e${Math.round(e)}`;
    }
    return fmt(v);
  });
}

function fmt(v) {
  if (v == null || !isFinite(v)) return '—';
  const a = Math.abs(v);
  if (a !== 0 && (a < 1e-3 || a >= 1e5)) return v.toExponential(2);
  return v.toPrecision(4).replace(/\.?0+$/, '');
}

/**
 * card：放圖的容器；opt.series：[{label, color, dash, width, scale}]；
 * opt.logX / opt.logY；opt.yLabel；opt.height；opt.xLabel；opt.yRange [min,max]；opt.y2 第二個 y 軸標籤。
 */
export function makeChart(card, opt) {
  const el = document.createElement('div');
  card.appendChild(el);
  const series = [{ label: opt.xLabel || 't' , value: (u, v) => fmt(v) }];
  for (const s of opt.series) {
    series.push({
      label: s.label,
      stroke: s.color,
      width: s.width ?? 1.4,
      dash: s.dash,
      scale: s.scale || 'y',
      points: { show: s.points ?? false, size: 5, fill: s.color },
      paths: s.points ? () => null : undefined,
      value: (u, v) => fmt(v),
      spanGaps: s.spanGaps ?? false,
      show: s.show ?? true,
    });
  }
  const scales = {
    x: { time: false, distr: opt.logX ? 3 : 1, ...(opt.xRange ? { range: opt.xRange } : {}) },
    y: { distr: opt.logY ? 3 : 1, ...(opt.yRange ? { range: opt.yRange } : {}) },
  };
  const axes = [
    { ...AXIS, label: opt.xLabel, labelSize: opt.xLabel ? 16 : 0, values: tick(opt.logX) },
    { ...AXIS, label: opt.yLabel, labelSize: 16, size: 56, values: tick(opt.logY) },
  ];
  if (opt.y2) {
    scales.y2 = { distr: opt.logY2 ? 3 : 1, ...(opt.y2Range ? { range: opt.y2Range } : {}) };
    axes.push({ ...AXIS, scale: 'y2', side: 1, label: opt.y2, labelSize: 16, size: 56, grid: { show: false }, values: tick(opt.logY2) });
  }
  const u = new uPlot({
    width: Math.max(200, card.clientWidth - 4),
    height: opt.height || 200,
    series, scales, axes,
    bands: opt.bands,
    cursor: { drag: { x: true, y: false, setScale: true } },
    legend: { live: true },
  }, [[], ...opt.series.map(() => [])], el);
  const ro = new ResizeObserver(() => {
    const w = Math.max(200, card.clientWidth - 4);
    if (Math.abs(w - u.width) > 2) u.setSize({ width: w, height: u.height });
  });
  ro.observe(card);
  return {
    u,
    set(data, resetScales = true) { u.setData(data, resetScales); },
  };
}

/**
 * 建立一張圖卡。說明預設收起，按標題列的 ⓘ 展開（或用頂端「顯示所有說明」全部展開）。
 * opt.wide：在兩欄的格線中占滿整列。
 */
export function chartCard(parent, title, note, opt = {}) {
  const card = document.createElement('div');
  card.className = `card chart-card${opt.wide ? ' wide' : ''}`;
  card.innerHTML = `<div class="card-h"><span class="ttl">${title}</span></div>`;
  parent.appendChild(card);
  const body = document.createElement('div');
  card.appendChild(body);
  if (note) {
    const btn = document.createElement('button');
    btn.className = 'info';
    btn.title = '說明';
    btn.textContent = 'ⓘ';
    card.querySelector('.card-h').appendChild(btn);
    const n = document.createElement('div');
    n.className = 'chart-note';
    n.innerHTML = note;
    card.appendChild(n);
    btn.onclick = () => card.classList.toggle('open');
  }
  return body;
}

/** 一組圖的區段標題與格線容器。 */
export function section(parent, title, desc) {
  const h = document.createElement('div');
  h.className = 'sec-h';
  h.innerHTML = `<h2>${title}</h2>${desc ? `<span>${desc}</span>` : ''}`;
  parent.appendChild(h);
  const grid = document.createElement('div');
  grid.className = 'charts';
  parent.appendChild(grid);
  return grid;
}

export { fmt };
