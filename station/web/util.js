// Small helpers shared by the drawing code.

// Size a canvas' pixel buffer to its on-screen size and return a cleared
// context that draws in CSS pixels. The on-screen size comes from the page
// layout (the canvas fills its box), never from the canvas' own attributes,
// so a display scale of 125% or 150% cannot make it grow.
export function fitCanvas(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  const pw = Math.round(w * dpr), ph = Math.round(h * dpr);
  if (canvas.width !== pw || canvas.height !== ph) { canvas.width = pw; canvas.height = ph; }
  const g = canvas.getContext('2d');
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, w, h);
  return { g, w, h };
}

// Theme colours for canvas drawing, read from the CSS variables so the
// canvases follow the dark / light switch. Call refreshColors() after a switch.
export const colors = {};
export function refreshColors() {
  const css = getComputedStyle(document.documentElement);
  for (const name of ['bg', 'panel', 'border', 'border-soft', 'fg', 'muted', 'accent',
                      'grid', 'axis', 's1', 's2', 's3', 's4']) {
    colors[name] = css.getPropertyValue('--' + name).trim();
  }
  colors.series = [colors.s1, colors.s2, colors.s3, colors.s4];
}

export const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

// "+0.123" / "-0.123": the sign always shown, so columns of numbers line up.
export const signed = (v, digits) => (v < 0 ? '-' : '+') + Math.abs(v).toFixed(digits);

// A "nice" step (1, 2 or 5 times a power of ten) near the wanted one.
export function niceStep(rough) {
  const power = Math.pow(10, Math.floor(Math.log10(rough)));
  const f = rough / power;
  return (f < 1.5 ? 1 : f < 3.5 ? 2 : f < 7.5 ? 5 : 10) * power;
}

export const $ = id => document.getElementById(id);
