#!/usr/bin/env python3
"""Real-time plot of the PID autotuning run (PlotJuggler replacement).

Listens for the UDP JSON samples emitted by the `sim` binary and serves a live
plot in the browser. Uses only the Python standard library; matplotlib is
optional and used just for the PNG snapshot endpoint.

Typical use: `sim` starts this automatically. Standalone:

    python3 tools/live_plot.py                 # listen on 9870, open a browser
    python3 tools/live_plot.py --headless      # no browser, just serve
    python3 tools/live_plot.py --http-port 9000

Any JSON object received is plotted: numeric fields become series, string
fields become phase bands, so it also works for other producers.

When started by `sim` (--exit-with-parent) the tool exits as soon as `sim`
does, however `sim` ends: it reads its stdin, a pipe that the kernel closes
when the parent process dies.
"""

import argparse
import json
import re
import signal
import math
import socket
import sys
import threading
import time
import webbrowser
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

DEFAULT_UDP_PORT = 9870
DEFAULT_HTTP_PORT = 8088
# Field used as the x axis when present in the samples.
TIME_FIELD = "time"


# Producers sometimes emit bare nan/inf (e.g. C++ std::to_string of a NaN),
# which is not valid JSON; accept it by turning those tokens into null.
_NON_FINITE_RE = re.compile(r'(?<![\w."])-?(?:nan|inf(?:inity)?)(?![\w."])', re.IGNORECASE)


def parse_sample(line):
    """Parses one JSON object, repairing bare nan/inf tokens if needed.

    Returns (payload, repaired). Raises json.JSONDecodeError if unparsable.
    """
    try:
        return json.loads(line), False
    except json.JSONDecodeError:
        return json.loads(_NON_FINITE_RE.sub("null", line)), True


class SampleStore:
    """Thread-safe store of incoming samples, ordered by arrival."""

    def __init__(self, max_samples):
        self._lock = threading.Lock()
        self._rows = deque(maxlen=max_samples)
        self._series = []            # numeric field names, in first-seen order
        self._state_fields = []      # string field names (e.g. test_phase)
        self._dropped = 0            # samples evicted by max_samples
        self._received = 0
        self._errors = 0
        self._repaired = 0
        self._first_wall = None
        self._last_wall = None
        self._cursor = 0             # monotonic index of the next row

    def add(self, payload):
        with self._lock:
            if not isinstance(payload, dict):
                self._errors += 1
                return
            for key, value in payload.items():
                if isinstance(value, bool):
                    continue
                if isinstance(value, (int, float)) or value is None:
                    if key != TIME_FIELD and key not in self._series:
                        self._series.append(key)
                elif isinstance(value, str):
                    if key not in self._state_fields:
                        self._state_fields.append(key)

            # Non-finite values would be re-serialised as invalid JSON for the
            # browser; store them as null (a gap in the plotted line).
            payload = {k: (None if isinstance(v, float) and not math.isfinite(v) else v)
                       for k, v in payload.items()}

            x = payload.get(TIME_FIELD)
            if not isinstance(x, (int, float)):
                x = self._cursor
            row = {
                "i": self._cursor,
                "x": float(x),
                "v": {k: payload.get(k) for k in self._series},
                "s": {k: payload.get(k) for k in self._state_fields},
            }
            if len(self._rows) == self._rows.maxlen:
                self._dropped += 1
            self._rows.append(row)
            self._cursor += 1
            self._received += 1
            now = time.time()
            if self._first_wall is None:
                self._first_wall = now
            self._last_wall = now

    def note_error(self):
        with self._lock:
            self._errors += 1

    def note_repaired(self):
        with self._lock:
            self._repaired += 1

    def snapshot(self, since, limit):
        """Returns rows with index >= `since` (at most `limit`)."""
        with self._lock:
            rows = [r for r in self._rows if r["i"] >= since]
            truncated = len(rows) > limit
            rows = rows[:limit]
            series = list(self._series)
            states = list(self._state_fields)
            cursor = rows[-1]["i"] + 1 if rows else max(since, self._first_index_locked())
            payload = {
                "cursor": cursor,
                "series": series,
                "states": states,
                "rows": [[r["x"]] + [r["v"].get(k) for k in series] for r in rows],
                "indices": [r["i"] for r in rows],
                "state_rows": [[r["s"].get(k) for k in states] for r in rows],
                "truncated": truncated,
                "stats": self._stats_locked(),
            }
            return payload

    def all_rows(self):
        with self._lock:
            return list(self._rows), list(self._series), list(self._state_fields)

    def _first_index_locked(self):
        return self._rows[0]["i"] if self._rows else 0

    def _stats_locked(self):
        rate = 0.0
        if self._first_wall and self._last_wall and self._last_wall > self._first_wall:
            rate = (self._received - 1) / (self._last_wall - self._first_wall)
        return {
            "received": self._received,
            "stored": len(self._rows),
            "dropped": self._dropped,
            "errors": self._errors,
            "repaired": self._repaired,
            "rate_hz": round(rate, 2),
            "age_s": round(time.time() - self._last_wall, 2) if self._last_wall else None,
        }

    def stats(self):
        with self._lock:
            return self._stats_locked()

    @property
    def last_wall(self):
        return self._last_wall


class UdpReceiver(threading.Thread):
    """Receives newline/datagram delimited JSON objects."""

    def __init__(self, host, port, store):
        super().__init__(daemon=True)
        self.store = store
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)
        self.sock.bind((host, port))
        self._stop = threading.Event()

    def run(self):
        while not self._stop.is_set():
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            for line in data.decode("utf-8", "replace").splitlines():
                line = line.strip()
                if not line:
                    continue
                try:
                    payload, repaired = parse_sample(line)
                except json.JSONDecodeError:
                    self.store.note_error()
                    continue
                if repaired:
                    self.store.note_repaired()
                self.store.add(payload)

    def stop(self):
        self._stop.set()
        try:
            self.sock.close()
        except OSError:
            pass


def render_png(store, title):
    """Static PNG of everything received so far (requires matplotlib)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows, series, states = store.all_rows()
    if not rows:
        raise ValueError("no samples received yet")

    xs = [r["x"] for r in rows]
    groups = group_series(series)
    fig, axes = plt.subplots(len(groups), 1, sharex=True, figsize=(11, 3 * len(groups)))
    if len(groups) == 1:
        axes = [axes]

    phase_field = states[0] if states else None
    for ax, (group_name, names) in zip(axes, groups):
        if phase_field:
            start = 0
            for idx in range(1, len(rows) + 1):
                at_end = idx == len(rows)
                if at_end or rows[idx]["s"].get(phase_field) != rows[start]["s"].get(phase_field):
                    phase = rows[start]["s"].get(phase_field)
                    if phase:
                        ax.axvspan(xs[start], xs[idx - 1],
                                   color=_phase_color(phase), alpha=0.12, linewidth=0)
                    start = idx
        for name in names:
            pts = [(r["x"], r["v"].get(name)) for r in rows
                   if isinstance(r["v"].get(name), (int, float))]
            if pts:
                ax.plot([p[0] for p in pts], [p[1] for p in pts], linewidth=1.4, label=name)
        ax.set_ylabel(group_name)
        ax.grid(True, linestyle="--", alpha=0.3)
        ax.legend(loc="upper left", fontsize=8, frameon=False)
    axes[-1].set_xlabel("time (s)")
    axes[0].set_title(title)
    fig.tight_layout()

    from io import BytesIO
    buf = BytesIO()
    fig.savefig(buf, format="png", dpi=110)
    plt.close(fig)
    return buf.getvalue()


def _phase_color(name):
    palette = ["#1f77b4", "#2ca02c", "#ff7f0e", "#9467bd", "#8c564b", "#e377c2", "#17becf"]
    return palette[sum(ord(c) for c in name) % len(palette)]


def group_series(series):
    """Groups series into charts by name, so related signals share an axis."""
    speed, jerk, other = [], [], []
    for name in series:
        low = name.lower()
        if "jerk" in low:
            jerk.append(name)
        elif any(k in low for k in ("speed", "step", "ref", "out", "setpoint")):
            speed.append(name)
        else:
            other.append(name)
    groups = []
    if speed:
        groups.append(("speed / reference", speed))
    if jerk:
        groups.append(("jerk", jerk))
    if other:
        groups.append(("other", other))
    return groups or [("signals", series)]


class Handler(BaseHTTPRequestHandler):
    store = None
    title = "PID autotuning - live"
    shutdown_event = None
    server_version = "AutotuningLivePlot/1.0"

    def log_message(self, fmt, *args):  # keep the console clean
        pass

    def _send(self, code, content_type, body, extra_headers=()):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for key, value in extra_headers:
            self.send_header(key, value)
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query)
        if parsed.path in ("/", "/index.html"):
            self._send(200, "text/html; charset=utf-8", PAGE_HTML.replace("__TITLE__", self.title).encode())
        elif parsed.path == "/data":
            since = int(query.get("since", ["0"])[0])
            limit = min(int(query.get("limit", ["20000"])[0]), 200000)
            body = json.dumps(self.store.snapshot(since, limit)).encode()
            self._send(200, "application/json", body)
        elif parsed.path == "/stats":
            self._send(200, "application/json", json.dumps(self.store.stats()).encode())
        elif parsed.path == "/snapshot.png":
            try:
                self._send(200, "image/png", render_png(self.store, self.title))
            except Exception as exc:  # matplotlib missing or no data yet
                self._send(503, "text/plain; charset=utf-8", f"snapshot unavailable: {exc}".encode())
        else:
            self._send(404, "text/plain; charset=utf-8", b"not found")

    def do_POST(self):
        if urlparse(self.path).path == "/shutdown":
            self._send(200, "text/plain; charset=utf-8", b"bye")
            self.shutdown_event.set()
        else:
            self._send(404, "text/plain; charset=utf-8", b"not found")


PAGE_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>__TITLE__</title>
<style>
  :root { color-scheme: dark; }
  body { margin: 0; background: #12151b; color: #e6e6e6;
         font: 13px/1.4 -apple-system, Segoe UI, Roboto, Helvetica, Arial, sans-serif; }
  header { display: flex; align-items: center; gap: 14px; flex-wrap: wrap;
           padding: 8px 14px; background: #1b2029; border-bottom: 1px solid #2c333f; }
  h1 { font-size: 14px; margin: 0 10px 0 0; font-weight: 600; }
  label { display: inline-flex; align-items: center; gap: 5px; white-space: nowrap; }
  button, input[type=number] { background: #232a35; color: #e6e6e6; border: 1px solid #39424f;
           border-radius: 4px; padding: 3px 8px; font: inherit; }
  button:hover { background: #2c3542; cursor: pointer; }
  button:disabled { opacity: 0.4; cursor: default; }
  button.on { background: #2f6f4f; border-color: #3c8a63; }
  #series { display: flex; gap: 12px; flex-wrap: wrap; padding: 6px 14px;
            background: #171c24; border-bottom: 1px solid #2c333f; }
  .sw { width: 11px; height: 11px; border-radius: 2px; display: inline-block; }
  #charts { padding: 6px 10px 14px; }
  .chart { position: relative; margin-bottom: 10px; }
  canvas { width: 100%; display: block; background: #161a21;
           border: 1px solid #262d38; border-radius: 6px; }
  #status { margin-left: auto; color: #9aa7b8; font-variant-numeric: tabular-nums; }
  #phase { font-weight: 600; }
  #tip { position: fixed; pointer-events: none; background: #0d1016ee; border: 1px solid #39424f;
         border-radius: 5px; padding: 6px 8px; display: none; white-space: pre; font-variant-numeric: tabular-nums; }
</style>
</head>
<body>
<header>
  <h1>__TITLE__</h1>
  <button id="follow" class="on">Follow live</button>
  <button id="pause">Pause</button>
  <label>Window <input id="window" type="number" value="60" min="1" step="5" style="width:72px"> s</label>
  <button id="fit">Fit all</button>
  <button id="snap">PNG snapshot</button>
  <button id="stop">Stop server</button>
  <span id="status">waiting for data…</span>
</header>
<div id="series"></div>
<div id="charts"></div>
<div id="tip"></div>
<script>
// ---------------------------------------------------------------- helpers
function niceTicks(min, max, count) {
  if (!isFinite(min) || !isFinite(max)) return [];
  if (min === max) { min -= 0.5; max += 0.5; }
  const raw = (max - min) / Math.max(1, count);
  const mag = Math.pow(10, Math.floor(Math.log10(raw)));
  const norm = raw / mag;
  const step = (norm < 1.5 ? 1 : norm < 3 ? 2 : norm < 7 ? 5 : 10) * mag;
  const ticks = [];
  for (let v = Math.ceil(min / step) * step; v <= max + step * 1e-6; v += step) {
    ticks.push(Math.abs(v) < step * 1e-6 ? 0 : v);
  }
  return ticks;
}
function phaseColor(name) {
  const palette = ['#1f77b4','#2ca02c','#ff7f0e','#9467bd','#8c564b','#e377c2','#17becf'];
  let sum = 0;
  for (const ch of name) sum += ch.charCodeAt(0);
  return palette[sum % palette.length];
}
function seriesColor(i) {
  const palette = ['#4fa3ff','#ffd166','#ff6b6b','#06d6a0','#c77dff','#f78c6b','#8ecae6'];
  return palette[i % palette.length];
}
function yRange(rows, cols, x0, x1) {
  let lo = Infinity, hi = -Infinity;
  for (const r of rows) {
    if (r[0] < x0 || r[0] > x1) continue;
    for (const c of cols) {
      const v = r[c];
      if (typeof v === 'number' && isFinite(v)) { if (v < lo) lo = v; if (v > hi) hi = v; }
    }
  }
  if (lo === Infinity) return null;
  if (lo === hi) return [lo - 1, hi + 1];
  const pad = (hi - lo) * 0.08;
  return [lo - pad, hi + pad];
}
function groupSeries(names) {
  const speed = [], jerk = [], other = [];
  for (const n of names) {
    const l = n.toLowerCase();
    if (l.includes('jerk')) jerk.push(n);
    else if (['speed','step','ref','out','setpoint'].some(k => l.includes(k))) speed.push(n);
    else other.push(n);
  }
  const groups = [];
  if (speed.length) groups.push({ title: 'speed / reference', names: speed });
  if (jerk.length) groups.push({ title: 'jerk', names: jerk });
  if (other.length) groups.push({ title: 'other', names: other });
  return groups.length ? groups : [{ title: 'signals', names: names }];
}
if (typeof module !== 'undefined') { module.exports = { niceTicks, phaseColor, yRange, groupSeries }; }

// ---------------------------------------------------------------- state
const S = { rows: [], states: [], series: [], stateNames: [], cursor: 0,
            hidden: new Set(), follow: true, paused: false, view: null, stats: {},
            groups: [], charts: [], mouse: null, lost: 0 };

async function poll() {
  if (!S.paused) {
    try {
      const res = await fetch('/data?since=' + S.cursor);
      const d = await res.json();
      if (d.series.length !== S.series.length) { S.series = d.series; rebuild(); }
      S.stateNames = d.states;
      for (let i = 0; i < d.rows.length; i++) { S.rows.push(d.rows[i]); S.states.push(d.state_rows[i]); }
      S.cursor = d.cursor;
      S.stats = d.stats;
      S.lost = 0;
      draw();
    } catch (e) {
      // Server gone (sim finished): keep the data already loaded, stop polling.
      if (++S.lost >= 3) { disconnected(); return; }
    }
  }
  setTimeout(poll, 100);
}

function disconnected() {
  S.paused = true;
  for (const id of ['pause', 'snap', 'stop']) document.getElementById(id).disabled = true;
  const st = document.getElementById('status');
  st.innerHTML = 'run finished — server closed · ' + (S.stats.received || 0) +
                 ' samples kept in this tab (zoom/pan still work)';
  st.style.color = '#f2c14e';
}

function rebuild() {
  S.groups = groupSeries(S.series);
  const box = document.getElementById('series');
  box.innerHTML = '';
  S.series.forEach((name, i) => {
    const id = 'cb' + i;
    const el = document.createElement('label');
    el.innerHTML = '<input type="checkbox" id="' + id + '" ' + (S.hidden.has(name) ? '' : 'checked') +
      '><span class="sw" style="background:' + seriesColor(i) + '"></span>' + name;
    el.querySelector('input').onchange = (ev) => {
      if (ev.target.checked) S.hidden.delete(name); else S.hidden.add(name);
      draw();
    };
    box.appendChild(el);
  });
  const charts = document.getElementById('charts');
  charts.innerHTML = '';
  S.charts = S.groups.map(g => {
    const wrap = document.createElement('div');
    wrap.className = 'chart';
    const cv = document.createElement('canvas');
    cv.height = 230;
    wrap.appendChild(cv);
    charts.appendChild(wrap);
    return { canvas: cv, group: g };
  });
}

function xDomain() {
  if (!S.rows.length) return [0, 1];
  const last = S.rows[S.rows.length - 1][0], first = S.rows[0][0];
  if (!S.follow && S.view) return S.view;
  const w = Math.max(1e-6, parseFloat(document.getElementById('window').value) || 60);
  return [Math.max(first, last - w), Math.max(last, first + 1e-6)];
}

function draw() {
  const [x0, x1] = xDomain();
  const phaseIdx = 0;
  for (const ch of S.charts) {
    const cv = ch.canvas, dpr = window.devicePixelRatio || 1;
    const w = cv.clientWidth, h = cv.clientHeight;
    if (cv.width !== w * dpr || cv.height !== h * dpr) { cv.width = w * dpr; cv.height = h * dpr; }
    const g = cv.getContext('2d');
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, w, h);
    const L = 62, R = 12, T = 18, B = 26;
    const cols = ch.group.names.map(n => ({ name: n, col: S.series.indexOf(n) + 1 }))
                              .filter(c => !S.hidden.has(c.name));
    const yr = yRange(S.rows, cols.map(c => c.col), x0, x1) || [0, 1];
    const X = t => L + (t - x0) / (x1 - x0) * (w - L - R);
    const Y = v => T + (1 - (v - yr[0]) / (yr[1] - yr[0])) * (h - T - B);

    // phase bands
    if (S.stateNames.length) {
      let start = null;
      for (let i = 0; i <= S.rows.length; i++) {
        const name = i < S.rows.length ? (S.states[i] || [])[phaseIdx] : null;
        if (start === null) { start = i; continue; }
        const prev = (S.states[start] || [])[phaseIdx];
        if (i === S.rows.length || name !== prev) {
          if (prev && prev !== '-' && S.rows[i - 1][0] >= x0 && S.rows[start][0] <= x1) {
            g.fillStyle = phaseColor(prev) + '22';
            const xa = Math.max(L, X(S.rows[start][0])), xb = Math.min(w - R, X(S.rows[i - 1][0]));
            g.fillRect(xa, T, Math.max(1, xb - xa), h - T - B);
            if (xb - xa > 42) {
              g.fillStyle = phaseColor(prev) + 'cc';
              g.font = '10px sans-serif';
              g.fillText(prev, xa + 3, T + 11);
            }
          }
          start = i;
        }
      }
    }

    // grid + axes
    g.strokeStyle = '#262d38'; g.fillStyle = '#8b97a8'; g.font = '10px sans-serif'; g.lineWidth = 1;
    for (const t of niceTicks(yr[0], yr[1], 5)) {
      const y = Y(t);
      g.beginPath(); g.moveTo(L, y); g.lineTo(w - R, y); g.stroke();
      g.fillText(t.toPrecision(4).replace(/\.?0+$/, ''), 6, y + 3);
    }
    for (const t of niceTicks(x0, x1, 8)) {
      const x = X(t);
      if (x < L || x > w - R) continue;
      g.beginPath(); g.moveTo(x, T); g.lineTo(x, h - B); g.stroke();
      g.fillText(t.toFixed(1), x - 10, h - 9);
    }
    g.strokeStyle = '#3b4554';
    g.strokeRect(L, T, w - L - R, h - T - B);
    g.fillStyle = '#c6d0dc'; g.fillText(ch.group.title, L + 4, T - 6);

    // series
    cols.forEach(c => {
      g.strokeStyle = seriesColor(S.series.indexOf(c.name));
      g.lineWidth = 1.6;
      g.beginPath();
      let pen = false;
      for (const r of S.rows) {
        if (r[0] < x0 || r[0] > x1) { pen = false; continue; }
        const v = r[c.col];
        if (typeof v !== 'number' || !isFinite(v)) { pen = false; continue; }
        const x = X(r[0]), y = Y(v);
        if (pen) g.lineTo(x, y); else { g.moveTo(x, y); pen = true; }
      }
      g.stroke();
    });

    // crosshair
    if (S.mouse && S.mouse.canvas === cv) {
      g.strokeStyle = '#6b7686'; g.setLineDash([3, 3]);
      g.beginPath(); g.moveTo(S.mouse.x, T); g.lineTo(S.mouse.x, h - B); g.stroke();
      g.setLineDash([]);
    }
  }
  const st = S.stats, last = S.rows.length ? S.rows[S.rows.length - 1] : null;
  const phase = last && S.states.length ? (S.states[S.states.length - 1] || [])[0] : null;
  document.getElementById('status').innerHTML =
    (last ? 't = ' + last[0].toFixed(2) + ' s · ' : '') +
    (phase ? 'phase <span id="phase" style="color:' + phaseColor(phase) + '">' + phase + '</span> · ' : '') +
    (st.received || 0) + ' samples · ' + (st.rate_hz || 0) + ' Hz' +
    (st.dropped ? ' · dropped ' + st.dropped : '') +
    (st.repaired ? ' · repaired ' + st.repaired : '') +
    (st.errors ? ' · bad ' + st.errors : '');
}

// ---------------------------------------------------------------- interaction
function nearestRow(t) {
  let best = null, bestD = Infinity;
  for (let i = 0; i < S.rows.length; i++) {
    const d = Math.abs(S.rows[i][0] - t);
    if (d < bestD) { bestD = d; best = i; }
  }
  return best;
}
document.addEventListener('mousemove', ev => {
  const tip = document.getElementById('tip');
  const ch = S.charts.find(c => c.canvas === ev.target);
  if (!ch || !S.rows.length) { tip.style.display = 'none'; S.mouse = null; draw(); return; }
  const rect = ch.canvas.getBoundingClientRect();
  const L = 62, R = 12;
  const [x0, x1] = xDomain();
  const t = x0 + (ev.clientX - rect.left - L) / (rect.width - L - R) * (x1 - x0);
  const i = nearestRow(t);
  S.mouse = { canvas: ch.canvas, x: ev.clientX - rect.left };
  let text = 't = ' + S.rows[i][0].toFixed(3) + ' s';
  S.series.forEach((n, k) => {
    const v = S.rows[i][k + 1];
    if (!S.hidden.has(n)) text += '\n' + n + ': ' + (typeof v === 'number' ? v.toPrecision(6) : '—');
  });
  (S.states[i] || []).forEach((s, k) => { if (s) text += '\n' + S.stateNames[k] + ': ' + s; });
  tip.textContent = text;
  tip.style.display = 'block';
  tip.style.left = Math.min(window.innerWidth - 220, ev.clientX + 14) + 'px';
  tip.style.top = (ev.clientY + 14) + 'px';
  draw();
});
document.getElementById('charts').addEventListener('wheel', ev => {
  if (!S.rows.length) return;
  ev.preventDefault();
  const [x0, x1] = xDomain();
  const rect = ev.target.getBoundingClientRect();
  const L = 62, R = 12;
  const focus = x0 + (ev.clientX - rect.left - L) / (rect.width - L - R) * (x1 - x0);
  const k = ev.deltaY > 0 ? 1.25 : 0.8;
  S.view = [focus - (focus - x0) * k, focus + (x1 - focus) * k];
  setFollow(false);
  draw();
}, { passive: false });

let drag = null;
document.getElementById('charts').addEventListener('mousedown', ev => {
  drag = { x: ev.clientX, view: xDomain(), width: ev.target.getBoundingClientRect().width };
});
document.addEventListener('mouseup', () => { drag = null; });
document.addEventListener('mousemove', ev => {
  if (!drag) return;
  const [x0, x1] = drag.view;
  const dx = (ev.clientX - drag.x) / (drag.width - 74) * (x1 - x0);
  S.view = [x0 - dx, x1 - dx];
  setFollow(false);
  draw();
});

function setFollow(on) {
  S.follow = on;
  document.getElementById('follow').classList.toggle('on', on);
  if (on) S.view = null;
}
document.getElementById('follow').onclick = () => { setFollow(!S.follow); draw(); };
document.getElementById('pause').onclick = (ev) => {
  S.paused = !S.paused;
  ev.target.classList.toggle('on', S.paused);
  ev.target.textContent = S.paused ? 'Resume' : 'Pause';
};
document.getElementById('fit').onclick = () => {
  if (S.rows.length) { S.view = [S.rows[0][0], S.rows[S.rows.length - 1][0]]; setFollow(false); draw(); }
};
document.getElementById('snap').onclick = () => window.open('/snapshot.png', '_blank');
document.getElementById('stop').onclick = async () => {
  await fetch('/shutdown', { method: 'POST' });
  document.getElementById('status').textContent = 'server stopped';
};
document.getElementById('window').onchange = () => { setFollow(true); draw(); };
window.addEventListener('resize', draw);
rebuild();
poll();
</script>
</body>
</html>
"""


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--udp-host", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=DEFAULT_UDP_PORT)
    parser.add_argument("--http-host", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=DEFAULT_HTTP_PORT,
                        help="0 picks a free port")
    parser.add_argument("--title", default="PID autotuning - live")
    parser.add_argument("--max-samples", type=int, default=500000)
    parser.add_argument("--headless", action="store_true", help="do not open a browser")
    parser.add_argument("--idle-exit", type=float, default=0.0,
                        help="exit after N seconds without new samples (0 = never)")
    parser.add_argument("--exit-with-parent", action="store_true",
                        help="exit when stdin reaches EOF (the launcher holds a pipe open)")
    args = parser.parse_args(argv)

    store = SampleStore(args.max_samples)
    try:
        receiver = UdpReceiver(args.udp_host, args.udp_port, store)
    except OSError as exc:
        print(f"live_plot: cannot bind UDP {args.udp_host}:{args.udp_port}: {exc}", file=sys.stderr)
        return 2
    receiver.start()

    shutdown_event = threading.Event()
    Handler.store = store
    Handler.title = args.title
    Handler.shutdown_event = shutdown_event
    httpd = bind_http(args.http_host, args.http_port)
    if httpd is None:
        receiver.stop()
        return 2
    httpd.daemon_threads = True
    url = f"http://{args.http_host}:{httpd.server_address[1]}/"

    # Stop cleanly on SIGTERM/SIGINT (the launcher sends SIGTERM on exit).
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: shutdown_event.set())
    if args.exit_with_parent:
        threading.Thread(target=watch_parent, args=(shutdown_event,), daemon=True).start()

    threading.Thread(target=httpd.serve_forever, kwargs={"poll_interval": 0.2}, daemon=True).start()
    # Handshake line for the launcher (and humans).
    print(f"LIVEPLOT_READY {url} udp={args.udp_host}:{args.udp_port}", flush=True)

    if not args.headless:
        try:
            webbrowser.open(url)
        except Exception:
            pass

    try:
        while not shutdown_event.wait(0.25):
            if args.idle_exit > 0 and store.last_wall and \
                    time.time() - store.last_wall > args.idle_exit:
                print("live_plot: idle timeout, exiting", flush=True)
                break
    finally:
        httpd.shutdown()
        receiver.stop()
        print("live_plot: stopped", flush=True)
    return 0


def bind_http(host, port):
    """Binds the HTTP server; falls back to an OS-chosen port if `port` is busy."""
    try:
        return ThreadingHTTPServer((host, port), Handler)
    except OSError as exc:
        if port != 0:
            print(f"live_plot: HTTP port {port} is busy ({exc}); using a free port instead",
                  flush=True)
            try:
                return ThreadingHTTPServer((host, 0), Handler)
            except OSError as exc2:
                exc = exc2
        print(f"live_plot: cannot bind HTTP {host}:{port}: {exc}", file=sys.stderr)
        return None


def watch_parent(shutdown_event):
    """Blocks until stdin hits EOF (launcher closed the pipe or died), then stops."""
    try:
        while sys.stdin.buffer.read(4096):
            pass
    except (OSError, ValueError):
        pass
    print("live_plot: launcher exited, shutting down", flush=True)
    shutdown_event.set()


if __name__ == "__main__":
    sys.exit(main())
