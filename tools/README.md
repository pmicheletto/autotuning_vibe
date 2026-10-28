# tools/live_plot.py — real-time plot (PlotJuggler replacement)

Receives the UDP JSON samples emitted by `sim` and serves a live plot in the
browser. Standard library only (matplotlib is optional, for PNG export).

`sim` starts it automatically and prints the URL:

```
$ ./build/sim
Live plot: http://127.0.0.1:8088/ (closes with this program; log /workspace/live_plot.log)
```

The plotter lives exactly as long as `sim`: it is stopped on normal exit and,
because it watches a pipe the kernel closes when `sim` dies, it also exits on
Ctrl-C, crashes or `kill -9` — nothing is left holding the ports. The browser
tab keeps the data it already received, so the finished run can still be
zoomed/panned after `sim` has exited. Use `--keep-plot` to have `sim` wait for
Enter after the report (the plot stays live until then).

Disable with `--no-plot` or `AUTOTUNING_NO_PLOT=1`; change the port with
`--plot-port N` (`0` = pick a free port; if the requested port is busy a free
one is used and reported). Without a graphical session the browser is not
opened, but the URL still works (e.g. over an SSH tunnel).

## Standalone

```bash
python3 tools/live_plot.py                      # UDP 9870, HTTP 8088, opens a browser
python3 tools/live_plot.py --headless           # no browser
python3 tools/live_plot.py --udp-port 9871 --http-port 9000
python3 tools/live_plot.py --idle-exit 60       # quit 60 s after the last sample
```

## Page

* **Follow live** / **Pause**, time **Window**, **Fit all**; mouse wheel zooms
  the time axis, drag pans, hovering shows all channel values at the cursor.
* Series are auto-grouped into charts (speed/reference, jerk, other) and can be
  toggled individually.
* The `test_phase` field is drawn as coloured background bands
  (identification, WaitTouchRef, Validate, ...).
* **PNG snapshot** renders the whole run server-side (`/snapshot.png`).
* **Stop server** terminates the tool early (standalone use; when launched by
  `sim` it stops together with `sim` anyway).

## Protocol

Any UDP datagram with one or more JSON objects (one per line) is accepted:
numeric fields become series, string fields become phase bands, `time` is the
x axis (falling back to arrival order). Bare `nan`/`inf` tokens from C/C++
producers are repaired to `null`. Endpoints: `/data?since=N`, `/stats`,
`/snapshot.png`, `POST /shutdown`. `--exit-with-parent` makes the tool exit
when its stdin reaches EOF (how `sim` binds the plotter's lifetime to its own).
